#define _GNU_SOURCE

#include "camera_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define CAMERA_BUFFER_COUNT 4

struct mapped_buffer {
    void *start;
    size_t length;
};

static int camera_fd = -1;
static struct mapped_buffer mapped_buffers[CAMERA_BUFFER_COUNT];
static unsigned int mapped_buffer_count = 0;
static uint8_t *secure_image_memory = NULL;
static int streaming_enabled = 0;

static int xioctl(unsigned long request, void *arg)
{
    int r;

    do {
        r = ioctl(camera_fd, request, arg);
    } while (r == -1 && errno == EINTR);

    return r;
}

static uint64_t timeval_to_ns(const struct timeval *tv)
{
    return (uint64_t)tv->tv_sec * 1000000000ULL +
           (uint64_t)tv->tv_usec * 1000ULL;
}

static uint64_t now_monotonic_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }

    return (uint64_t)ts.tv_sec * 1000000000ULL +
           (uint64_t)ts.tv_nsec;
}

static int queue_buffer(unsigned int index)
{
    struct v4l2_buffer buf;

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;

    return xioctl(VIDIOC_QBUF, &buf);
}

int camera_capture_init(const char *device,
                        unsigned int width,
                        unsigned int height)
{
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    unsigned int i;

    camera_capture_deinit();

    if (device == NULL) {
        errno = EINVAL;
        return -1;
    }

    camera_fd = open(device, O_RDWR | O_NONBLOCK);
    if (camera_fd < 0) {
        return -1;
    }

    memset(&cap, 0, sizeof(cap));
    if (xioctl(VIDIOC_QUERYCAP, &cap) < 0) {
        goto fail_close;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(cap.capabilities & V4L2_CAP_STREAMING)) {
        errno = ENOTSUP;
        goto fail_close;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(VIDIOC_S_FMT, &fmt) < 0) {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;

        if (xioctl(VIDIOC_S_FMT, &fmt) < 0) {
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

            if (xioctl(VIDIOC_S_FMT, &fmt) < 0) {
                goto fail_close;
            }
        }
    }

    memset(&req, 0, sizeof(req));
    req.count = CAMERA_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(VIDIOC_REQBUFS, &req) < 0) {
        goto fail_close;
    }

    if (req.count < 2) {
        errno = ENOMEM;
        goto fail_close;
    }

    mapped_buffer_count = 0;

    for (i = 0; i < req.count && i < CAMERA_BUFFER_COUNT; ++i) {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(VIDIOC_QUERYBUF, &buf) < 0) {
            goto fail_unmap;
        }

        mapped_buffers[i].length = buf.length;
        mapped_buffers[i].start = mmap(NULL,
                                       buf.length,
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED,
                                       camera_fd,
                                       buf.m.offset);

        if (mapped_buffers[i].start == MAP_FAILED) {
            mapped_buffers[i].start = NULL;
            goto fail_unmap;
        }

        mapped_buffer_count++;

        if (queue_buffer(i) < 0) {
            goto fail_unmap;
        }
    }

    secure_image_memory = malloc(CAMERA_MAX_IMAGE_BYTES);
    if (secure_image_memory == NULL) {
        goto fail_unmap;
    }

    if (mlock(secure_image_memory, CAMERA_MAX_IMAGE_BYTES) != 0) {
        fprintf(stderr,
                "Warning: mlock() failed for camera secure buffer: %s\n",
                strerror(errno));
    }

    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (xioctl(VIDIOC_STREAMON, &type) < 0) {
            goto fail_free_secure;
        }
    }

    streaming_enabled = 1;
    return 0;

fail_free_secure:
    free(secure_image_memory);
    secure_image_memory = NULL;

fail_unmap:
    for (i = 0; i < mapped_buffer_count; ++i) {
        if (mapped_buffers[i].start != NULL &&
            mapped_buffers[i].start != MAP_FAILED) {
            munmap(mapped_buffers[i].start, mapped_buffers[i].length);
        }

        mapped_buffers[i].start = NULL;
        mapped_buffers[i].length = 0;
    }

    mapped_buffer_count = 0;

fail_close:
    close(camera_fd);
    camera_fd = -1;

    return -1;
}

int camera_capture_snapshot(camera_image_t *image, int timeout_ms)
{
    struct pollfd pfd;
    struct v4l2_buffer buf;
    int ret;

    if (image == NULL || camera_fd < 0 || secure_image_memory == NULL) {
        errno = EINVAL;
        return -1;
    }

    memset(image, 0, sizeof(*image));

    pfd.fd = camera_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;

    do {
        ret = poll(&pfd, 1, timeout_ms);
    } while (ret < 0 && errno == EINTR);

    if (ret <= 0) {
        return -1;
    }

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(VIDIOC_DQBUF, &buf) < 0) {
        return -1;
    }

    if (buf.index >= mapped_buffer_count ||
        buf.bytesused == 0 ||
        buf.bytesused > CAMERA_MAX_IMAGE_BYTES) {
        if (xioctl(VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr,
                    "Warning: failed to requeue invalid camera buffer\n");
        }

        errno = EMSGSIZE;
        return -1;
    }


    memcpy(secure_image_memory,
           mapped_buffers[buf.index].start,
           buf.bytesused);

    image->data = secure_image_memory;
    image->size = buf.bytesused;

   
    if (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
        image->timestamp_ns = timeval_to_ns(&buf.timestamp);
    } else {
        image->timestamp_ns = now_monotonic_ns();
    }

    if (xioctl(VIDIOC_QBUF, &buf) < 0) {
        fprintf(stderr,
                "Warning: failed to requeue camera buffer %u: %s\n",
                buf.index,
                strerror(errno));
    }

    return 0;
}

void camera_capture_deinit(void)
{
    unsigned int i;

    if (camera_fd >= 0 && streaming_enabled) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(VIDIOC_STREAMOFF, &type);
    }

    streaming_enabled = 0;

    for (i = 0; i < mapped_buffer_count; ++i) {
        if (mapped_buffers[i].start != NULL &&
            mapped_buffers[i].start != MAP_FAILED) {
            munmap(mapped_buffers[i].start, mapped_buffers[i].length);
        }

        mapped_buffers[i].start = NULL;
        mapped_buffers[i].length = 0;
    }

    mapped_buffer_count = 0;

    if (secure_image_memory != NULL) {
        munlock(secure_image_memory, CAMERA_MAX_IMAGE_BYTES);
        free(secure_image_memory);
        secure_image_memory = NULL;
    }

    if (camera_fd >= 0) {
        close(camera_fd);
        camera_fd = -1;
    }
}
