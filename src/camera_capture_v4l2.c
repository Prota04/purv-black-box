#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L

#include "camera_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define CAMERA_BUFFER_COUNT 4
#define CAMERA_POLL_TIMEOUT_MS 100

#ifndef V4L2_PIX_FMT_SRGGB10P
#define V4L2_PIX_FMT_SRGGB10P v4l2_fourcc('p', 'R', 'A', 'A')
#endif

#ifndef MEDIA_BUS_FMT_SRGGB10_1X10
#define MEDIA_BUS_FMT_SRGGB10_1X10 0x300f
#endif

struct mapped_buffer {
    void *start;
    size_t length;
};

static int camera_fd = -1;
static struct mapped_buffer mapped_buffers[CAMERA_BUFFER_COUNT];
static unsigned int mapped_buffer_count = 0;

static uint8_t *secure_image_memory = NULL;
static size_t secure_image_capacity = 0;

static unsigned int configured_width = 0;
static unsigned int configured_height = 0;
static unsigned int configured_bytesperline = 0;
static uint32_t configured_pixelformat = 0;

static int streaming_enabled = 0;

/* Background camera-draining thread state. */
static pthread_t stream_thread;
static int stream_thread_started = 0;
static int stop_stream_thread = 0;

static pthread_mutex_t capture_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t capture_cond = PTHREAD_COND_INITIALIZER;

static int capture_requested = 0;
static int capture_ready = 0;
static int capture_error = 0;
static camera_image_t captured_image;

static int xioctl_fd(int fd, unsigned long request, void *arg)
{
    int result;

    do {
        result = ioctl(fd, request, arg);
    } while (result == -1 && errno == EINTR);

    return result;
}

static int xioctl(unsigned long request, void *arg)
{
    return xioctl_fd(camera_fd, request, arg);
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

static void add_milliseconds(struct timespec *time_value, int milliseconds)
{
    long nanoseconds;

    nanoseconds = (long)(milliseconds % 1000) * 1000000L;
    time_value->tv_sec += milliseconds / 1000;
    time_value->tv_nsec += nanoseconds;

    while (time_value->tv_nsec >= 1000000000L) {
        time_value->tv_sec++;
        time_value->tv_nsec -= 1000000000L;
    }
}

static int configure_imx219_subdevice(const char *sensor_subdevice,
                                      unsigned int width,
                                      unsigned int height)
{
    int subdev_fd;
    struct v4l2_subdev_format format;

    subdev_fd = open(sensor_subdevice, O_RDWR);
    if (subdev_fd < 0) {
        fprintf(stderr,
                "Camera: cannot open sensor subdevice %s: %s\n",
                sensor_subdevice,
                strerror(errno));
        return -1;
    }

    memset(&format, 0, sizeof(format));
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    format.format.width = width;
    format.format.height = height;
    format.format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
    format.format.field = V4L2_FIELD_NONE;
    format.format.colorspace = V4L2_COLORSPACE_RAW;

    if (xioctl_fd(subdev_fd, VIDIOC_SUBDEV_S_FMT, &format) < 0) {
        fprintf(stderr,
                "Camera: VIDIOC_SUBDEV_S_FMT failed on %s: %s\n",
                sensor_subdevice,
                strerror(errno));
        close(subdev_fd);
        return -1;
    }

    if (format.format.width != width ||
        format.format.height != height ||
        format.format.code != MEDIA_BUS_FMT_SRGGB10_1X10) {
        fprintf(stderr,
                "Camera: sensor rejected requested mode. "
                "Requested %ux%u SRGGB10, got %ux%u code=0x%x\n",
                width,
                height,
                format.format.width,
                format.format.height,
                format.format.code);
        close(subdev_fd);
        errno = EINVAL;
        return -1;
    }

    close(subdev_fd);
    return 0;
}

static int queue_buffer(unsigned int index)
{
    struct v4l2_buffer buffer;

    memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;

    return xioctl(VIDIOC_QBUF, &buffer);
}

static void signal_capture_failure(int error_number)
{
    pthread_mutex_lock(&capture_mutex);

    if (capture_requested) {
        capture_requested = 0;
        capture_ready = 0;
        capture_error = error_number != 0 ? error_number : EIO;
        pthread_cond_broadcast(&capture_cond);
    }

    pthread_mutex_unlock(&capture_mutex);
}

static void *camera_stream_thread(void *arg)
{
    (void)arg;

    while (1) {
        struct pollfd poll_fd;
        struct v4l2_buffer buffer;
        int poll_result;
        int should_capture;

        pthread_mutex_lock(&capture_mutex);
        if (stop_stream_thread) {
            pthread_mutex_unlock(&capture_mutex);
            break;
        }
        pthread_mutex_unlock(&capture_mutex);

        memset(&poll_fd, 0, sizeof(poll_fd));
        poll_fd.fd = camera_fd;
        poll_fd.events = POLLIN;

        do {
            poll_result = poll(&poll_fd, 1, CAMERA_POLL_TIMEOUT_MS);
        } while (poll_result < 0 && errno == EINTR);

        if (poll_result < 0) {
            int saved_errno = errno;
            fprintf(stderr, "Camera: poll failed: %s\n", strerror(saved_errno));
            signal_capture_failure(saved_errno);
            continue;
        }

        if (poll_result == 0) {
            continue;
        }

        if (!(poll_fd.revents & POLLIN)) {
            if (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                signal_capture_failure(EIO);
            }
            continue;
        }

        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;

        if (xioctl(VIDIOC_DQBUF, &buffer) < 0) {
            if (errno == EAGAIN) {
                continue;
            }

            {
                int saved_errno = errno;
                fprintf(stderr,
                        "Camera: VIDIOC_DQBUF failed: %s\n",
                        strerror(saved_errno));
                signal_capture_failure(saved_errno);
            }
            continue;
        }

        pthread_mutex_lock(&capture_mutex);
        should_capture = capture_requested && !capture_ready;
        pthread_mutex_unlock(&capture_mutex);

        if (should_capture) {
            if (buffer.index >= mapped_buffer_count ||
                buffer.bytesused == 0 ||
                buffer.bytesused > secure_image_capacity) {
                signal_capture_failure(EMSGSIZE);
            } else {
                camera_image_t image;
                uint64_t first_byte_timestamp_ns;

                memset(&image, 0, sizeof(image));

                /* This is the project latency endpoint: the next fresh frame
                 * has become available to userspace and we are about to copy
                 * its first byte into the dedicated secure image memory. */
                first_byte_timestamp_ns = now_monotonic_ns();

                memcpy(secure_image_memory,
                       mapped_buffers[buffer.index].start,
                       buffer.bytesused);

                image.data = secure_image_memory;
                image.size = buffer.bytesused;
                image.first_byte_timestamp_ns = first_byte_timestamp_ns;
                image.width = configured_width;
                image.height = configured_height;
                image.bytesperline = configured_bytesperline;
                image.pixelformat = configured_pixelformat;

                if (buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
                    image.frame_timestamp_ns = timeval_to_ns(&buffer.timestamp);
                } else {
                    image.frame_timestamp_ns = 0;
                }

                pthread_mutex_lock(&capture_mutex);

                /* The request may have timed out while memcpy was running. */
                if (capture_requested) {
                    captured_image = image;
                    capture_requested = 0;
                    capture_ready = 1;
                    capture_error = 0;
                    pthread_cond_broadcast(&capture_cond);
                }

                pthread_mutex_unlock(&capture_mutex);
            }
        }

        /* Always return the MMAP buffer to the driver so continuous streaming
         * never stops after the four buffers fill up. */
        if (xioctl(VIDIOC_QBUF, &buffer) < 0) {
            int saved_errno = errno;
            fprintf(stderr,
                    "Camera: VIDIOC_QBUF failed for buffer %u: %s\n",
                    buffer.index,
                    strerror(saved_errno));
            signal_capture_failure(saved_errno);
        }
    }

    return NULL;
}

int camera_capture_init(const char *video_device,
                        const char *sensor_subdevice,
                        unsigned int width,
                        unsigned int height)
{
    struct v4l2_capability capability;
    struct v4l2_format format;
    struct v4l2_requestbuffers request_buffers;
    uint32_t effective_capabilities;
    unsigned int i;
    int thread_result;

    camera_capture_deinit();

    if (video_device == NULL || sensor_subdevice == NULL ||
        width == 0 || height == 0) {
        errno = EINVAL;
        return -1;
    }

    /* On Media Controller based Unicam the sensor pad and the /dev/video0
     * format must match. Configure IMX219 first. */
    if (configure_imx219_subdevice(sensor_subdevice, width, height) != 0) {
        return -1;
    }

    camera_fd = open(video_device, O_RDWR | O_NONBLOCK);
    if (camera_fd < 0) {
        fprintf(stderr,
                "Camera: cannot open %s: %s\n",
                video_device,
                strerror(errno));
        return -1;
    }

    memset(&capability, 0, sizeof(capability));
    if (xioctl(VIDIOC_QUERYCAP, &capability) < 0) {
        perror("Camera: VIDIOC_QUERYCAP");
        goto fail_close;
    }

    effective_capabilities =
        (capability.capabilities & V4L2_CAP_DEVICE_CAPS)
            ? capability.device_caps
            : capability.capabilities;

    if (!(effective_capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(effective_capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr,
                "Camera: device does not support VIDEO_CAPTURE + STREAMING\n");
        errno = ENOTSUP;
        goto fail_close;
    }

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB10P; /* pRAA */
    format.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(VIDIOC_S_FMT, &format) < 0) {
        perror("Camera: VIDIOC_S_FMT pRAA");
        goto fail_close;
    }

    if (format.fmt.pix.width != width ||
        format.fmt.pix.height != height ||
        format.fmt.pix.pixelformat != V4L2_PIX_FMT_SRGGB10P) {
        fprintf(stderr,
                "Camera: video node rejected requested pRAA mode. "
                "Requested %ux%u, got %ux%u fourcc=%c%c%c%c\n",
                width,
                height,
                format.fmt.pix.width,
                format.fmt.pix.height,
                format.fmt.pix.pixelformat & 0xff,
                (format.fmt.pix.pixelformat >> 8) & 0xff,
                (format.fmt.pix.pixelformat >> 16) & 0xff,
                (format.fmt.pix.pixelformat >> 24) & 0xff);
        errno = EINVAL;
        goto fail_close;
    }

    configured_width = format.fmt.pix.width;
    configured_height = format.fmt.pix.height;
    configured_bytesperline = format.fmt.pix.bytesperline;
    configured_pixelformat = format.fmt.pix.pixelformat;

    secure_image_capacity = format.fmt.pix.sizeimage;
    if (secure_image_capacity == 0 ||
        secure_image_capacity > CAMERA_MAX_IMAGE_BYTES) {
        fprintf(stderr,
                "Camera: unsupported sizeimage=%zu bytes\n",
                secure_image_capacity);
        errno = EMSGSIZE;
        goto fail_close;
    }

    memset(&request_buffers, 0, sizeof(request_buffers));
    request_buffers.count = CAMERA_BUFFER_COUNT;
    request_buffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request_buffers.memory = V4L2_MEMORY_MMAP;

    if (xioctl(VIDIOC_REQBUFS, &request_buffers) < 0) {
        perror("Camera: VIDIOC_REQBUFS");
        goto fail_close;
    }

    if (request_buffers.count < 2) {
        fprintf(stderr,
                "Camera: driver allocated only %u MMAP buffers\n",
                request_buffers.count);
        errno = ENOMEM;
        goto fail_close;
    }

    mapped_buffer_count = 0;

    for (i = 0;
         i < request_buffers.count && i < CAMERA_BUFFER_COUNT;
         ++i) {
        struct v4l2_buffer buffer;

        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;

        if (xioctl(VIDIOC_QUERYBUF, &buffer) < 0) {
            perror("Camera: VIDIOC_QUERYBUF");
            goto fail_unmap;
        }

        mapped_buffers[i].length = buffer.length;
        mapped_buffers[i].start = mmap(NULL,
                                       buffer.length,
                                       PROT_READ | PROT_WRITE,
                                       MAP_SHARED,
                                       camera_fd,
                                       buffer.m.offset);

        if (mapped_buffers[i].start == MAP_FAILED) {
            mapped_buffers[i].start = NULL;
            perror("Camera: mmap");
            goto fail_unmap;
        }

        mapped_buffer_count++;

        if (queue_buffer(i) < 0) {
            perror("Camera: VIDIOC_QBUF");
            goto fail_unmap;
        }
    }

    secure_image_memory = malloc(secure_image_capacity);
    if (secure_image_memory == NULL) {
        perror("Camera: malloc secure image buffer");
        goto fail_unmap;
    }

    if (mlock(secure_image_memory, secure_image_capacity) != 0) {
        fprintf(stderr,
                "Warning: mlock() failed for camera secure buffer: %s\n",
                strerror(errno));
    }

    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (xioctl(VIDIOC_STREAMON, &type) < 0) {
            perror("Camera: VIDIOC_STREAMON");
            goto fail_free_secure;
        }
    }

    streaming_enabled = 1;

    pthread_mutex_lock(&capture_mutex);
    stop_stream_thread = 0;
    capture_requested = 0;
    capture_ready = 0;
    capture_error = 0;
    memset(&captured_image, 0, sizeof(captured_image));
    pthread_mutex_unlock(&capture_mutex);

    thread_result = pthread_create(&stream_thread,
                                   NULL,
                                   camera_stream_thread,
                                   NULL);
    if (thread_result != 0) {
        fprintf(stderr,
                "Camera: pthread_create failed: %s\n",
                strerror(thread_result));
        errno = thread_result;
        goto fail_streamoff;
    }

    stream_thread_started = 1;

    printf("Camera: continuous stream active: %ux%u %c%c%c%c, "
           "stride=%u, sizeimage=%zu bytes\n",
           configured_width,
           configured_height,
           configured_pixelformat & 0xff,
           (configured_pixelformat >> 8) & 0xff,
           (configured_pixelformat >> 16) & 0xff,
           (configured_pixelformat >> 24) & 0xff,
           configured_bytesperline,
           secure_image_capacity);

    return 0;

fail_streamoff:
    if (streaming_enabled) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(VIDIOC_STREAMOFF, &type);
        streaming_enabled = 0;
    }

fail_free_secure:
    if (secure_image_memory != NULL) {
        munlock(secure_image_memory, secure_image_capacity);
        free(secure_image_memory);
        secure_image_memory = NULL;
    }
    secure_image_capacity = 0;

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
    if (camera_fd >= 0) {
        close(camera_fd);
        camera_fd = -1;
    }

    configured_width = 0;
    configured_height = 0;
    configured_bytesperline = 0;
    configured_pixelformat = 0;

    return -1;
}

int camera_capture_snapshot(camera_image_t *image, int timeout_ms)
{
    struct timespec deadline;
    int wait_result;

    if (image == NULL || timeout_ms <= 0) {
        errno = EINVAL;
        return -1;
    }

    memset(image, 0, sizeof(*image));

    pthread_mutex_lock(&capture_mutex);

    if (camera_fd < 0 ||
        !streaming_enabled ||
        !stream_thread_started ||
        secure_image_memory == NULL) {
        pthread_mutex_unlock(&capture_mutex);
        errno = ENODEV;
        return -1;
    }

    if (capture_requested) {
        pthread_mutex_unlock(&capture_mutex);
        errno = EBUSY;
        return -1;
    }

    capture_ready = 0;
    capture_error = 0;
    capture_requested = 1;

    if (clock_gettime(CLOCK_REALTIME, &deadline) != 0) {
        capture_requested = 0;
        pthread_mutex_unlock(&capture_mutex);
        return -1;
    }

    add_milliseconds(&deadline, timeout_ms);

    while (!capture_ready && capture_error == 0 && !stop_stream_thread) {
        wait_result = pthread_cond_timedwait(&capture_cond,
                                             &capture_mutex,
                                             &deadline);

        if (wait_result == ETIMEDOUT) {
            capture_requested = 0;
            pthread_mutex_unlock(&capture_mutex);
            errno = ETIMEDOUT;
            return -1;
        }

        if (wait_result != 0) {
            capture_requested = 0;
            pthread_mutex_unlock(&capture_mutex);
            errno = wait_result;
            return -1;
        }
    }

    if (capture_error != 0) {
        int saved_error = capture_error;
        capture_requested = 0;
        capture_ready = 0;
        capture_error = 0;
        pthread_mutex_unlock(&capture_mutex);
        errno = saved_error;
        return -1;
    }

    if (!capture_ready) {
        capture_requested = 0;
        pthread_mutex_unlock(&capture_mutex);
        errno = ECANCELED;
        return -1;
    }

    *image = captured_image;
    capture_ready = 0;

    pthread_mutex_unlock(&capture_mutex);
    return 0;
}

void camera_capture_deinit(void)
{
    unsigned int i;

    if (stream_thread_started) {
        pthread_mutex_lock(&capture_mutex);
        stop_stream_thread = 1;
        capture_requested = 0;
        capture_ready = 0;
        capture_error = ECANCELED;
        pthread_cond_broadcast(&capture_cond);
        pthread_mutex_unlock(&capture_mutex);

        pthread_join(stream_thread, NULL);
        stream_thread_started = 0;
    }

    if (camera_fd >= 0 && streaming_enabled) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (xioctl(VIDIOC_STREAMOFF, &type) < 0) {
            fprintf(stderr,
                    "Camera: VIDIOC_STREAMOFF failed: %s\n",
                    strerror(errno));
        }
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
        munlock(secure_image_memory, secure_image_capacity);
        free(secure_image_memory);
        secure_image_memory = NULL;
    }

    secure_image_capacity = 0;

    if (camera_fd >= 0) {
        close(camera_fd);
        camera_fd = -1;
    }

    configured_width = 0;
    configured_height = 0;
    configured_bytesperline = 0;
    configured_pixelformat = 0;

    pthread_mutex_lock(&capture_mutex);
    stop_stream_thread = 0;
    capture_requested = 0;
    capture_ready = 0;
    capture_error = 0;
    memset(&captured_image, 0, sizeof(captured_image));
    pthread_mutex_unlock(&capture_mutex);
}