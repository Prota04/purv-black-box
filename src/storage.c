#define _POSIX_C_SOURCE 200809L
#define _GNU_SOURCE

#include "uapi.h"
#include "storage.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define CRASH_LOG_DIR "/var/log/blackbox"
#define MAX_PATH_LEN 512
#define ACCEL_G_PER_LSB 0.000732f

#ifndef V4L2_PIX_FMT_SRGGB10P
#define V4L2_PIX_FMT_SRGGB10P v4l2_fourcc('p', 'R', 'A', 'A')
#endif

char crash_dir[MAX_PATH_LEN];

static int create_directory(const char *path);

static int write_telemetry_to_file(const char *filepath,
                                   const lsm9ds1_raw_sample_t *data,
                                   size_t count,
                                   uint64_t threshold_timestamp_ns,
                                   uint64_t detection_timestamp_ns,
                                   const camera_image_t *image);

static int write_binary_file(const char *filepath,
                             const void *data,
                             size_t size);

static int write_praa_color_ppm(const char *filepath,
                                const camera_image_t *image);

int storage_task_save_crash_data(uint64_t threshold_timestamp_ns,
                                 uint64_t detection_timestamp_ns,
                                 const camera_image_t *image)
{
    char telemetry_path[MAX_PATH_LEN + 64];
    char raw_image_path[MAX_PATH_LEN + 64];
    char ppm_image_path[MAX_PATH_LEN + 64];

    int fd_dev = -1;
    int buffer_locked = 0;
    int ret = -1;

    lsm9ds1_raw_sample_t *buffer_data = NULL;
    ssize_t bytes_read;
    size_t sample_count;

    if (create_directory(CRASH_LOG_DIR) != 0) {
        fprintf(stderr,
                "Task 3: Failed to create base directory %s: %s\n",
                CRASH_LOG_DIR,
                strerror(errno));
        return -1;
    }

    snprintf(crash_dir,
             sizeof(crash_dir),
             "%s/crash_%llu",
             CRASH_LOG_DIR,
             (unsigned long long)detection_timestamp_ns);

    if (create_directory(crash_dir) != 0) {
        fprintf(stderr,
                "Task 3: Failed to create crash directory %s: %s\n",
                crash_dir,
                strerror(errno));
        return -1;
    }

    fd_dev = open(CRASH_BUFFER_PATH, O_RDWR);
    if (fd_dev < 0) {
        fprintf(stderr,
                "Task 3: Failed to open %s: %s\n",
                CRASH_BUFFER_PATH,
                strerror(errno));
        goto cleanup;
    }

    if (ioctl(fd_dev, CRASH_BUFFER_IOC_LOCK) < 0) {
        fprintf(stderr,
                "Task 3: Warning - failed to lock buffer via ioctl: %s\n",
                strerror(errno));
    } else {
        buffer_locked = 1;
    }

    buffer_data = malloc(BUFFER_SIZE * sizeof(lsm9ds1_raw_sample_t));
    if (buffer_data == NULL) {
        fprintf(stderr,
                "Task 3: Failed to allocate telemetry buffer\n");
        goto cleanup;
    }

    bytes_read = read(fd_dev,
                      buffer_data,
                      BUFFER_SIZE * sizeof(lsm9ds1_raw_sample_t));

    if (bytes_read < 0) {
        fprintf(stderr,
                "Task 3: Failed to read from %s: %s\n",
                CRASH_BUFFER_PATH,
                strerror(errno));
        goto cleanup;
    }

    sample_count = (size_t)bytes_read / sizeof(lsm9ds1_raw_sample_t);

    printf("Task 3: Read %zu telemetry samples from kernel buffer.\n",
           sample_count);

    snprintf(telemetry_path,
             sizeof(telemetry_path),
             "%s/accident_details.txt",
             crash_dir);

    if (write_telemetry_to_file(telemetry_path,
                                buffer_data,
                                sample_count,
                                threshold_timestamp_ns,
                                detection_timestamp_ns,
                                image) != 0) {
        fprintf(stderr,
                "Task 3: Failed to write telemetry file.\n");
        goto cleanup;
    }

    if (image != NULL && image->data != NULL && image->size > 0) {
        /* Save the exact bytes received from Unicam. This is the canonical
         * captured frame and preserves all 10-bit RAW data. */
        snprintf(raw_image_path,
                 sizeof(raw_image_path),
                 "%s/accident_site.raw",
                 crash_dir);

        if (write_binary_file(raw_image_path,
                              image->data,
                              image->size) != 0) {
            fprintf(stderr,
                    "Task 3: Failed to save RAW camera frame to %s: %s\n",
                    raw_image_path,
                    strerror(errno));
            goto cleanup;
        }

        /* pRAA is not JPEG. Generate a valid high-resolution grayscale PGM
         * preview from the upper 8 bits of each 10-bit Bayer sample. The RAW
         * file remains the lossless original capture. */
        if (image->pixelformat == V4L2_PIX_FMT_SRGGB10P) {
            snprintf(ppm_image_path,
                    sizeof(ppm_image_path),
                    "%s/accident_site.ppm",
                    crash_dir);

            if (write_praa_color_ppm(ppm_image_path, image) != 0) {
                fprintf(stderr,
                        "Task 3: Warning - failed to create color PPM image: %s\n",
                        strerror(errno));
            }
        }
    } else {
        fprintf(stderr,
                "Task 3: Warning - no camera frame is available.\n");
    }

    printf("Task 3: Crash data successfully saved to %s\n", crash_dir);
    ret = 0;

cleanup:
    if (buffer_locked && fd_dev >= 0) {
        if (ioctl(fd_dev, CRASH_BUFFER_IOC_UNLOCK) < 0) {
            fprintf(stderr,
                    "Task 3: Warning - failed to unlock/reset buffer: %s\n",
                    strerror(errno));
        }
    }

    if (buffer_data != NULL) {
        free(buffer_data);
    }

    if (fd_dev >= 0) {
        close(fd_dev);
    }

    return ret;
}

static int create_directory(const char *path)
{
    struct stat st;

    memset(&st, 0, sizeof(st));

    if (stat(path, &st) == 0) {
        if (!S_ISDIR(st.st_mode)) {
            errno = ENOTDIR;
            return -1;
        }
        return 0;
    }

    if (errno != ENOENT) {
        return -1;
    }

    /* Protected crash directory. */
    if (mkdir(path, 0700) != 0 && errno != EEXIST) {
        return -1;
    }

    chown(crash_dir, 1000, 1000);
    chmod(crash_dir, 0755);

    return 0;
}

static int write_telemetry_to_file(const char *filepath,
                                   const lsm9ds1_raw_sample_t *data,
                                   size_t count,
                                   uint64_t threshold_timestamp_ns,
                                   uint64_t detection_timestamp_ns,
                                   const camera_image_t *image)
{
    FILE *fp;
    float max_g = 0.0f;
    size_t i;

    fp = fopen(filepath, "w");
    if (fp == NULL) {
        return -1;
    }

    for (i = 0; i < count; ++i) {
        float ax = (float)data[i].accel_raw.x * ACCEL_G_PER_LSB;
        float ay = (float)data[i].accel_raw.y * ACCEL_G_PER_LSB;
        float az = (float)data[i].accel_raw.z * ACCEL_G_PER_LSB;
        float total_g = sqrtf(ax * ax + ay * ay + az * az);

        if (total_g > max_g) {
            max_g = total_g;
        }
    }

    fprintf(fp, "=== ACCIDENT DETAILS ===\n");
    fprintf(fp,
            "Samples Recorded: %zu (approx %.3f seconds)\n",
            count,
            count / 500.0);
    fprintf(fp, "Max total acceleration: %.3f g\n\n", max_g);

    fprintf(fp, "=== LATENCY ===\n");
    fprintf(fp,
            "threshold_timestamp_ns=%llu\n",
            (unsigned long long)threshold_timestamp_ns);
    fprintf(fp,
            "algorithm_detection_timestamp_ns=%llu\n",
            (unsigned long long)detection_timestamp_ns);

    if (image != NULL && image->first_byte_timestamp_ns != 0) {
        fprintf(fp,
                "camera_first_byte_timestamp_ns=%llu\n",
                (unsigned long long)image->first_byte_timestamp_ns);

        if (image->first_byte_timestamp_ns >= threshold_timestamp_ns) {
            uint64_t latency_ns =
                image->first_byte_timestamp_ns - threshold_timestamp_ns;

            fprintf(fp,
                    "threshold_to_camera_buffer_latency_ns=%llu\n",
                    (unsigned long long)latency_ns);
            fprintf(fp,
                    "threshold_to_camera_buffer_latency_ms=%.3f\n",
                    (double)latency_ns / 1000000.0);
        } else {
            fprintf(fp,
                    "threshold_to_camera_buffer_latency_ns=invalid_clock_mismatch\n");
        }

        if (image->first_byte_timestamp_ns >= detection_timestamp_ns) {
            uint64_t latency_ns =
                image->first_byte_timestamp_ns - detection_timestamp_ns;

            fprintf(fp,
                    "detection_to_camera_buffer_latency_ns=%llu\n",
                    (unsigned long long)latency_ns);
            fprintf(fp,
                    "detection_to_camera_buffer_latency_ms=%.3f\n",
                    (double)latency_ns / 1000000.0);
        }

        if (image->frame_timestamp_ns != 0) {
            fprintf(fp,
                    "v4l2_frame_timestamp_ns=%llu\n",
                    (unsigned long long)image->frame_timestamp_ns);
        }

        fprintf(fp, "camera_width=%u\n", image->width);
        fprintf(fp, "camera_height=%u\n", image->height);
        fprintf(fp, "camera_bytesperline=%u\n", image->bytesperline);
        fprintf(fp,
                "camera_fourcc=%c%c%c%c\n",
                image->pixelformat & 0xff,
                (image->pixelformat >> 8) & 0xff,
                (image->pixelformat >> 16) & 0xff,
                (image->pixelformat >> 24) & 0xff);
        fprintf(fp, "camera_frame_bytes=%zu\n", image->size);
    } else {
        fprintf(fp, "camera_first_byte_timestamp_ns=not_available\n");
    }

    fprintf(fp, "\n=== SENSOR HISTORY ===\n");
    fprintf(fp,
            "timestamp_ns,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n");

    for (i = 0; i < count; ++i) {
        fprintf(fp,
                "%llu,%d,%d,%d,%d,%d,%d\n",
                (unsigned long long)data[i].timestamp_ns,
                data[i].accel_raw.x,
                data[i].accel_raw.y,
                data[i].accel_raw.z,
                data[i].gyro_raw.x,
                data[i].gyro_raw.y,
                data[i].gyro_raw.z);
    }

    if (fflush(fp) != 0) {
        fclose(fp);
        return -1;
    }

    if (fsync(fileno(fp)) != 0) {
        fclose(fp);
        return -1;
    }

    fclose(fp);
    return 0;
}

static int write_binary_file(const char *filepath,
                             const void *data,
                             size_t size)
{
    int fd;
    const uint8_t *bytes;
    size_t remaining;

    if (data == NULL || size == 0) {
        errno = EINVAL;
        return -1;
    }

    fd = open(filepath, O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (fd < 0) {
        return -1;
    }

    bytes = (const uint8_t *)data;
    remaining = size;

    while (remaining > 0) {
        ssize_t written = write(fd, bytes, remaining);

        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }

            close(fd);
            return -1;
        }

        bytes += written;
        remaining -= (size_t)written;
    }

    if (fsync(fd) != 0) {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

static uint8_t get_pixel(const uint8_t *pixels,
                         unsigned int width,
                         unsigned int height,
                         int x,
                         int y)
{
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;

    if (x >= (int)width)
        x = (int)width - 1;
    if (y >= (int)height)
        y = (int)height - 1;

    return pixels[(size_t)y * width + (size_t)x];
}

static int write_praa_color_ppm(const char *filepath,
                                const camera_image_t *image)
{
    FILE *fp;
    uint8_t *raw8 = NULL;
    uint8_t *rgb_row = NULL;

    unsigned int x;
    unsigned int y;

    size_t stride;
    size_t minimum_row_bytes;

    if (image == NULL ||
        image->data == NULL ||
        image->width == 0 ||
        image->height == 0 ||
        image->pixelformat != V4L2_PIX_FMT_SRGGB10P ||
        (image->width % 4U) != 0U) {
        errno = EINVAL;
        return -1;
    }

    /*
     * pRAA / SRGGB10P:
     * 4 x 10-bit Bayer piksela su spakovana u 5 bajtova.
     *
     * Za preview koristimo gornjih 8 bita.
     */
    minimum_row_bytes =
        ((size_t)image->width / 4U) * 5U;

    stride = image->bytesperline != 0
                 ? (size_t)image->bytesperline
                 : minimum_row_bytes;

    if (stride < minimum_row_bytes ||
        stride * (size_t)image->height > image->size) {
        errno = EMSGSIZE;
        return -1;
    }

    /*
     * Prvo raspakujemo Bayer RAW10 u običnu
     * 8-bitnu Bayer matricu.
     */
    raw8 = malloc(
        (size_t)image->width *
        (size_t)image->height
    );

    if (raw8 == NULL) {
        return -1;
    }

    for (y = 0; y < image->height; ++y) {
        const uint8_t *input_row =
            image->data + (size_t)y * stride;

        uint8_t *output_row =
            raw8 + (size_t)y * image->width;

        size_t input_offset = 0;

        for (x = 0; x < image->width; x += 4U) {
            output_row[x + 0] =
                input_row[input_offset + 0];

            output_row[x + 1] =
                input_row[input_offset + 1];

            output_row[x + 2] =
                input_row[input_offset + 2];

            output_row[x + 3] =
                input_row[input_offset + 3];

            input_offset += 5U;
        }
    }

    fp = fopen(filepath, "wb");

    if (fp == NULL) {
        free(raw8);
        return -1;
    }

    /*
     * P6 = binary RGB PPM.
     */
    if (fprintf(fp,
                "P6\n%u %u\n255\n",
                image->width,
                image->height) < 0) {
        fclose(fp);
        free(raw8);
        return -1;
    }

    rgb_row = malloc(
        (size_t)image->width * 3U
    );

    if (rgb_row == NULL) {
        fclose(fp);
        free(raw8);
        return -1;
    }

    /*
     * SRGGB Bayer pattern:
     *
     * R G R G ...
     * G B G B ...
     * R G R G ...
     * G B G B ...
     *
     * Radimo jednostavni bilinear demosaicing.
     */
    for (y = 0; y < image->height; ++y) {

        for (x = 0; x < image->width; ++x) {

            uint8_t r;
            uint8_t g;
            uint8_t b;

            uint8_t center =
                get_pixel(raw8,
                          image->width,
                          image->height,
                          (int)x,
                          (int)y);

            /*
             * RED pixel:
             *
             * R G
             * G B
             */
            if ((y % 2U) == 0 &&
                (x % 2U) == 0) {

                r = center;

                g = (uint8_t)(
                    (
                        get_pixel(raw8, image->width, image->height,
                                  (int)x - 1, (int)y) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x + 1, (int)y) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x, (int)y - 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x, (int)y + 1)
                    ) / 4
                );

                b = (uint8_t)(
                    (
                        get_pixel(raw8, image->width, image->height,
                                  (int)x - 1, (int)y - 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x + 1, (int)y - 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x - 1, (int)y + 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x + 1, (int)y + 1)
                    ) / 4
                );
            }

            /*
             * GREEN pixel u RED redu:
             *
             * R G R
             *   B
             */
            else if ((y % 2U) == 0) {

                g = center;

                r = (uint8_t)(
                    (
                        get_pixel(raw8, image->width, image->height,
                                  (int)x - 1, (int)y) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x + 1, (int)y)
                    ) / 2
                );

                b = (uint8_t)(
                    (
                        get_pixel(raw8, image->width, image->height,
                                  (int)x, (int)y - 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x, (int)y + 1)
                    ) / 2
                );
            }

            /*
             * GREEN pixel u BLUE redu.
             */
            else if ((x % 2U) == 0) {

                g = center;

                r = (uint8_t)(
                    (
                        get_pixel(raw8, image->width, image->height,
                                  (int)x, (int)y - 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x, (int)y + 1)
                    ) / 2
                );

                b = (uint8_t)(
                    (
                        get_pixel(raw8, image->width, image->height,
                                  (int)x - 1, (int)y) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x + 1, (int)y)
                    ) / 2
                );
            }

            /*
             * BLUE pixel.
             */
            else {

                b = center;

                g = (uint8_t)(
                    (
                        get_pixel(raw8, image->width, image->height,
                                  (int)x - 1, (int)y) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x + 1, (int)y) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x, (int)y - 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x, (int)y + 1)
                    ) / 4
                );

                r = (uint8_t)(
                    (
                        get_pixel(raw8, image->width, image->height,
                                  (int)x - 1, (int)y - 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x + 1, (int)y - 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x - 1, (int)y + 1) +

                        get_pixel(raw8, image->width, image->height,
                                  (int)x + 1, (int)y + 1)
                    ) / 4
                );
            }

            rgb_row[x * 3U + 0U] = r;
            rgb_row[x * 3U + 1U] = g;
            rgb_row[x * 3U + 2U] = b;
        }

        if (fwrite(rgb_row,
                   1,
                   (size_t)image->width * 3U,
                   fp)
            != (size_t)image->width * 3U) {

            free(rgb_row);
            free(raw8);
            fclose(fp);
            return -1;
        }
    }

    free(rgb_row);
    free(raw8);

    if (fflush(fp) != 0) {
        fclose(fp);
        return -1;
    }

    if (fsync(fileno(fp)) != 0) {
        fclose(fp);
        return -1;
    }

    fclose(fp);

    return 0;
}