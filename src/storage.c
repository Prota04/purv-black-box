#define _POSIX_C_SOURCE 200809L
#define _GNU_SOURCE

#include "task3_storage.h"
#include "uapi.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define CRASH_LOG_DIR "/var/log/blackbox"
#define MAX_PATH_LEN 512
#define ACCEL_G_PER_LSB 0.000732f // Matches lsm9ds1.c configuration

static int create_directory(const char *path);
static int write_telemetry_to_file(const char *filepath, const lsm9ds1_raw_sample_t *data, size_t count);
static int move_file(const char *src, const char *dest);

int storage_task_save_crash_data(uint64_t crash_timestamp_ns, const char *camera_image_path) {
    char crash_dir[MAX_PATH_LEN];
    char telemetry_path[MAX_PATH_LEN];
    char image_dest_path[MAX_PATH_LEN];
    int fd_dev = -1;
    int ret = -1;
    
    lsm9ds1_raw_sample_t *buffer_data = NULL;
    ssize_t bytes_read;
    size_t sample_count;

    /* 1. Create the base directory if it doesn't exist */
    if (create_directory(CRASH_LOG_DIR) != 0) {
        fprintf(stderr, "Task 3: Failed to create base directory %s (Check permissions)\n", CRASH_LOG_DIR);
        return -1;
    }

    /* 2. Create the crash-specific directory */
    snprintf(crash_dir, sizeof(crash_dir), "%s/crash_%llu", CRASH_LOG_DIR, (unsigned long long)crash_timestamp_ns);
    if (create_directory(crash_dir) != 0) {
        fprintf(stderr, "Task 3: Failed to create crash directory %s\n", crash_dir);
        return -1;
    }

    /* 3. Open the crash buffer device */
    fd_dev = open(CRASH_BUFFER_PATH, O_RDWR);
    if (fd_dev < 0) {
        fprintf(stderr, "Task 3: Failed to open %s: %s\n", CRASH_BUFFER_PATH, strerror(errno));
        goto cleanup;
    }

    /* Ensure buffer is locked (Task 1 should ideally do this immediately upon crash detection) */
    if (ioctl(fd_dev, CRASH_BUFFER_IOC_LOCK) < 0) {
        fprintf(stderr, "Task 3: Warning - Failed to lock buffer via ioctl: %s\n", strerror(errno));
    }

    /* 4. Read the pre-crash telemetry data (Up to 5 seconds of history) */
    buffer_data = malloc(BUFFER_SIZE * sizeof(lsm9ds1_raw_sample_t));
    if (!buffer_data) {
        fprintf(stderr, "Task 3: Failed to allocate memory for telemetry buffer\n");
        goto cleanup;
    }

    bytes_read = read(fd_dev, buffer_data, BUFFER_SIZE * sizeof(lsm9ds1_raw_sample_t));
    if (bytes_read < 0) {
        fprintf(stderr, "Task 3: Failed to read from %s: %s\n", CRASH_BUFFER_PATH, strerror(errno));
        goto cleanup;
    }

    sample_count = bytes_read / sizeof(lsm9ds1_raw_sample_t);
    printf("Task 3: Read %zu telemetry samples from kernel buffer.\n", sample_count);

    /* 5. Write telemetry data to a text file */
    snprintf(telemetry_path, sizeof(telemetry_path), "%s/accident_details.txt", crash_dir);
    if (write_telemetry_to_file(telemetry_path, buffer_data, sample_count) != 0) {
        fprintf(stderr, "Task 3: Failed to write telemetry file.\n");
        goto cleanup;
    }

    /* 6. Move the camera image to the crash directory */
    snprintf(image_dest_path, sizeof(image_dest_path), "%s/accident_site.jpg", crash_dir);
    if (move_file(camera_image_path, image_dest_path) != 0) {
        fprintf(stderr, "Task 3: Warning - Failed to move camera image to %s.\n", crash_dir);
    }

    /* 7. Unlock and reset the buffer for future use */
    if (ioctl(fd_dev, CRASH_BUFFER_IOC_UNLOCK) < 0) {
        fprintf(stderr, "Task 3: Warning - Failed to unlock buffer via ioctl: %s\n", strerror(errno));
    }
    
    printf("Task 3: Crash data successfully saved to %s\n", crash_dir);
    ret = 0;

cleanup:
    if (buffer_data) free(buffer_data);
    if (fd_dev >= 0) close(fd_dev);
    return ret;
}

static int create_directory(const char *path) {
    struct stat st = {0};
    if (stat(path, &st) == -1) {
        if (mkdir(path, 0755) != 0 && errno != EEXIST) {
            return -1;
        }
    }
    return 0;
}

static int write_telemetry_to_file(const char *filepath, const lsm9ds1_raw_sample_t *data, size_t count) {
    FILE *fp = fopen(filepath, "w");
    if (!fp) return -1;

    float max_g = 0.0f;

    // Calculate exact force values at impact
    for (size_t i = 0; i < count; i++) {
        float ax = data[i].accel_raw.x * ACCEL_G_PER_LSB;
        float ay = data[i].accel_raw.y * ACCEL_G_PER_LSB;
        float az = data[i].accel_raw.z * ACCEL_G_PER_LSB;
        float g = sqrtf(ax*ax + ay*ay + az*az);
        if (g > max_g) max_g = g;
    }

    fprintf(fp, "=== ACCIDENT DETAILS ===\n");
    fprintf(fp, "Max Impact Force: %.2f G\n", max_g);
    fprintf(fp, "Samples Recorded: %zu (approx %.1f seconds prior to crash)\n\n", count, count / 500.0f);
    fprintf(fp, "=== TILT & SENSOR HISTORY ===\n");
    fprintf(fp, "timestamp_ns,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n");

    for (size_t i = 0; i < count; i++) {
        fprintf(fp, "%llu,%d,%d,%d,%d,%d,%d\n",
                (unsigned long long)data[i].timestamp_ns,
                data[i].accel_raw.x, data[i].accel_raw.y, data[i].accel_raw.z,
                data[i].gyro_raw.x, data[i].gyro_raw.y, data[i].gyro_raw.z);
    }

    fclose(fp);
    return 0;
}

static int move_file(const char *src, const char *dest) {
    if (rename(src, dest) == 0) return 0; // Fast path
    if (errno != EXDEV) return -1;        // Cross-device link fallback

    FILE *f_src = fopen(src, "rb");
    if (!f_src) return -1;

    FILE *f_dest = fopen(dest, "wb");
    if (!f_dest) { fclose(f_src); return -1; }

    char buffer[8192];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), f_src)) > 0) {
        if (fwrite(buffer, 1, bytes, f_dest) != bytes) {
            fclose(f_src); fclose(f_dest); return -1;
        }
    }

    fclose(f_src); fclose(f_dest);
    unlink(src);
    return 0;
}