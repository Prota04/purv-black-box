#define _POSIX_C_SOURCE 200809L

#include "uapi.h"
#include "accident_detection.h"
#include "task1_lsm9ds1.h"
#include "task2_camera.h"
#include "lsm9ds1.h"
#include "storage.h"
#include "camera_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define LSM9DS1_IMU_PRIORITY 99
#define EMERGENCY_PRIORITY 95

#define CAMERA_VIDEO_DEVICE "/dev/video0"
#define CAMERA_SENSOR_SUBDEVICE "/dev/v4l-subdev0"
#define CAMERA_WIDTH 1920U
#define CAMERA_HEIGHT 1080U

static pthread_t emergency_thread;

/* Shared crash timestamp. It is a CLOCK_MONOTONIC timestamp in ns. */
static pthread_mutex_t crash_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t g_threshold_timestamp_ns = 0;
static uint64_t g_detection_timestamp_ns = 0;

static void *emergency_task(void *arg)
{
    sigset_t signal_set;
    int signal_number;
    int result;
    uint64_t threshold_timestamp_ns;
    uint64_t detection_timestamp_ns;
    camera_image_t image;
    int captured;

    (void)arg;

    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGRTMIN);

    printf("Task 2 is waiting for an accident...\n");

    result = sigwait(&signal_set, &signal_number);
    if (result != 0) {
        fprintf(stderr,
                "sigwait failed: %s\n",
                strerror(result));
        return NULL;
    }

    /* Take the timestamp immediately after wake-up if you later want to
     * calculate Task1 -> Task2 scheduling latency. */
    printf("Task 2 awakened: accident notification received.\n");

    pthread_mutex_lock(&crash_data_mutex);
    threshold_timestamp_ns = g_threshold_timestamp_ns;
    detection_timestamp_ns = g_detection_timestamp_ns;
    pthread_mutex_unlock(&crash_data_mutex);

    memset(&image, 0, sizeof(image));

    /* Camera has already been streaming since main(). This requests the NEXT
     * fresh completed frame; it does not start or initialise the camera now. */
    captured = (trigger_camera_capture(&image) == 0);

    if (captured) {
        printf("Task 2: fresh camera frame copied to secure buffer: %zu bytes\n",
               image.size);

        if (image.first_byte_timestamp_ns >= threshold_timestamp_ns) {
            uint64_t total_latency_ns =
                image.first_byte_timestamp_ns - threshold_timestamp_ns;
            uint64_t post_detection_latency_ns =
                image.first_byte_timestamp_ns >= detection_timestamp_ns
                    ? image.first_byte_timestamp_ns - detection_timestamp_ns
                    : 0;

            printf("Task 2: >4G-threshold->first-image-byte latency = "
                   "%llu ns (%.3f ms)\n",
                   (unsigned long long)total_latency_ns,
                   (double)total_latency_ns / 1000000.0);

            printf("Task 2: algorithm-detection->first-image-byte latency = "
                   "%llu ns (%.3f ms)\n",
                   (unsigned long long)post_detection_latency_ns,
                   (double)post_detection_latency_ns / 1000000.0);
        } else {
            fprintf(stderr,
                    "Task 2: camera timestamp precedes detection timestamp "
                    "(clock mismatch or invalid timestamp).\n");
        }

        if (image.frame_timestamp_ns != 0) {
            printf("Task 2: V4L2 frame timestamp = %llu ns\n",
                   (unsigned long long)image.frame_timestamp_ns);
        }
    } else {
        fprintf(stderr, "Task 2: camera capture failed.\n");
    }

    display_sos_led_matrix();

    /* NOTE: This is still a synchronous call from Task 2. For the final RTOS
     * architecture Task 3 should become a separate SCHED_OTHER thread. The
     * camera changes here are independent of that later refactor. */
    printf("Task 3: Saving crash data...\n");

    if (storage_task_save_crash_data(threshold_timestamp_ns,
                                     detection_timestamp_ns,
                                     captured ? &image : NULL) != 0) {
        fprintf(stderr,
                "Task 3: Failed to save crash data.\n");
    } else {
        printf("Task 3: Crash data saved.\n");
    }

    return NULL;
}

int main(void)
{
    pthread_t imu_thread;
    pthread_attr_t attr;
    struct sched_param param;
    int result;
    int crash_fd = -1;
    int imu_fd = -1;
    int camera_enabled = 0;
    sigset_t signal_set;
    task1_lsm9ds1_args_t imu_args;

    crash_fd = open(CRASH_BUFFER_PATH, O_WRONLY);
    if (crash_fd < 0) {
        perror("open crash_buffer");
        return EXIT_FAILURE;
    }

    imu_fd = lsm9ds1_open("/dev/i2c-1");
    if (imu_fd < 0) {
        perror("lsm9ds1_open");
        close(crash_fd);
        return EXIT_FAILURE;
    }

    if (lsm9ds1_init(imu_fd) < 0) {
        perror("lsm9ds1_init");
        lsm9ds1_close(imu_fd);
        close(crash_fd);
        return EXIT_FAILURE;
    }

    /* Block SIGRTMIN before any worker threads are created so they inherit
     * the same blocked mask. emergency_task will synchronously wait for it. */
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGRTMIN);

    result = pthread_sigmask(SIG_BLOCK, &signal_set, NULL);
    if (result != 0) {
        fprintf(stderr,
                "pthread_sigmask failed: %s\n",
                strerror(result));
        lsm9ds1_close(imu_fd);
        close(crash_fd);
        return EXIT_FAILURE;
    }

    /* Start camera streaming BEFORE Task 1 begins monitoring accidents. */
    if (camera_capture_init(CAMERA_VIDEO_DEVICE,
                            CAMERA_SENSOR_SUBDEVICE,
                            CAMERA_WIDTH,
                            CAMERA_HEIGHT) != 0) {
        fprintf(stderr,
                "Warning: camera_capture_init failed: %s\n",
                strerror(errno));
        fprintf(stderr,
                "Camera capture will be disabled.\n");
    } else {
        camera_enabled = 1;
    }

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    /* Task 2: camera reaction + SOS, priority 95. */
    memset(&param, 0, sizeof(param));
    param.sched_priority = EMERGENCY_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);

    result = pthread_create(&emergency_thread,
                            &attr,
                            emergency_task,
                            NULL);
    if (result != 0) {
        fprintf(stderr,
                "emergency pthread_create failed: %s\n",
                strerror(result));
        pthread_attr_destroy(&attr);
        if (camera_enabled) {
            camera_capture_deinit();
        }
        lsm9ds1_close(imu_fd);
        close(crash_fd);
        return EXIT_FAILURE;
    }

    imu_args.imu_fd = imu_fd;
    imu_args.crash_fd = crash_fd;
    imu_args.emergency_thread = emergency_thread;
    imu_args.crash_data_mutex = &crash_data_mutex;
    imu_args.threshold_timestamp_ns = &g_threshold_timestamp_ns;
    imu_args.detection_timestamp_ns = &g_detection_timestamp_ns;

    memset(&param, 0, sizeof(param));
    param.sched_priority = LSM9DS1_IMU_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);

    result = pthread_create(&imu_thread,
                            &attr,
                            task1_lsm9ds1,
                            &imu_args);
    if (result != 0) {
        fprintf(stderr,
                "IMU pthread_create failed: %s\n",
                strerror(result));
        pthread_cancel(emergency_thread);
        pthread_join(emergency_thread, NULL);
        pthread_attr_destroy(&attr);
        if (camera_enabled) {
            camera_capture_deinit();
        }
        lsm9ds1_close(imu_fd);
        close(crash_fd);
        return EXIT_FAILURE;
    }

    pthread_attr_destroy(&attr);

    pthread_join(imu_thread, NULL);
    pthread_join(emergency_thread, NULL);

    if (camera_enabled) {
        camera_capture_deinit();
    }

    close(crash_fd);
    lsm9ds1_close(imu_fd);

    return EXIT_SUCCESS;
}
