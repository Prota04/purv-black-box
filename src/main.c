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

// Real-time priorities for the sensor detection and camera response tasks.
#define LSM9DS1_IMU_PRIORITY 99
#define TASK2_CAMERA_PRIORITY 95

// Camera device paths and capture resolution.
#define CAMERA_VIDEO_DEVICE "/dev/video0"
#define CAMERA_SENSOR_SUBDEVICE "/dev/v4l-subdev0"
#define CAMERA_WIDTH 1920U
#define CAMERA_HEIGHT 1080U

// Shared accident timestamps protected by a mutex.
// Both timestamps use CLOCK_MONOTONIC and are stored in nanoseconds.
//static pthread_mutex_t crash_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t latency_start_timestamp_ns = 0;
static uint64_t detection_timestamp_ns = 0;

int main(void)
{
    // Worker thread handles and their shared argument structures.
    pthread_t task1_lsm9ds1_thread;
    pthread_t task2_camera_thread;
    pthread_t task3_storage_thread;
    task1_lsm9ds1_args_t imu_args;
    task2_camera_args_t task2_args;
    storage_task_args_t storage_args;

    // Thread scheduling, device handles, and signal configuration.
    pthread_attr_t attr;
    struct sched_param param;
    int result;
    int crash_fd = -1;
    int imu_fd = -1;
    int camera_enabled = 0;
    sigset_t signal_set; // for SIGRTMIN

    // Open the kernel crash ring buffer used by Task 1.
    crash_fd = open(CRASH_BUFFER_PATH, O_WRONLY);
    if (crash_fd < 0) 
    {
        perror("open crash_buffer");
        return EXIT_FAILURE;
    }

    // Reset and unlock the crash buffer before starting a new monitoring session.
    if (ioctl(crash_fd, CRASH_BUFFER_IOC_UNLOCK) < 0) 
    {
        fprintf(stderr, "ioctl CRASH_BUFFER_IOC_UNLOCK failed: %s\n", strerror(errno));
        close(crash_fd);
        return EXIT_FAILURE;
    }

    // Open and initialize the LSM9DS1 IMU.
    imu_fd = lsm9ds1_open("/dev/i2c-1");
    if (imu_fd < 0) 
    {
        perror("lsm9ds1_open");
        close(crash_fd);
        return EXIT_FAILURE;
    }
    if (lsm9ds1_init(imu_fd) < 0) 
    {
        perror("lsm9ds1_init");
        lsm9ds1_close(imu_fd);
        close(crash_fd);
        return EXIT_FAILURE;
    }

    // Block SIGRTMIN before any worker threads are created so they inherit
    // the same blocked mask. task2_camera will synchronously wait for it.
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGRTMIN);
    result = pthread_sigmask(SIG_BLOCK, &signal_set, NULL);
    if (result != 0) 
    {
        fprintf(stderr, "pthread_sigmask failed: %s\n", strerror(result));
        lsm9ds1_close(imu_fd);
        close(crash_fd);
        return EXIT_FAILURE;
    }

    // Start camera streaming BEFORE Task 1 begins monitoring accidents.
    if (camera_capture_init(CAMERA_VIDEO_DEVICE, CAMERA_SENSOR_SUBDEVICE, CAMERA_WIDTH, CAMERA_HEIGHT) != 0) 
    {
        fprintf(stderr, "Warning: camera_capture_init failed: %s\n", strerror(errno));
        fprintf(stderr, "Camera capture will be disabled.\n");
    } 
    else
        camera_enabled = 1;

    // Initialize the shared data used for communication between Task 2 and Task 3.
    memset(&storage_args, 0, sizeof(storage_args));
    pthread_mutex_init(&storage_args.mutex, NULL);
    pthread_cond_init(&storage_args.condition, NULL);

    // Create Task 3 for crash data storage.
    result = pthread_create(&task3_storage_thread, NULL, storage_task, &storage_args);
    if (result != 0)
    {
        fprintf(stderr, "Task 3 pthread_create failed: %s\n", strerror(result));

        pthread_mutex_destroy(&storage_args.mutex);
        pthread_cond_destroy(&storage_args.condition);

        if (camera_enabled)
            camera_capture_deinit();
        lsm9ds1_close(imu_fd);
        close(crash_fd);

        return EXIT_FAILURE;
    }

    // Configure SCHED_FIFO attributes for the real-time tasks.
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    // Task 2: camera reaction + SOS, priority 95.
    memset(&param, 0, sizeof(param));
    param.sched_priority = TASK2_CAMERA_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);

    // Prepare the arguments needed by Task 2.
    task2_args.latency_start_timestamp_ns = &latency_start_timestamp_ns;
    task2_args.detection_timestamp_ns = &detection_timestamp_ns;
    task2_args.storage_args = &storage_args;

    // Create Task 2 with SCHED_FIFO priority 95.
    result = pthread_create(&task2_camera_thread, &attr, task2_camera, &task2_args);
    if (result != 0) 
    {
        fprintf(stderr, "emergency pthread_create failed: %s\n", strerror(result));
        pthread_attr_destroy(&attr);
        if (camera_enabled)
            camera_capture_deinit();
        lsm9ds1_close(imu_fd);
        close(crash_fd);
        return EXIT_FAILURE;
    }

    // Prepare the arguments needed by Task 1.
    imu_args.imu_fd = imu_fd;
    imu_args.crash_fd = crash_fd;
    imu_args.task2_camera_thread = task2_camera_thread;
    imu_args.latency_start_timestamp_ns = &latency_start_timestamp_ns;
    imu_args.detection_timestamp_ns = &detection_timestamp_ns;

    // Set Task 1 real-time priority.
    memset(&param, 0, sizeof(param));
    param.sched_priority = LSM9DS1_IMU_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);

    // Create Task 1 with SCHED_FIFO priority 99.
    result = pthread_create(&task1_lsm9ds1_thread, &attr, task1_lsm9ds1, &imu_args);
    if (result != 0) 
    {
        fprintf(stderr, "IMU pthread_create failed: %s\n", strerror(result));
        pthread_cancel(task2_camera_thread);
        pthread_join(task2_camera_thread, NULL);
        pthread_attr_destroy(&attr);
        if (camera_enabled)
            camera_capture_deinit();
        lsm9ds1_close(imu_fd);
        close(crash_fd);
        return EXIT_FAILURE;
    }

    pthread_attr_destroy(&attr);

    pthread_join(task1_lsm9ds1_thread, NULL);
    pthread_join(task2_camera_thread, NULL);
    pthread_join(task3_storage_thread, NULL);

    if (camera_enabled)
        camera_capture_deinit();

    close(crash_fd);
    lsm9ds1_close(imu_fd);

    pthread_cond_destroy(&storage_args.condition);
    pthread_mutex_destroy(&storage_args.mutex);

    return EXIT_SUCCESS;
}
