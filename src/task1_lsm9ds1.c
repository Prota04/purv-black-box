#define _POSIX_C_SOURCE 200809L

#include "lsm9ds1.h"
#include "task1_lsm9ds1.h"
#include "accident_detection.h"
#include "uapi.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define SAMPLE_PERIOD_NS (2L * 1000L * 1000L) // 500 Hz

// Calculates the next absolute activation time of the periodic IMU task.
static void add_nanoseconds(struct timespec *time_value, long nanoseconds)
{
    time_value->tv_nsec += nanoseconds;

    while (time_value->tv_nsec >= 1000000000L) 
    {
        time_value->tv_sec++;
        time_value->tv_nsec -= 1000000000L;
    }
}

/*
    Task 1 runs continuously at 500 Hz:
    - reads data from the IMU,
    - writes each sample to the kernel ring buffer,
    - checks whether an accident has occurred.

    If no accident is detected, the task waits for the next sampling period.
    If an accident is detected, it:
        - stores the relevant timestamps,
        - locks/freezes the ring buffer,
        - wakes up Task 2,
        - terminates Task 1.
*/
void *task1_lsm9ds1(void *arg)
{
    task1_lsm9ds1_args_t *args = arg;

    int imu_fd = args->imu_fd;
    int crash_fd = args->crash_fd;

    struct timespec next_activation;

    accident_detector_t detector;
    accident_detection_result_t detection;

    uint64_t impact_threshold_start_ns = 0;
    uint64_t rollover_threshold_start_ns = 0;

    accident_detection_init(&detector, 0.0f, 0.0f, 0.0f);

    if (clock_gettime(CLOCK_MONOTONIC, &next_activation) < 0) 
    {
        perror("clock_gettime");
        return NULL;
    }

    while(1)
    {
        lsm9ds1_raw_sample_t raw_sample = {0};
        lsm9ds1_sample_t sample = {0};

        accident_event_t event;
        struct timespec sample_time;

        ssize_t written;
        int sleep_result;

        // Read the latest accelerometer and gyroscope sample from the IMU.
        if (lsm9ds1_read_sample(imu_fd, &sample, &raw_sample) < 0) 
        {
            perror("lsm9ds1_read_sample");
            break;
        }

        // Get the timestamp of the current sample.
        if (clock_gettime(CLOCK_MONOTONIC, &sample_time) < 0) 
        {
            perror("clock_gettime");
            break;
        }

        // Convert the sample timestamp to nanoseconds.
        raw_sample.timestamp_ns = (__u64)sample_time.tv_sec * 1000000000ULL + (__u64)sample_time.tv_nsec;

        // Write the current sample to the kernel ring buffer.
        written = write(crash_fd, &raw_sample, sizeof(raw_sample));
        if (written != (ssize_t)sizeof(raw_sample))
        {
            perror("write crash_buffer");
            break;
        }

        // Check for a collision or rollover event.
        event = accident_detection_update(&detector, &sample, &detection);

        // Store the timestamp of the first sample that exceeds the 4G threshold.
        // This timestamp is used as the starting point for measuring total latency.
        if (detection.total_g >= IMPACT_THRESHOLD_G)
        {
            if (impact_threshold_start_ns == 0)
                impact_threshold_start_ns = raw_sample.timestamp_ns;
        }
        else
            impact_threshold_start_ns = 0;


        if (detection.tilt_deg >= ROLLOVER_ANGLE_DEG)
        {
            if (rollover_threshold_start_ns == 0)
                rollover_threshold_start_ns = raw_sample.timestamp_ns;
        }
        else
            rollover_threshold_start_ns = 0;

        // Accident detected and confirmed.
        if (event != ACCIDENT_EVENT_NONE) 
        {

            int signal_result;

            *args->detection_timestamp_ns = raw_sample.timestamp_ns;

            if ((event & ACCIDENT_EVENT_IMPACT) && impact_threshold_start_ns != 0)
                *args->latency_start_timestamp_ns = impact_threshold_start_ns;
            else if ((event & ACCIDENT_EVENT_ROLLOVER) && rollover_threshold_start_ns != 0)
                *args->latency_start_timestamp_ns = rollover_threshold_start_ns;
            else
                *args->latency_start_timestamp_ns = raw_sample.timestamp_ns;
            

            // Freeze the kernel ring buffer to prevent the 5 seconds
            // of pre-accident data from being overwritten.
            if (ioctl(crash_fd, CRASH_BUFFER_IOC_LOCK) < 0)
                fprintf(stderr, "ioctl CRASH_BUFFER_IOC_LOCK failed: %s\n", strerror(errno));

            // Wake up Task 2 to activate the camera and SOS signal.
            signal_result = pthread_kill(args->task2_camera_thread, SIGRTMIN);
            if (signal_result != 0)
                fprintf(stderr, "pthread_kill failed: %s\n", strerror(signal_result));
            
            if (event & ACCIDENT_EVENT_IMPACT)
                printf("Impact detected.\n");
            if (event & ACCIDENT_EVENT_ROLLOVER)
                printf("Rollover detected.\n");

            break;
        }

        add_nanoseconds(&next_activation, SAMPLE_PERIOD_NS);

        sleep_result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_activation, NULL);

        if (sleep_result != 0) 
        {
            fprintf(stderr, "clock_nanosleep failed: %s\n", strerror(sleep_result));
            break;
        }
    }

    return NULL;
}
