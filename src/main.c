#define _POSIX_C_SOURCE 200809L

#include "uapi.h"
#include "accident_detection.h"

#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <semaphore.h>
#include <signal.h>

#define LSM9DS1_IMU_PRIORITY 99
#define EMERGENCY_PRIORITY 95 //FILIP
#define SAMPLE_PERIOD_NS (2L * 1000L * 1000L)
#define TEST_CYCLES 500

static pthread_t emergency_thread;

//static sem_t emergency_sem;

static void add_nanoseconds(struct timespec* time, long nanoseconds)
{
    time->tv_nsec += nanoseconds;

    while (time->tv_nsec >= 1000000000L) 
    {
        time->tv_sec++;
        time->tv_nsec -= 1000000000L;
    }
}

static void* imu_task (void* arg)
{
    int crash_fd = *(int *)arg;
    struct timespec next_activation;

    accident_detector_t detector;
    accident_detection_result_t detection;
    accident_event_t detected_events = ACCIDENT_EVENT_NONE;

    int i;

    accident_detection_init(&detector, 0.0f, 0.0f, 0.0f);

    if(clock_gettime(CLOCK_MONOTONIC, &next_activation) < 0)
    {
        perror("clock_gettime");
        return NULL;
    }

    for(i = 0; i < TEST_CYCLES; ++i)
    {
        //read sensor
        //timestamp sample
        //write sample to /dev/crash_buffer
        //run accident detection
        //notify task 2 if acc occur

        lsm9ds1_raw_sample_t raw_sample = {0};
        lsm9ds1_sample_t sample = {0};
        accident_event_t event;

        struct timespec sample_time;
        ssize_t written;

        sample.accel_g.x = 0.0f;
        sample.accel_g.y = 0.0f;
        sample.accel_g.z = 1.0f;

        sample.gyro_rad_s.x = 0.0f;
        sample.gyro_rad_s.y = 0.0f;
        sample.gyro_rad_s.z = 0.0f;

        if (i == 250 || i == 251)
        {
            sample.accel_g.x = 4.5f;
            sample.accel_g.y = 0.0f;
            sample.accel_g.z = 0.0f;
        }

        raw_sample.accel_raw.x = (__s16)(100 + i);
        raw_sample.accel_raw.y = (__s16)(200 + i);
        raw_sample.accel_raw.z = (__s16)(300 + i);

        raw_sample.gyro_raw.x = (__s16)(400 + i);
        raw_sample.gyro_raw.y = (__s16)(500 + i);
        raw_sample.gyro_raw.z = (__s16)(600 + i);

        /*
         * REAL SENSOR VERSION:
         *
         * When the Sense HAT is available, the temporary
         * raw and physical values above will be replaced by:
         *
         * if (lsm9ds1_read_sample(imu_fd, &sample, &raw_sample) < 0)
         * {
         *     perror("lsm9ds1_read_sample");
         *     break;
         * }
         */

        if(clock_gettime(CLOCK_MONOTONIC, &sample_time) < 0)
        {
            perror("clock_gettime");
            break;
        }

        raw_sample.timestamp_ns =
            (__u64)sample_time.tv_sec * 1000000000ULL +
            (__u64)sample_time.tv_nsec;

        written = write(crash_fd, &raw_sample, sizeof(raw_sample));

        if (written != (ssize_t)sizeof(raw_sample))
        {
            perror("write crash_buffer");
            break;
        }

        event = accident_detection_update(&detector, &sample, &detection);

        if (event != ACCIDENT_EVENT_NONE)
        {
            int signal_result;

            detected_events = (accident_event_t)(detected_events | event);

            signal_result = pthread_kill(emergency_thread, SIGRTMIN);

            if(signal_result != 0)
            {
                fprintf(stderr, "pthread_kill failed: %s\n", strerror(signal_result));
            }

            // ioctl(crash_fd, LOCK_BUFFER);

            /*
            if (sem_post(&emergency_sem) < 0)
            {
                perror("sem_post");
            }
            */

            //ioctl(crash_fd, LOCK_BUFFER);
            //sem_post(&emergency_sem);
            
        }

        add_nanoseconds(&next_activation, SAMPLE_PERIOD_NS);

        int result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_activation, NULL);

        if(result != 0)
        {
            fprintf(stderr, "clock_nanosleep failed: %s\n", strerror(result));
            break;
        }
    }

    printf("IMU task finished after %d cycles\n", i);

    if (detected_events & ACCIDENT_EVENT_IMPACT)
    {
        printf("Impact detected successfully.\n");
    }

    if (detected_events & ACCIDENT_EVENT_ROLLOVER)
    {
        printf("Rollover detected successfully.\n");
    }

    return NULL;

}

static void *emergency_task(void *arg) // FILIP
{
    sigset_t signal_set;
    int signal_number;
    int result;

    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGRTMIN);

    printf("Task 2 is waiting for an accident...\n");

    result = sigwait(&signal_set, &signal_number);

    if(result != 0)
    {
        fprintf(stderr, "sigwait failed: %s\n", strerror(result));
        return NULL;
    }

    /*
    if (sem_wait(&emergency_sem) < 0)
    {
        perror("sem_wait");
        return NULL;
    }
    */
    printf("Task 2 awakened: accident notification received.\n");

    //trigger camera
    //display SOS
    //notify Task 3

    return NULL;
}

int main()
{
    pthread_t imu_thread;
    //pthread_t emergency_thread; //FILIP

    pthread_attr_t attr;
    struct sched_param param;
    int result;
    int crash_fd;
    sigset_t signal_set;

    crash_fd = open(CRASH_BUFFER_PATH, O_WRONLY);

    if(crash_fd < 0)
    {
        perror("open crash_buffer");
        return EXIT_FAILURE;
    }

    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGRTMIN);
    result = pthread_sigmask( SIG_BLOCK, &signal_set, NULL);
    if (result != 0)
    {
        fprintf(stderr, "pthread_sigmask failed: %s\n", strerror(result));
        close(crash_fd);
        return EXIT_FAILURE;
    }

    /*
    if (sem_init(&emergency_sem, 0, 0) < 0)
    {
        perror("sem_init");
        close(crash_fd);
        return EXIT_FAILURE;
    }
    */

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr,SCHED_FIFO);

    //TASK 2 FILIP :Camera trigger + SOS
    memset(&param, 0, sizeof(param));
    param.sched_priority = EMERGENCY_PRIORITY;
    pthread_attr_setschedparam(&attr,&param);
    result = pthread_create(&emergency_thread, &attr, emergency_task, NULL);
    if (result != 0)
    {
        fprintf(stderr, "emergency pthread_create failed: %s\n", strerror(result));

        pthread_attr_destroy(&attr);
        //sem_destroy(&emergency_sem);
        close(crash_fd);

        return EXIT_FAILURE;
    }


    memset(&param, 0, sizeof(param));
    param.sched_priority = LSM9DS1_IMU_PRIORITY;
    pthread_attr_setschedparam(&attr, &param);
    result = pthread_create(&imu_thread, &attr, imu_task, &crash_fd);
    if(result != 0)
    {
        fprintf(stderr, "pthread_create failed: %s\n", strerror(result));
        pthread_attr_destroy(&attr);
        return EXIT_FAILURE;
    }

    pthread_attr_destroy(&attr);

    pthread_join(imu_thread, NULL);
    pthread_join(emergency_thread, NULL);

    //sem_destroy(&emergency_sem);

    close(crash_fd);

    return EXIT_SUCCESS;
}
