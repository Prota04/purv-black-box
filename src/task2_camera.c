#include "task2_camera.h"
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static pthread_t camera_thread;
static pthread_mutex_t camera_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t camera_cond = PTHREAD_COND_INITIALIZER;
static int accident_flag = 0;
static accident_event_t current_event = ACCIDENT_EVENT_NONE;

static void trigger_camera_capture(void)
{
    int result = system(
        "rpicam-still "
        "--nopreview "
        "--timeout 10 "
        "--width 1920 "
        "--height 1080 "
        "--awb auto "
        "--rotation 180 "
        "--tuning-file "
        "/usr/share/libcamera/ipa/rpi/vc4/imx219_noir.json "
        "--output test.jpg"
    );

    if (result != 0) {
        fprintf(stderr, "Error: The image was not captured.\n");
    }
}

static void display_sos_led_matrix(void)
{
    printf("SOS displayed on LED matrix.\n");
}

void *task2_camera_thread_func(void *arg)
{
    struct sched_param param;
    param.sched_priority = 95;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        perror("Failed to set Task 2 SCHED_FIFO priority");
    }

    while (1) {
        pthread_mutex_lock(&camera_mutex);
        while (!accident_flag) {
            pthread_cond_wait(&camera_cond, &camera_mutex);
        }
        
        accident_event_t event = current_event;
        accident_flag = 0;
        pthread_mutex_unlock(&camera_mutex);

        if (event & (ACCIDENT_EVENT_IMPACT | ACCIDENT_EVENT_ROLLOVER)) {
            trigger_camera_capture();
            display_sos_led_matrix();
        }
    }

    return NULL;
}

void task2_camera_init(void)
{
    accident_flag = 0;
    current_event = ACCIDENT_EVENT_NONE;
}

void task2_camera_start(void)
{
    pthread_create(&camera_thread, NULL, task2_camera_thread_func, NULL);
}

void task2_camera_trigger(accident_event_t event)
{
    pthread_mutex_lock(&camera_mutex);
    current_event = event;
    accident_flag = 1;
    pthread_cond_signal(&camera_cond);
    pthread_mutex_unlock(&camera_mutex);
}