#define _POSIX_C_SOURCE 200809L

#include "task2_camera.h"
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8
#define MESSAGE_WIDTH 11
#define COLOR_RED 0xF800

static pthread_t camera_thread;
static pthread_mutex_t camera_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t camera_cond = PTHREAD_COND_INITIALIZER;
static int accident_flag = 0;
static accident_event_t current_event = ACCIDENT_EVENT_NONE;

static const uint8_t sos[5][MESSAGE_WIDTH] = {
    {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1},
    {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0},
    {1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1},
    {0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
    {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1}
};

static int open_sense_hat_framebuffer(char *device_path, size_t path_size)
{
    int index;

    for (index = 0; index < 10; ++index) {
        char name_path[64];
        char name[128];
        FILE *name_file;
        int framebuffer;

        snprintf(name_path, sizeof(name_path),
                 "/sys/class/graphics/fb%d/name", index);

        name_file = fopen(name_path, "r");
        if (name_file == NULL) {
            continue;
        }

        if (fgets(name, sizeof(name), name_file) == NULL) {
            fclose(name_file);
            continue;
        }

        fclose(name_file);

        if (strstr(name, "RPi-Sense FB") == NULL &&
            strstr(name, "rpisense") == NULL) {
            continue;
        }

        snprintf(device_path, path_size, "/dev/fb%d", index);

        framebuffer = open(device_path, O_RDWR);
        if (framebuffer >= 0) {
            return framebuffer;
        }
    }

    errno = ENODEV;
    return -1;
}

static int write_frame(int framebuffer,
                       uint16_t pixels[MATRIX_HEIGHT][MATRIX_WIDTH])
{
    const uint8_t *data = (const uint8_t *)pixels;
    size_t remaining = MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t);

    if (lseek(framebuffer, 0, SEEK_SET) < 0) {
        return -1;
    }

    while (remaining > 0) {
        ssize_t written = write(framebuffer, data, remaining);

        if (written < 0) {
            return -1;
        }

        data += written;
        remaining -= (size_t)written;
    }

    return 0;
}

static void create_scroll_frame(
    uint16_t pixels[MATRIX_HEIGHT][MATRIX_WIDTH],
    int message_x)
{
    int x;
    int y;

    memset(pixels, 0,
           MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));

    for (y = 0; y < 5; ++y) {
        for (x = 0; x < MESSAGE_WIDTH; ++x) {
            int screen_x = message_x + x;
            int screen_y = y + 1;

            if (sos[y][x] &&
                screen_x >= 0 && screen_x < MATRIX_WIDTH) {
                pixels[screen_y][screen_x] = COLOR_RED;
            }
        }
    }
}

void trigger_camera_capture(void)
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

void display_sos_led_matrix(void)
{
    const struct timespec frame_delay = {
        .tv_sec = 0,
        .tv_nsec = 150L * 1000L * 1000L
    };
    uint16_t pixels[MATRIX_HEIGHT][MATRIX_WIDTH];
    char framebuffer_path[32];
    int framebuffer;
    int message_x;

    framebuffer = open_sense_hat_framebuffer(
        framebuffer_path, sizeof(framebuffer_path));

    if (framebuffer < 0) {
        fprintf(stderr,
                "Could not find the Sense HAT framebuffer: %s\n",
                strerror(errno));
        return;
    }

    for (message_x = MATRIX_WIDTH;
            message_x >= -MESSAGE_WIDTH;
            --message_x) {
        create_scroll_frame(pixels, message_x);

        if (write_frame(framebuffer, pixels) < 0) {
            fprintf(stderr, "LED write failed: %s\n",
                    strerror(errno));
            close(framebuffer);
            return;
        }

        nanosleep(&frame_delay, NULL);
    }

    memset(pixels, 0, sizeof(pixels));
    write_frame(framebuffer, pixels);
    close(framebuffer);
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