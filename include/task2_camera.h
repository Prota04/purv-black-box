#ifndef TASK2_CAMERA_H
#define TASK2_CAMERA_H

#include <pthread.h>
#include "accident_detection.h"

#define CAMERA_IMAGE_PATH "test.jpg"

void task2_camera_init(void);
void task2_camera_start(void);
void task2_camera_trigger(accident_event_t event);
void *task2_camera_thread_func(void *arg);
void trigger_camera_capture(void);
void display_sos_led_matrix(void);

#endif