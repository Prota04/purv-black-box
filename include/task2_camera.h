#ifndef TASK2_CAMERA_H
#define TASK2_CAMERA_H

#include <pthread.h>

#include "accident_detection.h"
#include "camera_capture.h"

void task2_camera_init(void);
void task2_camera_start(void);
void task2_camera_trigger(accident_event_t event);
void *task2_camera_thread_func(void *arg);


int trigger_camera_capture(camera_image_t *image);

void display_sos_led_matrix(void);

#endif 