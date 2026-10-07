#ifndef TASK2_CAMERA_H
#define TASK2_CAMERA_H

#include "camera_capture.h"
#include <pthread.h>
#include "storage.h"

typedef struct 
{
    uint64_t *latency_start_timestamp_ns;
    uint64_t *detection_timestamp_ns;

    storage_task_args_t *storage_args;
} task2_camera_args_t;

int trigger_camera_capture(camera_image_t *image);
void display_sos_led_matrix(void);
void *task2_camera(void *arg);

#endif /* TASK2_CAMERA_H */
