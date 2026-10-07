#ifndef TASK3_STORAGE_H
#define TASK3_STORAGE_H

#include <stdint.h>

#include "camera_capture.h"
#include <pthread.h>

typedef struct
{
    pthread_mutex_t mutex;
    pthread_cond_t condition;

    int data_ready;
    int captured;

    uint64_t detection_timestamp_ns;
    camera_image_t image;

} storage_task_args_t;

void *storage_task(void *arg);

int storage_task_save_crash_data(uint64_t detection_timestamp_ns,
                                 const camera_image_t *image);

#endif /* TASK3_STORAGE_H */
