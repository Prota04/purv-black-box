#ifndef TASK3_STORAGE_H
#define TASK3_STORAGE_H

#include <stdint.h>

#include "camera_capture.h"

int storage_task_save_crash_data(uint64_t threshold_timestamp_ns,
                                 uint64_t detection_timestamp_ns,
                                 const camera_image_t *image);

#endif /* TASK3_STORAGE_H */