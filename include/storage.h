#ifndef TASK3_STORAGE_H
#define TASK3_STORAGE_H

#include <stdint.h>

int storage_task_save_crash_data(uint64_t crash_timestamp_ns, const char *camera_image_path);

#endif