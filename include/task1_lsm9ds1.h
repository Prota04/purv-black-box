#ifndef TASK1_LSM9DS1_H
#define TASK1_LSM9DS1_H

#include <pthread.h>
#include <stdint.h>

#define TASK1_LSM9DS1_PRIORITY 99

typedef struct 
{
    int imu_fd;
    int crash_fd;

    pthread_t task2_camera_thread;
    
    uint64_t *latency_start_timestamp_ns;
    uint64_t *detection_timestamp_ns;

} task1_lsm9ds1_args_t;

void *task1_lsm9ds1(void *arg);

#endif
