#ifndef UAPI_H
#define UAPI_H

#include <linux/types.h>

#define LSM9DS1_AG_ADDRESS 0x6A
#define LSM9DS1_WHO_AM_I_EXPECTED 0x68

#define CRASH_BUFFER_PATH "/dev/crash_buffer"
#define BUFFER_SIZE       2500 

typedef struct {
    __s16 x;
    __s16 y;
    __s16 z;
} lsm9ds1_raw_vector_t;

typedef struct {
    __u64 timestamp_ns;
    lsm9ds1_raw_vector_t accel_raw;
    lsm9ds1_raw_vector_t gyro_raw;
} lsm9ds1_raw_sample_t;

#endif