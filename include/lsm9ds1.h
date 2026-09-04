#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <stdint.h>
#include "uapi.h"

typedef struct {
    float x;
    float y;
    float z;
} lsm9ds1_vector_t;

typedef struct {
    lsm9ds1_vector_t accel_g;
    lsm9ds1_vector_t gyro_rad_s;
} lsm9ds1_sample_t;

int lsm9ds1_open(const char *i2c_device);
void lsm9ds1_close(int fd);

int lsm9ds1_read_who_am_i(int fd, uint8_t *value);
int lsm9ds1_init(int fd);

int lsm9ds1_read_accel_raw(int fd, lsm9ds1_raw_vector_t *raw);
int lsm9ds1_read_gyro_raw(int fd, lsm9ds1_raw_vector_t *raw);
int lsm9ds1_read_sample(int fd, lsm9ds1_sample_t *sample, lsm9ds1_raw_sample_t *raw_sample);

#endif