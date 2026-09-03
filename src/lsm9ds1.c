#define _POSIX_C_SOURCE 200809L

#include "lsm9ds1.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

/* Accelerometer/gyroscope register map. */
#define REG_WHO_AM_I      0x0F
#define REG_CTRL_REG1_G   0x10
#define REG_OUT_X_L_G     0x18
#define REG_CTRL_REG4     0x1E
#define REG_CTRL_REG5_XL  0x1F
#define REG_CTRL_REG6_XL  0x20
#define REG_CTRL_REG8     0x22
#define REG_OUT_X_L_XL    0x28

/*
 * Configuration used here:
 *   accelerometer: 952 Hz, +/-16 g  -> 0.122 mg/LSB
 *   gyroscope:     952 Hz, 500 dps -> 17.50 mdps/LSB
 */
#define ACCEL_G_PER_LSB       0.000732f
#define GYRO_DPS_PER_LSB      0.01750f
#define DEG_TO_RAD            0.01745329251994329577f

static int write_register(int fd, uint8_t reg, uint8_t value)
{
    uint8_t message[2] = {reg, value};

    if (write(fd, message, sizeof(message)) != (ssize_t)sizeof(message)) {
        return -1;
    }

    return 0;
}

static int read_registers(int fd, uint8_t start_reg, uint8_t *data, size_t length)
{
    /*
     * Bit 7 is the multiple-register read flag. Without it, reading six bytes
     * may repeatedly return one register and produce impossible XYZ values.
     */
    uint8_t address = (length > 1) ? (uint8_t)(start_reg | 0x80U) : start_reg;

    if (write(fd, &address, 1) != 1) {
        return -1;
    }

    if (read(fd, data, length) != (ssize_t)length) {
        return -1;
    }

    return 0;
}

static int16_t little_endian_i16(const uint8_t *bytes)
{
    uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    return (int16_t)value;
}

static int read_raw_vector(int fd, uint8_t first_register,
                           lsm9ds1_raw_vector_t *raw)
{
    uint8_t bytes[6];

    if (raw == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (read_registers(fd, first_register, bytes, sizeof(bytes)) < 0) {
        return -1;
    }

    raw->x = little_endian_i16(&bytes[0]);
    raw->y = little_endian_i16(&bytes[2]);
    raw->z = little_endian_i16(&bytes[4]);

    return 0;
}

int lsm9ds1_open(const char *i2c_device)
{
    int fd;

    if (i2c_device == NULL) {
        errno = EINVAL;
        return -1;
    }

    fd = open(i2c_device, O_RDWR);
    if (fd < 0) {
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, LSM9DS1_AG_ADDRESS) < 0) {
        int saved_errno = errno;
        close(fd);
        errno = saved_errno;
        return -1;
    }

    return fd;
}

void lsm9ds1_close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}

int lsm9ds1_read_who_am_i(int fd, uint8_t *value)
{
    if (value == NULL) {
        errno = EINVAL;
        return -1;
    }

    return read_registers(fd, REG_WHO_AM_I, value, 1);
}

int lsm9ds1_init(int fd)
{
    uint8_t who_am_i;

    if (lsm9ds1_read_who_am_i(fd, &who_am_i) < 0) {
        return -1;
    }

    if (who_am_i != LSM9DS1_WHO_AM_I_EXPECTED) {
        errno = ENODEV;
        return -1;
    }

    /*
     * CTRL_REG8 = 0x44:
     *   BDU = 1       - low/high output bytes stay from the same sample
     *   IF_ADD_INC=1  - automatic address increment
     */
    if (write_register(fd, REG_CTRL_REG8, 0x44) < 0) {
        return -1;
    }

    /* Enable X, Y and Z axes. */
    if (write_register(fd, REG_CTRL_REG4, 0x38) < 0 ||
        write_register(fd, REG_CTRL_REG5_XL, 0x38) < 0) {
        return -1;
    }

    /* Gyroscope: ODR=952 Hz, full scale=500 dps, BW=00. */
    if (write_register(fd, REG_CTRL_REG1_G, 0xC8) < 0) {
        return -1;
    }

    /* Accelerometer: ODR=952 Hz, full scale=+/-16 g. */
    if (write_register(fd, REG_CTRL_REG6_XL, 0xC8) < 0) {
        return -1;
    }

    /* Allow the first configured samples to become available. */
    {
        const struct timespec delay = {.tv_sec = 0, .tv_nsec = 20L * 1000L * 1000L};
        nanosleep(&delay, NULL);
    }

    return 0;
}

int lsm9ds1_read_accel_raw(int fd, lsm9ds1_raw_vector_t *raw)
{
    return read_raw_vector(fd, REG_OUT_X_L_XL, raw);
}

int lsm9ds1_read_gyro_raw(int fd, lsm9ds1_raw_vector_t *raw)
{
    return read_raw_vector(fd, REG_OUT_X_L_G, raw);
}

int lsm9ds1_read_sample(int fd, lsm9ds1_sample_t *sample)
{
    if (sample == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (lsm9ds1_read_accel_raw(fd, &sample->accel_raw) < 0 ||
        lsm9ds1_read_gyro_raw(fd, &sample->gyro_raw) < 0) {
        return -1;
    }

    sample->accel_g.x = sample->accel_raw.x * ACCEL_G_PER_LSB;
    sample->accel_g.y = sample->accel_raw.y * ACCEL_G_PER_LSB;
    sample->accel_g.z = sample->accel_raw.z * ACCEL_G_PER_LSB;

    sample->gyro_rad_s.x =
        sample->gyro_raw.x * GYRO_DPS_PER_LSB * DEG_TO_RAD;
    sample->gyro_rad_s.y =
        sample->gyro_raw.y * GYRO_DPS_PER_LSB * DEG_TO_RAD;
    sample->gyro_rad_s.z =
        sample->gyro_raw.z * GYRO_DPS_PER_LSB * DEG_TO_RAD;

    return 0;
}