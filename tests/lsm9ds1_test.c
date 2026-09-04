#define _POSIX_C_SOURCE 200809L

#include "lsm9ds1.h"
#include "uapi.h"

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* Sampling configuration: 2 ms = 500 Hz. */
#define SAMPLE_PERIOD_NS          (2L * 1000L * 1000L)

/* Print every 50 samples: 50 * 2 ms = 100 ms. */
#define DISPLAY_EVERY_SAMPLES     50

/* Impact detection configuration. */
#define IMPACT_THRESHOLD_G        4.0f
#define IMPACT_REQUIRED_SAMPLES   3

/* Rollover detection configuration. */
#define ROLLOVER_ANGLE_DEG        60.0f
#define ROTATION_THRESHOLD_DPS    100.0f

/* 250 samples * 2 ms = 500 ms. */
#define ROLLOVER_REQUIRED_SAMPLES 250

/* Remember a rapid rotation for one second. */
#define ROTATION_MEMORY_SAMPLES   500

#define RAD_TO_DEG                57.29577951308232f

static volatile sig_atomic_t keep_running = 1;

static void handle_stop_signal(int signal_number)
{
    (void)signal_number;
    keep_running = 0;
}

static void add_nanoseconds(struct timespec *time, long nanoseconds)
{
    time->tv_nsec += nanoseconds;

    while (time->tv_nsec >= 1000000000L) {
        time->tv_sec++;
        time->tv_nsec -= 1000000000L;
    }
}

int main(int argc, char **argv)
{
    const char *device =
        (argc > 1) ? argv[1] : "/dev/i2c-1";

    struct sigaction action;
    struct timespec next_activation;

    int impact_samples = 0;
    int rollover_samples = 0;
    int rotation_memory = 0;

    int impact_active = 0;
    int rollover_detected = 0;

    unsigned long sample_number = 0;

    uint8_t who_am_i;
    int fd;

    /*
     * Allow the program to stop safely using Ctrl+C.
     */
    memset(&action, 0, sizeof(action));
    action.sa_handler = handle_stop_signal;

    sigemptyset(&action.sa_mask);

    if (sigaction(SIGINT, &action, NULL) < 0 ||
        sigaction(SIGTERM, &action, NULL) < 0) {
        fprintf(stderr,
                "Cannot install signal handler: %s\n",
                strerror(errno));
        return 1;
    }

    /*
     * Open the I2C device.
     */
    fd = lsm9ds1_open(device);

    if (fd < 0) {
        fprintf(stderr,
                "Cannot open %s: %s\n",
                device,
                strerror(errno));
        return 1;
    }

    /*
     * Read and display the sensor identification register.
     */
    if (lsm9ds1_read_who_am_i(fd, &who_am_i) < 0) {
        fprintf(stderr,
                "WHO_AM_I read failed: %s\n",
                strerror(errno));

        lsm9ds1_close(fd);
        return 1;
    }

    printf("WHO_AM_I = 0x%02X\n", who_am_i);

    /*
     * Configure the accelerometer and gyroscope.
     */
    if (lsm9ds1_init(fd) < 0) {
        fprintf(stderr,
                "LSM9DS1 initialization failed: %s\n",
                strerror(errno));

        lsm9ds1_close(fd);
        return 1;
    }

    /*
     * Use an absolute activation time to prevent timing drift.
     */
    if (clock_gettime(CLOCK_MONOTONIC, &next_activation) < 0) {
        fprintf(stderr,
                "clock_gettime failed: %s\n",
                strerror(errno));

        lsm9ds1_close(fd);
        return 1;
    }

    printf("Sampling started at 500 Hz.\n");
    printf("Press Ctrl+C to stop.\n\n");

    while (keep_running) {
        lsm9ds1_sample_t sample;
        lsm9ds1_raw_sample_t raw_sample;

        float total_g;
        float roll;
        float pitch;
        float angular_speed_rad;
        float angular_speed_deg;

        int sleep_result;

        /*
         * Read one accelerometer and gyroscope sample.
         */
        if (lsm9ds1_read_sample(fd, &sample, &raw_sample) < 0) {
            fprintf(stderr,
                    "Sensor read failed: %s\n",
                    strerror(errno));
            break;
        }

        sample_number++;

        /*
         * Calculate the total acceleration magnitude.
         */
        total_g = sqrtf(
            sample.accel_g.x * sample.accel_g.x +
            sample.accel_g.y * sample.accel_g.y +
            sample.accel_g.z * sample.accel_g.z
        );

        /*
         * Calculate roll and pitch from the accelerometer.
         *
         * These angles are reliable mainly when gravity is the
         * dominant acceleration acting on the sensor.
         */
        roll = atan2f(
            sample.accel_g.y,
            sample.accel_g.z
        ) * RAD_TO_DEG;

        pitch = atan2f(
            -sample.accel_g.x,
            sqrtf(
                sample.accel_g.y * sample.accel_g.y +
                sample.accel_g.z * sample.accel_g.z
            )
        ) * RAD_TO_DEG;

        /*
         * Calculate the magnitude of angular velocity.
         *
         * gyro_rad_s already contains converted values in rad/s.
         */
        angular_speed_rad = sqrtf(
            sample.gyro_rad_s.x * sample.gyro_rad_s.x +
            sample.gyro_rad_s.y * sample.gyro_rad_s.y +
            sample.gyro_rad_s.z * sample.gyro_rad_s.z
        );

        angular_speed_deg = angular_speed_rad * RAD_TO_DEG;

        /*
         * Impact detection.
         *
         * Three consecutive samples represent approximately 6 ms
         * at the sampling rate of 500 Hz.
         */
        if (total_g >= IMPACT_THRESHOLD_G) {
            impact_samples++;
        } else {
            impact_samples = 0;
            impact_active = 0;
        }

        if (impact_samples >= IMPACT_REQUIRED_SAMPLES &&
            !impact_active) {
            impact_active = 1;

            printf(
                "\nWARNING: Impact detected! "
                "Acceleration = %.3f g\n\n",
                total_g
            );
        }

        /*
         * Remember that rapid rotation occurred during the
         * previous second.
         */
        if (angular_speed_deg >= ROTATION_THRESHOLD_DPS) {
            rotation_memory = ROTATION_MEMORY_SAMPLES;
        } else if (rotation_memory > 0) {
            rotation_memory--;
        }

        /*
         * Count how long the vehicle remains at a large angle.
         */
        if (fabsf(roll) >= ROLLOVER_ANGLE_DEG ||
            fabsf(pitch) >= ROLLOVER_ANGLE_DEG) {
            rollover_samples++;
        } else {
            rollover_samples = 0;
        }

        /*
         * Confirm rollover only if:
         *   1. rapid rotation was recently detected;
         *   2. the large angle lasts at least 500 ms.
         */
        if (rotation_memory > 0 &&
            rollover_samples >= ROLLOVER_REQUIRED_SAMPLES &&
            !rollover_detected) {
            rollover_detected = 1;

            printf(
                "\nWARNING: Rollover detected! "
                "Roll = %.2f deg, Pitch = %.2f deg\n\n",
                roll,
                pitch
            );
        }

        /*
         * Sampling occurs every 2 ms, but printing every 2 ms
         * would unnecessarily slow down the program.
         * Print values every 100 ms instead.
         */
        if (sample_number % DISPLAY_EVERY_SAMPLES == 0) {
            printf(
                "ACCEL raw=(%6d, %6d, %6d) "
                "g=(%7.3f, %7.3f, %7.3f) "
                "total=%6.3f g\n",
                raw_sample.accel_raw.x,
                raw_sample.accel_raw.y,
                raw_sample.accel_raw.z,
                sample.accel_g.x,
                sample.accel_g.y,
                sample.accel_g.z,
                total_g
            );

            printf(
                "GYRO raw=(%6d, %6d, %6d) "
                "rad/s=(%7.3f, %7.3f, %7.3f) "
                "angular_speed=%7.2f deg/s "
                "rapid_rotation=%d\n",
                raw_sample.gyro_raw.x,
                raw_sample.gyro_raw.y,
                raw_sample.gyro_raw.z,
                sample.gyro_rad_s.x,
                sample.gyro_rad_s.y,
                sample.gyro_rad_s.z,
                angular_speed_deg,
                rotation_memory > 0
            );

            printf(
                "ORIENTATION roll=%7.2f deg "
                "pitch=%7.2f deg "
                "rollover_samples=%d\n\n",
                roll,
                pitch,
                rollover_samples
            );
        }

        /*
         * Calculate the next absolute activation time.
         */
        add_nanoseconds(
            &next_activation,
            SAMPLE_PERIOD_NS
        );

        /*
         * Sleep until the next 2 ms activation point.
         */
        do {
            sleep_result = clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &next_activation,
                NULL
            );
        } while (sleep_result == EINTR && keep_running);

        if (sleep_result != 0 &&
            sleep_result != EINTR) {
            fprintf(stderr,
                    "clock_nanosleep failed: %s\n",
                    strerror(sleep_result));
            break;
        }
    }

    printf("\nStopping IMU test.\n");

    lsm9ds1_close(fd);
    return 0;
}