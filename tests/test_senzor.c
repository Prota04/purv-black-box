#define _POSIX_C_SOURCE 200809L

#include "lsm9ds1.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define SAMPLE_PERIOD_NS           (2L * 1000L * 1000L)
#define SAMPLE_PERIOD_S            0.002f
#define DISPLAY_EVERY_SAMPLES      50
#define CALIBRATION_SAMPLES        1000

#define IMPACT_THRESHOLD_G         4.0f
#define IMPACT_REQUIRED_SAMPLES    2

#define ROLLOVER_ANGLE_DEG         60.0f
#define ROLLOVER_REQUIRED_SAMPLES  250

#define DEFAULT_TEST_SECONDS       30
#define COMPLEMENTARY_ALPHA        0.98f
#define ACCEL_MIN_RELIABLE_G       0.80f
#define ACCEL_MAX_RELIABLE_G       1.20f
#define RAD_TO_DEG                 57.29577951308232f
#define DEG_TO_RAD                 0.01745329251994329577f

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

static uint64_t elapsed_nanoseconds(const struct timespec *start,
                                    const struct timespec *current)
{
    int64_t seconds =
        (int64_t)current->tv_sec - (int64_t)start->tv_sec;
    int64_t nanoseconds =
        (int64_t)current->tv_nsec - (int64_t)start->tv_nsec;

    return (uint64_t)(seconds * 1000000000LL + nanoseconds);
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}

static float normalize_angle(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static float calculate_tilt(float roll_deg, float pitch_deg)
{
    float roll_rad = roll_deg * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    float cosine = cosf(roll_rad) * cosf(pitch_rad);

    cosine = clamp_float(cosine, -1.0f, 1.0f);
    return acosf(cosine) * RAD_TO_DEG;
}

static int sleep_until(const struct timespec *activation_time)
{
    int result;

    do {
        result = clock_nanosleep(
            CLOCK_MONOTONIC,
            TIMER_ABSTIME,
            activation_time,
            NULL
        );
    } while (result == EINTR && keep_running);

    return result;
}

static int calibrate_gyroscope(int fd,
                               float *bias_x,
                               float *bias_y,
                               float *bias_z)
{
    struct timespec next_activation;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    int i;

    if (clock_gettime(CLOCK_MONOTONIC, &next_activation) < 0) {
        return -1;
    }

    for (i = 0; i < CALIBRATION_SAMPLES && keep_running; ++i) {
        lsm9ds1_sample_t sample;
        lsm9ds1_raw_sample_t raw_sample;
        int sleep_result;

        if (lsm9ds1_read_sample(fd, &sample, &raw_sample) < 0) {
            return -1;
        }

        sum_x += sample.gyro_rad_s.x;
        sum_y += sample.gyro_rad_s.y;
        sum_z += sample.gyro_rad_s.z;

        add_nanoseconds(&next_activation, SAMPLE_PERIOD_NS);
        sleep_result = sleep_until(&next_activation);

        if (sleep_result != 0 && sleep_result != EINTR) {
            errno = sleep_result;
            return -1;
        }
    }

    if (!keep_running) {
        errno = EINTR;
        return -1;
    }

    *bias_x = (float)(sum_x / CALIBRATION_SAMPLES);
    *bias_y = (float)(sum_y / CALIBRATION_SAMPLES);
    *bias_z = (float)(sum_z / CALIBRATION_SAMPLES);

    return 0;
}

static int parse_duration(const char *text, int *duration_seconds)
{
    char *end;
    long value;

    errno = 0;
    value = strtol(text, &end, 10);

    if (errno != 0 || text[0] == '\0' || end[0] != '\0' ||
        value < 0 || value > 86400) {
        return -1;
    }

    *duration_seconds = (int)value;
    return 0;
}

int main(int argc, char **argv)
{
    const char *device =
        (argc > 1) ? argv[1] : "/dev/i2c-1";
    const char *csv_path =
        (argc > 2) ? argv[2] : "imu_test.csv";

    int duration_seconds = DEFAULT_TEST_SECONDS;
    struct sigaction action;
    struct timespec start_time;
    struct timespec next_activation;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    float filtered_roll = 0.0f;
    float filtered_pitch = 0.0f;
    int orientation_initialized = 0;

    int impact_samples = 0;
    int rollover_samples = 0;
    int impact_detected = 0;
    int rollover_detected = 0;

    unsigned long sample_number = 0;
    uint8_t who_am_i;
    FILE *csv;
    int fd;

    if (argc > 4) {
        fprintf(
            stderr,
            "Usage: %s [i2c-device] [csv-file] [duration-seconds]\n",
            argv[0]
        );
        return 1;
    }

    if (argc > 3 &&
        parse_duration(argv[3], &duration_seconds) < 0) {
        fprintf(stderr, "Invalid duration: %s\n", argv[3]);
        return 1;
    }

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

    fd = lsm9ds1_open(device);

    if (fd < 0) {
        fprintf(stderr,
                "Cannot open %s: %s\n",
                device,
                strerror(errno));
        return 1;
    }

    if (lsm9ds1_read_who_am_i(fd, &who_am_i) < 0) {
        fprintf(stderr,
                "WHO_AM_I read failed: %s\n",
                strerror(errno));
        lsm9ds1_close(fd);
        return 1;
    }

    printf("WHO_AM_I = 0x%02X\n", who_am_i);

    if (lsm9ds1_init(fd) < 0) {
        fprintf(stderr,
                "LSM9DS1 initialization failed: %s\n",
                strerror(errno));
        lsm9ds1_close(fd);
        return 1;
    }

    puts("Keep the board flat and completely still for 2 seconds.");
    puts("Calibrating gyroscope...");

    if (calibrate_gyroscope(
            fd,
            &gyro_bias_x,
            &gyro_bias_y,
            &gyro_bias_z) < 0) {
        fprintf(stderr,
                "Gyroscope calibration failed: %s\n",
                strerror(errno));
        lsm9ds1_close(fd);
        return 1;
    }

    printf(
        "Gyro bias: x=%.6f, y=%.6f, z=%.6f rad/s\n",
        gyro_bias_x,
        gyro_bias_y,
        gyro_bias_z
    );

    csv = fopen(csv_path, "w");

    if (csv == NULL) {
        fprintf(stderr,
                "Cannot create %s: %s\n",
                csv_path,
                strerror(errno));
        lsm9ds1_close(fd);
        return 1;
    }

    /*
     * CSV output is used only by this test program. The final Task 1
     * must write sensor samples to the kernel ring buffer instead.
     */
    setvbuf(csv, NULL, _IOFBF, 1024 * 1024);

    fprintf(
        csv,
        "timestamp_ns,ax_g,ay_g,az_g,total_g,"
        "gx_rad_s,gy_rad_s,gz_rad_s,angular_speed_deg_s,"
        "roll_deg,pitch_deg,tilt_deg,impact,rollover\n"
    );

    if (clock_gettime(CLOCK_MONOTONIC, &start_time) < 0) {
        fprintf(stderr,
                "clock_gettime failed: %s\n",
                strerror(errno));
        fclose(csv);
        lsm9ds1_close(fd);
        return 1;
    }

    next_activation = start_time;

    printf("Sampling started at 500 Hz for %d seconds.\n",
           duration_seconds);
    printf("Results will be saved to %s.\n", csv_path);
    printf("Press Ctrl+C to stop earlier.\n\n");

    while (keep_running) {
        lsm9ds1_sample_t sample;
        lsm9ds1_raw_sample_t raw_sample;
        struct timespec sample_time;

        float total_g;
        float accel_roll;
        float accel_pitch;
        float tilt;

        float gyro_x;
        float gyro_y;
        float gyro_z;
        float angular_speed_rad;
        float angular_speed_deg;

        int accel_reliable;
        int sleep_result;

        if (lsm9ds1_read_sample(fd, &sample, &raw_sample) < 0) {
            fprintf(stderr,
                    "Sensor read failed: %s\n",
                    strerror(errno));
            break;
        }

        if (clock_gettime(CLOCK_MONOTONIC, &sample_time) < 0) {
            fprintf(stderr,
                    "clock_gettime failed: %s\n",
                    strerror(errno));
            break;
        }

        raw_sample.timestamp_ns = elapsed_nanoseconds(
            &start_time,
            &sample_time
        );

        gyro_x = sample.gyro_rad_s.x - gyro_bias_x;
        gyro_y = sample.gyro_rad_s.y - gyro_bias_y;
        gyro_z = sample.gyro_rad_s.z - gyro_bias_z;

        total_g = sqrtf(
            sample.accel_g.x * sample.accel_g.x +
            sample.accel_g.y * sample.accel_g.y +
            sample.accel_g.z * sample.accel_g.z
        );

        accel_roll = atan2f(
            sample.accel_g.y,
            sample.accel_g.z
        ) * RAD_TO_DEG;

        accel_pitch = atan2f(
            -sample.accel_g.x,
            sqrtf(
                sample.accel_g.y * sample.accel_g.y +
                sample.accel_g.z * sample.accel_g.z
            )
        ) * RAD_TO_DEG;

        accel_reliable =
            total_g >= ACCEL_MIN_RELIABLE_G &&
            total_g <= ACCEL_MAX_RELIABLE_G;

        if (!orientation_initialized) {
            filtered_roll = accel_roll;
            filtered_pitch = accel_pitch;
            orientation_initialized = 1;
        } else {
            filtered_roll +=
                gyro_x * SAMPLE_PERIOD_S * RAD_TO_DEG;
            filtered_pitch +=
                gyro_y * SAMPLE_PERIOD_S * RAD_TO_DEG;

            if (accel_reliable) {
                filtered_roll =
                    COMPLEMENTARY_ALPHA * filtered_roll +
                    (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;

                filtered_pitch =
                    COMPLEMENTARY_ALPHA * filtered_pitch +
                    (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
            }
        }

        filtered_roll = normalize_angle(filtered_roll);
        filtered_pitch = normalize_angle(filtered_pitch);
        tilt = calculate_tilt(filtered_roll, filtered_pitch);

        angular_speed_rad = sqrtf(
            gyro_x * gyro_x +
            gyro_y * gyro_y +
            gyro_z * gyro_z
        );

        angular_speed_deg =
            angular_speed_rad * RAD_TO_DEG;

        /*
         * Impact: total acceleration of at least 4 g for 4 ms.
         */
        if (total_g >= IMPACT_THRESHOLD_G) {
            impact_samples++;
        } else {
            impact_samples = 0;
        }

        if (impact_samples >= IMPACT_REQUIRED_SAMPLES &&
            !impact_detected) {
            impact_detected = 1;

            printf(
                "\nWARNING: IMPACT DETECTED! "
                "total=%.3f g\n\n",
                total_g
            );

            fflush(csv);
        }

        /*
         * Rollover: tilt alone is the final condition. Angular speed
         * is not required, so slow rollover is also detected.
         */
        if (tilt >= ROLLOVER_ANGLE_DEG) {
            rollover_samples++;
        } else {
            rollover_samples = 0;
        }

        if (rollover_samples >= ROLLOVER_REQUIRED_SAMPLES &&
            !rollover_detected) {
            rollover_detected = 1;

            printf(
                "\nWARNING: ROLLOVER DETECTED! "
                "roll=%.2f deg, pitch=%.2f deg, tilt=%.2f deg\n\n",
                filtered_roll,
                filtered_pitch,
                tilt
            );

            fflush(csv);
        }

        fprintf(
            csv,
            "%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.3f,"
            "%.3f,%.3f,%.3f,%d,%d\n",
            raw_sample.timestamp_ns,
            sample.accel_g.x,
            sample.accel_g.y,
            sample.accel_g.z,
            total_g,
            gyro_x,
            gyro_y,
            gyro_z,
            angular_speed_deg,
            filtered_roll,
            filtered_pitch,
            tilt,
            impact_detected,
            rollover_detected
        );

        if (sample_number % DISPLAY_EVERY_SAMPLES == 0) {
            printf(
                "ACCEL g=(%7.3f, %7.3f, %7.3f) "
                "total=%6.3f g\n",
                sample.accel_g.x,
                sample.accel_g.y,
                sample.accel_g.z,
                total_g
            );

            printf(
                "GYRO corrected=(%7.3f, %7.3f, %7.3f) rad/s "
                "speed=%7.2f deg/s\n",
                gyro_x,
                gyro_y,
                gyro_z,
                angular_speed_deg
            );

            printf(
                "ORIENTATION roll=%7.2f deg "
                "pitch=%7.2f deg "
                "tilt=%7.2f deg "
                "impact=%d rollover=%d\n\n",
                filtered_roll,
                filtered_pitch,
                tilt,
                impact_detected,
                rollover_detected
            );
        }

        sample_number++;

        if (duration_seconds > 0 &&
            raw_sample.timestamp_ns >=
                (uint64_t)duration_seconds * 1000000000ULL) {
            break;
        }

        add_nanoseconds(
            &next_activation,
            SAMPLE_PERIOD_NS
        );

        sleep_result = sleep_until(&next_activation);

        if (sleep_result != 0 &&
            sleep_result != EINTR) {
            fprintf(stderr,
                    "clock_nanosleep failed: %s\n",
                    strerror(sleep_result));
            break;
        }
    }

    printf("\nStopping IMU test.\n");
    printf("Recorded %lu samples in %s.\n",
           sample_number,
           csv_path);

    fclose(csv);
    lsm9ds1_close(fd);
    return 0;
}
