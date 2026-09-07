#define _POSIX_C_SOURCE 200809L

#include "lsm9ds1.h"
#include "accident_detection.h"

#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define SAMPLE_PERIOD_NS           (2L * 1000L * 1000L)
#define DISPLAY_EVERY_SAMPLES      50
#define CALIBRATION_SAMPLES        1000
#define DEFAULT_TEST_SECONDS       30

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
    accident_detector_t detector;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

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

    accident_detection_init(
        &detector,
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
        accident_detection_result_t detection;
        accident_event_t event;
        struct timespec sample_time;
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

        event = accident_detection_update(
            &detector,
            &sample,
            &detection
        );

        if (event & ACCIDENT_EVENT_IMPACT) {
            printf(
                "\nWARNING: IMPACT DETECTED! total=%.3f g\n\n",
                detection.total_g
            );
            fflush(csv);
        }

        if (event & ACCIDENT_EVENT_ROLLOVER) {
            printf(
                "\nWARNING: ROLLOVER DETECTED! "
                "roll=%.2f deg, pitch=%.2f deg, tilt=%.2f deg\n\n",
                detection.roll_deg,
                detection.pitch_deg,
                detection.tilt_deg
            );
            fflush(csv);
        }

        fprintf(
            csv,
            "%llu,%.6f,%.6f,%.6f,%.6f,"
            "%.6f,%.6f,%.6f,%.3f,"
            "%.3f,%.3f,%.3f,%d,%d\n",
            (unsigned long long)raw_sample.timestamp_ns,
            sample.accel_g.x,
            sample.accel_g.y,
            sample.accel_g.z,
            detection.total_g,
            detection.gyro_x_rad_s,
            detection.gyro_y_rad_s,
            detection.gyro_z_rad_s,
            detection.angular_speed_deg_s,
            detection.roll_deg,
            detection.pitch_deg,
            detection.tilt_deg,
            detection.impact_detected,
            detection.rollover_detected
        );

        if (sample_number % DISPLAY_EVERY_SAMPLES == 0) {
            printf(
                "ACCEL g=(%7.3f, %7.3f, %7.3f) total=%6.3f g\n",
                sample.accel_g.x,
                sample.accel_g.y,
                sample.accel_g.z,
                detection.total_g
            );

            printf(
                "GYRO corrected=(%7.3f, %7.3f, %7.3f) rad/s "
                "speed=%7.2f deg/s\n",
                detection.gyro_x_rad_s,
                detection.gyro_y_rad_s,
                detection.gyro_z_rad_s,
                detection.angular_speed_deg_s
            );

            printf(
                "ORIENTATION roll=%7.2f deg "
                "pitch=%7.2f deg "
                "tilt=%7.2f deg "
                "impact=%d rollover=%d\n\n",
                detection.roll_deg,
                detection.pitch_deg,
                detection.tilt_deg,
                detection.impact_detected,
                detection.rollover_detected
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
