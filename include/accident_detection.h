#ifndef ACCIDENT_DETECTION_H
#define ACCIDENT_DETECTION_H

#include "lsm9ds1.h"

#define DETECTION_SAMPLE_PERIOD_S            0.002f

#define IMPACT_THRESHOLD_G                   4.0f
#define IMPACT_REQUIRED_SAMPLES              2

#define ROLLOVER_ANGLE_DEG                   60.0f
#define ROLLOVER_REQUIRED_SAMPLES            250

#define COMPLEMENTARY_ALPHA                  0.98f
#define ACCEL_MIN_RELIABLE_G                 0.80f
#define ACCEL_MAX_RELIABLE_G                 1.20f

typedef enum {
    ACCIDENT_EVENT_NONE     = 0,
    ACCIDENT_EVENT_IMPACT   = 1 << 0,
    ACCIDENT_EVENT_ROLLOVER = 1 << 1
} accident_event_t;

typedef struct {
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    float filtered_roll_deg;
    float filtered_pitch_deg;
    int orientation_initialized;

    int impact_samples;
    int rollover_samples;
    int impact_detected;
    int rollover_detected;
} accident_detector_t;

typedef struct {
    float total_g;

    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
    float angular_speed_deg_s;

    float roll_deg;
    float pitch_deg;
    float tilt_deg;

    int impact_detected;
    int rollover_detected;
} accident_detection_result_t;

void accident_detection_init(accident_detector_t *detector,
                             float gyro_bias_x,
                             float gyro_bias_y,
                             float gyro_bias_z);

accident_event_t accident_detection_update(
    accident_detector_t *detector,
    const lsm9ds1_sample_t *sample,
    accident_detection_result_t *result);

#endif
