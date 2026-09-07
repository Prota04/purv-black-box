#include "accident_detection.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_TO_DEG 57.29577951308232f
#define DEG_TO_RAD 0.01745329251994329577f

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

void accident_detection_init(accident_detector_t *detector,
                             float gyro_bias_x,
                             float gyro_bias_y,
                             float gyro_bias_z)
{
    if (detector == NULL) {
        return;
    }

    memset(detector, 0, sizeof(*detector));
    detector->gyro_bias_x = gyro_bias_x;
    detector->gyro_bias_y = gyro_bias_y;
    detector->gyro_bias_z = gyro_bias_z;
}

accident_event_t accident_detection_update(
    accident_detector_t *detector,
    const lsm9ds1_sample_t *sample,
    accident_detection_result_t *result)
{
    float total_g;
    float accel_roll;
    float accel_pitch;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float angular_speed_rad;
    float tilt;
    int accel_reliable;
    accident_event_t event = ACCIDENT_EVENT_NONE;

    if (detector == NULL || sample == NULL || result == NULL) {
        return ACCIDENT_EVENT_NONE;
    }

    gyro_x = sample->gyro_rad_s.x - detector->gyro_bias_x;
    gyro_y = sample->gyro_rad_s.y - detector->gyro_bias_y;
    gyro_z = sample->gyro_rad_s.z - detector->gyro_bias_z;

    total_g = sqrtf(
        sample->accel_g.x * sample->accel_g.x +
        sample->accel_g.y * sample->accel_g.y +
        sample->accel_g.z * sample->accel_g.z
    );

    accel_roll = atan2f(
        sample->accel_g.y,
        sample->accel_g.z
    ) * RAD_TO_DEG;

    accel_pitch = atan2f(
        -sample->accel_g.x,
        sqrtf(
            sample->accel_g.y * sample->accel_g.y +
            sample->accel_g.z * sample->accel_g.z
        )
    ) * RAD_TO_DEG;

    accel_reliable =
        total_g >= ACCEL_MIN_RELIABLE_G &&
        total_g <= ACCEL_MAX_RELIABLE_G;

    if (!detector->orientation_initialized) {
        detector->filtered_roll_deg = accel_roll;
        detector->filtered_pitch_deg = accel_pitch;
        detector->orientation_initialized = 1;
    } else {
        detector->filtered_roll_deg +=
            gyro_x * DETECTION_SAMPLE_PERIOD_S * RAD_TO_DEG;
        detector->filtered_pitch_deg +=
            gyro_y * DETECTION_SAMPLE_PERIOD_S * RAD_TO_DEG;

        if (accel_reliable) {
            detector->filtered_roll_deg =
                COMPLEMENTARY_ALPHA * detector->filtered_roll_deg +
                (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;

            detector->filtered_pitch_deg =
                COMPLEMENTARY_ALPHA * detector->filtered_pitch_deg +
                (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
        }
    }

    detector->filtered_roll_deg =
        normalize_angle(detector->filtered_roll_deg);
    detector->filtered_pitch_deg =
        normalize_angle(detector->filtered_pitch_deg);

    tilt = calculate_tilt(
        detector->filtered_roll_deg,
        detector->filtered_pitch_deg
    );

    angular_speed_rad = sqrtf(
        gyro_x * gyro_x +
        gyro_y * gyro_y +
        gyro_z * gyro_z
    );

    if (total_g >= IMPACT_THRESHOLD_G) {
        detector->impact_samples++;
    } else {
        detector->impact_samples = 0;
    }

    if (detector->impact_samples >= IMPACT_REQUIRED_SAMPLES &&
        !detector->impact_detected) {
        detector->impact_detected = 1;
        event = (accident_event_t)(event | ACCIDENT_EVENT_IMPACT);
    }

    if (tilt >= ROLLOVER_ANGLE_DEG) {
        detector->rollover_samples++;
    } else {
        detector->rollover_samples = 0;
    }

    if (detector->rollover_samples >= ROLLOVER_REQUIRED_SAMPLES &&
        !detector->rollover_detected) {
        detector->rollover_detected = 1;
        event = (accident_event_t)(event | ACCIDENT_EVENT_ROLLOVER);
    }

    result->total_g = total_g;
    result->gyro_x_rad_s = gyro_x;
    result->gyro_y_rad_s = gyro_y;
    result->gyro_z_rad_s = gyro_z;
    result->angular_speed_deg_s = angular_speed_rad * RAD_TO_DEG;
    result->roll_deg = detector->filtered_roll_deg;
    result->pitch_deg = detector->filtered_pitch_deg;
    result->tilt_deg = tilt;
    result->impact_detected = detector->impact_detected;
    result->rollover_detected = detector->rollover_detected;

    return event;
}
