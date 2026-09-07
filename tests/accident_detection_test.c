#include "accident_detection.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define DEG_TO_RAD 0.01745329251994329577f

static int failures = 0;

static void expect_true(int condition, const char *name)
{
    if (condition) {
        printf("[PASS] %s\n", name);
    } else {
        printf("[FAIL] %s\n", name);
        failures++;
    }
}

static lsm9ds1_sample_t level_sample(void)
{
    lsm9ds1_sample_t sample = {0};
    sample.accel_g.z = 1.0f;
    return sample;
}

static lsm9ds1_sample_t roll_sample(float roll_deg, float roll_rate_deg_s)
{
    lsm9ds1_sample_t sample = {0};
    float angle_rad = roll_deg * DEG_TO_RAD;

    sample.accel_g.y = sinf(angle_rad);
    sample.accel_g.z = cosf(angle_rad);
    sample.gyro_rad_s.x = roll_rate_deg_s * DEG_TO_RAD;

    return sample;
}

static void test_normal_motion(void)
{
    accident_detector_t detector;
    accident_detection_result_t result;
    lsm9ds1_sample_t sample = level_sample();
    accident_event_t event = ACCIDENT_EVENT_NONE;
    int i;

    accident_detection_init(&detector, 0.0f, 0.0f, 0.0f);

    for (i = 0; i < 1000; ++i) {
        event = accident_detection_update(&detector, &sample, &result);
        if (event != ACCIDENT_EVENT_NONE) {
            break;
        }
    }

    expect_true(event == ACCIDENT_EVENT_NONE,
                "Normal 1 g motion does not trigger an accident");
    expect_true(!result.impact_detected && !result.rollover_detected,
                "Normal motion leaves both detection flags clear");
}

static void test_impact_requires_two_samples(void)
{
    accident_detector_t detector;
    accident_detection_result_t result;
    lsm9ds1_sample_t sample = level_sample();
    accident_event_t first;
    accident_event_t second;

    accident_detection_init(&detector, 0.0f, 0.0f, 0.0f);
    accident_detection_update(&detector, &sample, &result);

    sample.accel_g.x = 4.5f;
    sample.accel_g.y = 0.0f;
    sample.accel_g.z = 0.0f;

    first = accident_detection_update(&detector, &sample, &result);
    second = accident_detection_update(&detector, &sample, &result);

    expect_true((first & ACCIDENT_EVENT_IMPACT) == 0,
                "One >=4 g sample is not enough for impact");
    expect_true((second & ACCIDENT_EVENT_IMPACT) != 0,
                "Second consecutive >=4 g sample triggers impact");
    expect_true(result.impact_detected,
                "Impact state remains latched after detection");
}

static void test_short_tilt_does_not_trigger_rollover(void)
{
    accident_detector_t detector;
    accident_detection_result_t result;
    lsm9ds1_sample_t sample;
    accident_event_t event = ACCIDENT_EVENT_NONE;
    int i;

    accident_detection_init(&detector, 0.0f, 0.0f, 0.0f);
    sample = roll_sample(70.0f, 0.0f);

    for (i = 0; i < ROLLOVER_REQUIRED_SAMPLES - 1; ++i) {
        event = accident_detection_update(&detector, &sample, &result);
    }

    expect_true((event & ACCIDENT_EVENT_ROLLOVER) == 0,
                "Tilt >60 degrees for less than 250 samples does not trigger rollover");

    event = accident_detection_update(&detector, &sample, &result);

    expect_true((event & ACCIDENT_EVENT_ROLLOVER) != 0,
                "250th consecutive tilted sample triggers rollover");
}

static void test_slow_rollover(void)
{
    accident_detector_t detector;
    accident_detection_result_t result;
    lsm9ds1_sample_t sample = level_sample();
    accident_event_t event = ACCIDENT_EVENT_NONE;
    const int ramp_samples = 1000;
    const float final_roll_deg = 70.0f;
    const float duration_s = ramp_samples * DETECTION_SAMPLE_PERIOD_S;
    const float roll_rate_deg_s = final_roll_deg / duration_s;
    int i;

    accident_detection_init(&detector, 0.0f, 0.0f, 0.0f);
    accident_detection_update(&detector, &sample, &result);

    for (i = 1; i <= ramp_samples; ++i) {
        float angle = final_roll_deg * (float)i / (float)ramp_samples;
        sample = roll_sample(angle, roll_rate_deg_s);
        event = accident_detection_update(&detector, &sample, &result);
        if (event & ACCIDENT_EVENT_ROLLOVER) {
            break;
        }
    }

    while (!(event & ACCIDENT_EVENT_ROLLOVER) && i < ramp_samples + 500) {
        sample = roll_sample(final_roll_deg, 0.0f);
        event = accident_detection_update(&detector, &sample, &result);
        ++i;
    }

    expect_true((event & ACCIDENT_EVENT_ROLLOVER) != 0,
                "Slow rollover is detected without requiring high angular speed");
    expect_true(result.tilt_deg >= ROLLOVER_ANGLE_DEG,
                "Slow rollover detection occurs while tilt is above threshold");
}

int main(void)
{
    test_normal_motion();
    test_impact_requires_two_samples();
    test_short_tilt_does_not_trigger_rollover();
    test_slow_rollover();

    if (failures == 0) {
        puts("\nAll accident-detection tests passed.");
        return EXIT_SUCCESS;
    }

    printf("\n%d accident-detection test(s) failed.\n", failures);
    return EXIT_FAILURE;
}
