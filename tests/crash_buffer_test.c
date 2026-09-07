#include "uapi.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int failures = 0;

static void pass(const char *message)
{
    printf("[PASS] %s\n", message);
}

static void fail(const char *message)
{
    printf("[FAIL] %s\n", message);
    failures++;
}

static lsm9ds1_raw_sample_t make_sample(uint64_t index)
{
    lsm9ds1_raw_sample_t sample;

    memset(&sample, 0, sizeof(sample));
    sample.timestamp_ns = index;

    sample.accel_raw.x = (__s16)(100 + index);
    sample.accel_raw.y = (__s16)(200 + index);
    sample.accel_raw.z = (__s16)(300 + index);

    sample.gyro_raw.x = (__s16)(400 + index);
    sample.gyro_raw.y = (__s16)(500 + index);
    sample.gyro_raw.z = (__s16)(600 + index);

    return sample;
}

static int same_sample(const lsm9ds1_raw_sample_t *actual,
                       const lsm9ds1_raw_sample_t *expected)
{
    return actual->timestamp_ns == expected->timestamp_ns &&
           actual->accel_raw.x == expected->accel_raw.x &&
           actual->accel_raw.y == expected->accel_raw.y &&
           actual->accel_raw.z == expected->accel_raw.z &&
           actual->gyro_raw.x == expected->gyro_raw.x &&
           actual->gyro_raw.y == expected->gyro_raw.y &&
           actual->gyro_raw.z == expected->gyro_raw.z;
}

static int write_one(int fd, uint64_t index)
{
    lsm9ds1_raw_sample_t sample = make_sample(index);
    ssize_t written = write(fd, &sample, sizeof(sample));

    if (written != (ssize_t)sizeof(sample)) {
        fprintf(stderr,
                "write(sample=%llu) failed: %s\n",
                (unsigned long long)index,
                (written < 0) ? strerror(errno) : "short write");
        return -1;
    }

    return 0;
}

static ssize_t read_all(int fd, lsm9ds1_raw_sample_t *samples, size_t capacity)
{
    ssize_t bytes = read(fd, samples, capacity * sizeof(*samples));

    if (bytes < 0) {
        fprintf(stderr, "read failed: %s\n", strerror(errno));
        return -1;
    }

    if ((size_t)bytes % sizeof(*samples) != 0) {
        fprintf(stderr, "driver returned a partial sample\n");
        return -1;
    }

    return bytes / (ssize_t)sizeof(*samples);
}

int main(void)
{
    lsm9ds1_raw_sample_t *samples;
    ssize_t count;
    uint64_t i;
    int fd;

    printf("Testing existing %s implementation only.\n", CRASH_BUFFER_PATH);
    printf("Run this after freshly unloading/reloading the kernel module.\n\n");

    fd = open(CRASH_BUFFER_PATH, O_RDWR);
    if (fd < 0) {
        fprintf(stderr,
                "Cannot open %s: %s\n",
                CRASH_BUFFER_PATH,
                strerror(errno));
        return EXIT_FAILURE;
    }

    samples = calloc(BUFFER_SIZE, sizeof(*samples));
    if (samples == NULL) {
        fprintf(stderr, "calloc failed\n");
        close(fd);
        return EXIT_FAILURE;
    }

    /* Test 1: a freshly loaded buffer should be empty. */
    count = read_all(fd, samples, BUFFER_SIZE);
    if (count == 0) {
        pass("Fresh buffer is empty");
    } else {
        fail("Fresh buffer is empty (reload the module before running the test)");
    }

    /* Test 2: three writes must be returned oldest -> newest. */
    for (i = 0; i < 3; ++i) {
        if (write_one(fd, i) < 0) {
            free(samples);
            close(fd);
            return EXIT_FAILURE;
        }
    }

    count = read_all(fd, samples, BUFFER_SIZE);
    if (count == 3) {
        pass("Three written samples are readable");
    } else {
        fail("Read returns exactly three samples after three writes");
    }

    if (count == 3) {
        int order_ok = 1;

        for (i = 0; i < 3; ++i) {
            lsm9ds1_raw_sample_t expected = make_sample(i);
            if (!same_sample(&samples[i], &expected)) {
                order_ok = 0;
                break;
            }
        }

        if (order_ok) {
            pass("Samples are read in chronological order");
        } else {
            fail("Samples are read in chronological order");
        }
    }

    /*
     * Test 3: continue until 2505 total samples have been written.
     * With a 2500-sample ring buffer, samples 0..4 must be overwritten,
     * so the retained range must be 5..2504.
     */
    for (i = 3; i < (uint64_t)BUFFER_SIZE + 5U; ++i) {
        if (write_one(fd, i) < 0) {
            free(samples);
            close(fd);
            return EXIT_FAILURE;
        }
    }

    count = read_all(fd, samples, BUFFER_SIZE);
    if (count == BUFFER_SIZE) {
        pass("Buffer capacity is limited to BUFFER_SIZE samples");
    } else {
        fail("Full buffer read returns BUFFER_SIZE samples");
    }

    if (count == BUFFER_SIZE) {
        int wrap_ok = 1;

        for (i = 0; i < BUFFER_SIZE; ++i) {
            uint64_t expected_index = i + 5U;
            lsm9ds1_raw_sample_t expected = make_sample(expected_index);

            if (!same_sample(&samples[i], &expected)) {
                fprintf(stderr,
                        "Mismatch at position %llu: expected sample %llu, got timestamp %llu\n",
                        (unsigned long long)i,
                        (unsigned long long)expected_index,
                        (unsigned long long)samples[i].timestamp_ns);
                wrap_ok = 0;
                break;
            }
        }

        if (wrap_ok) {
            pass("When full, oldest samples are overwritten and order stays chronological");
        } else {
            fail("Ring-buffer wraparound preserves the newest 2500 samples in order");
        }
    }

    /* Test 4: current read implementation does not consume samples. */
    count = read_all(fd, samples, BUFFER_SIZE);
    if (count == BUFFER_SIZE && samples[0].timestamp_ns == 5U) {
        pass("Reading is non-destructive in the current implementation");
    } else {
        fail("Repeated read returns the same current buffer contents");
    }

    free(samples);
    close(fd);

    if (failures == 0) {
        puts("\nAll crash-buffer tests passed.");
        return EXIT_SUCCESS;
    }

    printf("\n%d crash-buffer test(s) failed.\n", failures);
    return EXIT_FAILURE;
}
