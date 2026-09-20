#include "uapi.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define EXPECTED_SAMPLES 500

int main(void)
{
    lsm9ds1_raw_sample_t *samples;
    ssize_t bytes_read;
    ssize_t sample_count;
    int fd;

    fd = open(CRASH_BUFFER_PATH, O_RDONLY);

    if (fd < 0)
    {
        perror("open crash_buffer");
        return EXIT_FAILURE;
    }

    samples = calloc(BUFFER_SIZE, sizeof(*samples));

    if (samples == NULL)
    {
        perror("calloc");
        close(fd);
        return EXIT_FAILURE;
    }

    bytes_read = read(fd, samples, BUFFER_SIZE * sizeof(*samples));

    if (bytes_read < 0)
    {
        perror("read crash_buffer");
        free(samples);
        close(fd);
        return EXIT_FAILURE;
    }

    sample_count = bytes_read / (ssize_t)sizeof(*samples);

    printf("Samples in crash buffer: %ld\n", (long)sample_count);

    if (sample_count > 0)
    {
        printf("\nFirst sample:\n");
        printf("timestamp = %llu\n",
               (unsigned long long)samples[0].timestamp_ns);

        printf("accel = (%d, %d, %d)\n",
               samples[0].accel_raw.x,
               samples[0].accel_raw.y,
               samples[0].accel_raw.z);

        printf("gyro  = (%d, %d, %d)\n",
               samples[0].gyro_raw.x,
               samples[0].gyro_raw.y,
               samples[0].gyro_raw.z);

        printf("\nLast sample:\n");
        printf("timestamp = %llu\n",
               (unsigned long long)
               samples[sample_count - 1].timestamp_ns);

        printf("accel = (%d, %d, %d)\n",
               samples[sample_count - 1].accel_raw.x,
               samples[sample_count - 1].accel_raw.y,
               samples[sample_count - 1].accel_raw.z);

        printf("gyro  = (%d, %d, %d)\n",
               samples[sample_count - 1].gyro_raw.x,
               samples[sample_count - 1].gyro_raw.y,
               samples[sample_count - 1].gyro_raw.z);
    }

    if (sample_count == EXPECTED_SAMPLES)
    {
        printf("\n[PASS] Task 1 wrote 500 samples.\n");
    }
    else
    {
        printf("\n[FAIL] Expected 500 samples, got %ld.\n", (long)sample_count);
    }

    free(samples);
    close(fd);

    return EXIT_SUCCESS;
}
