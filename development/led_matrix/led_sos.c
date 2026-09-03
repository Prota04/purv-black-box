#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8
#define MESSAGE_WIDTH 11

#define COLOR_OFF 0x0000
#define COLOR_RED 0xF800

/*
 * A 3x5 "SOS" message:
 *
 * ###  ###  ###
 * #    # #  #
 * ###  # #  ###
 *   #  # #    #
 * ###  ###  ###
 */
static const uint8_t sos[5][MESSAGE_WIDTH] = {
    {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1},
    {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0},
    {1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1},
    {0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1},
    {1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1}
};

static int open_sense_hat_framebuffer(char *device_path, size_t path_size)
{
    int index;

    for (index = 0; index < 10; ++index) {
        char name_path[64];
        char name[128];
        FILE *name_file;
        int framebuffer;

        snprintf(name_path, sizeof(name_path),
                 "/sys/class/graphics/fb%d/name", index);

        name_file = fopen(name_path, "r");
        if (name_file == NULL) {
            continue;
        }

        if (fgets(name, sizeof(name), name_file) == NULL) {
            fclose(name_file);
            continue;
        }

        fclose(name_file);

        if (strstr(name, "RPi-Sense FB") == NULL &&
            strstr(name, "rpisense") == NULL) {
            continue;
        }

        snprintf(device_path, path_size, "/dev/fb%d", index);

        framebuffer = open(device_path, O_RDWR);
        if (framebuffer >= 0) {
            return framebuffer;
        }
    }

    errno = ENODEV;
    return -1;
}

static int write_frame(int framebuffer,
                       uint16_t pixels[MATRIX_HEIGHT][MATRIX_WIDTH])
{
    const uint8_t *data = (const uint8_t *)pixels;
    size_t remaining = MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t);

    if (lseek(framebuffer, 0, SEEK_SET) < 0) {
        return -1;
    }

    while (remaining > 0) {
        ssize_t written = write(framebuffer, data, remaining);

        if (written < 0) {
            return -1;
        }

        data += written;
        remaining -= (size_t)written;
    }

    return 0;
}

static void create_scroll_frame(
    uint16_t pixels[MATRIX_HEIGHT][MATRIX_WIDTH],
    int message_x)
{
    int x;
    int y;

    memset(pixels, 0,
           MATRIX_WIDTH * MATRIX_HEIGHT * sizeof(uint16_t));

    for (y = 0; y < 5; ++y) {
        for (x = 0; x < MESSAGE_WIDTH; ++x) {
            int screen_x = message_x + x;
            int screen_y = y + 1;

            if (sos[y][x] &&
                screen_x >= 0 && screen_x < MATRIX_WIDTH) {
                pixels[screen_y][screen_x] = COLOR_RED;
            }
        }
    }
}

int main(void)
{
    const struct timespec frame_delay = {
        .tv_sec = 0,
        .tv_nsec = 150L * 1000L * 1000L
    };
    uint16_t pixels[MATRIX_HEIGHT][MATRIX_WIDTH];
    char framebuffer_path[32];
    int framebuffer;
    int message_x;

    framebuffer = open_sense_hat_framebuffer(
        framebuffer_path, sizeof(framebuffer_path));

    if (framebuffer < 0) {
        fprintf(stderr,
                "Could not find the Sense HAT framebuffer: %s\n",
                strerror(errno));
        fprintf(stderr,
                "Check it with: cat /sys/class/graphics/fb*/name\n");
        return EXIT_FAILURE;
    }

    printf("Using %s\n", framebuffer_path);
    printf("Displaying SOS on the LED matrix...\n");


    for (message_x = MATRIX_WIDTH;
            message_x >= -MESSAGE_WIDTH;
            --message_x) {
        create_scroll_frame(pixels, message_x);

        if (write_frame(framebuffer, pixels) < 0) {
            fprintf(stderr, "LED write failed: %s\n",
                    strerror(errno));
            close(framebuffer);
            return EXIT_FAILURE;
        }

        nanosleep(&frame_delay, NULL);
    }

    /* Clear the LED matrix before exiting. */
    memset(pixels, 0, sizeof(pixels));

    if (write_frame(framebuffer, pixels) < 0) {
        fprintf(stderr, "Could not clear the LED matrix: %s\n",
                strerror(errno));
        close(framebuffer);
        return EXIT_FAILURE;
    }

    close(framebuffer);
    printf("LED test completed.\n");

    return EXIT_SUCCESS;
}
