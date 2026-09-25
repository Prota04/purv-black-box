#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <stddef.h>
#include <stdint.h>

#define CAMERA_MAX_IMAGE_BYTES (8U * 1024U * 1024U)

typedef struct {
    uint8_t *data;
    size_t size;

    /* Monotonic timestamp taken immediately before the captured frame
     * starts being copied into secure_image_memory. This is the timestamp
     * used for the project latency measurement. */
    uint64_t first_byte_timestamp_ns;

    /* Timestamp supplied by the V4L2 driver for the captured frame when
     * V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC is available. */
    uint64_t frame_timestamp_ns;

    unsigned int width;
    unsigned int height;
    unsigned int bytesperline;
    uint32_t pixelformat;
} camera_image_t;

/*
 * Configures the IMX219 sensor subdevice for SRGGB10_1X10 and the Unicam
 * video node for packed 10-bit Bayer (pRAA), allocates MMAP buffers, starts
 * streaming, and starts a background thread that continuously DQBUF/QBUF's
 * frames so the camera never stalls while waiting for an accident.
 */
int camera_capture_init(const char *video_device,
                        const char *sensor_subdevice,
                        unsigned int width,
                        unsigned int height);

/*
 * Requests the NEXT fresh frame after this call. The background camera
 * thread copies that frame into a protected userspace buffer and wakes the
 * caller. timeout_ms must be > 0.
 */
int camera_capture_snapshot(camera_image_t *image, int timeout_ms);

void camera_capture_deinit(void);

#endif /* CAMERA_CAPTURE_H */