#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <stddef.h>
#include <stdint.h>


#define CAMERA_MAX_IMAGE_BYTES (8 * 1024 * 1024)

typedef struct {
    uint8_t *data;
    size_t size;
    uint64_t timestamp_ns;
} camera_image_t;


int camera_capture_init(const char *device,
                        unsigned int width,
                        unsigned int height);


int camera_capture_snapshot(camera_image_t *image, int timeout_ms);

void camera_capture_deinit(void);

#endif /* CAMERA_CAPTURE_H */