#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    int result;

    printf("Starting the camera...\n");

    result = system(
        "rpicam-still "
        "--nopreview "
        "--timeout 10 "
        "--width 1920 "
        "--height 1080 "
        "--awb auto "
        "--rotation 180 "
        "--tuning-file "
        "/usr/share/libcamera/ipa/rpi/vc4/imx219_noir.json "
        "--output test.jpg"
    );

    if (result != 0) {
        fprintf(stderr, "Error: The image was not captured.\n");
        fprintf(stderr,
                "Check the camera using: "
                "rpicam-hello --list-cameras\n");

        return EXIT_FAILURE;
    }

    printf("The image was successfully saved as test.jpg\n");

    return EXIT_SUCCESS;
}
