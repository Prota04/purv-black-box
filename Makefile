CROSS_COMPILE := arm-linux-gnueabihf-
CC := $(CROSS_COMPILE)gcc

CFLAGS := -Wall -Wextra -Wpedantic -Iinclude -I.
LDLIBS := -lm

BUILD_DIR := build

IMU_TARGET := $(BUILD_DIR)/test_imu
DETECTION_TARGET := $(BUILD_DIR)/test_detection
BUFFER_TARGET := $(BUILD_DIR)/test_crash_buffer
BLACK_BOX_TARGET := $(BUILD_DIR)/black_box
TASK1_BUFFER_TARGET := $(BUILD_DIR)/test_task1_buffer

all: $(IMU_TARGET) \
     $(DETECTION_TARGET) \
     $(BUFFER_TARGET) \
     $(BLACK_BOX_TARGET) \
     $(TASK1_BUFFER_TARGET) \
     dev

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(IMU_TARGET): src/lsm9ds1.c \
               src/accident_detection.c \
               tests/lsm9ds1_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(DETECTION_TARGET): src/accident_detection.c \
                     tests/accident_detection_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

$(BUFFER_TARGET): tests/crash_buffer_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(BLACK_BOX_TARGET): src/main.c \
                     src/lsm9ds1.c \
                     src/accident_detection.c \
                     src/task2_camera.c \
                     src/storage.c \
                     src/camera_capture_v4l2.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS) -pthread

$(TASK1_BUFFER_TARGET): tests/task1_buffer_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

dev:
	$(MAKE) -C dev

clean:
	rm -rf $(BUILD_DIR)
	$(MAKE) -C dev clean

.PHONY: all clean dev