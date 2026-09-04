CC = gcc
CFLAGS = -Wall -Iinclude -I.
BUILD_DIR = build
TARGET = $(BUILD_DIR)/test_imu

SRCS = src/lsm9ds1.c tests/lsm9ds1_test.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) -lm

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS)
	rm -rf $(BUILD_DIR)