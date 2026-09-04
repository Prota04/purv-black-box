#include "uapi.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define DEVICE_NAME "crash_buffer"
#define CLASS_NAME "crash_class"

#define CRASH_BUFFER_PATH "/dev/crash_buffer"
#define BUFFER_SIZE       2500 

MODULE_LICENSE("GPL");
MODULE_AUTHOR("purv");
MODULE_DESCRIPTION("Ring Buffer Kernel Module for Crash Data Logging");
MODULE_VERSION("1.0");

static int major_number;
static lsm9ds1_raw_sample_t *ring_buffer = NULL;
static size_t head = 0;
static size_t tail = 0;
static size_t count = 0;

static struct class *crash_class = NULL;
static struct device *crash_device = NULL;

static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static struct file_operations fops = {
    .open = dev_open,
    .read = dev_read,
    .write = dev_write,
    .release = dev_release,
};

static int __init crash_buffer_init(void)
{
    printk(KERN_INFO "CrashBuffer: Initializing driver\n");

    /* Allocate memory for the ring buffer in kernel space */
    ring_buffer = kmalloc_array(BUFFER_SIZE, sizeof(lsm9ds1_raw_sample_t), GFP_KERNEL);
    if (!ring_buffer) {
        printk(KERN_ERR "CrashBuffer: Failed to allocate memory for ring buffer\n");
        return -ENOMEM;
    }

    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ALERT "CrashBuffer: Failed to register major number\n");
        kfree(ring_buffer);
        return major_number;
    }

    crash_class = class_create(CLASS_NAME);
    if (IS_ERR(crash_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        kfree(ring_buffer);
        return PTR_ERR(crash_class);
    }

    crash_device = device_create(crash_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(crash_device)) {
        class_destroy(crash_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        kfree(ring_buffer);
        return PTR_ERR(crash_device);
    }

    printk(KERN_INFO "CrashBuffer: Device /dev/%s created successfully\n", DEVICE_NAME);
    return 0;
}

static void __exit crash_buffer_exit(void)
{
    device_destroy(crash_class, MKDEV(major_number, 0));
    class_unregister(crash_class);
    class_destroy(crash_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    kfree(ring_buffer);
    printk(KERN_INFO "CrashBuffer: Module removed\n");
}

static int dev_open(struct inode *inodep, struct file *filep)
{
    return 0;
}

static int dev_release(struct inode *inodep, struct file *filep)
{
    return 0;
}

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
    lsm9ds1_raw_sample_t dummy_sample;

    if (len != sizeof(lsm9ds1_raw_sample_t)) {
        return -EINVAL;
    }

    if (copy_from_user(&dummy_sample, buffer, sizeof(lsm9ds1_raw_sample_t))) {
        return -EFAULT;
    }

    /* Write data into ring buffer and overwrite oldest sample if buffer is full */
    ring_buffer[head] = dummy_sample;
    head = (head + 1) % BUFFER_SIZE;

    if (count < BUFFER_SIZE) {
        count++;
    } else {
        tail = (tail + 1) % BUFFER_SIZE;
    }

    return sizeof(lsm9ds1_raw_sample_t);
}

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
    size_t bytes_to_read;
    size_t i;
    lsm9ds1_raw_sample_t *temp_buf;

    if (count == 0) {
        return 0;
    }

    bytes_to_read = count * sizeof(lsm9ds1_raw_sample_t);
    if (len < bytes_to_read) {
        bytes_to_read = (len / sizeof(lsm9ds1_raw_sample_t)) * sizeof(lsm9ds1_raw_sample_t);
    }

    temp_buf = kmalloc(bytes_to_read, GFP_KERNEL);
    if (!temp_buf) {
        return -ENOMEM;
    }

    /* Order samples chronologically from oldest to newest */
    for (i = 0; i < (bytes_to_read / sizeof(lsm9ds1_raw_sample_t)); i++) {
        size_t idx = (tail + i) % BUFFER_SIZE;
        temp_buf[i] = ring_buffer[idx];
    }

    if (copy_to_user(buffer, temp_buf, bytes_to_read)) {
        kfree(temp_buf);
        return -EFAULT;
    }

    kfree(temp_buf);
    return bytes_to_read;
}

module_init(crash_buffer_init);
module_exit(crash_buffer_exit);