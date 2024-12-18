#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <string.h>
#include <time.h>

#define GSENSOR_IOCTL_MAGIC       'a'
#define GSENSOR_IOCTL_START       _IO(GSENSOR_IOCTL_MAGIC, 0x03)

#define EVENT_DEVICE "/dev/input/event0"
#define SENSOR_DEVICE "/dev/mma8452_daemon"

#define DELAY_MS 500 // 延迟时间，单位毫秒

void msleep(int milliseconds) {
    struct timespec req, rem;
    req.tv_sec = milliseconds / 1000;
    req.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&req, &rem);
}

int start_sensor() {
    int fd = open(SENSOR_DEVICE, O_RDWR);
    if (fd < 0) {
        perror("Failed to open sensor device");
        return -1;
    }

    if (ioctl(fd, GSENSOR_IOCTL_START) < 0) {
        perror("Failed to start sensor");
        close(fd);
        return -1;
    }

    printf("Sensor started successfully\n");
    close(fd);
    return 0;
}

int main() {
    int fd_event;
    struct input_event ev;

    int x = 0, y = 0, z = 0;       // 存储 X、Y、Z 数据
    int got_x = 0, got_y = 0, got_z = 0; // 标志位，记录是否已读取

    // 1. 启动传感器
    if (start_sensor() < 0) {
        return -1;
    }

    // 2. 打开 event0 设备文件
    fd_event = open(EVENT_DEVICE, O_RDONLY);
    if (fd_event < 0) {
        perror("Failed to open event device");
        return -1;
    }

    printf("Reading accelerometer data from %s...\n", EVENT_DEVICE);

    // 3. 循环读取数据
    while (1) {
        ssize_t bytes = read(fd_event, &ev, sizeof(struct input_event));
        if (bytes < 0) {
            perror("Failed to read event");
            close(fd_event);
            return -1;
        }

        // 检查事件类型和代码
        if (ev.type == EV_ABS) { // 绝对坐标事件
            switch (ev.code) {
                case ABS_X:
                    x = ev.value;
                    got_x = 1;
                    break;
                case ABS_Y:
                    y = ev.value;
                    got_y = 1;
                    break;
                case ABS_Z:
                    z = ev.value;
                    got_z = 1;
                    break;
                default:
                    break;
            }
        }

        // 当 X、Y、Z 数据都获取到时一起打印
        if (got_x && got_y && got_z) {
            printf("Accelerometer Data - X: %d, Y: %d, Z: %d\n", x, y, z);
            
            // 重置标志位，准备下一次读取
            got_x = got_y = got_z = 0;

            // 延迟，避免过快打印
            msleep(DELAY_MS);
        }
    }

    close(fd_event);
    return 0;
}
