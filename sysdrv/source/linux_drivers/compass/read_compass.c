/***************************************************************
Author       : hao2062
Date         : 2024-12-19 11:25:15
LastEditors  : hao2062 | 894357340@qq.com
LastEditTime : 2024-12-19 16:47:43
FilePath     : /rv1106_rv1103_240529/sysdrv/source/linux_drivers/compass/read_compass.c
Description  : 磁力计测试程序
Copyright (c) 2024 by 894357340@qq.com, All Rights Reserved. 
***************************************************************/
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <string.h>
#include <time.h>

#define COMPASS_IOCTL_MAGIC       'c'
// #define ECS_IOCTL_SET_MODE        _IOW(COMPASS_IOCTL_MAGIC, 0x04, char)

#define ECS_IOCTL_APP_SET_DELAY				_IOW(COMPASS_IOCTL_MAGIC, 0x18, short)

#define EVENT_DEVICE "/dev/input/event1"  // 磁力计的输入设备
#define SENSOR_DEVICE "/dev/compass"      // 磁力计的传感器设备

// #define QMC6308_MODE_CONTINUOUS   0x03    // 设置为连续测量模式
#define DELAY_MS 500                     // 延迟时间，单位毫秒

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

    // char mode = QMC6308_MODE_CONTINUOUS; // 设置模式为连续测量

    int x = 30;
    int ret = ioctl(fd, ECS_IOCTL_APP_SET_DELAY, &x); // 调用 ioctl 并获取返回值
    if (ret < 0) {
        perror("Failed to start sensor");
        printf("ioctl returned: %d\n", ret); // 打印返回值
        close(fd);
        return -1;
    }

    printf("ioctl succeeded, returned: %d\n", ret); // 如果成功，打印返回值

    printf("Sensor started successfully in continuous mode\n");
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

    // 2. 打开磁力计的 event1 设备文件
    fd_event = open(EVENT_DEVICE, O_RDONLY);
    if (fd_event < 0) {
        perror("Failed to open event device");
        return -1;
    }

    printf("Reading magnetometer data from %s...\n", EVENT_DEVICE);

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
                case ABS_HAT0X: // 磁力计 X 轴数据
                    x = ev.value;
                    got_x = 1;
                    break;
                case ABS_HAT0Y: // 磁力计 Y 轴数据
                    y = ev.value;
                    got_y = 1;
                    break;
                case ABS_BRAKE: // 磁力计 Z 轴数据
                    z = ev.value;
                    got_z = 1;
                    break;
                default:
                    break;
            }
        }

        // 当 X、Y、Z 数据都获取到时一起打印
        if (got_x && got_y && got_z) {
            printf("Magnetometer Data - X: %d, Y: %d, Z: %d\n", x, y, z);
            
            // 重置标志位，准备下一次读取
            got_x = got_y = got_z = 0;

            // 延迟，避免过快打印
            msleep(DELAY_MS);
        }
    }

    close(fd_event);
    return 0;
}
