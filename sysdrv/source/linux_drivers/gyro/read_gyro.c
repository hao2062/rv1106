#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#define L3G4200D_IOCTL_BASE 77
#define L3G4200D_IOCTL_SET_ENABLE _IOW(L3G4200D_IOCTL_BASE, 2, int)

#define EVENT_DEVICE "/dev/input/event1"
#define SENSOR_DEVICE "/dev/gyrosensor"

#define DELAY_MS 500 // 延迟时间，单位毫秒

volatile sig_atomic_t stop_flag = 0; // 用于捕获退出信号

void msleep(int milliseconds) {
    struct timespec req, rem;
    req.tv_sec = milliseconds / 1000;
    req.tv_nsec = (milliseconds % 1000) * 1000000L;
    nanosleep(&req, &rem);
}

// 捕获 Ctrl+C 信号，安全退出
void handle_sigint(int sig) {
    printf("\nExiting program safely...\n");
    stop_flag = 1;
}

// 启动或停止传感器
int start_sensor(int enable) {
    int fd = open(SENSOR_DEVICE, O_RDWR);
    if (fd < 0) {
        perror("Failed to open sensor device");
        return -1;
    }

    if (ioctl(fd, L3G4200D_IOCTL_SET_ENABLE, &enable) < 0) {
        perror("Failed to set sensor status");
        close(fd);
        return -1;
    }

    if (enable)
        printf("Sensor started successfully\n");
    else
        printf("Sensor stopped successfully\n");

    close(fd);
    return 0;
}

int main() {
    int fd_event;
    struct input_event ev;

    int gyro_x = 0, gyro_y = 0, gyro_z = 0; // 用于存储角速度数据
    int got_gyro = 0;                       // 标志位，是否读取到了完整的数据

    // 1. 捕获 SIGINT 信号（Ctrl+C）
    signal(SIGINT, handle_sigint);

    // 2. 启动传感器
    if (start_sensor(1) < 0) {
        return -1;
    }

    // 3. 打开事件设备文件
    fd_event = open(EVENT_DEVICE, O_RDONLY);
    if (fd_event < 0) {
        perror("Failed to open event device");
        start_sensor(0); // 关闭传感器
        return -1;
    }

    printf("Reading gyroscope data (angular velocity) from %s...\n", EVENT_DEVICE);

    // 4. 循环读取事件数据
    while (!stop_flag) {
        ssize_t bytes = read(fd_event, &ev, sizeof(struct input_event));
        if (bytes < sizeof(struct input_event)) {
            perror("Failed to read input event");
            break;
        }

        // 检查事件类型
        if (ev.type == EV_REL) { // 相对旋转事件（角速度）
            switch (ev.code) {
                case REL_RX:
                    gyro_x = ev.value; // 获取 X 轴角速度
                    got_gyro = 1;
                    break;
                case REL_RY:
                    gyro_y = ev.value; // 获取 Y 轴角速度
                    got_gyro = 1;
                    break;
                case REL_RZ:
                    gyro_z = ev.value; // 获取 Z 轴角速度
                    got_gyro = 1;
                    break;
                default:
                    break;
            }
        }

        // 当读取到完整的角速度数据时打印
        if (got_gyro) {
            printf("Angular Velocity (°/s) - X: %d, Y: %d, Z: %d\n", gyro_x, gyro_y, gyro_z);
            got_gyro = 0;
            msleep(DELAY_MS); // 延迟一段时间，避免频繁输出
        }
    }

    // 5. 退出时关闭传感器并清理资源
    printf("Stopping sensor...\n");
    start_sensor(0); // 停止传感器
    close(fd_event);

    return 0;
}
