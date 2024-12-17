#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#define DEVICE "/dev/gpio_test_driver4"

int test_read_function(int fd) {
    char buf[2]; // 1 byte for state + 1 byte for string terminator
    ssize_t bytesRead;

    // Loop to read GPIO state
    for (int i = 0; i < 5; i++) { // Example: read 5 times
        bytesRead = read(fd, buf, sizeof(buf) - 1);
        if (bytesRead < 0) {
            perror("Failed to read GPIO state");
            return -1; // Return -1 on read failure
        }
        
        if (bytesRead == 0) {
            printf("No data read from GPIO\n");
        } else {
            buf[bytesRead] = '\0'; // Add string terminator
            printf("GPIO state: %c\n", buf[0]); // Print only the state character
        }
        sleep(1); // Pause for 1 second before next read
    }
    return 0;
}

int test_poll_function(int fd) {
    // printf("Start Waiting...\n");

    struct pollfd fds;
    fds.fd = fd;
    fds.events = POLLPRI | POLLERR; // 监听上升沿和错误事件

    while (1) {
        int ret = poll(&fds, 1, -1); // 阻塞等待
        if (ret > 0) {
            if (fds.revents & POLLPRI) {
                printf("Detected rising edge event 2\n");
            }
            if (fds.revents & POLLERR) {
                printf("Detected faling edge event 2\n");
            }
        } else if (ret < 0) {
            perror("poll error");
            break;
        }
    }
    return 0;
}

int main() {
    int fd = open(DEVICE, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }

    printf("Start Test Read Function.\n");

    // 测试 read 功能
    if (test_read_function(fd) < 0) {
        close(fd);
        return -1; // 如果测试读功能失败，关闭设备并退出
    }

    // 关闭文件描述符
    close(fd);

    // 重新打开设备
    fd = open(DEVICE, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open device");
        return -1;
    }

    printf("Start Test Polling Function.\n");

    // 测试 poll 功能
    test_poll_function(fd);

    // 关闭文件描述符
    close(fd);
    return 0;
}
