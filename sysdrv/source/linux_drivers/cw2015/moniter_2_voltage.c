#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>

// 定义文件路径
#define CW2015_VOLTAGE_PATH "/sys/class/power_supply/cw2015-battery/voltage_now"
#define CW2015_CAPACITY_PATH "/sys/class/power_supply/cw2015-battery/capacity"
#define ADC_RAW_PATH "/sys/bus/iio/devices/iio:device0/in_voltage1_raw"
#define ADC_SCALE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage_scale"
#define LOG_FILE_PATH "voltage_log.txt" // 日志文件路径

// 读取文件内容
int read_file(const char *path, char *buffer, size_t size) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("Error opening file");
        return -1;
    }
    ssize_t len = read(fd, buffer, size - 1);
    if (len < 0) {
        perror("Error reading file");
        close(fd);
        return -1;
    }
    buffer[len] = '\0'; // 确保字符串结束
    close(fd);
    return 0;
}

// 获取CW2015电压
float get_cw2015_voltage() {
    char buffer[32];
    if (read_file(CW2015_VOLTAGE_PATH, buffer, sizeof(buffer)) < 0) {
        return -1;
    }
    return atof(buffer) / 1000; // 转换为毫伏
}

// 获取CW2015电量
int get_cw2015_capacity() {
    char buffer[32];
    if (read_file(CW2015_CAPACITY_PATH, buffer, sizeof(buffer)) < 0) {
        return -1;
    }
    return atoi(buffer); // 转换为整数，单位是百分比
}

// 获取ADC电压
float get_adc_voltage() {
    char buffer[32];
    float raw, scale;

    // 读取RAW值
    if (read_file(ADC_RAW_PATH, buffer, sizeof(buffer)) < 0) {
        return -1;
    }
    raw = atof(buffer);

    // 读取SCALE值
    if (read_file(ADC_SCALE_PATH, buffer, sizeof(buffer)) < 0) {
        return -1;
    }
    scale = atof(buffer);

    return raw * scale; // 计算实际电压
}

// 写入日志到文件
void write_log(const char *log) {
    FILE *file = fopen(LOG_FILE_PATH, "a"); // 以追加模式打开文件
    if (file == NULL) {
        perror("Error opening log file");
        return;
    }
    fprintf(file, "%s", log);
    fclose(file);
}

int main() {
    while (1) {
        char log_buffer[256];
        time_t now = time(NULL);
        struct tm *t = localtime(&now);

        // 获取CW2015电压
        float cw2015_voltage = get_cw2015_voltage();
        if (cw2015_voltage < 0) {
            fprintf(stderr, "Failed to read CW2015 voltage\n");
            continue;
        }

        // 获取CW2015电量
        int cw2015_capacity = get_cw2015_capacity();
        if (cw2015_capacity < 0) {
            fprintf(stderr, "Failed to read CW2015 capacity\n");
            continue;
        }

        // 获取ADC电压
        float adc_voltage = get_adc_voltage() * 3; // 乘以分压系数
        if (adc_voltage < 0) {
            fprintf(stderr, "Failed to read ADC voltage\n");
            continue;
        }

        // 计算误差
        float error = adc_voltage - cw2015_voltage;

        // 格式化时间
        char time_str[64];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", t);

        // 格式化日志内容
        snprintf(log_buffer, sizeof(log_buffer),
                 "[%s] CW2015 Voltage: %.3f mV, ADC Voltage: %.3f mV, Error: %.3f mV, Capacity: %d%%\n",
                 time_str, cw2015_voltage, adc_voltage, error, cw2015_capacity);

        // 打印到终端
        printf("%s", log_buffer);

        // 写入到日志文件
        write_log(log_buffer);

        // 每秒读取一次
        sleep(1);
    }

    return 0;
}
