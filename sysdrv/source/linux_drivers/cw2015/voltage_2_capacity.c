#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// ADC 原始值和分辨率文件路径
#define ADC_RAW_PATH "/sys/bus/iio/devices/iio:device0/in_voltage1_raw"
#define ADC_SCALE_PATH "/sys/bus/iio/devices/iio:device0/in_voltage_scale"

// 电压和电量映射表路径
#define MAPPING_TABLE_PATH "output_table.txt"

// 映射表结构体
typedef struct {
    float voltage;  // 电压 (mV)
    int capacity;   // 电量 (%)
} Mapping;

// 获取ADC电压
float get_adc_voltage() {
    char buffer[32];
    float raw, scale;

    // 读取RAW值
    FILE *raw_file = fopen(ADC_RAW_PATH, "r");
    if (!raw_file) {
        perror("Failed to open ADC_RAW_PATH");
        return -1;
    }
    if (fgets(buffer, sizeof(buffer), raw_file) == NULL) {
        perror("Failed to read ADC_RAW_PATH");
        fclose(raw_file);
        return -1;
    }
    fclose(raw_file);
    raw = atof(buffer);

    // 读取SCALE值
    FILE *scale_file = fopen(ADC_SCALE_PATH, "r");
    if (!scale_file) {
        perror("Failed to open ADC_SCALE_PATH");
        return -1;
    }
    if (fgets(buffer, sizeof(buffer), scale_file) == NULL) {
        perror("Failed to read ADC_SCALE_PATH");
        fclose(scale_file);
        return -1;
    }
    fclose(scale_file);
    scale = atof(buffer);

    return raw * scale; // 计算实际电压
}

// 读取映射表
int load_mapping_table(const char *path, Mapping **table, int *size) {
    FILE *file = fopen(path, "r");
    if (!file) {
        perror("Failed to open mapping table file");
        return -1;
    }

    char line[128];
    int capacity;
    float voltage;
    int count = 0;
    Mapping *temp_table = NULL;

    // 动态分配内存并读取数据
    while (fgets(line, sizeof(line), file)) {
        if (sscanf(line, "%f %d", &voltage, &capacity) == 2) {
            temp_table = realloc(temp_table, (count + 1) * sizeof(Mapping));
            temp_table[count].voltage = voltage;
            temp_table[count].capacity = capacity;
            count++;
        }
    }
    fclose(file);

    *table = temp_table;
    *size = count;

    return 0;
}

// 根据电压查找对应的电量（线性插值）
int find_capacity(float voltage, Mapping *table, int size) {
    if (size == 0 || !table) {
        return -1; // 映射表为空
    }

    // 如果电压超出范围，返回最小或最大电量
    if (voltage <= table[size - 1].voltage) {
        return table[size - 1].capacity;
    }
    if (voltage >= table[0].voltage) {
        return table[0].capacity;
    }

    // 查找电压范围
    for (int i = 0; i < size - 1; i++) {
        if (voltage <= table[i].voltage && voltage > table[i + 1].voltage) {
            // 线性插值计算电量
            float v1 = table[i].voltage;
            float v2 = table[i + 1].voltage;
            int c1 = table[i].capacity;
            int c2 = table[i + 1].capacity;

            return c1 + (voltage - v1) * (c2 - c1) / (v2 - v1);
        }
    }
    return -1; // 未找到合适范围
}

int main() {
    Mapping *table = NULL;
    int size = 0;

    // 加载映射表
    if (load_mapping_table(MAPPING_TABLE_PATH, &table, &size) < 0) {
        fprintf(stderr, "Failed to load mapping table.\n");
        return 1;
    }

    printf("Mapping table loaded successfully (%d entries).\n", size);

    // 持续读取 ADC 电压并计算对应电量
    while (1) {
        float voltage = get_adc_voltage()*3;
        if (voltage < 0) {
            fprintf(stderr, "Failed to read ADC voltage.\n");
            break;
        }

        int capacity = find_capacity(voltage, table, size);
        if (capacity < 0) {
            fprintf(stderr, "Failed to find capacity for voltage: %.2f mV\n", voltage);
        } else {
            printf("Voltage: %.2f mV -> Capacity: %d%%\n", voltage, capacity);
        }

        usleep(500000); // 延迟 500 毫秒
    }

    // 释放映射表内存
    free(table);
    return 0;
}
