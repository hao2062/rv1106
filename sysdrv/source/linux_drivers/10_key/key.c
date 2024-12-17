#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#define DEVICE_NAME "gpio_test_driver"

static int major;
static int gpio_pin;
static int irq_number;
static int gpio_value = 0;
static wait_queue_head_t wait_queue;
static int event_flag = 0;

struct gpio_test_data {
    int gpio_pin;
    struct device *dev;
};

static irqreturn_t gpio_irq_handler(int irq, void *dev_id) {
    gpio_value = gpio_get_value(gpio_pin);
    event_flag = 1; // 标记事件发生
    wake_up_interruptible(&wait_queue); // 唤醒阻塞的 poll
    return IRQ_HANDLED;
}

static ssize_t gpio_read(struct file *file, char __user *buf, size_t count, loff_t *ppos) {
    char value_str[2];
    int len;

    gpio_value = gpio_get_value(gpio_pin); // 读取当前 GPIO 电平
    value_str[0] = gpio_value ? '1' : '0';
    value_str[1] = '\n';
    len = min(count, (size_t)2);

    if (copy_to_user(buf, value_str, len)) {
        return -EFAULT;
    }

    return len;
}

static ssize_t gpio_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos) {
    char kbuf[2];

    if (count > 2) {
        return -EINVAL;
    }

    if (copy_from_user(kbuf, buf, count)) {
        return -EFAULT;
    }

    if (kbuf[0] == '1') {
        gpio_set_value(gpio_pin, 1);
    } else if (kbuf[0] == '0') {
        gpio_set_value(gpio_pin, 0);
    } else {
        return -EINVAL;
    }

    return count;
}

static unsigned int gpio_poll(struct file *file, poll_table *wait) {
    unsigned int mask = 0;

    poll_wait(file, &wait_queue, wait);

    if (event_flag) {
        mask |= POLLIN | POLLRDNORM; // 数据可读
        event_flag = 0; // 重置事件标志
    }

    return mask;
}

static int gpio_open(struct inode *inode, struct file *file) {
    return 0;
}

static int gpio_release(struct inode *inode, struct file *file) {
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = gpio_read,
    .write = gpio_write,
    .poll = gpio_poll,
    .open = gpio_open,
    .release = gpio_release,
};

static int gpio_test_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    struct gpio_test_data *data;
    int ret;

	pr_info("Function: gpio_test_probe.");

    // 分配并初始化驱动私有数据
    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        dev_err(dev, "Failed to allocate memory for driver data\n");
        return -ENOMEM;
    }

    dev_set_drvdata(dev, data);

    // 从设备树中获取 GPIO 引脚
    data->gpio_pin = of_get_named_gpio(dev->of_node, "gpio", 0);
    if (!gpio_is_valid(data->gpio_pin)) {
        dev_err(dev, "Invalid GPIO pin\n");
        return -ENODEV;
    }

    gpio_pin = data->gpio_pin;

    // 请求 GPIO 并配置为输入
    ret = devm_gpio_request_one(dev, gpio_pin, GPIOF_IN, "gpio_test");
    if (ret) {
        dev_err(dev, "Failed to request GPIO\n");
        return ret;
    }

    // 注册 IRQ
    irq_number = gpio_to_irq(gpio_pin);
    if (irq_number < 0) {
        dev_err(dev, "Failed to get IRQ number\n");
        return irq_number;
    }

    ret = devm_request_irq(dev, irq_number, gpio_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "gpio_test", data);
    if (ret) {
        dev_err(dev, "Failed to request IRQ\n");
        return ret;
    }

    // 初始化字符设备
    major = register_chrdev(0, DEVICE_NAME, &fops);
    if (major < 0) {
        dev_err(dev, "Failed to register character device\n");
        return major;
    }

    init_waitqueue_head(&wait_queue);

    dev_info(dev, "GPIO test driver initialized, major number: %d\n", major);
    return 0;
}

static int gpio_test_remove(struct platform_device *pdev) {
    unregister_chrdev(major, DEVICE_NAME);
    dev_info(&pdev->dev, "GPIO test driver removed\n");
    return 0;
}

static const struct of_device_id gpio_test_of_match[] = {
    { .compatible = "mycompany,gpio-test" },
    {},
};
MODULE_DEVICE_TABLE(of, gpio_test_of_match);

static struct platform_driver gpio_test_driver = {
    .driver = {
        .name = "gpio_test",
        .of_match_table = gpio_test_of_match,
    },
    .probe = gpio_test_probe,
    .remove = gpio_test_remove,
};

module_platform_driver(gpio_test_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("GPIO Test Driver with Device Tree Support");
MODULE_VERSION("1.0");
