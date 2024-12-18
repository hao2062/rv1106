/* drivers/input/sensors/access/qmi8658_acc.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: oeh<oeh@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
// 头文件
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>
#include <linux/qmi8658.h>

// 设置加速度计的输出速率
static int qmi8658_set_rate(struct i2c_client *client, int rate)
{
	const short hz[] = {448, 224, 112, 56, 28};
	const int   d[] = {0x24, 0x25,0x26, 0x27, 0x28};
	int i =0, data =0, result =0;

	/* always use poll mode, no need to set rate */
	return 0;

	if ((rate < 1) || (rate > 480))
		return -1;

	while ((rate > hz[i]) && (i < 5))
		i++;
	data = d[i];

	result = sensor_write_reg(client, QMI8658_REG_CTRL2, data);
	if (result)
		return -1;

	return 0;
}

// 启用加速度计
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
//	struct sensor_private_data *sensor =
//	    (struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	u8 databuf[2] = {0};

	databuf[0] = sensor_read_reg(client, QMI8658_REG_CTRL7);

	// 是否启用加速度计
	if (!enable) {
		pr_info("disable sensor.");
		databuf[0] = (databuf[0] & 0xFE) | 0x80; //accel disable;		
	} else {
		pr_info("enable sensor.");
		databuf[0] = (databuf[0] & 0xFE) | 0x81; //accel enable;		
		qmi8658_set_rate(client, rate);
	}

	result = sensor_write_reg(client, QMI8658_REG_CTRL7, databuf[0]);
	if (result) {
		dev_err(&client->dev, "%s:fail to set pwrm1\n", __func__);
		return -1;
	}
	dev_err(&client->dev, "zhoucs %s:activer OK, enable=%d, result=%d\n", __func__, enable, result);
	return result;
}

// 初始化加速度计
static int sensor_init(struct i2c_client *client)
{
	int res = 0;
	u8 read_data = 0,reg_value = 0;

	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);

	read_data = sensor_read_reg(client, sensor->ops->id_reg);

	if (read_data != sensor->ops->id_data) {
		dev_err(&client->dev, "%s:check id err,read_data:%d,ops->id_data:%d\n", 
				__func__, read_data, sensor->ops->id_data);
		return -1;
	}else{
		dev_err(&client->dev, "%s:check id success -->qmi8658\n", __func__);
	}

	res = sensor_write_reg(client, QMI8658_REG_RESET, QMI8658_SW_RESET);
	if (res) {
		dev_err(&client->dev, "set QMI8658_SW_RESET error,res: %d!\n", res);
		return res;
	}
	msleep(15);

	res = sensor_write_reg(client, QMI8658_REG_CTRL1, QMI8658_ADDR_AI_BE);
	if (res) {
		dev_err(&client->dev, "set QMI8658_REG_CTRL1 error,res: %d!\n", res);
		return res;
	}

	reg_value = QMI8658_ACC_RANGE_DEF|QMI8658_ODR_470HZ_REG_VALUE;// ±8g,odr=479hz
	res = sensor_write_reg(client, QMI8658_REG_CTRL2, reg_value);
	if (res) {
		dev_err(&client->dev, "set QMI8658_REG_CTRL2 error,res: %d!\n", res);
		return res;
	}

	reg_value = QMI8658_GYR_RANGE_2048DPS|QMI8658_ODR_470HZ_REG_VALUE;//2048dps,odr=479hz
	res = sensor_write_reg(client, QMI8658_REG_CTRL3, reg_value);
	if (res) {
		dev_err(&client->dev, "set QMI8658_REG_CTRL3 error,res: %d!\n", res);
		return res;
	}

	// 修改：改成默认启用传感器
	// res = sensor->ops->active(client, 0, sensor->pdata->poll_delay_ms);
	res = sensor->ops->active(client, 1, sensor->pdata->poll_delay_ms);

	if (res) {
		dev_err(&client->dev, "%s:line=%d,error\n", __func__, __LINE__);
		return res;
	}
	return res;
}

// 上报加速度计
static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);

	if (sensor->status_cur == SENSOR_ON) {
		/* Report acceleration sensor information */
		input_report_abs(sensor->input_dev, ABS_X, axis->x);
		input_report_abs(sensor->input_dev, ABS_Y, axis->y);
		input_report_abs(sensor->input_dev, ABS_Z, axis->z);
		input_sync(sensor->input_dev);
	}

	return 0;
}

// 读取并报告加速度计的值
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	short x, y, z;
	struct sensor_axis axis;
	u8 buffer[6] = {0};
	char value = 0, count = 0;

	// pr_info("Function: sensor_report_value.");

	if (sensor->ops->read_len < 6) {
		dev_err(&client->dev, "%s:lenth is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}

	memset(buffer, 0, 6);

	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	while(((value & 0x03)!= 0x03)&&(count++ < 10)){
		value = sensor_read_reg(client, QMI8658_REG_STATUSINT);
	}
	*buffer = QMI8658_REG_GYR_XOUT_LSB;
	ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
	*buffer = sensor->ops->read_reg;
	ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
	if (ret < 0)
		return ret;

	x = ((buffer[1] << 8) & 0xFF00) + (buffer[0] & 0xFF);
	y = ((buffer[3] << 8) & 0xFF00) + (buffer[2] & 0xFF);
	z = ((buffer[5] << 8) & 0xFF00) + (buffer[4] & 0xFF);
	
	// 添加日志打印读取到的原始加速度数据
	// pr_info("sensor_report_value: Raw Accel Data - X: %d, Y: %d, Z: %d\n", x, y, z);

	//dev_err(&client->dev, "raw_acc=%4d,%4d,%4d\n", (signed short)x, (signed short)y, (signed short)z);

	axis.x = (pdata->orientation[0]) * x + (pdata->orientation[1]) * y + (pdata->orientation[2]) * z;
	axis.y = (pdata->orientation[3]) * x + (pdata->orientation[4]) * y + (pdata->orientation[5]) * z;
	axis.z = (pdata->orientation[6]) * x + (pdata->orientation[7]) * y + (pdata->orientation[8]) * z;

	gsensor_report_value(client, &axis);

	mutex_lock(&(sensor->data_mutex));
	sensor->axis = axis;
	mutex_unlock(&(sensor->data_mutex));

	if ((sensor->pdata->irq_enable) && (sensor->ops->int_status_reg >= 0))
		value = sensor_read_reg(client, sensor->ops->int_status_reg);

	return ret;
}

// 传感器的操作结构体
struct sensor_operate gsensor_qmi8658_ops = {
	.name				= "qmi8658_acc",
	.type				= SENSOR_TYPE_ACCEL,
	.id_i2c				= ACCEL_ID_QMI8658,
	.read_reg			= QMI8658_REG_ACC_XOUT_LSB,
	.read_len			= 6,
	.id_reg				= QMI8658_REG_WHO_AM_I,
	.id_data 			= SENSOR_CHIP_ID_QMI8658,
	.precision			= QMI8658_PRECISION,
	.ctrl_reg 			= QMI8658_REG_CTRL7,
	.int_status_reg 	= QMI8658_REG_STATUS0,
	.range				= {-32768, 32768},
	.trig				= IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
	.active				= sensor_active,
	.init				= sensor_init,
	.report 			= sensor_report_value,
};

/****************operate according to sensor chip:end************/
static int gsensor_qmi8658_probe(struct i2c_client *client,
				 const struct i2c_device_id *devid)
{
	pr_info("Function: gsensor_qmi8658_probe.");
	return sensor_register_device(client, NULL, devid, &gsensor_qmi8658_ops);
}

static int gsensor_qmi8658_remove(struct i2c_client *client)
{
	return sensor_unregister_device(client, NULL, &gsensor_qmi8658_ops);
}

static const struct i2c_device_id gsensor_qmi8658_id[] = {
	{"qmi8658_acc", ACCEL_ID_QMI8658},
	{}
};

static struct i2c_driver gsensor_qmi8658_driver = {
	.probe = gsensor_qmi8658_probe,
	.remove = gsensor_qmi8658_remove,
	.shutdown = sensor_shutdown,
	.id_table = gsensor_qmi8658_id,
	.driver = {
		.name = "gsensor_qmi8658",
#ifdef CONFIG_PM
		.pm = &sensor_pm_ops,
#endif
	},
};

module_i2c_driver(gsensor_qmi8658_driver);

MODULE_AUTHOR("ouenhui <oeh@rock-chips.com");
MODULE_DESCRIPTION("qmi8658_acc 3-Axis accelerometer driver");
MODULE_LICENSE("GPL");
