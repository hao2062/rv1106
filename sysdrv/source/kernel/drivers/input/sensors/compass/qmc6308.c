/*
 * Device driver core for the QMC6308 chip for low field magnetic sensing.
 *
 * Copyright (C) 2022 GiraffAI
 *
 * Author: Amarnath Revanna <amarnath.revanna@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>

#include <linux/sensor-dev.h>

#define QST_SENSOR_INFO_SIZE	2
#define QST_SENSOR_CONF_SIZE	3
#define SENSOR_DATA_SIZE		8
#define YPR_DATA_SIZE			12
#define RWBUF_SIZE				16

#define ACC_DATA_FLAG			0
#define MAG_DATA_FLAG			1
#define ORI_DATA_FLAG			2
#define AKM_NUM_SENSORS		3

#define ACC_DATA_READY			(1 << (ACC_DATA_FLAG))
#define MAG_DATA_READY		(1 << (MAG_DATA_FLAG))
#define ORI_DATA_READY			(1 << (ORI_DATA_FLAG))


// #define QMC6308_Xdata_LSB				0x01
// #define QMC6308_Xdata_MSB				0x02
// #define QMC6308_Ydata_LSB				0x03
// #define QMC6308_Ydata_MSB				0x04
// #define QMC6308_Zdata_LSB				0x05
// #define QMC6308_Zdata_MSB				0x06

#define QMC6308_Xdata_LSB   0x00
#define QMC6308_Xdata_MSB   0x01
#define QMC6308_Ydata_LSB   0x02
#define QMC6308_Ydata_MSB   0x03
#define QMC6308_Zdata_LSB   0x04
#define QMC6308_Zdata_MSB   0x05


// #define QMC6308_REG_STATUS             0x09
#define QMC6308_REG_STATUS             0x06


// #define QMC6308_REG_CONTROL1 			0x0a
// #define QMC6308_REG_CONTROL2			0x0b

#define QMC6308_REG_CONTROL1  0x09
#define QMC6308_REG_CONTROL2  0x0A


// #define QMC6308_WHO_AM_I				0x00 //Chip ID
#define QMC6308_WHO_AM_I				0x0d //Chip ID

#define QMC6308_DATA_SIZE				6
// #define QMC6308_ID_VAL             	0x80 //Chip ID data
#define QMC6308_ID_VAL             	0xff //Chip ID data

/* Mode configuration */
#define QMC6308_MODE_SUSPEND			0x00
#define QMC6308_MODE_NORMAL			0x01
#define QMC6308_MODE_SINGLE			0x02
#define QMC6308_MODE_CONTINUOUS		0x03

// data output rates for 6308
#define QMC6308_ODR_10HZ  				(0x00 << 2)
#define QMC6308_ODR_50HZ  				(0x01 << 2)
#define QMC6308_ODR_100HZ 				(0x02 << 2)	
#define QMC6308_ODR_200HZ 				(0x03 << 2)

#define QMC6308_OSR1_8 				(0x00 << 4)
#define QMC6308_OSR1_4 				(0x01 << 4)
#define QMC6308_OSR1_2					(0x02 << 4)
#define QMC6308_OSR1_1					(0x03 << 4)

#define QMC6308_OSR2_1 				(0x00 << 6)
#define QMC6308_OSR2_2 				(0x01 << 6)
#define QMC6308_OSR2_4					(0x02 << 6)
#define QMC6308_OSR2_8					(0x03 << 6)//

#define QMC6308_SET_RESET_MODE			0x00
#define QMC6308_SET_ONLY_MODE			0x01
#define QMC6308_SET_RESET_OFF			0x03

#define QMC6308_RNG_30G				(0x00 << 2)
#define QMC6308_RNG_12G				(0x01 << 2)
#define QMC6308_RNG_8G					(0x02 << 2)
#define QMC6308_RNG_2G					(0x03 << 2)

#define QMC6308_SELF_TEST              0x01
#define QMC6308_SOFR_RST               0x01

#define COMPASS_IOCTL_MAGIC                   'c'

/* IOCTLs for AKM library */
#define ECS_IOCTL_WRITE				_IOW(COMPASS_IOCTL_MAGIC, 0x01, char*)
#define ECS_IOCTL_READ					_IOWR(COMPASS_IOCTL_MAGIC, 0x02, char*)
#define ECS_IOCTL_RESET				_IO(COMPASS_IOCTL_MAGIC, 0x03)
#define ECS_IOCTL_SET_MODE			_IOW(COMPASS_IOCTL_MAGIC, 0x04, short)
#define ECS_IOCTL_GETDATA				_IOR(COMPASS_IOCTL_MAGIC, 0x05, char[SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR				_IOW(COMPASS_IOCTL_MAGIC, 0x06, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS	_IOR(COMPASS_IOCTL_MAGIC, 0x07, int)
#define ECS_IOCTL_GET_CLOSE_STATUS	_IOR(COMPASS_IOCTL_MAGIC, 0x08, int)
#define ECS_IOCTL_GET_LAYOUT			_IOR(COMPASS_IOCTL_MAGIC, 0x09, char)
#define ECS_IOCTL_GET_ACCEL			_IOR(COMPASS_IOCTL_MAGIC, 0x0A, short[3])
#define ECS_IOCTL_GET_OUTBIT			_IOR(COMPASS_IOCTL_MAGIC, 0x0B, char)
#define ECS_IOCTL_GET_DELAY			_IOR(COMPASS_IOCTL_MAGIC, 0x30, short)
#define ECS_IOCTL_GET_PROJECT_NAME	_IOR(COMPASS_IOCTL_MAGIC, 0x0D, char[64])
#define ECS_IOCTL_GET_MATRIX			_IOR(COMPASS_IOCTL_MAGIC, 0x0E, short [4][3][3])
#define ECS_IOCTL_GET_PLATFORM_DATA	_IOR(COMPASS_IOCTL_MAGIC, 0x0E, struct akm_platform_data)
#define ECS_IOCTL_GET_INFO				_IOR(COMPASS_IOCTL_MAGIC, 0x27, unsigned char[QST_SENSOR_INFO_SIZE])
#define ECS_IOCTL_GET_CONF				_IOR(COMPASS_IOCTL_MAGIC, 0x28, unsigned char[QST_SENSOR_CONF_SIZE])

static struct i2c_client *this_client;
static struct miscdevice compass_dev_device;
//static struct sensor_private_data *g_sensor[SENSOR_NUM_TYPES];
static int g_akm_rbuf_ready = 0;
static int g_akm_rbuf[12];

static int mag_layout = 0;//set mag layout



/****************operate according to sensor chip:start************/
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor = 
		(struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	//int status = 0;
	// sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	// if(!enable) {
	// 	status = ~QMC6308_MODE_CONTINUOUS;
	// 	sensor->ops->ctrl_data &= status;
	// } else {
	// 	status = QMC6308_MODE_NORMAL;
	// 	sensor->ops->ctrl_data |= status;
	// }

	if (enable)
		sensor->ops->ctrl_data = 0xc3;
	else
		sensor->ops->ctrl_data = QMC6308_MODE_SUSPEND;

	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (result)
		dev_err(&client->dev, "%s:fail to active sensor\n", __func__);

	return result;

}

static int sensor_init(struct i2c_client *client)
{
	struct sensor_private_data *sensor = 
		(struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	u8 chip_id = 0;
	
	// pr_info("Function: sensor_init");

	this_client = client;

	result = sensor->ops->active(client, 0, 0);
	if (result) {
		dev_err(&client->dev, "%s:sensor active fail\n", __func__);
		return result;
	}
	
	// 这里可能是设置默认启动传感器的？
	// sensor->status_cur = SENSOR_OFF;
	sensor->status_cur = SENSOR_ON;
	// sensor->stop_work = 0;	
	
	chip_id = sensor_read_reg(client, QMC6308_WHO_AM_I);
	if(chip_id != QMC6308_ID_VAL) {
		printk("%s: read chip id=0x%x, it is not 0x%x---\n", __func__, chip_id, QMC6308_ID_VAL);
		return -1;
	}

	//sensor_write_reg(client, 0x29, 0x06);
	
	// sensor_write_reg(client, QMC6308_REG_CONTROL1, QMC6308_OSR2_8);
	// sensor_write_reg(client, QMC6308_REG_CONTROL2, QMC6308_RNG_8G);
	// sensor_write_reg(client, 0x0d, 0x40);
	// sensor_write_reg(client, QMC6308_REG_CONTROL2, 0x00);
	// sensor_write_reg(client, QMC6308_REG_CONTROL1, 0xc3);
	// sensor_write_reg(client, 0x0d, 0x40);
	sensor_write_reg(client, QMC6308_REG_CONTROL2, 0x40);
	sensor_write_reg(client, QMC6308_REG_CONTROL1, 0x1d);

	// pr_info("misc_register in qmc6308.c");
	result = misc_register(&compass_dev_device);
	if (result < 0) {
		pr_err("%s:fail to register misc device %s\n", __func__, compass_dev_device.name);
		result = -1;
	}

	return result;
}

#if 0
static int compass_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    	(struct sensor_private_data *) i2c_get_clientdata(client);

	
	if (!g_akm_rbuf_ready) {
		pr_info("g_akm_rbuf not ready..............\n");
		return -1;
	}
	
	if (sensor->status_cur == SENSOR_ON) {
		
		//input_report_rel(sensor->input_dev, ABS_RX, axis->x);
		//input_report_rel(sensor->input_dev, ABS_RY, axis->y);
		//input_report_rel(sensor->input_dev, ABS_RZ, axis->z);
		input_report_abs(sensor->input_dev, ABS_HAT0X, sensor->axis.x);
		input_report_abs(sensor->input_dev, ABS_HAT0Y, sensor->axis.y);
		input_report_abs(sensor->input_dev, ABS_BRAKE, sensor->axis.z);
		input_report_abs(sensor->input_dev, ABS_HAT1X, g_akm_rbuf[6]);
		//printk("%s:x=%d,y=%d,z=%d\n",__func__,sensor->axis.x, sensor->axis.y, sensor->axis.z);
		
		input_sync(sensor->input_dev);
	}
	return 0;

}
#endif

static void mag_axis_convert(int *out, const short *in, int layout)
{
    if ((!out) || (!in)) {
        return;
    }
    switch (layout) {
    case 0:
        //x'=x y'=y z'=z
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];
        break;
    case 1:
        //x'=-y y'=x z'=z
        out[0] = -in[1];
        out[1] = in[0];
        out[2] = in[2];
        break;
    case 2:
        //x'=-x y'=-y z'=z
        out[0] = -in[0];
        out[1] = -in[1];
        out[2] = in[2];
        break;
    case 3:
        //x'=y y'=-x z'=z
        out[0] = in[1];
        out[1] = -in[0];
        out[2] = in[2];
        break;
    case 4:
        //x'=-x y'=y z'=-z
        out[0] = -in[0];
        out[1] = in[1];
        out[2] = -in[2];
        break;
    case 5:
        //x'=y y'=x z'=-z
        out[0] = in[1];
        out[1] = in[0];
        out[2] = -in[2];
        break;
    case 6:
        //x'=x y'=-y z'=-z
        out[0] = in[0];
        out[1] = -in[1];
        out[2] = -in[2];
        break;
    case 7:
        //x'=-y y'=-x z'=-z
        out[0] = -in[1];
        out[1] = -in[0];
        out[2] = -in[2];
        break;
    default:
        //x'=x y'=y z'=z
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];
        break;
    }
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor = 
		(struct sensor_private_data *) i2c_get_clientdata(client);
	u8 status = 0;
	u8 buffer[6] = {0};
	int trytimes = 10;
	int result = 0;
	short mag_rawdata[3] = {0};
	int mag_out[3] = {0};
	u8 control1_val = 0;
	u8 control2_val = 0;

	// pr_info("sensor_report_value");

	// 读取并打印 CONTROL1 和 CONTROL2 寄存器的值
	control1_val = sensor_read_reg(client, QMC6308_REG_CONTROL1);
	control2_val = sensor_read_reg(client, QMC6308_REG_CONTROL2);
	// pr_info("CONTROL1 register value: 0x%x\n", control1_val);
	// pr_info("CONTROL2 register value: 0x%x\n", control2_val);

	// 检查 CONTROL1 和 CONTROL2 是否已设置为连续模式
	// CONTROL1 设置为连续模式：MODE = 01, ODR = 100Hz, RNG = 8G, OSR = 512
	if (control1_val != 0x1D) { // 对应 MODE:01, ODR:100Hz, RNG:8G, OSR:512
		// pr_info("Reconfiguring CONTROL1 register to continuous mode (0x1D)\n");
		sensor_write_reg(client, QMC6308_REG_CONTROL1, 0x1D);
	}

	// CONTROL2 设置为正常工作模式，禁用软复位等
	if (control2_val != 0x40) { // 对应 ROL_PNT=1, INT_ENB=0
		// pr_info("Reconfiguring CONTROL2 register to normal mode (0x40)\n");
		sensor_write_reg(client, QMC6308_REG_CONTROL2, 0x40);
	}

	// 再次读取以验证
	control1_val = sensor_read_reg(client, QMC6308_REG_CONTROL1);
	control2_val = sensor_read_reg(client, QMC6308_REG_CONTROL2);
	// pr_info("After reconfiguration, CONTROL1: 0x%x, CONTROL2: 0x%x\n", 
	        control1_val, control2_val);

	// 检查 STATUS 寄存器
	do {
		status = sensor_read_reg(client, QMC6308_REG_STATUS);
		// pr_info("STATUS register value: 0x%x\n", status); // 打印 STATUS 寄存器值
		if (!(status & 0x01)) { // 数据是否准备好
			msleep(5); // 等待
			continue;
		} else
			break;
	} while (trytimes--);

	if (!(status & 0x01)) {
		printk("%s: sensor rx data no ready\n", __func__);
		return -1;
	}

	// 读取传感器数据
	*buffer = sensor->ops->read_reg;
	result = sensor_rx_data(client, buffer, sensor->ops->read_len);
	if (result < 0) {
		return result;
	}
		
	mutex_lock(&sensor->data_mutex);
	memcpy(sensor->sensor_data, buffer, sensor->ops->read_len);
	mutex_unlock(&sensor->data_mutex);

	// 解析原始数据
	mag_rawdata[0] = (short)(buffer[1] * 256 + buffer[0]);
	mag_rawdata[1] = (short)(buffer[3] * 256 + buffer[2]);
	mag_rawdata[2] = (short)(buffer[5] * 256 + buffer[4]);

	// pr_info("Raw Magnetometer Data - X: %d, Y: %d, Z: %d\n", 
	        mag_rawdata[0], mag_rawdata[1], mag_rawdata[2]);

	mag_axis_convert(mag_out, mag_rawdata, mag_layout);

	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_HAT0X, mag_out[0]);
	input_report_abs(sensor->input_dev, ABS_HAT0Y, mag_out[1]);
	input_report_abs(sensor->input_dev, ABS_BRAKE, mag_out[2]);
	input_sync(sensor->input_dev);

	return result;
}



// static int sensor_report_value(struct i2c_client *client)
// {
// 	struct sensor_private_data *sensor = 
// 		(struct sensor_private_data *) i2c_get_clientdata(client);
// 	// struct sensor_platform_data *pdata = sensor->pdata;
// 	u8 status = 0;
// 	u8 buffer[6] = {0};
// 	//short x, y, z;
// 	//struct sensor_axis axis;
// 	int trytimes = 10;
// 	int result = 0;
// 	short mag_rawdata[3]={0};
// 	int mag_out[3]={0};
	

// 	#if 0
// 	mutex_lock(&sensor->data_mutex);
// 	compass_report_value(client);
// 	mutex_unlock(&sensor->data_mutex);
// 	#endif

// 	pr_info("sensor_report_value");

// 	do{
// 		status = sensor_read_reg(client, QMC6308_REG_STATUS);
// 		if(!(status & 0x01)){
// 			msleep(5);
// 			continue;
// 		}
// 		else
// 			break;
// 	}while(trytimes--);
	
// 	if(!(status & 0x01)) {
// 		printk("%s: sensor rx data no ready\n", __func__);
// 		return -1;
// 	}	
	
// 	*buffer = sensor->ops->read_reg;
// 	result = sensor_rx_data(client, buffer, sensor->ops->read_len);
// 	if (result < 0)
// 	{
// 		return result;
// 	}
		
// 	mutex_lock(&sensor->data_mutex);
// 	memcpy(sensor->sensor_data, buffer, sensor->ops->read_len);
// 	mutex_unlock(&sensor->data_mutex);
// 	//上报的mgas值
// 	// x = (short)(buffer[1] * 256 + buffer[0]);
// 	// y = (short)(buffer[3] * 256 + buffer[2]);
// 	// z = (short)(buffer[5] * 256 + buffer[4]);
// 	mag_rawdata[0] = (short)(buffer[1] * 256 + buffer[0]);
// 	mag_rawdata[1] = (short)(buffer[3] * 256 + buffer[2]);
// 	mag_rawdata[2] = (short)(buffer[5] * 256 + buffer[4]);

// 	mag_axis_convert(mag_out, mag_rawdata, mag_layout);

//     /* Report acceleration sensor information */
//     input_report_abs(sensor->input_dev, ABS_HAT0X, mag_out[0]);
//     input_report_abs(sensor->input_dev, ABS_HAT0Y, mag_out[1]);
//     input_report_abs(sensor->input_dev, ABS_BRAKE, mag_out[2]);
//     input_sync(sensor->input_dev);

// 	#if 0
// 	axis.x = mag_out[0];
// 	axis.y = mag_out[1];
// 	axis.z = mag_out[2];
// 	//printk("%s: (x, y, z) [%d %d %d]---\n", __func__, x, y, z);

// 	/* orientation 取决去 layout 的值 */
// 	// axis.x = ((pdata->orientation[0]) * x + (pdata->orientation[1]) * y + (pdata->orientation[2]) * z);
// 	// axis.y = ((pdata->orientation[3]) * x + (pdata->orientation[4]) * y + (pdata->orientation[5]) * z);
// 	// axis.z = ((pdata->orientation[6]) * x + (pdata->orientation[7]) * y + (pdata->orientation[8]) * z);
	 	
// 	mutex_lock(&(sensor->data_mutex));
// 	sensor->axis = axis;
// 	mutex_unlock(&(sensor->data_mutex));

// 	mutex_lock(&sensor->data_mutex);
// 	memcpy(sensor->sensor_data, buffer, sensor->ops->read_len);
// 	mutex_unlock(&sensor->data_mutex);
// 	#endif

// 	return result;
// }

static int compass_qmc6308_get_openstatus(void)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(this_client);

	wait_event_interruptible(sensor->flags.open_wq, (atomic_read(&sensor->flags.open_flag) != 0));

	return atomic_read(&sensor->flags.open_flag);
}

static int compass_qmc6308_get_closestatus(void)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(this_client);

	wait_event_interruptible(sensor->flags.open_wq, (atomic_read(&sensor->flags.open_flag) <= 0));

	return atomic_read(&sensor->flags.open_flag);
}
static void compass_set_YPR(int *rbuf)
{
	/* No events are reported */
	if (!rbuf[0]) {
		pr_err("%s:Don't waste a time.", __func__);
		return;
	}

	g_akm_rbuf_ready = 1;
	memcpy(g_akm_rbuf, rbuf, 12 * sizeof(int));
}


static int compass_dev_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int compass_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int compass_qmc6308_set_mode(struct i2c_client *client, char mode)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(this_client);
	int result = 0;
	u8 reg_value = 0;

	switch (mode & 0x03) {
	case QMC6308_MODE_NORMAL:
	case QMC6308_MODE_SINGLE:
	case QMC6308_MODE_CONTINUOUS:
		if (sensor->status_cur == SENSOR_OFF) {
			sensor->stop_work = 0;
			sensor->status_cur = SENSOR_ON;
			printk("compass qmc6308 start \n");
			schedule_delayed_work(&sensor->delaywork, msecs_to_jiffies(sensor->pdata->poll_delay_ms));
		}
		break;

	case QMC6308_MODE_SUSPEND:
		if (sensor->status_cur == SENSOR_ON) {
			sensor->stop_work = 1;
			cancel_delayed_work_sync(&sensor->delaywork);
			printk("compass qmc6308 stop \n");
			g_akm_rbuf_ready = 0;
			sensor->status_cur = SENSOR_OFF;
		}
 		break;
	}

	switch (mode & 0x03) {
	case QMC6308_MODE_NORMAL:
		reg_value = 0xc3;
		result = sensor_write_reg(client, sensor->ops->ctrl_reg, reg_value);
		if (result)
			pr_err("%s:i2c error,mode=%d\n", __func__, mode);
		break;
	case QMC6308_MODE_SINGLE:
		reg_value = 0xc3;
		result = sensor_write_reg(client, sensor->ops->ctrl_reg, reg_value);
		if (result)
			pr_err("%s:i2c error,mode=%d\n", __func__, mode);
		break;
	case QMC6308_MODE_CONTINUOUS:
		reg_value = 0xc3;
		result = sensor_write_reg(client, sensor->ops->ctrl_reg, reg_value);
		if (result)
			pr_err("%s:i2c error,mode=%d\n", __func__, mode);
		break;
	case QMC6308_MODE_SUSPEND:
		/* Set powerdown mode */
		result = sensor_write_reg(client, sensor->ops->ctrl_reg, QMC6308_MODE_SUSPEND);
		if (result)
			pr_err("%s:i2c error,mode=%d\n", __func__, mode);
		udelay(100);
		break;
	default:
		pr_err("%s: Unknown mode(%d)", __func__, mode);
		result = -EINVAL;
		break;
	}

	return result;
}


/* ioctl - I/O control */
static long compass_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct sensor_private_data *sensor = 
		(struct sensor_private_data *)i2c_get_clientdata(this_client);
	struct i2c_client *client = this_client;
	void __user *argp = (void __user *)arg;
	int result = 0;
	struct akm_platform_data compass;
	unsigned char sense_info[QST_SENSOR_INFO_SIZE];
	unsigned char sense_conf[QST_SENSOR_CONF_SIZE];
	/* NOTE: In this function the size of "char" should be 1-byte. */
	char compass_data[SENSOR_DATA_SIZE];	/* for GETDATA */
	char rwbuf[RWBUF_SIZE];		/* for READ/WRITE */
	char mode;		/* for SET_MODE*/
	int value[12];		/* for SET_YPR */
	int status;		/* for OPEN/CLOSE_STATUS */
	int ret = -1;		/* Return value. */
	int16_t acc_buf[3];		/* for GET_ACCEL */
	int64_t delay[AKM_NUM_SENSORS];	/* for GET_DELAY */

	char layout;		/* for GET_LAYOUT */
	char outbit;		/* for GET_OUTBIT */

	// pr_info("Function: compass_dev_ioctl.");

	switch (cmd) {
	case ECS_IOCTL_WRITE:
	case ECS_IOCTL_READ:
		
		if (!argp)
			return -EINVAL;
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_MODE:
		
			if (!argp)
			return -EINVAL;
		if (copy_from_user(&mode, argp, sizeof(mode)))
			return -EFAULT;
		break;
	case ECS_IOCTL_SET_YPR:
		
		if (!argp)
			return -EINVAL;
		if (copy_from_user(&value, argp, sizeof(value)))
			return -EFAULT;

		break;
	case ECS_IOCTL_GET_INFO:
	case ECS_IOCTL_GET_CONF:
	case ECS_IOCTL_GETDATA:
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
	case ECS_IOCTL_GET_DELAY:
	case ECS_IOCTL_GET_LAYOUT:
	case ECS_IOCTL_GET_OUTBIT:
	case ECS_IOCTL_GET_ACCEL:
		
		/* Just check buffer pointer */
		if (!argp) {
			pr_err("%s:invalid argument\n", __func__);
			return -EINVAL;
		}
		break;
	default:
		break;
	}
	
	switch (cmd) {
	case ECS_IOCTL_GET_INFO:
		
		sense_info[0] = QMC6308_WHO_AM_I;
		mutex_lock(&sensor->operation_mutex);
		ret = sensor_rx_data(client, &sense_info[0], 1);
		mutex_unlock(&sensor->operation_mutex);
		if (ret < 0) {
			pr_err("%s:fait to get sense_info\n", __func__);
			return ret;
		}
		/* Check read data */
		if (sense_info[0] != QMC6308_ID_VAL) {
			dev_err(&client->dev,
				"%s: The device is not AKM Compass.", __func__);
			return -ENXIO;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		
		/*sense_conf[0] = AK8963_FUSE_ASAX;
		mutex_lock(&sensor->operation_mutex);
		ret = sensor_rx_data(client, &sense_conf[0], AKM_SENSOR_CONF_SIZE);
		mutex_unlock(&sensor->operation_mutex);
		if (ret < 0) {
			pr_err("%s:fait to get sense_conf\n", __func__);
			return ret;
		}*/
		break;
	case ECS_IOCTL_WRITE:
		
		mutex_lock(&sensor->operation_mutex);
		if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE - 1))) {
			mutex_unlock(&sensor->operation_mutex);
			return -EINVAL;
		}
		ret = sensor_tx_data(client, &rwbuf[1], rwbuf[0]);
		if (ret < 0) {
			mutex_unlock(&sensor->operation_mutex);
			pr_err("%s:fait to tx data\n", __func__);
			return ret;
		}
		mutex_unlock(&sensor->operation_mutex);
		break;
	case ECS_IOCTL_READ:
		
		mutex_lock(&sensor->operation_mutex);
		if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE - 1))) {
			mutex_unlock(&sensor->operation_mutex);
			pr_err("%s:data is error\n", __func__);
			return -EINVAL;
		}
		ret = sensor_rx_data(client, &rwbuf[1], rwbuf[0]);
		if (ret < 0) {
			mutex_unlock(&sensor->operation_mutex);
			pr_err("%s:fait to rx data\n", __func__);
			return ret;
		}
		mutex_unlock(&sensor->operation_mutex);
		break;
	case ECS_IOCTL_SET_MODE:
		mutex_lock(&sensor->operation_mutex);
		if (sensor->ops->ctrl_data != mode) {
			ret = compass_qmc6308_set_mode(client, mode);
			if (ret < 0) {
				pr_err("%s:fait to set mode\n", __func__);
				mutex_unlock(&sensor->operation_mutex);
				return ret;
			}

			sensor->ops->ctrl_data = mode;
		}
		mutex_unlock(&sensor->operation_mutex);
		break;
	case ECS_IOCTL_GETDATA:
			mutex_lock(&sensor->data_mutex);
			memcpy(compass_data, sensor->sensor_data, SENSOR_DATA_SIZE);
			mutex_unlock(&sensor->data_mutex);
			break;
	case ECS_IOCTL_SET_YPR:
		
			mutex_lock(&sensor->data_mutex);
			compass_set_YPR(value);
			mutex_unlock(&sensor->data_mutex);
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		
		status = compass_qmc6308_get_openstatus();
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		
		status = compass_qmc6308_get_closestatus();
		break;
	case ECS_IOCTL_GET_DELAY:
		
		mutex_lock(&sensor->operation_mutex);
		delay[0] = sensor->flags.delay;
		delay[1] = sensor->flags.delay;
		delay[2] = sensor->flags.delay;
		mutex_unlock(&sensor->operation_mutex);
		break;

	case ECS_IOCTL_GET_PLATFORM_DATA:	
		ret = copy_to_user(argp, &compass, sizeof(compass));
		if (ret < 0) {
			pr_err("%s:error,ret=%d\n", __func__, ret);
			return ret;
		}
		break;
	case ECS_IOCTL_GET_LAYOUT:
		
		layout = sensor->pdata->layout;
		break;
	case ECS_IOCTL_GET_OUTBIT:
		
		outbit = 1;
		break;
	case ECS_IOCTL_RESET:
		
		/*ret = compass_akm_reset(client);
		if (ret < 0)
			return ret;*/
		break;
	case ECS_IOCTL_GET_ACCEL:
		break;

	default:
		return -ENOTTY;
	}

	switch (cmd) {
	
	case ECS_IOCTL_READ:
		
		if (copy_to_user(argp, &rwbuf, rwbuf[0] + 1))
			return -EFAULT;
		break;
	case ECS_IOCTL_GETDATA:
		
		if (copy_to_user(argp, &compass_data, sizeof(compass_data)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		
		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_DELAY:
		
		if (copy_to_user(argp, &delay, sizeof(delay)))
			return -EFAULT;
		break;
	case ECS_IOCTL_GET_LAYOUT:
		
		if (copy_to_user(argp, &layout, sizeof(layout))) {
			pr_err("%s:error:%d\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OUTBIT:
		
		if (copy_to_user(argp, &outbit, sizeof(outbit))) {
			pr_err("%s:error:%d\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_ACCEL:
		
		if (copy_to_user(argp, &acc_buf, sizeof(acc_buf))) {
			pr_err("%s:error:%d\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_INFO:
		
		if (copy_to_user(argp, &sense_info,	sizeof(sense_info))) {
			pr_err("%s:error:%d\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_CONF:
		
		if (copy_to_user(argp, &sense_conf,	sizeof(sense_conf))) {
			pr_err("%s:error:%d\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;

	default:
		break;
	}
	return result;
}


static const struct file_operations compass_dev_fops = {
	.owner = THIS_MODULE,
	.open = compass_dev_open,
	.release = compass_dev_release,
	.unlocked_ioctl = compass_dev_ioctl,
};

static struct miscdevice compass_dev_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "qmc6308_dev",
	.fops = &compass_dev_fops,
};

static struct sensor_operate compass_qmc6308_ops = {
	.name				= "qmc6308",
	.type				= SENSOR_TYPE_COMPASS,
	.id_i2c				= COMPASS_ID_QMC6308,    	
	.read_reg			= QMC6308_Xdata_LSB,		
	.read_len			= QMC6308_DATA_SIZE,   		
	.id_reg				= QMC6308_WHO_AM_I,      	
	.id_data			= QMC6308_ID_VAL,		 	
	.precision			= 16,
	.ctrl_reg			= QMC6308_REG_CONTROL1,		
	.ctrl_data			= QMC6308_MODE_CONTINUOUS,
	.int_status_reg 	= SENSOR_UNKNOW_DATA,	
	.range				= {-0x32768, 0x32767},
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
	.misc_dev			= NULL,
};

/****************operate according to sensor chip:end************/
static int compass_qmc6308_probe(struct i2c_client *client,
				 const struct i2c_device_id *devid)
{	
	// pr_info("Funtion: compass_qmc6308_probe");
	return sensor_register_device(client, NULL, devid, &compass_qmc6308_ops);
}

static int compass_qmc6308_remove(struct i2c_client *client)
{
	return sensor_unregister_device(client, NULL, &compass_qmc6308_ops);
}

static const struct i2c_device_id compass_qmc6308_id[] = {
	{"mag_qmc6308", COMPASS_ID_QMC6308},
	{}
};

static struct i2c_driver compass_qmc6308_driver = {
	.probe = compass_qmc6308_probe,
	.remove = compass_qmc6308_remove,
	.shutdown = sensor_shutdown,
	.id_table = compass_qmc6308_id,
	.driver = {
		.name = "compass_qmc6308",
	#ifdef CONFIG_PM
		.pm = &sensor_pm_ops,
	#endif
	},
};


module_i2c_driver(compass_qmc6308_driver);

MODULE_AUTHOR("Amarnath Revanna");
MODULE_DESCRIPTION("QMC6308 Core Driver");
MODULE_LICENSE("GPL");
