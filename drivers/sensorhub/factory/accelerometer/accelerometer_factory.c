/*
 *  Copyright (C) 2020, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include "../../sensor/accelerometer.h"
#include "../../sensorhub/shub_device.h"
#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"
#include "../../utility/shub_dev_core.h"
#include "../../utility/shub_utility.h"
#include "../../comm/shub_comm.h"
#include "accelerometer_factory.h"

#include <linux/delay.h>
#include <linux/slab.h>

#define MAX_ACCEL_1G 4096
#define MAX_ACCEL_2G 8192
#define MIN_ACCEL_2G -8192
#define MAX_ACCEL_4G 16384

#define CALIBRATION_DATA_AMOUNT 20
/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

static struct device *accel_sysfs_device;
static struct device_attribute **chipset_attrs;

static ssize_t accel_calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_ACCELEROMETER);
	struct accelerometer_data *data = sensor->data;

	ret = sensor->funcs->open_calibration_file();
	if (ret < 0)
		shub_errf("calibration open failed(%d)", ret);

	shub_infof("Cal data : %d %d %d - %d", data->cal_data.x, data->cal_data.y, data->cal_data.z, ret);

	return sprintf(buf, "%d %d %d %d\n", ret, data->cal_data.x, data->cal_data.y, data->cal_data.z);
}

static int accel_do_calibrate(int enable)
{
	int iSum[3] = {0, };
	int ret = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_ACCELEROMETER);
	uint32_t backup_sampling_period = sensor->sampling_period;
	uint32_t backup_max_report_latency = sensor->max_report_latency;
	struct accelerometer_data *data = sensor->data;

	if (enable) {
		int count;

		data->cal_data.x = 0;
		data->cal_data.y = 0;
		data->cal_data.z = 0;
		set_accel_cal(data);

		batch_sensor(SENSOR_TYPE_ACCELEROMETER, 10, 0);
		enable_sensor(SENSOR_TYPE_ACCELEROMETER, NULL, 0);

		msleep(300);

		for (count = 0; count < CALIBRATION_DATA_AMOUNT; count++) {
			iSum[0] += sensor->event_buffer.x;
			iSum[1] += sensor->event_buffer.y;
			iSum[2] += sensor->event_buffer.z;
			mdelay(10);
		}

		batch_sensor(SENSOR_TYPE_ACCELEROMETER, backup_sampling_period, backup_max_report_latency);
		disable_sensor(SENSOR_TYPE_ACCELEROMETER, NULL, 0);

		data->cal_data.x = (iSum[0] / CALIBRATION_DATA_AMOUNT);
		data->cal_data.y = (iSum[1] / CALIBRATION_DATA_AMOUNT);
		data->cal_data.z = (iSum[2] / CALIBRATION_DATA_AMOUNT);

		if (data->cal_data.z > 0)
			data->cal_data.z -= MAX_ACCEL_1G;
		else if (data->cal_data.z < 0)
			data->cal_data.z += MAX_ACCEL_1G;

	} else {
		data->cal_data.x = 0;
		data->cal_data.y = 0;
		data->cal_data.z = 0;
	}

	shub_infof("do accel calibrate %d, %d, %d", data->cal_data.x, data->cal_data.y, data->cal_data.z);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(ACCEL_CALIBRATION_FILE_PATH, O_CREAT | O_TRUNC | O_WRONLY, 0660);
	if (IS_ERR(cal_filp)) {
		shub_errf("Can't open calibration file");
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);
		return ret;
	}

	ret = vfs_write(cal_filp, (char *)&data->cal_data, 3 * sizeof(int), &cal_filp->f_pos);
	if (ret != 3 * sizeof(int)) {
		shub_errf("Can't write the accelcal to file");
		ret = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);
	set_accel_cal(data);
	return ret;
}

static ssize_t accel_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	int64_t enable;

	ret = kstrtoll(buf, 10, &enable);
	if (ret < 0)
		return ret;

	ret = accel_do_calibrate((int)enable);
	if (ret < 0)
		shub_errf("accel_do_calibrate() failed");

	return size;
}

static ssize_t raw_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_ACCELEROMETER);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", sensor->event_buffer.x, sensor->event_buffer.y,
			sensor->event_buffer.z);
}

static ssize_t accel_reactive_alert_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	bool success = false;
	struct accelerometer_data *data = get_sensor(SENSOR_TYPE_ACCELEROMETER)->data;

	if (data->is_accel_alert == true)
		success = true;
	else
		success = false;

	data->is_accel_alert = false;
	return sprintf(buf, "%u\n", success);
}

static ssize_t accel_reactive_alert_store(struct device *dev, struct device_attribute *attr, const char *buf,
					  size_t size)
{
	int ret = 0;
	char *buffer = NULL;
	int buffer_length = 0;
	struct accelerometer_data *data = get_sensor(SENSOR_TYPE_ACCELEROMETER)->data;

	if (sysfs_streq(buf, "1")) {
		shub_infof("on");
	} else if (sysfs_streq(buf, "0")) {
		shub_infof("off");
	} else if (sysfs_streq(buf, "2")) {
		shub_infof("factory");

		data->is_accel_alert = 0;

		ret = shub_send_command_wait(CMD_GETVALUE, SENSOR_TYPE_ACCELEROMETER, SENSOR_FACTORY, 3000, NULL, 0,
					     &buffer, &buffer_length);

		if (ret < 0) {
			shub_errf("shub_send_command_wait Fail %d", ret);
			goto exit;
		}

		if (buffer == NULL) {
			shub_errf("buffer is null");
			ret = -EINVAL;
			goto exit;
		}

		if (buffer_length < 1) {
			shub_errf("length err %d", buffer_length);
			ret = -EINVAL;
			goto exit;
		}

		data->is_accel_alert = *buffer;

		shub_infof("factory test success!");
	} else {
		shub_errf("invalid value %d", *buf);
		ret = -EINVAL;
	}

exit:
	kfree(buffer);
	return size;
}

static ssize_t accel_lowpassfilter_store(struct device *dev, struct device_attribute *attr, const char *buf,
					 size_t size)
{
	int ret = 0;
	int new_enable = 1;
	char temp = 0;

	if (sysfs_streq(buf, "1"))
		new_enable = 1;
	else if (sysfs_streq(buf, "0"))
		new_enable = 0;
	else
		shub_infof(" invalid value!");

	temp = new_enable;

	ret = shub_send_command_wait(CMD_SETVALUE, SENSOR_TYPE_ACCELEROMETER, ACCELOMETER_LPF_ON_OFF, 0, &temp,
				     sizeof(char), NULL, NULL);

	if (ret < 0)
		shub_errf("shub_send_command_wait Fail %d", ret);

	return size;
}

static DEVICE_ATTR_RO(raw_data);
static DEVICE_ATTR(calibration, 0664, accel_calibration_show, accel_calibration_store);
static DEVICE_ATTR(reactive_alert, 0664, accel_reactive_alert_show, accel_reactive_alert_store);
static DEVICE_ATTR(lowpassfilter, 0220, NULL, accel_lowpassfilter_store);

static struct device_attribute *acc_attrs[] = {
	&dev_attr_calibration,
	&dev_attr_raw_data,
	&dev_attr_reactive_alert,
	&dev_attr_lowpassfilter,
	NULL,
};

typedef struct device_attribute** (*get_chipset_dev_attrs)(char *);
get_chipset_dev_attrs get_acc_chipset_dev_attrs[] = {
	get_accelerometer_icm42605m_dev_attrs,
	get_accelerometer_lsm6dsl_dev_attrs,
	get_accelerometer_lis2dlc12_dev_attrs,
};

void initialize_accelerometer_sysfs(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_ACCELEROMETER);
	int ret, i;

	ret = sensor_device_create(&accel_sysfs_device, NULL, "accelerometer_sensor");
	if (ret < 0) {
		shub_errf("fail to creat %s sysfs device", sensor->name);
		return;
	}

	ret = add_sensor_device_attr(accel_sysfs_device, acc_attrs);
	if (ret < 0) {
		shub_errf("fail to add %s sysfs device attr", sensor->name);
		return;
	}

	for (i = 0; i < ARRAY_LEN(get_acc_chipset_dev_attrs); i++) {
		chipset_attrs = get_acc_chipset_dev_attrs[i](sensor->chipset_name);
		if (chipset_attrs) {
			ret = add_sensor_device_attr(accel_sysfs_device, chipset_attrs);
			if (ret < 0) {
				shub_errf("fail to add sysfs chipset device attr(%d)", i);
				return;
			}
			break;
		}
	}
}

void remove_accelerometer_sysfs(void)
{
	if (chipset_attrs)
		remove_sensor_device_attr(accel_sysfs_device, chipset_attrs);
	remove_sensor_device_attr(accel_sysfs_device, acc_attrs);
	sensor_device_destroy(accel_sysfs_device);
	accel_sysfs_device = NULL;
}

void initialize_accelerometer_factory(bool en)
{
	if (!get_sensor_probe_state(SENSOR_TYPE_ACCELEROMETER))
		return;

	if (en)
		initialize_accelerometer_sysfs();
	else
		remove_accelerometer_sysfs();
}
