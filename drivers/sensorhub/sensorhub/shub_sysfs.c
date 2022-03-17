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
#include "../comm/shub_comm.h"
#include "../sensorhub/shub_device.h"
#include "../sensormanager/shub_sensor_type.h"
#include "../utility/shub_dev_core.h"
#include "../utility/shub_utility.h"
#include "../utility/shub_wait_event.h"
#include "../vendor/shub_vendor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "shub_device.h"

#include <linux/slab.h>
#ifdef CONFIG_SHUB_FIRMWARE_DOWNLOAD
#include "shub_firmware.h"
#endif

#define FACTORY_DATA_MAX 100
static char buffer[FACTORY_DATA_MAX];

ssize_t mcu_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct shub_data_t *data = get_shub_data();
	bool is_success = false;
	int ret = 0;
	int prev_reset_cnt;

	prev_reset_cnt = data->cnt_reset;

	reset_mcu(RESET_TYPE_KERNEL_SYSFS);

	ret = shub_wait_event_timeout(&data->reset_lock, 2000);

	shub_infof("");
	if (!ret && is_shub_working() && prev_reset_cnt != data->cnt_reset) {
		is_success = true;
	}

	return sprintf(buf, "%s\n", (is_success ? "OK" : "NG"));
}

static ssize_t show_reset_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	if (get_reset_type() == RESET_TYPE_KERNEL_NO_EVENT) {
		ret = sprintf(buf, "No Event\n");
	} else if (get_reset_type() == RESET_TYPE_KERNEL_COM_FAIL) {
		ret = sprintf(buf, "Com Fail\n");
	} else if (get_reset_type() == RESET_TYPE_HUB_CRASHED) {
		ret = sprintf(buf, "CHUB RESET\n");
	}
	init_reset_type();
	return ret;
}

static ssize_t fs_ready_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	shub_infof("");
	fs_ready_cb();

	return size;
}

ssize_t mcu_revision_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_SHUB_FIRMWARE_DOWNLOAD
	return sprintf(buf, "%s01%u,%s01%u\n", SENSORHUB_VENDOR, get_firmware_rev(), SENSORHUB_VENDOR,
		       get_kernel_fw_rev());
#else
	return sprintf(buf, "N,N\n");
#endif
}

ssize_t mcu_model_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", SENSORHUB_NAME);
}

ssize_t mcu_factorytest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *temp_buffer = NULL;
	int temp_buffer_length = 0;

	if (sysfs_streq(buf, "1")) {

		ret = shub_send_command_wait(CMD_GETVALUE, TYPE_MCU, SENSOR_FACTORY, 1000, NULL, 0, &temp_buffer,
					     &temp_buffer_length);
		if (ret < 0) {
			shub_errf("fail %d", ret);
			if (temp_buffer != NULL)
				kfree(temp_buffer);
			return ret;
		}
	} else {
		shub_errf("invalid value %d", *buf);
		return -EINVAL;
	}

	shub_infof("MCU Factory Test - %d, length = %d", ret, temp_buffer_length);
	memcpy(buffer, temp_buffer, temp_buffer_length);

	if (temp_buffer != NULL) {
		kfree(temp_buffer);
	}

	return size;
}

ssize_t mcu_factorytest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	bool bMcuTestSuccessed = false;

	shub_info("MCU Factory Test Data : %u, %u, %u, %u, %u", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

	/* system clock, RTC, I2C Master, I2C Slave, externel pin */
	if ((buffer[0] == 1) && (buffer[1] == 1) && (buffer[2] == 1) && (buffer[3] == 1) && (buffer[4] == 1)) {
		bMcuTestSuccessed = true;
	}

	shub_infof("MCU Factory Test Result - %s, %s, %s\n", SENSORHUB_NAME, (bMcuTestSuccessed ? "OK" : "NG"), "OK");

	return sprintf(buf, "%s,%s,%s\n", SENSORHUB_NAME, (bMcuTestSuccessed ? "OK" : "NG"), "OK");
}

#ifdef YUMNOTYET
ssize_t mcu_sleep_factorytest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct shub_data_t *data = dev_get_drvdata(dev);
	int ret = 0;
	char *temp_buffer = NULL;
	int temp_buffer_length = 0;

	if (sysfs_streq(buf, "1")) {
		ret = shub_send_command_wait(CMD_GETVALUE, TYPE_MCU, MCU_SLEEP_TEST, 1000, NULL, 0, &temp_buffer,
					     &temp_buffer_length);
		if (ret < 0) {
			shub_errf("fail %d", ret);
			if (temp_buffer != NULL)
				kfree(temp_buffer);

			return -ERROR;
		}
	} else {
		shub_errf("invalid value %d", *buf);
		return -EINVAL;
	}

	shub_infof("MCU Sleep Factory Test - %d, length = %d", 1, temp_buffer_length);
	buffer[0] = temp_buffer_length;
	buffer[1] = 0;
	memcpy(&buffer[2], temp_buffer, temp_buffer_length);

	if (temp_buffer != NULL) {
		kfree(temp_buffer);
	}

	return size;
}

ssize_t mcu_sleep_factorytest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int data_index, sensor_type = 0;
	struct shub_data_t *data = dev_get_drvdata(dev);
	struct sensor_value fsb[SENSOR_TYPE_MAX];
	u16 length = 0;

	length = (((u16)(buffer[1])) << 8) + ((u16)(buffer[0]));
	memset(fsb, 0, sizeof(struct sensor_value) * SENSOR_TYPE_MAX);

	shub_infof("length = %d", length);

	for (data_index = 2; data_index < length;) {
		sensor_type = buffer[data_index++];

		if ((sensor_type < 0) || (sensor_type > (SENSOR_TYPE_MAX - 1))) {
			shub_errf("type %d, data frame error", sensor_type);
			goto exit;
		}

		// get_sensordata((char *)buffer, &data_index, sensor_type, &(fsb[sensor_type]));
		// get_timestamp((char *)buffer, &data_index, &(fsb[sensor_type]), sensor_type);
	}

	fsb[SENSOR_TYPE_PRESSURE].pressure -= data->buf[SENSOR_TYPE_PRESSURE].pressure_cal;

exit:
	shub_infof("Result\n"
		   "[shub] accel %d,%d,%d\n"
		   "[shub] gyro %d,%d,%d\n"
		   "[shub] mag %d,%d,%d\n"
		   "[shub] baro %d,%d\n"
		   "[shub] prox %u,%u\n"
		   "[shub] light %u,%u,%u,%u,%u,%u"
		   "[shub]: temp %d,%d,%d\n",
		   fsb[SENSOR_TYPE_ACCELEROMETER].x, fsb[SENSOR_TYPE_ACCELEROMETER].y, fsb[SENSOR_TYPE_ACCELEROMETER].z,
		   fsb[SENSOR_TYPE_GYROSCOPE].x, fsb[SENSOR_TYPE_GYROSCOPE].y, fsb[SENSOR_TYPE_GYROSCOPE].z,
		   fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_x, fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_y,
		   fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_z, fsb[SENSOR_TYPE_PRESSURE].pressure,
		   fsb[SENSOR_TYPE_PRESSURE].temperature, fsb[SENSOR_TYPE_PROXIMITY].prox,
		   fsb[SENSOR_TYPE_PROXIMITY].prox_ex, fsb[SENSOR_TYPE_LIGHT].r, fsb[SENSOR_TYPE_LIGHT].g,
		   fsb[SENSOR_TYPE_LIGHT].b, fsb[SENSOR_TYPE_LIGHT].w, fsb[SENSOR_TYPE_LIGHT].a_time,
		   fsb[SENSOR_TYPE_LIGHT].a_gain, fsb[SENSOR_TYPE_TEMPERATURE].x, fsb[SENSOR_TYPE_TEMPERATURE].y,
		   fsb[SENSOR_TYPE_TEMPERATURE].z);

	return sprintf(buf,
		       "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,"
		       "%u,%u,%u,%u,%u,%u,0,0,0,0,%d,%d\n",
		       fsb[SENSOR_TYPE_ACCELEROMETER].x, fsb[SENSOR_TYPE_ACCELEROMETER].y,
		       fsb[SENSOR_TYPE_ACCELEROMETER].z, fsb[SENSOR_TYPE_GYROSCOPE].x, fsb[SENSOR_TYPE_GYROSCOPE].y,
		       fsb[SENSOR_TYPE_GYROSCOPE].z, fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_x,
		       fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_y, fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_z,
		       fsb[SENSOR_TYPE_PRESSURE].pressure, fsb[SENSOR_TYPE_PRESSURE].temperature,
		       fsb[SENSOR_TYPE_PROXIMITY].prox_ex, fsb[SENSOR_TYPE_LIGHT].r, fsb[SENSOR_TYPE_LIGHT].g,
		       fsb[SENSOR_TYPE_LIGHT].b, fsb[SENSOR_TYPE_LIGHT].w, fsb[SENSOR_TYPE_LIGHT].a_time,
		       fsb[SENSOR_TYPE_LIGHT].a_gain, fsb[SENSOR_TYPE_TEMPERATURE].x, fsb[SENSOR_TYPE_TEMPERATURE].y);
}
#endif

static DEVICE_ATTR(mcu_rev, S_IRUGO, mcu_revision_show, NULL);
static DEVICE_ATTR(mcu_name, S_IRUGO, mcu_model_name_show, NULL);
static DEVICE_ATTR(mcu_reset, S_IRUGO, mcu_reset_show, NULL);
static DEVICE_ATTR(reset_info, S_IRUGO, show_reset_info, NULL);
static DEVICE_ATTR(fs_ready, 0220, NULL, fs_ready_store);
static DEVICE_ATTR(mcu_test, S_IRUGO | S_IWUSR | S_IWGRP, mcu_factorytest_show, mcu_factorytest_store);
#ifdef YUMNOTYET
static DEVICE_ATTR(mcu_sleep_test, S_IRUGO | S_IWUSR | S_IWGRP, mcu_sleep_factorytest_show,
		   mcu_sleep_factorytest_store);
#endif

static struct device_attribute *shub_attrs[] = {
    &dev_attr_mcu_rev,
    &dev_attr_mcu_name,
    &dev_attr_mcu_reset,
    &dev_attr_reset_info,
    &dev_attr_fs_ready,
    &dev_attr_mcu_test,
#ifdef YUMNOTYET
    &dev_attr_mcu_sleep_test,
#endif
    NULL,
};

int init_shub_sysfs(struct device *shub_dev)
{
	struct shub_data_t *data = get_shub_data();
	int ret;

	ret = sensor_device_create(&data->sysfs_dev, data, "ssp_sensor");
	if (ret < 0) {
		shub_errf("fail to creat ssp_sensor device");
		return ret;
	}

	ret = add_sensor_device_attr(data->sysfs_dev, shub_attrs);
	if (ret < 0) {
		shub_errf("fail to add shub device attr");
	}

	return ret;
}

void remove_shub_sysfs(struct device *shub_dev)
{
	struct shub_data_t *data = get_shub_data();

	remove_sensor_device_attr(data->sysfs_dev, shub_attrs);
	sensor_device_destroy(data->sysfs_dev);
}
