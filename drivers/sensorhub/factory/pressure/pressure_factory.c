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

#include "../../comm/shub_comm.h"
#include "../../sensorhub/shub_device.h"
#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"
#include "../../utility/shub_dev_core.h"
#include "../../utility/shub_utility.h"
#include "pressure_factory.h"

#include <linux/slab.h>

#define PR_ABS_MAX 8388607 /* 24 bit 2'compl */
#define PR_ABS_MIN -8388608

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

static struct device *pressure_sysfs_device;
static struct device_attribute **chipset_attrs;

static ssize_t sea_level_pressure_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_PRESSURE);

	ret = sscanf(buf, "%9d", &event->pressure_sealevel);
	if (ret < 0) {
		shub_errf("- failed = %d", ret);
		return -EINVAL;
	}

	if (event->pressure_sealevel == 0) {
		shub_infof("our->temperature = 0\n");
		event->pressure_sealevel = -1;
	}

	shub_infof("sea_level_pressure = %d\n", event->pressure_sealevel);

	return size;
}

static ssize_t pressure_cabratioin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_event *event = get_sensor_event(SENSOR_NAME_PRESSURE);
	struct shub_sensor *sensor = get_sensor(SENSOR_NAME_PRESSURE);

	sensor->funcs->open_calibration_file();

	return sprintf(buf, "%d\n", event->pressure_cal);
}

static ssize_t pressure_cabratioin_store(struct device *dev, struct device_attribute *attr, const char *buf,
					 size_t size)
{
	int pressure_cal = 0;
	int ret = 0;
	struct sensor_event *event = get_sensor_event(SENSOR_NAME_PRESSURE);

	ret = kstrtoint(buf, 10, &pressure_cal);
	if (ret < 0) {
		shub_errf("kstrtoint failed.(%d)", ret);
		return ret;
	}

	if (pressure_cal < PR_ABS_MIN || pressure_cal > PR_ABS_MAX) {
		return -EINVAL;
	}

	event->pressure_cal = (s32)pressure_cal;

	return size;
}

static ssize_t pressure_selftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *buffer = NULL;
	int buffer_length = 0;
	int ret = 0;

	ret = shub_send_command_wait(CMD_GETVALUE, SENSOR_TYPE_PRESSURE, SENSOR_FACTORY, 3000, NULL, 0, &buffer,
				     &buffer_length);

	if (ret < 0) {
		shub_errf("shub_send_command_wait Fail %d", ret);
		goto exit;
	}

	if (buffer == NULL) {
		shub_errf("buffer is null");
		ret = -EINVAL;
		goto exit;
	}

	shub_infof("%u", *buffer);
	ret = snprintf(buf, PAGE_SIZE, "%d", *buffer);

exit:
	if (buffer != NULL) {
		kfree(buffer);
	}

	return ret;
}

static DEVICE_ATTR(calibration, S_IRUGO | S_IWUSR | S_IWGRP, pressure_cabratioin_show, pressure_cabratioin_store);
static DEVICE_ATTR(sea_level_pressure, S_IWUSR | S_IWGRP, NULL, sea_level_pressure_store);
static DEVICE_ATTR(selftest, S_IRUGO, pressure_selftest_show, NULL);

static struct device_attribute *pressure_attrs[] = {
	&dev_attr_calibration,
	&dev_attr_sea_level_pressure,
	&dev_attr_selftest,
	NULL,
};

typedef struct device_attribute** (*get_chipset_dev_attrs)(char *);
get_chipset_dev_attrs get_pressure_chipset_dev_attrs[] = {
	get_pressure_lps22hh_dev_attrs,
	get_pressure_lps25h_dev_attrs,
};

void initialize_pressure_sysfs(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PRESSURE);
	int ret, i;

	ret = sensor_device_create(&pressure_sysfs_device, NULL, "barometer_sensor");
	if (ret < 0) {
		shub_errf("fail to creat %s sysfs device", sensor->name);
		return;
	}

	ret = add_sensor_device_attr(pressure_sysfs_device, pressure_attrs);
	if (ret < 0) {
		shub_errf("fail to add %s sysfs device attr", sensor->name);
		return;
	}

	for (i = 0; i < ARRAY_LEN(get_pressure_chipset_dev_attrs); i++) {
		chipset_attrs = get_pressure_chipset_dev_attrs[i](sensor->chipset_name);
		if (chipset_attrs) {
			ret = add_sensor_device_attr(pressure_sysfs_device, chipset_attrs);
			if (ret < 0) {
				shub_errf("fail to add sysfs chipset device attr(%d)", i);
				return;
			}
			break;
		}
	}
}

void remove_pressure_sysfs(void)
{
	if (chipset_attrs)
		remove_sensor_device_attr(pressure_sysfs_device, chipset_attrs);
	remove_sensor_device_attr(pressure_sysfs_device, pressure_attrs);
	sensor_device_destroy(pressure_sysfs_device);
	pressure_sysfs_device = NULL;
}

void initialize_pressure_factory(bool en)
{
	if (!get_sensor_probe_state(SENSOR_TYPE_PRESSURE))
		return;

	if (en)
		initialize_pressure_sysfs();
	else
		remove_pressure_sysfs();
}
