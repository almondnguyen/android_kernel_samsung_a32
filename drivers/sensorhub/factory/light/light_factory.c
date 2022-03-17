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

#include "../../sensor/light.h"
#include "../../comm/shub_comm.h"
#include "../../sensorhub/shub_device.h"
#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"
#include "../../utility/shub_dev_core.h"
#include "../../utility/shub_utility.h"
#include "light_factory.h"

#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/
static struct device *light_sysfs_device;
static struct device_attribute **chipset_attrs;
static u32 light_position[6];

static ssize_t lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_LIGHT);

	return sprintf(buf, "%u,%u,%u,%u,%u,%u\n", event->r, event->g, event->b, event->w, event->a_time,
		       event->a_gain);
}

static ssize_t raw_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_LIGHT);

	return sprintf(buf, "%u,%u,%u,%u,%u,%u\n", event->r, event->g, event->b, event->w, event->a_time,
		       event->a_gain);
}

static ssize_t light_circle_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u.%u %u.%u %u.%u\n", light_position[0], light_position[1],
		       light_position[2], light_position[3], light_position[4],
		       light_position[5]);
}

static ssize_t hall_ic_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	u8 hall_ic = 0;

	if (!get_sensor_probe_state(SENSOR_TYPE_LIGHT_AUTOBRIGHTNESS))
		return -ENOENT;
	if (!buf)
		return -EINVAL;

	if (kstrtou8(buf, 10, &hall_ic) < 0)
		return -EINVAL;

	shub_infof("%d", hall_ic);

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_LIGHT_AUTOBRIGHTNESS, HALL_IC_STATUS, (char *)&hall_ic,
				sizeof(hall_ic));
	if (ret < 0) {
		shub_errf("CMD fail %d\n", ret);
		return size;
	}

	return size;
}

static ssize_t coef_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	char *coef_buf = NULL;
	int coef_buf_length = 0;
	int temp_coef[7] = {0, };
	struct light_data *data = get_sensor(SENSOR_TYPE_LIGHT)->data;

	if (data->light_coef) {
		ret = shub_send_command_wait(CMD_GETVALUE, SENSOR_TYPE_LIGHT, LIGHT_COEF, 1000, NULL, 0, &coef_buf,
					&coef_buf_length);

		if (ret < 0) {
			shub_errf("shub_send_command_wait Fail %d", ret);
			kfree(coef_buf);
			return ret;
		}

		if (coef_buf == NULL) {
			shub_errf("buffer is null");
			return -EINVAL;
		}

		if (coef_buf_length != 28) {
			shub_errf("buffer length error %d", coef_buf_length);
			kfree(coef_buf);
			return -EINVAL;
		}

		memcpy(temp_coef, coef_buf, sizeof(temp_coef));

		shub_infof("%d %d %d %d %d %d %d\n", temp_coef[0], temp_coef[1], temp_coef[2], temp_coef[3],
			   temp_coef[4], temp_coef[5], temp_coef[6]);

		ret = snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d\n", temp_coef[0], temp_coef[1], temp_coef[2],
			       temp_coef[3], temp_coef[4], temp_coef[5], temp_coef[6]);

		kfree(coef_buf);
	} else {
		ret = snprintf(buf, PAGE_SIZE, "0,0,0,0,0,0,0\n");
	}

	return ret;
}

static DEVICE_ATTR_RO(lux);
static DEVICE_ATTR_RO(raw_data);
static DEVICE_ATTR_RO(light_circle);
static DEVICE_ATTR_RO(coef);
static DEVICE_ATTR(hall_ic, 0220, NULL, hall_ic_store);

static struct device_attribute *light_attrs[] = {
	&dev_attr_lux,
	&dev_attr_raw_data,
	&dev_attr_hall_ic,
	NULL,
	NULL,
	NULL,
};

typedef struct device_attribute **(*get_chipset_dev_attrs)(char *);
get_chipset_dev_attrs get_light_chipset_dev_attrs[] = {
	get_light_stk3a5x_dev_attrs,
	get_light_stk3x6x_dev_attrs,
	get_light_stk3328_dev_attrs,
	get_light_veml3328_dev_attrs,
	get_light_tcs3701_dev_attrs,
};

static void check_light_dev_attr(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_LIGHT);
	struct light_data *data = sensor->data;
	struct device_node *np = get_shub_device()->of_node;
	int attrs_size = ARRAY_LEN(light_attrs);

	if (!of_property_read_u32_array(np, "light-position", light_position, ARRAY_LEN(light_position))) {
		light_attrs[attrs_size-3] = &dev_attr_light_circle;
		shub_infof("light-position - %u.%u %u.%u %u.%u", light_position[0], light_position[1],
			   light_position[2], light_position[3], light_position[4], light_position[5]);
	}

	if (data->light_coef)
		light_attrs[attrs_size-2] = &dev_attr_coef;
}

void initialize_light_sysfs(void)
{
	int ret, i;
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_LIGHT);

	ret = sensor_device_create(&light_sysfs_device, NULL, "light_sensor");
	if (ret < 0) {
		shub_errf("fail to creat %s sysfs device", sensor->name);
		return;
	}

	check_light_dev_attr();

	ret = add_sensor_device_attr(light_sysfs_device, light_attrs);
	if (ret < 0) {
		shub_errf("fail to add %s sysfs device attr", sensor->name);
		return;
	}

	for (i = 0; i < ARRAY_LEN(get_light_chipset_dev_attrs); i++) {
		chipset_attrs = get_light_chipset_dev_attrs[i](sensor->chipset_name);
		if (chipset_attrs) {
			ret = add_sensor_device_attr(light_sysfs_device, chipset_attrs);
			if (ret < 0) {
				shub_errf("fail to add sysfs chipset device attr(%d)", i);
				return;
			}
			break;
		}
	}
}

void remove_light_sysfs(void)
{
	if (chipset_attrs)
		remove_sensor_device_attr(light_sysfs_device, chipset_attrs);
	remove_sensor_device_attr(light_sysfs_device, light_attrs);
	sensor_device_destroy(light_sysfs_device);
	light_sysfs_device = NULL;
}

void initialize_light_factory(bool en)
{
	if (!get_sensor_probe_state(SENSOR_TYPE_LIGHT))
		return;
	if (en)
		initialize_light_sysfs();
	else
		remove_light_sysfs();
}
