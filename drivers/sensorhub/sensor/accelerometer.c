#include "../comm/shub_comm.h"
#include "../sensorhub/shub_device.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_dev_core.h"
#include "../utility/shub_utility.h"
#include "../others/shub_motor_callback.h"
#include "accelerometer.h"

#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

void parse_dt_accelerometer(struct device *dev)
{
	struct accelerometer_data *data = get_sensor(SENSOR_TYPE_ACCELEROMETER)->data;
	struct device_node *np = dev->of_node;
	int accel_motor_coef = 0;

	if (data->chipset_funcs->parse_dt)
		data->chipset_funcs->parse_dt(dev);

	if (!of_property_read_u32(np, "acc-motor-coef", &accel_motor_coef))
		set_motor_coef(accel_motor_coef);
	shub_infof("acc-motor-coef[%d]", accel_motor_coef);
}

int set_accel_position(int position)
{
	int ret = 0;
	struct accelerometer_data *data = get_sensor(SENSOR_TYPE_ACCELEROMETER)->data;

	data->position = position;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_ACCELEROMETER, SENSOR_AXIS, (char *)&(data->position),
				sizeof(data->position));
	if (ret < 0) {
		shub_errf("CMD fail %d\n", ret);
		return ret;
	}

	shub_infof("A : %u", data->position);

	return ret;
}

int get_accel_position(void)
{
	struct accelerometer_data *data = get_sensor(SENSOR_TYPE_ACCELEROMETER)->data;

	return data->position;
}

static int open_accel_calibration_file(void)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;
	struct accelerometer_data *data = get_sensor(SENSOR_TYPE_ACCELEROMETER)->data;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(ACCEL_CALIBRATION_FILE_PATH, O_RDONLY, 0660);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);

		data->cal_data.x = 0;
		data->cal_data.y = 0;
		data->cal_data.z = 0;

		return ret;
	}

	ret = vfs_read(cal_filp, (char *)&data->cal_data, 3 * sizeof(int), &cal_filp->f_pos);
	if (ret != 3 * sizeof(int))
		ret = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	shub_infof("open accel calibration %d, %d, %d\n", data->cal_data.x, data->cal_data.y, data->cal_data.z);

	if ((data->cal_data.x == 0) && (data->cal_data.y == 0) && (data->cal_data.z == 0))
		return -EINVAL;

	return ret;
}

int set_accel_cal(struct accelerometer_data *data)
{
	int ret = 0;
	s16 accel_cal[3] = {0, };

	if (!get_sensor_probe_state(SENSOR_TYPE_ACCELEROMETER)) {
		shub_infof("[SHUB] Skip this function!!!, accel sensor is not connected\n");
		return ret;
	}

	accel_cal[0] = data->cal_data.x;
	accel_cal[1] = data->cal_data.y;
	accel_cal[2] = data->cal_data.z;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_ACCELEROMETER, CAL_DATA, (char *)accel_cal, 6 * sizeof(char));

	if (ret < 0) {
		shub_errf("CMD Fail %d", ret);
		return ret;
	}
	shub_info("[SHUB] Set accel cal data %d, %d, %d\n", data->cal_data.x, data->cal_data.y, data->cal_data.z);

	return ret;
}


typedef struct accelerometer_chipset_funcs *(get_accelerometer_function_pointer)(char *);
get_accelerometer_function_pointer *get_acc_funcs_ary[] = {
	get_accelometer_icm42605m_function_pointer,
	get_accelometer_lsm6dsl_function_pointer,
	get_accelometer_lis2dlc12_function_pointer,
};

static int init_accelerometer_chipset(char *name)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_ACCELEROMETER);
	struct accelerometer_data *data = (struct accelerometer_data *)sensor->data;
	struct accelerometer_chipset_funcs *funcs;
	int i;

	shub_infof("");

	if (data->chipset_funcs)
		return 0;

	strcpy(sensor->chipset_name, name);

	for (i = 0; i < ARRAY_LEN(get_acc_funcs_ary); i++) {
		funcs = get_acc_funcs_ary[i](name);
		if (funcs) {
			data->chipset_funcs = funcs;
			break;
		}
	}

	if (!data->chipset_funcs) {
		shub_errf("cannot find accelerometer sensor chipset (%s)", name);
		return -EINVAL;
	}

	parse_dt_accelerometer(get_shub_device());

	return 0;
}

int sync_accelerometer_status(void)
{
	int ret = 0;
	struct accelerometer_data *data = get_sensor(SENSOR_TYPE_ACCELEROMETER)->data;

	shub_infof();
	ret = set_accel_position(data->position);
	if (ret < 0) {
		shub_errf("set position failed");
		return ret;
	}

	ret = set_accel_cal(data);
	if (ret < 0) {
		shub_errf("set_mag_cal failed");
		return ret;
	}

	return ret;
}

void print_accelerometer_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_ACCELEROMETER);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_ACCELEROMETER);

	shub_info("%s(%u) : %d, %d, %d (%lld) (%ums, %dms)", sensor->name, SENSOR_TYPE_ACCELEROMETER, event->x,
		  event->y, event->z, event->timestamp, sensor->sampling_period, sensor->max_report_latency);
}

void init_accelerometer(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_ACCELEROMETER);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "accelerometer_sensor");
		sensor->receive_event_size = 6;
		sensor->report_event_size = 6;

		sensor->data = kzalloc(sizeof(struct accelerometer_data), GFP_KERNEL);
		sensor->funcs = kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->sync_status = sync_accelerometer_status;
		sensor->funcs->print_debug = print_accelerometer_debug;
		sensor->funcs->set_position = set_accel_position;
		sensor->funcs->get_position = get_accel_position;
		sensor->funcs->init_chipset = init_accelerometer_chipset;
		sensor->funcs->open_calibration_file = open_accel_calibration_file;
	} else {
		kfree(sensor->data);
		sensor->data = NULL;

		kfree(sensor->funcs);
		sensor->funcs = NULL;
	}
}
