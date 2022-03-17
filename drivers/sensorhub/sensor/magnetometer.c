#include "../comm/shub_comm.h"
#include "../sensorhub/shub_device.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_dev_core.h"
#include "../utility/shub_utility.h"
#include "../utility/shub_wakelock.h"
#include "magnetometer.h"

#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#define MAG_CALIBRATION_FILE_PATH "/efs/FactoryApp/mag_cal_data"

typedef struct magnetometer_chipset_funcs *(get_magnetometer_function_pointer)(char *);
get_magnetometer_function_pointer *get_mag_funcs_ary[] = {
	get_magnetic_ak09918c_function_pointer,
	get_magnetic_yas539_function_pointer,
	get_magnetic_mmc5603_function_pointer,
};

static void parse_dt_magnetometer(struct device *dev)
{
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	shub_infof("");
	if (data->chipset_funcs->parse_dt)
		data->chipset_funcs->parse_dt(dev);
}

static int set_mag_position(int position)
{
	int ret = 0;
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	data->position = position;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_GEOMAGNETIC_FIELD, SENSOR_AXIS, (char *)&(data->position),
				sizeof(data->position));
	if (ret < 0) {
		shub_errf("CMD fail %d\n", ret);
		return ret;
	}

	shub_infof("%u", data->position);

	return ret;
}

static int get_mag_position(void)
{
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	return data->position;
}

int set_mag_matrix(struct magnetometer_data *data)
{
	int ret = 0;

	shub_infof();

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_GEOMAGNETIC_FIELD, MAGNETIC_STATIC_MATRIX,
				(char *)data->mag_matrix, data->mag_matrix_len);
	shub_infof("%u", data->position);

	if (ret < 0) {
		shub_errf("failed %d", ret);
		return ret;
	}

	return 0;
}

static int open_mag_calibration_file(void)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(MAG_CALIBRATION_FILE_PATH, O_RDONLY | O_NOFOLLOW | O_NONBLOCK, 0660);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);
		shub_errf("Can't open calibration file %d", ret);

		memset(data->cal_data, 0, data->cal_data_len);
		return ret;
	}

	ret = vfs_read(cal_filp, data->cal_data, data->cal_data_len, &cal_filp->f_pos);
	if (ret != data->cal_data_len) {
		ret = -EIO;
		shub_errf("Can't read calibration file %d", ret);
		memset(data->cal_data, 0, data->cal_data_len);
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return ret;
}

static int save_mag_calibration_file(void)
{
	int ret = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(MAG_CALIBRATION_FILE_PATH, O_CREAT | O_TRUNC | O_WRONLY | O_NOFOLLOW | O_NONBLOCK, 0660);
	if (IS_ERR(cal_filp)) {
		shub_errf("Can't open calibration file");
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);
		return -EIO;
	}

	ret = vfs_write(cal_filp, data->cal_data, data->cal_data_len, &cal_filp->f_pos);
	if (ret != data->cal_data_len) {
		shub_errf("Can't write mag cal to file");
		ret = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return ret;
}

static int parsing_mag_calibration(char *dataframe, int *index)
{
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	shub_infof("Mag caldata received from MCU(%d)", data->cal_data_len);
	memcpy(data->cal_data, dataframe + (*index), data->cal_data_len);
	shub_wake_lock();
	save_mag_calibration_file();
	shub_wake_unlock();
	(*index) += data->cal_data_len;

	return 0;
}

static int set_mag_cal(struct magnetometer_data *data)
{
	int ret = 0;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_GEOMAGNETIC_FIELD, CAL_DATA,
				(char *)data->cal_data, data->cal_data_len);
	if (ret < 0)
		shub_errf("shub_send_command_wait fail %d", ret);

	return ret;
}

int init_magnetometer_chipset(char *name)
{
	int i;
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD);
	struct magnetometer_data *data = sensor->data;
	struct magnetometer_chipset_funcs *funcs;

	shub_infof("");

	if (data->chipset_funcs)
		return 0;

	strcpy(sensor->chipset_name, name);

	for (i = 0; i < ARRAY_LEN(get_mag_funcs_ary); i++) {
		funcs = get_mag_funcs_ary[i](name);
		if (funcs) {
			data->chipset_funcs = funcs;
			if (data->chipset_funcs->init)
				data->chipset_funcs->init();
			break;
		}
	}

	if (!data->chipset_funcs) {
		shub_errf("cannot find magnetometer sensor chipset.");
		return -EINVAL;
	}

	if (data->cal_data_len)
		data->cal_data = kzalloc(data->cal_data_len, GFP_KERNEL);

	if (data->mag_matrix_len)
		data->mag_matrix = kzalloc(data->mag_matrix_len, GFP_KERNEL);

	parse_dt_magnetometer(get_shub_device());

	return 0;
}

static int sync_magnetometer_status(void)
{
	int ret = 0;
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	shub_infof();
	ret = set_mag_position(data->position);
	if (ret < 0) {
		shub_errf("set_position failed");
		return ret;
	}

	ret = set_mag_matrix(data);
	if (ret < 0) {
		shub_errf("initialize magnetic sensor failed");
		return ret;
	}

	ret = set_mag_cal(data);
	if (ret < 0)
		shub_errf("set_mag_cal failed\n");

	return ret;
}

static void print_magnetometer_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_GEOMAGNETIC_FIELD);

	shub_info("%s(%u) : %d, %d, %d, %d (%lld) (%ums, %dms)", sensor->name, SENSOR_TYPE_GEOMAGNETIC_FIELD,
		  event->cal_x, event->cal_y, event->cal_z, event->accuracy, event->timestamp, sensor->sampling_period,
		  sensor->max_report_latency);
}

void init_magnetometer(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "geomagnetic_sensor");
		sensor->receive_event_size = 7;
		sensor->report_event_size = 7;

		sensor->data = kzalloc(sizeof(struct magnetometer_data), GFP_KERNEL);
		sensor->funcs = kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->sync_status = sync_magnetometer_status;
		sensor->funcs->set_position = set_mag_position;
		sensor->funcs->get_position = get_mag_position;
		sensor->funcs->print_debug = print_magnetometer_debug;
		sensor->funcs->parsing_data = parsing_mag_calibration;
		sensor->funcs->init_chipset = init_magnetometer_chipset;
		sensor->funcs->open_calibration_file = open_mag_calibration_file;

	} else {
		struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

		kfree(data->cal_data);
		data->cal_data = NULL;

		kfree(data->mag_matrix);
		data->mag_matrix = NULL;

		kfree(sensor->data);
		sensor->data = NULL;

		kfree(sensor->funcs);
		sensor->funcs = NULL;
	}
}

void init_magnetometer_power(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_GEOMAGNETIC_POWER);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "geomagnetic_power");
		sensor->receive_event_size = 6;
		sensor->report_event_size = 6;
	}
}
