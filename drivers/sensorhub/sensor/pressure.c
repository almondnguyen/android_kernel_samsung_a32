#include "../comm/shub_comm.h"
#include "../sensorhub/shub_device.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_utility.h"

#include <linux/of_gpio.h>
#include <linux/slab.h>

#define CALIBRATION_FILE_PATH "/efs/FactoryApp/baro_delta"

static int open_pressure_calibration_file(void)
{
	char chBuf[10] = {0, };
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_PRESSURE);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0660);
	if (IS_ERR(cal_filp)) {
		ret = PTR_ERR(cal_filp);
		if (ret != -ENOENT) {
			shub_errf("Can't open calibration file(%d)\n", ret);
		}
		set_fs(old_fs);
		return ret;
	}
	ret = vfs_read(cal_filp, chBuf, 10 * sizeof(char), &cal_filp->f_pos);
	if (ret < 0) {
		shub_errf("Can't read the cal data from file (%d)\n", ret);
		filp_close(cal_filp, current->files);
		set_fs(old_fs);
		return ret;
	}
	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ret = kstrtoint(chBuf, 10, &event->pressure_cal);
	if (ret < 0) {
		shub_errf("kstrtoint failed. %d", ret);
		return ret;
	}

	shub_infof("open pressure calibration %d", event->pressure_cal);

	return ret;
}

int init_pressure_chipset(char *name)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PRESSURE);

	shub_infof("");

	strcpy(sensor->chipset_name, name);

	return 0;
}

int sync_pressure_status()
{
	shub_infof();
	return 0;
}

void print_pressure_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PRESSURE);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_PRESSURE);

	shub_info("%s(%u) : %d, %d (%lld) (%ums, %dms)", sensor->name, SENSOR_TYPE_PRESSURE, event->pressure,
		  event->temperature, event->timestamp, sensor->sampling_period, sensor->max_report_latency);
}

void init_pressure(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PRESSURE);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "pressure_sensor");
		sensor->receive_event_size = 6;
		sensor->report_event_size = 14;

		sensor->funcs = (struct sensor_funcs *)kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->print_debug = print_pressure_debug;
		sensor->funcs->init_chipset = init_pressure_chipset;
		sensor->funcs->open_calibration_file = open_pressure_calibration_file;
	} else {
		kfree(sensor->funcs);
		sensor->funcs = NULL;
	}

	return;
}