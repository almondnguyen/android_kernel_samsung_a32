#include "../comm/shub_comm.h"
#include "../sensorhub/shub_device.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_dev_core.h"
#include "../utility/shub_utility.h"
#include "proximity.h"

#include <linux/of_gpio.h>
#include <linux/slab.h>

static void init_proximity_variable(struct proximity_data *data)
{
	if (data->chipset_funcs && data->chipset_funcs->init_proximity_variable)
		data->chipset_funcs->init_proximity_variable(data);
}

static void parse_dt_proximity(struct device *dev)
{
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	if (data->chipset_funcs && data->chipset_funcs->parse_dt)
		data->chipset_funcs->parse_dt(dev);
}

#define TEMPERATURE_THRESH 50
#define COMPENSATION_VALUE 40
#define BATTERY_TEMP_PATH  "/sys/class/power_supply/battery/temp"

int get_proximity_thresh_temperature_compensation(void)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *temperature_filp = NULL;
	char temp_str[10] = {0, };
	int temp_value = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	temperature_filp = filp_open(BATTERY_TEMP_PATH, O_RDONLY | O_NOFOLLOW | O_NONBLOCK, 0660);
	if (IS_ERR(temperature_filp)) {
		temperature_filp = filp_open(BATTERY_TEMP_PATH, O_RDONLY | O_NOFOLLOW | O_NONBLOCK, 0660);

		if (IS_ERR(temperature_filp)) {
			set_fs(old_fs);
			shub_errf("can't proximity open temperature file %d", PTR_ERR(temperature_filp));
			return 0;
		}
	}

	ret = vfs_read(temperature_filp, (char *)&temp_str, sizeof(temp_str), &temperature_filp->f_pos);
	if (ret != sizeof(temp_value))
		ret = 0;

	ret = kstrtos32(temp_str, 10, &temp_value);
	if (ret < 0) {
		shub_errf("kstrtou32 failed(%d)", ret);
		ret = 0;
	} else if (temp_value < TEMPERATURE_THRESH)
		ret = COMPENSATION_VALUE;

	shub_infof("%s temp value %d compensation %d", temp_str, temp_value, ret);

	filp_close(temperature_filp, current->files);
	set_fs(old_fs);

	return ret;
}

void set_proximity_threshold(void)
{
	int ret = 0;
	u8 prox_th_mode = -1;
	u16 prox_th[PROX_THRESH_SIZE] = {0, };
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	if (!get_sensor_probe_state(SENSOR_TYPE_PROXIMITY)) {
		shub_infof("proximity sensor is not connected");
		return;
	}

	memcpy(prox_th, data->prox_threshold, sizeof(prox_th));

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_PROXIMITY, PROXIMITY_THRESHOLD, (char *)prox_th,
				sizeof(prox_th));
	if (ret < 0) {
		shub_err("SENSOR_PROXTHRESHOLD CMD fail %d", ret);
		return;
	}

	if (data->need_compensation) {
		int compensation = get_proximity_thresh_temperature_compensation();

		prox_th[0] += compensation;
		prox_th[1] += compensation;
	}

	if (data->chipset_funcs && data->chipset_funcs->get_proximity_threshold_mode)
		prox_th_mode = data->chipset_funcs->get_proximity_threshold_mode();

	shub_info("Proximity Threshold[%d] - %u, %u", prox_th_mode, data->prox_threshold[PROX_THRESH_HIGH],
		  data->prox_threshold[PROX_THRESH_LOW]);
}

typedef struct proximity_chipset_funcs *(get_proximity_function_pointer)(char *);

get_proximity_function_pointer *get_prox_funcs_ary[] = {
	get_proximity_stk3x6x_function_pointer,
	get_proximity_gp2ap110s_function_pointer,
	get_proximity_stk3328_function_pointer,
};

int init_proximity_chipset(char *name)
{
	int i;
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PROXIMITY);
	struct proximity_data *data = sensor->data;
	struct proximity_chipset_funcs *funcs;

	shub_infof("");

	if (data->chipset_funcs)
		return 0;

	strcpy(sensor->chipset_name, name);

	for (i = 0; i < ARRAY_LEN(get_prox_funcs_ary); i++) {
		funcs = get_prox_funcs_ary[i](name);
		if (funcs) {
			data->chipset_funcs = funcs;
			if (data->chipset_funcs->init)
				data->chipset_funcs->init(data);
			break;
		}
	}

	if (!data->chipset_funcs) {
		shub_errf("cannot find proximity sensor chipset.");
		return -EINVAL;
	}

	parse_dt_proximity(get_shub_device());
	init_proximity_variable(data);

	return 0;
}

static int sync_proximity_status(void)
{
	int ret = 0;
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	shub_infof();

	set_proximity_threshold();
	if (data->chipset_funcs && data->chipset_funcs->sync_proximity_state)
		data->chipset_funcs->sync_proximity_state(data);

	return ret;
}

static void print_debug_proximity(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PROXIMITY);
	if (!sensor)
		return;

	shub_infof("%s(%u) : %d, %d (%lld) (%ums, %dms)", sensor->name, SENSOR_TYPE_PROXIMITY,
		   sensor->event_buffer.prox, sensor->event_buffer.prox_ex, sensor->event_buffer.timestamp,
		   sensor->sampling_period, sensor->max_report_latency);
}

static int enable_proximity(void)
{
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	set_proximity_threshold();
	if (data->chipset_funcs && data->chipset_funcs->pre_enable_proximity)
		data->chipset_funcs->pre_enable_proximity(data);

	return 0;
}

void print_proximity_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PROXIMITY);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_PROXIMITY);

	shub_info("%s(%u) : %d, %d (%lld) (%ums, %dms)", sensor->name, SENSOR_TYPE_PROXIMITY, event->prox,
		  event->prox_ex, event->timestamp, sensor->sampling_period, sensor->max_report_latency);
}

void report_event_proximity(void)
{
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_PROXIMITY);
	shub_infof("Proximity Sensor Detect : %u, raw : %u", event->prox, event->prox_ex);
}

int parsing_proximity_threshold(char *dataframe, int *index)
{
	u16 thresh[2] = {0, };
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	memcpy(thresh, dataframe + (*index), sizeof(thresh));
	data->prox_threshold[0] = thresh[0];
	data->prox_threshold[1] = thresh[1];

	if (data->chipset_funcs->set_proximity_threshold_mode)
		data->chipset_funcs->set_proximity_threshold_mode(3);

	(*index) += sizeof(thresh);
	shub_infof("prox threshold received %u %u", data->prox_threshold[0], data->prox_threshold[1]);

	return 0;
}

int open_proximity_calibration(void)
{
	int ret = 0;
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	if (data->chipset_funcs->open_calibration_file)
		ret = data->chipset_funcs->open_calibration_file();
	return ret;
}

void init_proximity(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PROXIMITY);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "proximity_sensor");
		sensor->receive_event_size = 3;
		sensor->report_event_size = 1;

		sensor->data = kzalloc(sizeof(struct proximity_data), GFP_KERNEL);
		sensor->funcs = kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->enable = enable_proximity;
		sensor->funcs->sync_status = sync_proximity_status;
		sensor->funcs->print_debug = print_debug_proximity;
		sensor->funcs->report_event = report_event_proximity;
		sensor->funcs->parsing_data = parsing_proximity_threshold;
		sensor->funcs->init_chipset = init_proximity_chipset;
		sensor->funcs->open_calibration_file = open_proximity_calibration;
	} else {
		struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

		kfree(data->threshold_data);
		data->threshold_data  = NULL;
		kfree(sensor->data);
		sensor->data = NULL;

		kfree(sensor->funcs);
		sensor->funcs = NULL;
	}
}
