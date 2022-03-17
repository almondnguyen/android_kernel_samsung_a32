#include "../comm/shub_comm.h"
#include "../sensorhub/shub_device.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_utility.h"
#include "light.h"

#include <linux/of_gpio.h>
#include <linux/slab.h>

static void init_light_variable(struct light_data *data)
{
	data->brightness = -1;
}

static void parse_dt_light(struct device *dev)
{
	struct light_data *data = get_sensor(SENSOR_TYPE_LIGHT)->data;
	struct device_node *np = dev->of_node;
	int coef[LIGHT_COEF_SIZE] = {0, };

	if (!of_property_read_u32_array(np, "light-coef", coef, LIGHT_COEF_SIZE)) {
		data->light_coef = kcalloc(LIGHT_COEF_SIZE, sizeof(int), GFP_KERNEL);
		memcpy(data->light_coef, coef, sizeof(coef));
	}

	if (of_property_read_u32(np, "brightness-array-len", &data->brightness_array_len)) {
		shub_errf("no brightness array len");
		data->brightness_array_len = 0;
		data->brightness_array = NULL;
	} else {
		data->brightness_array = kcalloc(data->brightness_array_len, sizeof(u32), GFP_KERNEL);
		if (of_property_read_u32_array(np, "brightness-array", data->brightness_array,
					       data->brightness_array_len)) {
			shub_errf("no brightness array");
			data->brightness_array_len = 0;
			kfree(data->brightness_array);
			data->brightness_array = NULL;
		}
	}
}

void set_light_coef(struct light_data *data)
{
	int ret = 0;

	if (!get_sensor_probe_state(SENSOR_TYPE_LIGHT)) {
		shub_infof("light sensor is not connected");
		return;
	}

	if (!data->light_coef)
		return;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_LIGHT, LIGHT_COEF,
				(char *)data->light_coef, LIGHT_COEF_SIZE);
	if (ret < 0) {
		shub_errf("MSG2SSP_AP_SET_LIGHT_COEF CMD fail %d\n", ret);
		return;
	}

	shub_infof("%d %d %d %d %d %d %d\n", data->light_coef[0], data->light_coef[1], data->light_coef[2],
		   data->light_coef[3], data->light_coef[4], data->light_coef[5], data->light_coef[6]);
}

int set_light_brightness(struct light_data *data)
{
	int ret = 0;

	if (!get_sensor_probe_state(SENSOR_TYPE_LIGHT)) {
		shub_infof("light sensor is not connected");
		return ret;
	}

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_LIGHT, LIGHT_BRIGHTNESS, (char *)&data->brightness,
				sizeof(data->brightness));
	if (ret < 0) {
		shub_errf("CMD fail %d\n", ret);
		return ret;
	}

	shub_infof("%d", data->brightness);

	return ret;
}

#ifdef CONFIG_SENSORS_SSP_LIGHT_JPNCONCEPT
int set_light_region(struct light_data *data)
{
	int ret = 0;
	char region = 0;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_LIGHT, LIGHT_PRJ_REGION, (char *)&region, sizeof(region));

	if (ret < 0) {
		shub_errf("CMD fail %d\n", ret);
		return ret;
	}

	shub_infof("%d", region);
	int ret;
}
#endif


int init_light_chipset(char *name)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_LIGHT);
	struct light_data *data = sensor->data;

	shub_infof("");
	strcpy(sensor->chipset_name, name);

	parse_dt_light(get_shub_device());
	init_light_variable(data);
	return 0;
}

static int sync_light_status(void)
{
	int ret = 0;
	struct light_data *data = get_sensor(SENSOR_TYPE_LIGHT)->data;

	set_light_coef(data);
	set_light_brightness(data);
#ifdef CONFIG_SENSORS_SSP_LIGHT_JPNCONCEPT
	set_light_region(data);
#endif
	return ret;
}

static int enable_light(void)
{
	struct light_data *data = get_sensor(SENSOR_TYPE_LIGHT)->data;

	data->light_log_cnt = 0;
	return 0;
}

static void report_event_light(void)
{
	struct light_data *data = get_sensor(SENSOR_TYPE_LIGHT)->data;
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_LIGHT);

	if (data->light_log_cnt < 3) {
		shub_info("Light Sensor : lux=%u brightness=%u r=%d g=%d b=%d c=%d atime=%d again=%d", event->lux,
			  event->brightness, event->r, event->g, event->b, event->w, event->a_time, event->a_gain);

		data->light_log_cnt++;
	}
}

void print_light_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_LIGHT);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_LIGHT);

	shub_info("%s(%u) : %u, %u (%lld) (%ums, %dms)", sensor->name, SENSOR_TYPE_LIGHT, event->lux, event->cct,
		  event->timestamp, sensor->sampling_period, sensor->max_report_latency);
}

int inject_light_additional_data(char *buf, int count)
{
	int cur_level = 0;
	int cal_brightness = 0;
	int32_t brightness;
	int i, ret = 0;
	struct light_data *data = get_sensor(SENSOR_TYPE_LIGHT)->data;

	if (count < 4) {
		shub_errf("brightness length error %d", count);
		return -EINVAL;
	}
	brightness = *((int32_t *)(buf));
	cal_brightness = brightness / 10;
	cal_brightness *= 10;

	// shub_errf("br %d, cal_br %d", brightness, cal_brightness);
	// set current level for changing itime
	for (i = 0; i < data->brightness_array_len; i++) {
		if (brightness <= data->brightness_array[i]) {
			cur_level = i + 1;
			// shub_infof("brightness %d <= %d , level %d", brightness, data->brightness_array[i],
			// cur_level);
			break;
		}
	}

	if (data->last_brightness_level != cur_level) {
		data->brightness = brightness;
		// update last level
		data->last_brightness_level = cur_level;
		ret = set_light_brightness(data);
		data->brightness = cal_brightness;
	} else if (data->brightness != cal_brightness) {
		data->brightness = brightness;
		ret = set_light_brightness(data);
		data->brightness = cal_brightness;
	}

	return ret;
}

void init_light(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_LIGHT);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "light_sensor");
		sensor->receive_event_size = 28;
		sensor->report_event_size = 4;

		sensor->data = kzalloc(sizeof(struct light_data), GFP_KERNEL);
		sensor->funcs = kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->sync_status = sync_light_status;
		sensor->funcs->enable = enable_light;
		sensor->funcs->report_event = report_event_light;
		sensor->funcs->print_debug = print_light_debug;
		sensor->funcs->inject_additional_data = inject_light_additional_data;
		sensor->funcs->init_chipset = init_light_chipset;
	} else {
		struct light_data *data = get_sensor(SENSOR_TYPE_LIGHT)->data;

		kfree(data->light_coef);
		data->light_coef = NULL;
		kfree(sensor->data);
		sensor->data = NULL;

		kfree(sensor->funcs);
		sensor->funcs = NULL;
	}

	return;
}
