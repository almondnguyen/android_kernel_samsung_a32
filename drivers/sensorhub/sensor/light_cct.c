#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_utility.h"
#include "light.h"

#include <linux/slab.h>

static int light_cct_log_cnt;

static int enable_light_cct(void)
{
	light_cct_log_cnt = 0;

	return 0;
}

static void report_event_light_cct(void)
{
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_LIGHT_CCT);

	if (light_cct_log_cnt < 3) {
		shub_info("Light cct Sensor : lux=%u r=%d g=%d b=%d c=%d atime=%d again=%d", event->lux, event->r,
			  event->g, event->b, event->w, event->a_time, event->a_gain);

		light_cct_log_cnt++;
	}
}

void print_light_cct_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_LIGHT_CCT);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_LIGHT_CCT);

	shub_info("%s(%u) : %u, %u, %u (%lld) (%ums, %dms)", sensor->name, SENSOR_TYPE_LIGHT_CCT, event->lux,
		  event->cct, event->raw_lux, event->timestamp, sensor->sampling_period, sensor->max_report_latency);
}

void init_light_cct(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_LIGHT_CCT);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "light_cct_sensor");
		sensor->receive_event_size = 24;
		sensor->report_event_size = 12;

		sensor->funcs = kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->enable = enable_light_cct;
		sensor->funcs->report_event = report_event_light_cct;
		sensor->funcs->print_debug = print_light_cct_debug;
	} else {
		kfree(sensor->funcs);
		sensor->funcs = NULL;
	}

	return;
}
