#include <linux/slab.h>

#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_utility.h"

static void report_event_step_counter(void)
{
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_STEP_COUNTER);

	event->step_total += event->step_diff;
}

void print_step_counter_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_STEP_COUNTER);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_STEP_COUNTER);

	shub_info("%s(%u) : %u (%lld) (%ums, %dms)", sensor->name, SENSOR_TYPE_STEP_COUNTER,
		  event->step_diff, event->timestamp,
		  sensor->sampling_period, sensor->max_report_latency);
}

void init_step_counter(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_STEP_COUNTER);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "step_cnt_sensor");
		sensor->receive_event_size = 4;
		sensor->report_event_size = 12;

		sensor->funcs = (struct sensor_funcs *)kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->report_event = report_event_step_counter;
		sensor->funcs->print_debug = print_step_counter_debug;
	}
}

void init_step_detector(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_STEP_DETECTOR);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "step_det_sensor");
		sensor->receive_event_size = 1;
		sensor->report_event_size = 1;
	}
}
