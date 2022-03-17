#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_utility.h"

#include <linux/slab.h>

void print_gyroscope_uncal_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);

	shub_info("%s(%u) : %d, %d, %d, %d, %d, %d (%lld) (%ums, %dms)", sensor->name,
		  SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, event->uncal_x, event->uncal_y, event->uncal_z, event->offset_x,
		  event->offset_y, event->offset_z, event->timestamp, sensor->sampling_period,
		  sensor->max_report_latency);
}

void init_gyroscope_uncal(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "uncal_gyro_sensor");
		sensor->receive_event_size = 12;
		sensor->report_event_size = 12;

		sensor->funcs = (struct sensor_funcs *)kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->print_debug = print_gyroscope_uncal_debug;
	}
}
