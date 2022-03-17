#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_utility.h"

void init_super(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_SENSORHUB);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "sensorhub_sensor");
		sensor->receive_event_size = 0;
		sensor->report_event_size = 2;
	}
}
