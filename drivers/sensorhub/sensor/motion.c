#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_utility.h"

void init_sigificant_motion(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_SIGNIFICANT_MOTION);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "sig_motion_sensor");
		sensor->receive_event_size = 1;
		sensor->report_event_size = 1;
	}
}

void init_tilt_detector(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_TILT_DETECTOR);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "tilt_detector");
		sensor->receive_event_size = 1;
		sensor->report_event_size = 1;
	}
}

void init_pick_up_gesture(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PICK_UP_GESTURE);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "pickup_gesture");
		sensor->receive_event_size = 1;
		sensor->report_event_size = 1;
	}
}

void init_call_gesture(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_CALL_GESTURE);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "call_gesture");
		sensor->receive_event_size = 1;
		sensor->report_event_size = 1;
	}
}

void init_wake_up_motion(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_WAKE_UP_MOTION);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "wake_up_motion");
		sensor->receive_event_size = 1;
		sensor->report_event_size = 1;
	}
}

void init_protos_motion(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PROTOS_MOTION);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "protos_motion");
		sensor->receive_event_size = 1;
		sensor->report_event_size = 1;
	}
}

void init_pocket_mode(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_POCKET_MODE);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "pocket_mode_sensor");
		sensor->receive_event_size = 5;
		sensor->report_event_size = 5;
	}
}
