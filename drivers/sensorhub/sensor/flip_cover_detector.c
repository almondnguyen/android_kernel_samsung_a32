#include <linux/slab.h>

#include "../comm/shub_comm.h"
#include "../utility/shub_utility.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../factory/flip_cover_detector/flip_cover_detector_factory.h"
#include "flip_cover_detector.h"

int sync_flip_cover_detector_status(void)
{
	int ret = 0;
	struct flip_cover_detector_data *data = get_sensor(SENSOR_TYPE_FLIP_COVER_DETECTOR)->data;
	int status = data->nfc_cover_status;

	ret = shub_send_command(CMD_SETVALUE, SENSOR_TYPE_FLIP_COVER_DETECTOR, LIBRARY_DATA,
				(char *)&status, sizeof(status));
	if (ret < 0)
		shub_errf("send nfc_cover_status failed");

	return ret;
}

void print_flip_cover_detector_debug(void)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_FLIP_COVER_DETECTOR);
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_FLIP_COVER_DETECTOR);

	shub_info("%s(%u) : %d, %d, %d / %d, %d, %d, %d (%lld) (%ums, %dms)",
		  sensor->name, SENSOR_TYPE_FLIP_COVER_DETECTOR,
		  event->value, (int)event->magX, (int)event->stable_min_max,
		  (int)event->uncal_mag_x, (int)event->uncal_mag_y, (int)event->uncal_mag_z, event->saturation,
		  event->timestamp, sensor->sampling_period, sensor->max_report_latency);
}

void report_event_flip_cover_detector(void)
{
	struct flip_cover_detector_data *data = get_sensor(SENSOR_TYPE_FLIP_COVER_DETECTOR)->data;

	if (data->factory_cover_status)
		check_cover_detection_factory();
}

void init_flip_cover_detector(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_FLIP_COVER_DETECTOR);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "flip_cover_detector");
		sensor->receive_event_size = 22;
		sensor->report_event_size = 9;

		sensor->data = kzalloc(sizeof(struct flip_cover_detector_data), GFP_KERNEL);
		sensor->funcs = kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->sync_status = sync_flip_cover_detector_status;
		sensor->funcs->report_event = report_event_flip_cover_detector;
		sensor->funcs->print_debug = print_flip_cover_detector_debug;
	}
}
