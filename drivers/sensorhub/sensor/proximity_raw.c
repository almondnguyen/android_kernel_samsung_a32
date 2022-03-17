#include <linux/slab.h>

#include "../sensor/proximity.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"

#define PROX_AVG_READ_NUM       80
enum {
	PROX_RAW_NUM = 0,
	PROX_RAW_MIN,
	PROX_RAW_SUM,
	PROX_RAW_MAX,
	PROX_RAW_DATA_SIZE,
};

void report_event_proximity_raw(void)
{
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_PROXIMITY_RAW);
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY_RAW)->data;

	if (data->prox_raw_avg[PROX_RAW_NUM]++ >= PROX_AVG_READ_NUM) {
		data->prox_raw_avg[PROX_RAW_SUM] /= PROX_AVG_READ_NUM;
		event->prox_raw[1] = (u16)data->prox_raw_avg[1];
		event->prox_raw[2] = (u16)data->prox_raw_avg[2];
		event->prox_raw[3] = (u16)data->prox_raw_avg[3];

		data->prox_raw_avg[PROX_RAW_NUM] = 0;
		data->prox_raw_avg[PROX_RAW_MIN] = 0;
		data->prox_raw_avg[PROX_RAW_SUM] = 0;
		data->prox_raw_avg[PROX_RAW_MAX] = 0;
	} else {
		data->prox_raw_avg[PROX_RAW_SUM] += event->prox_raw[0];

		if (data->prox_raw_avg[PROX_RAW_NUM] == 1)
			data->prox_raw_avg[PROX_RAW_MIN] = event->prox_raw[0];
		else if (event->prox_raw[0] < data->prox_raw_avg[PROX_RAW_MIN])
			data->prox_raw_avg[PROX_RAW_MIN] = event->prox_raw[0];

		if (event->prox_raw[0] > data->prox_raw_avg[PROX_RAW_MAX])
			data->prox_raw_avg[PROX_RAW_MAX] = event->prox_raw[0];
	}
}

void init_proximity_raw(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PROXIMITY_RAW);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "proximity_raw");
		sensor->receive_event_size = 2;
		sensor->report_event_size = 0;

		sensor->data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;
		sensor->funcs = (struct sensor_funcs *)kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->report_event = report_event_proximity_raw;
	}
}