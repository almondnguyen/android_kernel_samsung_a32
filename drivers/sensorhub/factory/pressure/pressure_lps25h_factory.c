#include "../../comm/shub_comm.h"
#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"
#include "../../utility/shub_utility.h"
#include "pressure_factory.h"

#include <linux/slab.h>

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define LPS25H_NAME   "LPS25H"
#define LPS25H_VENDOR "STM"

ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", LPS25H_NAME);
}

ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", LPS25H_VENDOR);
}

ssize_t pressure_temperature_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	s32 temperature = 0;
	s32 float_temperature = 0;
	s32 temp = 0;
	struct sensor_event *event = get_sensor_event(SENSOR_NAME_PRESSURE);

	temp = (s32)(event->temperature);
	temperature = (4250) + ((temp / (120 * 4)) * 100); //(42.5f) + (temperature/(120 * 4));
	float_temperature = ((temperature % 100) > 0 ? (temperature % 100) : -(temperature % 100));

	return sprintf(buf, "%d.%02d\n", (temperature / 100), float_temperature);
}

static DEVICE_ATTR(name, S_IRUGO, name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, vendor_show, NULL);
static DEVICE_ATTR(temperature, S_IRUGO, pressure_temperature_show, NULL);

static struct device_attribute *pressure_lps25h_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_temperature,
	NULL,
};

struct device_attribute **get_pressure_lps25h_dev_attrs(char *name)
{
	if (strcmp(name, LPS25H_NAME) != 0)
		return NULL;

	return pressure_lps25h_attrs;
}
