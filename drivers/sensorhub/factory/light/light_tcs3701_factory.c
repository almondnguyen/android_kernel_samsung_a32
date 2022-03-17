#include <linux/device.h>

#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"

/*************************************************************************/
/* factory Test                                                          */
/*************************************************************************/

#define TCS3701_NAME	"TCS3701"
#define TCS3701_VENDOR	"AMS"

static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", TCS3701_NAME);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", TCS3701_VENDOR);
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(vendor);

static struct device_attribute *light_tcs3701_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	NULL,
};

struct device_attribute **get_light_tcs3701_dev_attrs(char *name)
{
	if (strcmp(name, TCS3701_NAME) != 0)
		return NULL;

	return light_tcs3701_attrs;
}
