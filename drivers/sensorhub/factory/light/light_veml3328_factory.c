#include <linux/device.h>

#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"

/*************************************************************************/
/* factory Test                                                          */
/*************************************************************************/
#define VEML3328_NAME	"VEML3328"
#define VEML3328_VENDOR	"Capella"

static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VEML3328_NAME);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VEML3328_VENDOR);
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(vendor);

static struct device_attribute *light_veml3328_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	NULL,
};

struct device_attribute **get_light_veml3328_dev_attrs(char *name)
{
	if (strcmp(name, VEML3328_NAME) != 0)
		return NULL;

	return light_veml3328_attrs;
}
