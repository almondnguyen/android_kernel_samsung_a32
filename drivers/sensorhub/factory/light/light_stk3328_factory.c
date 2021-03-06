#include <linux/device.h>

#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"

/*************************************************************************/
/* factory Test                                                          */
/*************************************************************************/

#define STK3328_NAME	"STK3328"
#define STK3328_VENDOR	"Sitronix"

static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", STK3328_NAME);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", STK3328_VENDOR);
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(vendor);

static struct device_attribute *light_stk3328_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	NULL,
};

struct device_attribute **get_light_stk3328_dev_attrs(char *name)
{
	if (strcmp(name, STK3328_NAME) != 0)
		return NULL;

	return light_stk3328_attrs;
}
