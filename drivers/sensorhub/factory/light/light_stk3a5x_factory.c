#include <linux/device.h>

#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"

/*************************************************************************/
/* factory Test                                                          */
/*************************************************************************/

#define STK33519_NAME	"STK33519"
#define STK33519_VENDOR	"Sitronix"

static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", STK33519_NAME);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", STK33519_VENDOR);
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(vendor);

static struct device_attribute *light_stk3a5x_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	NULL,
};

struct device_attribute **get_light_stk3a5x_dev_attrs(char *name)
{
	if (strcmp(name, STK33519_NAME) != 0)
		return NULL;

	return light_stk3a5x_attrs;
}
