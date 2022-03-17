#include <linux/device.h>

#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"

/*************************************************************************/
/* factory Test                                                          */
/*************************************************************************/

static char *stk3x6x_name_ary[] = {
	"STK33617",
	"STK31610"
};

#define STK3X6X_VENDOR	"Sitronix"

static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_LIGHT);

	return sprintf(buf, "%s\n", sensor->chipset_name);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", STK3X6X_VENDOR);
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(vendor);

static struct device_attribute *light_stk3x6x_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	NULL,
};

struct device_attribute **get_light_stk3x6x_dev_attrs(char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(stk3x6x_name_ary); i++) {
		if (strcmp(name, stk3x6x_name_ary[i]) == 0)
			return light_stk3x6x_attrs;

	}
	return NULL;
}
