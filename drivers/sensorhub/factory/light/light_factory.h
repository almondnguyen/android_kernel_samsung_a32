#ifndef __SHUB_LIGHT_FACTORY_H_
#define __SHUB_LIGHT_FACTORY_H_

#include <linux/device.h>
#include <linux/types.h>

struct device_attribute **get_light_stk3a5x_dev_attrs(char *name);
struct device_attribute **get_light_stk3x6x_dev_attrs(char *name);
struct device_attribute **get_light_stk3328_dev_attrs(char *name);
struct device_attribute **get_light_veml3328_dev_attrs(char *name);
struct device_attribute **get_light_tcs3701_dev_attrs(char *name);

#endif
