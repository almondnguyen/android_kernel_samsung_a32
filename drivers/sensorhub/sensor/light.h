#ifndef __SHUB_LIGHT_H_
#define __SHUB_LIGHT_H_

#include <linux/types.h>
#include <linux/device.h>

#define LIGHT_COEF_SIZE 7

struct light_data {
	int *light_coef;
	int light_log_cnt;
	int brightness;
	int last_brightness_level;
	int brightness_array_len;
	u32 *brightness_array;
};

#endif /* __SHUB_LIGHT_H_ */
