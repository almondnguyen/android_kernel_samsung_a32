#ifndef __SHUB_LIGHT_AUTOBRIGHTNESS_H__
#define __SHUB_LIGHT_AUTOBRIGHTNESS_H__

#include <linux/types.h>

struct light_autobrightness_data {
	bool camera_lux_en;
	int camera_lux;
	int camera_lux_hysteresis[2];
	int camera_br_hysteresis[2];
	int light_ab_log_cnt;
};

#endif /* __SHUB_LIGHT_AUTOBRIGHTNESS_H_ */
