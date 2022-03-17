/*
 *  Copyright (C) 2020, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#ifndef __SHUB_MAGNETOMETER_H_
#define __SHUB_MAGNETOMETER_H_

#include <linux/device.h>

struct magnetometer_data {
	struct magnetometer_chipset_funcs *chipset_funcs;

	int position;
	void *cal_data;
	int cal_data_len;

	u8 *mag_matrix;
	int mag_matrix_len;
};

struct magnetometer_chipset_funcs {
	void (*init)(void);
	void (*parse_dt)(struct device *dev);
};

struct magnetometer_chipset_funcs *get_magnetic_ak09918c_function_pointer(char *name);
struct magnetometer_chipset_funcs *get_magnetic_mmc5603_function_pointer(char *name);
struct magnetometer_chipset_funcs *get_magnetic_yas539_function_pointer(char *name);

int set_mag_matrix(struct magnetometer_data *data);

/* magnetometer calibration data */
struct calibration_data_ak09918c {
	u8 accuracy;
	s16 offset_x;
	s16 offset_y;
	s16 offset_z;
	s16 flucv_x;
	s16 flucv_y;
	s16 flucv_z;
} __attribute__((__packed__));

struct calibration_data_mmc5603 {
	s32 offset_x;
	s32 offset_y;
	s32 offset_z;
	s32 radius;
} __attribute__((__packed__));

struct calibration_data_yas539 {
	s16 offset_x;
	s16 offset_y;
	s16 offset_z;
	u8 accuracy;
} __attribute__((__packed__));

#endif /* __SHUB_MAGNETOMETER_H_ */
