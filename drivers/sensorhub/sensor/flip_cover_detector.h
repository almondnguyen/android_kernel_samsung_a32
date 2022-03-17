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

#ifndef __SHUB_FLIP_COVER_DETECTOR_H__
#define __SHUB_FLIP_COVER_DETECTOR_H__

#define MAX_MATRIX_LEN 54

#include <linux/device.h>

struct flip_cover_detector_data {
	int factory_cover_status;
	int nfc_cover_status;
};

#endif /* __SHUB_FLIP_COVER_DETECTOR_H_ */
