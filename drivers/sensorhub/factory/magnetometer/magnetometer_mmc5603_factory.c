#include "../../comm/shub_comm.h"
#include "../../sensor/magnetometer.h"
#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"
#include "../../utility/shub_utility.h"

#include <linux/delay.h>
#include <linux/slab.h>

#define MMC5603_NAME   "MMC5633"
#define MMC5603_VENDOR "Memsic"

#define GM_MMC_DATA_SPEC_MIN -6500
#define GM_MMC_DATA_SPEC_MAX 6500

#define GM_SELFTEST_X_SPEC_MIN 50
#define GM_SELFTEST_X_SPEC_MAX 150
#define GM_SELFTEST_Y_SPEC_MIN 50
#define GM_SELFTEST_Y_SPEC_MAX 150
#define GM_SELFTEST_Z_SPEC_MIN 50
#define GM_SELFTEST_Z_SPEC_MAX 150

int check_mmc5603_adc_data_spec(int type)
{
	struct sensor_event *event = get_sensor_event(type);

	if ((event->x == 0) && (event->y == 0) && (event->z == 0)) {
		return -1;
	} else if ((event->x > GM_MMC_DATA_SPEC_MAX) || (event->x < GM_MMC_DATA_SPEC_MIN) ||
		   (event->y > GM_MMC_DATA_SPEC_MAX) || (event->y < GM_MMC_DATA_SPEC_MIN) ||
		   (event->z > GM_MMC_DATA_SPEC_MAX) || (event->z < GM_MMC_DATA_SPEC_MIN)) {
		return -1;
	} else {
		return 0;
	}
}

static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", MMC5603_NAME);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", MMC5603_VENDOR);
}

static ssize_t matrix_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	return sprintf(buf, "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
		       data->mag_matrix[0], data->mag_matrix[1], data->mag_matrix[2], data->mag_matrix[3],
		       data->mag_matrix[4], data->mag_matrix[5], data->mag_matrix[6], data->mag_matrix[7],
		       data->mag_matrix[8], data->mag_matrix[9], data->mag_matrix[10], data->mag_matrix[11],
		       data->mag_matrix[12], data->mag_matrix[13], data->mag_matrix[14], data->mag_matrix[15],
		       data->mag_matrix[16], data->mag_matrix[17], data->mag_matrix[18], data->mag_matrix[19],
		       data->mag_matrix[20], data->mag_matrix[21], data->mag_matrix[22], data->mag_matrix[23],
		       data->mag_matrix[24], data->mag_matrix[25], data->mag_matrix[26]);
}

static ssize_t matrix_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	u8 val[27] = {0, };
	int ret = 0;
	int i;
	char *token;
	char *str;
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	str = (char *)buf;

	for (i = 0; i < 27; i++) {
		token = strsep(&str, "\n ");
		if (token == NULL) {
			shub_err("too few arguments (27 needed)");
			return -EINVAL;
		}

		ret = kstrtou8(token, 10, &val[i]);
		if (ret < 0) {
			shub_err("kstros8 error %d", ret);
			return ret;
		}
	}

	for (i = 0; i < 27; i++)
		data->mag_matrix[i] = val[i];

	shub_info("%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
		  data->mag_matrix[0], data->mag_matrix[1], data->mag_matrix[2], data->mag_matrix[3],
		  data->mag_matrix[4], data->mag_matrix[5], data->mag_matrix[6], data->mag_matrix[7],
		  data->mag_matrix[8], data->mag_matrix[9], data->mag_matrix[10], data->mag_matrix[11],
		  data->mag_matrix[12], data->mag_matrix[13], data->mag_matrix[14], data->mag_matrix[15],
		  data->mag_matrix[16], data->mag_matrix[17], data->mag_matrix[18], data->mag_matrix[19],
		  data->mag_matrix[20], data->mag_matrix[21], data->mag_matrix[22], data->mag_matrix[23],
		  data->mag_matrix[24], data->mag_matrix[25], data->mag_matrix[26]);
	set_mag_matrix(data);

	return ret;
}

static ssize_t selftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	s8 result[4] = {-1, -1, -1, -1};
	char *buf_selftest = NULL;
	int buf_selftest_length = 0;
	char bufAdc[8] = {0, };
	s16 ratio_X = 0, ratio_Y = 0, ratio_Z = 0;
	s16 iADC_X = 0, iADC_Y = 0, iADC_Z = 0;
	s16 srdiff_X = 0, srdiff_Y = 0, srdiff_Z = 0;
	s16 ref_X = 0, ref_Y = 0, ref_Z = 0;
	int srsum_X = 0, srsum_Y = 0, srsum_Z = 0;
	s32 dMsDelay = 20;
	int ret = 0;
	int spec_out_retries = 0;
	struct sensor_event *event = get_sensor_event(SENSOR_TYPE_GEOMAGNETIC_POWER);

	shub_infof("");

	result[0] = 0;

Retry_selftest:
	ret = shub_send_command_wait(CMD_GETVALUE, SENSOR_TYPE_GEOMAGNETIC_FIELD, SENSOR_FACTORY, 1000, NULL, 0,
				     &buf_selftest, &buf_selftest_length);

	if (ret < 0) {
		shub_err("shub_send_command_wait Fail %d", ret);
		goto exit;
	}

	if (buf_selftest == NULL) {
		shub_err("buffer is null");
		goto exit;
	}

	if (buf_selftest_length < 25) {
		shub_err("buffer length error %d", buf_selftest_length);
		goto exit;
	}

	result[2] = 0;

	/* read 6bytes ratio data */
	ratio_X = (s16)((buf_selftest[1] << 8) + buf_selftest[2]);
	ratio_Y = (s16)((buf_selftest[3] << 8) + buf_selftest[4]);
	ratio_Z = (s16)((buf_selftest[5] << 8) + buf_selftest[6]);

	shub_info("self test ratioX = %d, ratioY = %d, ratioZ = %d\n", ratio_X, ratio_Y, ratio_Z);

	/* read 6bytes set/reset diff data*/
	srdiff_X = (s16)((buf_selftest[7] << 8) + buf_selftest[8]);
	srdiff_Y = (s16)((buf_selftest[9] << 8) + buf_selftest[10]);
	srdiff_Z = (s16)((buf_selftest[11] << 8) + buf_selftest[12]);

	shub_info("self test srdiff_X = %+8d.%06d, srdiff_Y = %+8d.%06d, srdiff_Z = %+8d.%06d\n",
		  (int)(srdiff_X * 100) / 1024, (int)(srdiff_X * 100) % 1024, (int)(srdiff_Y * 100) / 1024,
		  (int)(srdiff_Y * 100) % 1024, (int)(srdiff_Z * 100) / 1024, (int)(srdiff_Z * 100) % 1024);

	/* read 12bytes set/reset sum data*/
	srsum_X =
	    (int)((buf_selftest[16] << 24) + (buf_selftest[15] << 16) + (buf_selftest[14] << 8) + buf_selftest[13]);
	srsum_Y =
	    (int)((buf_selftest[20] << 24) + (buf_selftest[19] << 16) + (buf_selftest[18] << 8) + buf_selftest[17]);
	srsum_Z =
	    (int)((buf_selftest[24] << 24) + (buf_selftest[23] << 16) + (buf_selftest[22] << 8) + buf_selftest[21]);

	shub_info("self test srsum_X = %+8d.%06d, srsum_Y = %+8d.%06d, srsum_Z = %+8d.%06d\n",
		  (int)(srsum_X * 100) / 1024, (int)(srsum_X * 100) % 1024, (int)(srsum_Y * 100) / 1024,
		  (int)(srsum_Y * 100) % 1024, (int)(srsum_Z * 100) / 1024, (int)(srsum_Z * 100) % 1024);

	/* read 6bytes set/reset ref data*/
	ref_X = (s16)((buf_selftest[25] << 8) + buf_selftest[26]);
	ref_Y = (s16)((buf_selftest[27] << 8) + buf_selftest[28]);
	ref_Z = (s16)((buf_selftest[29] << 8) + buf_selftest[30]);

	shub_info("self test ref_X = %+8d.%06d, ref_Y = %+8d.%06d, ref_Z = %+8d.%06d\n", (int)(ref_X * 100) / 1024,
		  (int)(ref_X * 100) % 1024, (int)(ref_Y * 100) / 1024, (int)(ref_Y * 100) % 1024,
		  (int)(ref_Z * 100) / 1024, (int)(ref_Z * 100) % 1024);

	if ((ratio_X >= GM_SELFTEST_X_SPEC_MIN) && (ratio_X <= GM_SELFTEST_X_SPEC_MAX))
		shub_info("x passed self test, expect 50<=x<=150\n");
	else
		shub_info("x failed self test, expect 50<=x<=150\n");
	if ((ratio_Y >= GM_SELFTEST_Y_SPEC_MIN) && (ratio_Y <= GM_SELFTEST_Y_SPEC_MAX))
		shub_info("y passed self test, expect 50<=y<=150\n");
	else
		shub_info("y failed self test, expect 50<=y<=150\n");

	if ((ratio_Z >= GM_SELFTEST_Z_SPEC_MIN) && (ratio_Z <= GM_SELFTEST_Z_SPEC_MAX))
		shub_info("z passed self test, expect 50<=z<=150\n");
	else
		shub_info("z failed self test, expect 50<=z<=150\n");

	// If selftest is passed, set result[1] to 1.
	if (buf_selftest[0] == 0)
		result[1] = 0;
	else
		shub_err("self test fail %d", buf_selftest[0] /*Selftest result*/);

	if ((result[1] == -1) && (spec_out_retries++ < 5)) {
		shub_err("selftest spec out. Retry = %d", spec_out_retries);
		goto Retry_selftest;
	}

	spec_out_retries = 10;

	/* ADC */
	memcpy(&bufAdc[0], &dMsDelay, 4);

	event->x = 0;
	event->y = 0;
	event->z = 0;

	if (!get_sensor_enabled(SENSOR_TYPE_GEOMAGNETIC_POWER))
		batch_sensor(SENSOR_TYPE_GEOMAGNETIC_POWER, 20, 0);
	enable_sensor(SENSOR_TYPE_GEOMAGNETIC_POWER, NULL, 0);

	do {
		msleep(60);
		if (check_mmc5603_adc_data_spec(SENSOR_TYPE_GEOMAGNETIC_POWER) == 0) { // success
			break;
		}
	} while (--spec_out_retries);

	if (spec_out_retries > 0)
		result[3] = 0;

	iADC_X = event->x;
	iADC_Y = event->y;
	iADC_Z = event->z;

	disable_sensor(SENSOR_TYPE_GEOMAGNETIC_POWER, NULL, 0);

	shub_info("-adc, x = %d, y = %d, z = %d, retry = %d\n", iADC_X, iADC_Y, iADC_Z, spec_out_retries);

exit:
	shub_info("out. Result = %d %d %d %d\n", result[0], result[1], result[2], result[3]);

	ret = sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", result[0], result[1], ratio_X, ratio_Y, ratio_Z,
		      result[2], result[3], iADC_X, iADC_Y, iADC_Z);

	kfree(buf_selftest);


	return ret;
}

static ssize_t hw_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;
	struct calibration_data_mmc5603 *cal_data = data->cal_data;

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", cal_data->offset_x, cal_data->offset_y, cal_data->offset_z);
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(vendor);
static DEVICE_ATTR_RO(selftest);
static DEVICE_ATTR_RO(hw_offset);
static DEVICE_ATTR(matrix, 0664, matrix_show, matrix_store);

static struct device_attribute *mag_mmc5603_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_selftest,
	&dev_attr_matrix,
	&dev_attr_hw_offset,
	NULL,
};

struct device_attribute **get_magnetometer_mmc5603_dev_attrs(char *name)
{
	if (strcmp(name, MMC5603_NAME) != 0)
		return NULL;

	return mag_mmc5603_attrs;
}
