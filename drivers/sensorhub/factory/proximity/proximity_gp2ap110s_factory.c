#include <linux/delay.h>
#include <linux/uaccess.h>

#include "../../sensor/proximity.h"
#include "../../sensormanager/shub_sensor.h"
#include "../../sensormanager/shub_sensor_manager.h"
#include "../../utility/shub_utility.h"
#include "proximity_factory.h"

#define GP2AP110S_NAME    "GP2AP110S"
#define GP2AP110S_VENDOR  "SHARP"

#define PROX_SETTINGS_FILE_PATH     "/efs/FactoryApp/prox_settings"

static int save_proximity_setting_mode(void)
{
	struct file *filp = NULL;
	mm_segment_t old_fs;
	int ret = -1;
	char buf[3] = "";
	int buf_len = 0;
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;
	struct proximity_gp2ap110s_data *thd_data = data->threshold_data;

	buf_len = snprintf(buf, sizeof(buf), "%d", thd_data->prox_setting_mode);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(PROX_SETTINGS_FILE_PATH,
				O_CREAT | O_TRUNC | O_RDWR | O_SYNC, 0666);
	if (filp == NULL) {
		shub_infof("filp is NULL");
		return ret;
	}

	if (IS_ERR(filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(filp);
		shub_errf("Can't open prox settings file (%d)", ret);
		return ret;
	}

	ret = vfs_write(filp, buf, buf_len, &filp->f_pos);
	if (ret != buf_len) {
		shub_errf("Can't write the prox settings data to file, ret=%d", ret);
		ret = -EIO;
	}

	filp_close(filp, current->files);
	set_fs(old_fs);

	msleep(150);

	return ret;
}

static ssize_t name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", GP2AP110S_NAME);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", GP2AP110S_VENDOR);
}

static ssize_t proximity_modify_settings_show(struct device *dev,
					      struct device_attribute *attr, char *buf)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_PROXIMITY);
	struct proximity_data *data = sensor->data;
	struct proximity_gp2ap110s_data *thd_data = data->threshold_data;

	sensor->funcs->open_calibration_file();
	return thd_data->prox_setting_mode;
}

static ssize_t proximity_modify_settings_store(struct device *dev,
					       struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	u8 mode;
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;
	struct proximity_gp2ap110s_data *thd_data = data->threshold_data;

	shub_infof("%s\n", buf);

	ret = kstrtou8(buf, 10, &mode);
	if (ret < 0)
		return ret;

	if (mode <= 0 || mode > 2) {
		shub_errf("invalid value %d", mode);
		return -EINVAL;
	}

	shub_infof("prox_setting %d", mode);

	ret = save_proximity_setting_mode();
	if (mode == 2)
		memcpy(data->prox_threshold, thd_data->prox_mode_thresh, sizeof(data->prox_threshold));

	return size;
}

static ssize_t proximity_settings_thresh_high_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;
	struct proximity_gp2ap110s_data *thd_data = data->threshold_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", thd_data->prox_setting_thresh[PROX_THRESH_HIGH]);
}

static ssize_t proximity_settings_thresh_high_store(struct device *dev, struct device_attribute *attr, const char *buf,
						    size_t size)
{
	int ret;
	u16 settings_thresh;
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;
	struct proximity_gp2ap110s_data *thd_data = data->threshold_data;

	ret = kstrtou16(buf, 10, &settings_thresh);
	if (ret < 0) {
		shub_errf("kstrto16 failed.(%d)", ret);
		return -EINVAL;
	}

	thd_data->prox_setting_thresh[PROX_THRESH_HIGH] = settings_thresh;

	shub_infof("new prox setting high threshold %u", thd_data->prox_setting_thresh[PROX_THRESH_HIGH]);

	return size;
}

static ssize_t proximity_settings_thresh_low_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;
	struct proximity_gp2ap110s_data *thd_data = data->threshold_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", thd_data->prox_setting_thresh[PROX_THRESH_LOW]);
}

static ssize_t proximity_settings_thresh_low_store(struct device *dev, struct device_attribute *attr, const char *buf,
						   size_t size)
{
	int ret;
	u16 settings_thresh;
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;
	struct proximity_gp2ap110s_data *thd_data = data->threshold_data;

	ret = kstrtou16(buf, 10, &settings_thresh);
	if (ret < 0) {
		shub_errf("kstrto16 failed.(%d)", ret);
		return -ENOENT;
	}

	thd_data->prox_setting_thresh[PROX_THRESH_LOW] = settings_thresh;

	shub_infof("new prox setting low threshold %u", thd_data->prox_setting_thresh[PROX_THRESH_LOW]);

	return size;
}

static DEVICE_ATTR_RO(name);
static DEVICE_ATTR_RO(vendor);
static DEVICE_ATTR(modify_settings, 0664, proximity_modify_settings_show, proximity_modify_settings_store);
static DEVICE_ATTR(settings_thd_high, 0664, proximity_settings_thresh_high_show, proximity_settings_thresh_high_store);
static DEVICE_ATTR(settings_thd_low, 0664, proximity_settings_thresh_low_show, proximity_settings_thresh_low_store);

static struct device_attribute *proximity_gp2ap110s_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_modify_settings,
	&dev_attr_settings_thd_high,
	&dev_attr_settings_thd_low,
	NULL,
};

struct device_attribute **get_proximity_gp2ap110s_dev_attrs(char *name)
{
	if (strcmp(name, GP2AP110S_NAME) != 0)
		return NULL;

	return proximity_gp2ap110s_attrs;
}
