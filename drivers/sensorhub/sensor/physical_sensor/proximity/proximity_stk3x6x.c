#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>

#include "../../../sensor/proximity.h"
#include "../../../sensormanager/shub_sensor.h"
#include "../../../sensormanager/shub_sensor_manager.h"
#include "../../../comm/shub_comm.h"
#include "../../../sensorhub/shub_device.h"
#include "../../../utility/shub_utility.h"

#define STK3X6X_NAME   "STK33617"
#define STK3X6X_VENDOR "Sitronix"

#define PROX_CALIBRATION_FILE_PATH       "/efs/FactoryApp/prox_cal_data"
#define PROX_CALIBRATION_FILE_MODE2_PATH "/efs/FactoryApp/prox_cal_data2"

static u8 prox_thresh_mode;
static u16 prox_thresh_addval[PROX_THRESH_SIZE];

void init_proximity_stk3x6x_variable(struct proximity_data *data)
{
	struct proximity_stk3x6x_data *thd_data = data->threshold_data;

	data->need_compensation = true;
	thd_data->prox_cal_mode = 0;
}

void parse_dt_proximity_stk3x6x(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	if (of_property_read_u16_array(np, "prox-stk33617-thresh", data->prox_threshold, PROX_THRESH_SIZE))
		shub_err("no prox-stk33617-thresh, set as 0");

	shub_info("thresh %u, %u", data->prox_threshold[PROX_THRESH_HIGH], data->prox_threshold[PROX_THRESH_LOW]);

	if (of_property_read_u16_array(np, "prox-stk33617-thresh-addval", prox_thresh_addval,
				       sizeof(prox_thresh_addval)))
		shub_errf("no prox-stk33617-thresh_addval, set as 0");

	shub_infof("thresh-addval - %u, %u", prox_thresh_addval[PROX_THRESH_HIGH], prox_thresh_addval[PROX_THRESH_LOW]);
}

u8 get_proximity_stk3x6x_threshold_mode(void)
{
	return prox_thresh_mode;
}

void set_proximity_stk3x6x_threshold_mode(u8 mode)
{
	prox_thresh_mode = mode;
}

int proximity_open_calibration(void)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(PROX_CALIBRATION_FILE_PATH,
				O_RDONLY | O_NOFOLLOW | O_NONBLOCK, 0660);
	if (IS_ERR(cal_filp)) {
		cal_filp = filp_open(PROX_CALIBRATION_FILE_MODE2_PATH,
				O_RDONLY | O_NOFOLLOW | O_NONBLOCK, 0660);

		prox_thresh_mode = 2;
		if (IS_ERR(cal_filp)) {
			set_fs(old_fs);
			ret = PTR_ERR(cal_filp);
			prox_thresh_mode = 0;
			shub_errf("can't proximity open calibration file %d", ret);
			return ret;
		}
	} else {
		prox_thresh_mode = 1;
	}

	ret = vfs_read(cal_filp, (char *)&data->prox_threshold, sizeof(data->prox_threshold), &cal_filp->f_pos);
	if (ret != sizeof(data->prox_threshold))
		ret = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	shub_infof("mode %d, thresh %d, %d", prox_thresh_mode, data->prox_threshold[0], data->prox_threshold[1]);

	return ret;
}

int save_prox_cal_threshold_data(struct proximity_data *data)
{
	int ret = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	struct proximity_stk3x6x_data *thd_data = data->threshold_data;

	shub_infof("mode %d thresh %d, %d ", thd_data->prox_cal_mode,
		data->prox_threshold[0], data->prox_threshold[1]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (thd_data->prox_cal_mode == 1) {
		cal_filp = filp_open(PROX_CALIBRATION_FILE_PATH,
				O_CREAT | O_TRUNC | O_WRONLY | O_NOFOLLOW |
				O_NONBLOCK, 0660);
	} else if (thd_data->prox_cal_mode == 2) {
		cal_filp = filp_open(PROX_CALIBRATION_FILE_MODE2_PATH,
				O_CREAT | O_TRUNC | O_WRONLY | O_NOFOLLOW |
				O_NONBLOCK, 0660);
	} else {
		set_fs(old_fs);
		return -EINVAL;
	}

	thd_data->prox_cal_mode = 0;
	if (IS_ERR(cal_filp)) {
		shub_errf("can't open calibration file");
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);
		return -EIO;
	}

	ret = vfs_write(cal_filp, (char *)&data->prox_threshold, sizeof(data->prox_threshold), &cal_filp->f_pos);
	if (ret != sizeof(data->prox_threshold)) {
		shub_errf("can't write prox cal to file");
		ret = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);
	return ret;
}

void pre_report_event_proximity_stk3x6x(void)
{
	struct proximity_data *data = get_sensor(SENSOR_TYPE_PROXIMITY)->data;

	save_prox_cal_threshold_data(data);
	proximity_open_calibration();
}

void init_proximity_stk3x6x(struct proximity_data *data)
{
	if (data->threshold_data == NULL) {
		data->threshold_data =
		    (struct proximity_stk3x6x_data *)kzalloc(sizeof(struct proximity_stk3x6x_data), GFP_KERNEL);
	}
}

struct proximity_chipset_funcs prox_stk3x6x_ops = {
	.init = init_proximity_stk3x6x,
	.init_proximity_variable = init_proximity_stk3x6x_variable,
	.get_proximity_threshold_mode = get_proximity_stk3x6x_threshold_mode,
	.set_proximity_threshold_mode = set_proximity_stk3x6x_threshold_mode,
	.pre_report_event_proximity = pre_report_event_proximity_stk3x6x,
	.parse_dt = parse_dt_proximity_stk3x6x,
	.open_calibration_file = proximity_open_calibration,
};

struct proximity_chipset_funcs *get_proximity_stk3x6x_function_pointer(char *name)
{
	if (strcmp(name, STK3X6X_NAME) != 0)
		return NULL;

	return &prox_stk3x6x_ops;
}
