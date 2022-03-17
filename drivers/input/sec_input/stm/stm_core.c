/* drivers/input/sec_input/stm/stm_core.c
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *
 * Core file for Samsung TSC driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "stm_dev.h"
#include "stm_reg.h"

#if IS_ENABLED(CONFIG_INPUT_SEC_SECURE_TOUCH)
irqreturn_t secure_filter_interrupt(struct stm_ts_data *ts)
{
	if (atomic_read(&ts->secure_enabled) == SECURE_TOUCH_ENABLE) {
		if (atomic_cmpxchg(&ts->secure_pending_irqs, 0, 1) == 0) {
			sysfs_notify(&ts->plat_data->input_dev->dev.kobj, NULL, "secure_touch");

		} else {
			input_info(true, &ts->client->dev, "%s: pending irq:%d\n",
					__func__, (int)atomic_read(&ts->secure_pending_irqs));
		}

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/**
 * Sysfs attr group for secure touch & interrupt handler for Secure world.
 * @atomic : syncronization for secure_enabled
 * @pm_runtime : set rpm_resume or rpm_ilde
 */
ssize_t secure_touch_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stm_ts_data *ts = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d", atomic_read(&ts->secure_enabled));
}

ssize_t secure_touch_enable_store(struct device *dev,
		struct device_attribute *addr, const char *buf, size_t count)
{
	struct stm_ts_data *ts = dev_get_drvdata(dev);
	int ret;
	unsigned long data;

	if (count > 2) {
		input_err(true, &ts->client->dev,
				"%s: cmd length is over (%s,%d)!!\n",
				__func__, buf, (int)strlen(buf));
		return -EINVAL;
	}

	ret = kstrtoul(buf, 10, &data);
	if (ret != 0) {
		input_err(true, &ts->client->dev, "%s: failed to read:%d\n",
				__func__, ret);
		return -EINVAL;
	}

	if (data == 1) {
		if (ts->reset_is_on_going) {
			input_err(true, &ts->client->dev, "%s: reset is on going because i2c fail\n", __func__);
			return -EBUSY;
		}

		/* Enable Secure World */
		if (atomic_read(&ts->secure_enabled) == SECURE_TOUCH_ENABLE) {
			input_err(true, &ts->client->dev, "%s: already enabled\n", __func__);
			return -EBUSY;
		}

		sec_delay(200);
		
		/* syncronize_irq -> disable_irq + enable_irq
		 * concern about timing issue.
		 */
		disable_irq(ts->client->irq);

		/* Release All Finger */
		stm_ts_release_all_finger(ts);

		if (pm_runtime_get_sync(ts->client->adapter->dev.parent) < 0) {
			enable_irq(ts->client->irq);
			input_err(true, &ts->client->dev, "%s: failed to get pm_runtime\n", __func__);
			return -EIO;
		}

#if IS_ENABLED(CONFIG_INPUT_SEC_NOTIFIER)
		sec_input_notify(&ts->stm_input_nb, NOTIFIER_SECURE_TOUCH_ENABLE, NULL);
#endif
		reinit_completion(&ts->secure_powerdown);
		reinit_completion(&ts->secure_interrupt);

		atomic_set(&ts->secure_enabled, 1);
		atomic_set(&ts->secure_pending_irqs, 0);

		enable_irq(ts->client->irq);

		input_info(true, &ts->client->dev, "%s: secure touch enable\n", __func__);
	} else if (data == 0) {
		/* Disable Secure World */
		if (atomic_read(&ts->secure_enabled) == SECURE_TOUCH_DISABLE) {
			input_err(true, &ts->client->dev, "%s: already disabled\n", __func__);
			return count;
		}

		sec_delay(200);

		pm_runtime_put_sync(ts->client->adapter->dev.parent);
		atomic_set(&ts->secure_enabled, 0);

		sysfs_notify(&ts->plat_data->input_dev->dev.kobj, NULL, "secure_touch");

		sec_delay(10);

		stm_ts_irq_thread(ts->client->irq, ts);
		complete(&ts->secure_interrupt);
		complete(&ts->secure_powerdown);

		input_info(true, &ts->client->dev, "%s: secure touch disable\n", __func__);

#if IS_ENABLED(CONFIG_INPUT_SEC_NOTIFIER)
		sec_input_notify(&ts->stm_input_nb, NOTIFIER_SECURE_TOUCH_DISABLE, NULL);
#endif
	} else {
		input_err(true, &ts->client->dev, "%s: unsupport value:%ld\n", __func__, data);
		return -EINVAL;
	}

	return count;
}

ssize_t secure_touch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct stm_ts_data *ts = dev_get_drvdata(dev);
	int val = 0;

	if (atomic_read(&ts->secure_enabled) == SECURE_TOUCH_DISABLE) {
		input_err(true, &ts->client->dev, "%s: disabled\n", __func__);
		return -EBADF;
	}

	if (atomic_cmpxchg(&ts->secure_pending_irqs, -1, 0) == -1) {
		input_err(true, &ts->client->dev, "%s: pending irq -1\n", __func__);
		return -EINVAL;
	}

	if (atomic_cmpxchg(&ts->secure_pending_irqs, 1, 0) == 1) {
		val = 1;
		input_err(true, &ts->client->dev, "%s: pending irq is %d\n",
				__func__, atomic_read(&ts->secure_pending_irqs));
	}

	complete(&ts->secure_interrupt);

	return snprintf(buf, PAGE_SIZE, "%u", val);
}

ssize_t secure_ownership_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "1");
}

int secure_touch_init(struct stm_ts_data *ts)
{
	input_info(true, &ts->client->dev, "%s\n", __func__);

	init_completion(&ts->secure_interrupt);
	init_completion(&ts->secure_powerdown);

	return 0;
}

void secure_touch_stop(struct stm_ts_data *ts, bool stop)
{
	if (atomic_read(&ts->secure_enabled)) {
		atomic_set(&ts->secure_pending_irqs, -1);

		sysfs_notify(&ts->plat_data->input_dev->dev.kobj, NULL, "secure_touch");

		if (stop)
			wait_for_completion_interruptible(&ts->secure_powerdown);

		input_info(true, &ts->client->dev, "%s: %d\n", __func__, stop);
	}
}

static DEVICE_ATTR(secure_touch_enable, (S_IRUGO | S_IWUSR | S_IWGRP),
		secure_touch_enable_show, secure_touch_enable_store);
static DEVICE_ATTR(secure_touch, S_IRUGO, secure_touch_show, NULL);
static DEVICE_ATTR(secure_ownership, S_IRUGO, secure_ownership_show, NULL);
static struct attribute *secure_attr[] = {
	&dev_attr_secure_touch_enable.attr,
	&dev_attr_secure_touch.attr,
	&dev_attr_secure_ownership.attr,
	NULL,
};

static struct attribute_group secure_attr_group = {
	.attrs = secure_attr,
};
#endif

#if IS_ENABLED(CONFIG_SAMSUNG_TUI)
extern int stui_i2c_lock(struct i2c_adapter *adap);
extern int stui_i2c_unlock(struct i2c_adapter *adap);

int stui_tsp_enter(void)
{
	struct stm_ts_data *ts = dev_get_drvdata(ptsp);
	int ret = 0;

	if (!ts)
		return -EINVAL;

#if IS_ENABLED(CONFIG_INPUT_SEC_NOTIFIER)
	sec_input_notify(&ts->stm_input_nb, NOTIFIER_SECURE_TOUCH_ENABLE, NULL);
#endif

	disable_irq(ts->irq);
	stm_ts_release_all_finger(ts);

	ret = stui_i2c_lock(ts->client->adapter);
	if (ret) {
		pr_err("[STUI] stui_i2c_lock failed : %d\n", ret);
#if IS_ENABLED(CONFIG_INPUT_SEC_NOTIFIER)
		sec_input_notify(&ts->stm_input_nb, NOTIFIER_SECURE_TOUCH_DISABLE, NULL);
#endif
		enable_irq(ts->irq);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(stui_tsp_enter);

int stui_tsp_exit(void)
{
	struct stm_ts_data *ts = dev_get_drvdata(ptsp);
	int ret = 0;

	if (!ts)
		return -EINVAL;

	ret = stui_i2c_unlock(ts->client->adapter);
	if (ret)
		pr_err("[STUI] stui_i2c_unlock failed : %d\n", ret);

	enable_irq(ts->irq);

#if IS_ENABLED(CONFIG_INPUT_SEC_NOTIFIER)
	sec_input_notify(&ts->stm_input_nb, NOTIFIER_SECURE_TOUCH_DISABLE, NULL);
#endif

	return ret;
}
EXPORT_SYMBOL(stui_tsp_exit);
#endif

int stm_ts_i2c_write(struct stm_ts_data *ts, u8 *reg, int cnum, u8 *data, int len)
{
	int ret;
	unsigned char retry;
	struct i2c_msg msg;
	int i;
	const int len_max = 0xffff;
	u8 *buf;
	u8 *buff;

	if (len + 1 > len_max) {
		input_err(true, &ts->client->dev,
				"%s: The i2c buffer size is exceeded.\n", __func__);
		return -ENOMEM;
	}

	if (!ts->plat_data->resume_done.done) {
		ret = wait_for_completion_interruptible_timeout(&ts->plat_data->resume_done, msecs_to_jiffies(500));
		if (ret <= 0) {
			input_err(true, &ts->client->dev, "%s: LPM: pm resume is not handled:%d\n", __func__, ret);
			return -EIO;
		}
	}

#if IS_ENABLED(CONFIG_INPUT_SEC_SECURE_TOUCH)
	if (atomic_read(&ts->secure_enabled) == SECURE_TOUCH_ENABLE) {
		input_err(true, &ts->client->dev,
				"%s: TSP no accessible from Linux, TUI is enabled!\n", __func__);
		return -EBUSY;
	}
#endif
#if IS_ENABLED(CONFIG_SAMSUNG_TUI)
	if (STUI_MODE_TOUCH_SEC & stui_get_mode())
		return -EBUSY;
#endif
	buf = kzalloc(cnum, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, reg, cnum);

	buff = kzalloc(len + cnum, GFP_KERNEL);
	if (!buff) {
		kfree(buff);
		return -ENOMEM;
	}

	if (ts->plat_data->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	memcpy(buff, buf, cnum);
	memcpy(buff + cnum, data, len);

	msg.addr = ts->client->addr;
	msg.flags = 0 | I2C_M_DMA_SAFE;
	msg.len = len + cnum;
	msg.buf = buff;

	mutex_lock(&ts->i2c_mutex);
	for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(ts->client->adapter, &msg, 1);
		if (ret == 1)
			break;

		if (ts->plat_data->power_state == SEC_INPUT_STATE_POWER_OFF) {
			input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
			mutex_unlock(&ts->i2c_mutex);
			goto err;
		}

		usleep_range(1 * 1000, 1 * 1000);

		if (retry > 1) {
			input_err(true, &ts->client->dev, "%s: I2C retry %d, ret:%d\n", __func__, retry + 1, ret);
			ts->plat_data->hw_param.comm_err_count++;
			if (ts->debug_flag & SEC_TS_DEBUG_SEND_UEVENT) {
				char result[32];

				snprintf(result, sizeof(result), "RESULT=I2C");
				sec_cmd_send_event_to_user(&ts->sec, NULL, result);
			}
		}
	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts->client->dev, "%s: I2C write over retry limit\n", __func__);
		ret = -EIO;
		if (ts->probe_done && !ts->reset_is_on_going && !ts->plat_data->shutdown_called)
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
	}

	if (ts->debug_flag & SEC_TS_DEBUG_PRINT_I2C_WRITE_CMD) {
		pr_info("sec_input:i2c_cmd: W: %02X | ", *reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}

	if (ret == 1) {
		kfree(buf);
		return 0;
	}
err:
	kfree(buf);
	return -EIO;
}

int stm_ts_i2c_read(struct stm_ts_data *ts, u8 *reg, int cnum, u8 *data, int len)
{
	int ret;
	unsigned char retry;
	struct i2c_msg msg[2];
	int remain = len;
	int i;
	u8 *buff;
	u8 *buf;

	if (!ts->plat_data->resume_done.done) {
		ret = wait_for_completion_interruptible_timeout(&ts->plat_data->resume_done, msecs_to_jiffies(500));
		if (ret <= 0) {
			input_err(true, &ts->client->dev, "%s: LPM: pm resume is not handled:%d\n", __func__, ret);
			return -EIO;
		}
	}

#if IS_ENABLED(CONFIG_INPUT_SEC_SECURE_TOUCH)
	if (atomic_read(&ts->secure_enabled) == SECURE_TOUCH_ENABLE) {
		input_err(true, &ts->client->dev,
				"%s: TSP no accessible from Linux, TUI is enabled!\n", __func__);
		return -EBUSY;
	}
#endif
#if IS_ENABLED(CONFIG_SAMSUNG_TUI)
	if (STUI_MODE_TOUCH_SEC & stui_get_mode())
		return -EBUSY;
#endif
	buf = kzalloc(cnum, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	
	memcpy(buf, reg, cnum);

	buff = kzalloc(len, GFP_KERNEL);
	if (!buff) {
		kfree(buf);
		return -ENOMEM;
	}

	if (ts->plat_data->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF\n", __func__);
		goto err;
	}

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0 | I2C_M_DMA_SAFE;
	msg[0].len = cnum;
	msg[0].buf = buf;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD | I2C_M_DMA_SAFE;
	msg[1].buf = buff;

	mutex_lock(&ts->i2c_mutex);
	if (len <= ts->plat_data->i2c_burstmax) {
		msg[1].len = len;
		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 2);
			if (ret == 2)
				break;
			usleep_range(1 * 1000, 1 * 1000);
			if (ts->plat_data->power_state == SEC_INPUT_STATE_POWER_OFF) {
				input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				goto err;
			}

			if (retry > 1) {
				input_err(true, &ts->client->dev, "%s: I2C retry %d, ret:%d\n",
					__func__, retry + 1, ret);
				ts->plat_data->hw_param.comm_err_count++;
				if (ts->debug_flag & SEC_TS_DEBUG_SEND_UEVENT) {
					char result[32];

					snprintf(result, sizeof(result), "RESULT=I2C");
					sec_cmd_send_event_to_user(&ts->sec, NULL, result);
				}
			}
		}
	} else {
		/*
		 * I2C read buffer is 256 byte. do not support long buffer over than 256.
		 * So, try to seperate reading data about 256 bytes.
		 */
		for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
			ret = i2c_transfer(ts->client->adapter, msg, 1);
			if (ret == 1)
				break;
			usleep_range(1 * 1000, 1 * 1000);
			if (ts->plat_data->power_state == SEC_INPUT_STATE_POWER_OFF) {
				input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
				mutex_unlock(&ts->i2c_mutex);
				goto err;
			}

			if (retry > 1) {
				input_err(true, &ts->client->dev, "%s: I2C retry %d, ret:%d\n",
					__func__, retry + 1, ret);
				ts->plat_data->hw_param.comm_err_count++;
			}
		}

		do {
			if (remain > ts->plat_data->i2c_burstmax)
				msg[1].len = ts->plat_data->i2c_burstmax;
			else
				msg[1].len = remain;

			remain -= ts->plat_data->i2c_burstmax;

			for (retry = 0; retry < SEC_TS_I2C_RETRY_CNT; retry++) {
				ret = i2c_transfer(ts->client->adapter, &msg[1], 1);
				if (ret == 1)
					break;
				usleep_range(1 * 1000, 1 * 1000);
				if (ts->plat_data->power_state == SEC_INPUT_STATE_POWER_OFF) {
					input_err(true, &ts->client->dev, "%s: POWER_STATUS : OFF, retry:%d\n", __func__, retry);
					mutex_unlock(&ts->i2c_mutex);
					goto err;
				}

				if (retry > 1) {
					input_err(true, &ts->client->dev, "%s: I2C retry %d, ret:%d\n",
						__func__, retry + 1, ret);
					ts->plat_data->hw_param.comm_err_count++;
				}
			}
			msg[1].buf += msg[1].len;
		} while (remain > 0);
	}

	mutex_unlock(&ts->i2c_mutex);

	if (retry == SEC_TS_I2C_RETRY_CNT) {
		input_err(true, &ts->client->dev, "%s: I2C read over retry limit\n", __func__);
		ret = -EIO;
		if (ts->probe_done && !ts->reset_is_on_going && !ts->plat_data->shutdown_called)
			schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(TOUCH_RESET_DWORK_TIME));
	}

	memcpy(data, buff, len);
	if (ts->debug_flag & SEC_TS_DEBUG_PRINT_I2C_READ_CMD) {
		pr_info("sec_input:i2c_cmd: R: %02X | ", *reg);
		for (i = 0; i < len; i++)
			pr_cont("%02X ", data[i]);
		pr_cont("\n");
	}

	kfree(buf);
	kfree(buff);
	return ret;
err:
	kfree(buf);
	kfree(buff);
	return -EIO;
}

void stm_ts_reinit(void *data) 
{
	struct stm_ts_data *ts = (struct stm_ts_data *)data;
	int ret = 0;
	int retry = 3;

	do {
		ret = stm_ts_systemreset(ts, 0);
		if (ret < 0)
			stm_ts_reset(ts, 20);
		else
			break;
	} while (--retry);

	if (retry == 0) {
		input_err(true, &ts->client->dev, "%s: Failed to system reset\n", __func__);
		goto out;
	}

	input_info(true, &ts->client->dev,
		"%s: charger=0x%x, touch_functions=0x%x, Power mode=0x%x\n",
		__func__, ts->plat_data->wirelesscharger_mode, ts->plat_data->touch_functions, ts->plat_data->power_state);

	ts->plat_data->touch_noise_status = 0;
	ts->plat_data->touch_pre_noise_status = 0;
	ts->plat_data->wet_mode = 0;

	ts->stm_ts_command(ts, STM_TS_CMD_CLEAR_ALL_EVENT, true);
	stm_ts_release_all_finger(ts);

	if (ts->plat_data->wirelesscharger_mode != TYPE_WIRELESS_CHARGER_NONE) {
		ret = stm_ts_set_charger_mode(ts);
		if (ret < 0)
			goto out;
	}

	stm_ts_set_cover_type(ts, ts->plat_data->touch_functions & STM_TS_TOUCHTYPE_BIT_COVER);

	stm_ts_set_custom_library(ts);
	stm_ts_set_press_property(ts);
	stm_ts_set_fod_finger_merge(ts);

	if (ts->plat_data->support_fod && ts->plat_data->fod_data.set_val)
		stm_ts_set_fod_rect(ts);

	/* Power mode */
	if (ts->plat_data->power_state == SEC_INPUT_STATE_LPM) {
		stm_ts_set_opmode(ts, STM_TS_OPMODE_LOWPOWER);
		sec_delay(50);
		if (ts->plat_data->lowpower_mode & SEC_TS_MODE_SPONGE_AOD)
			stm_ts_set_aod_rect(ts);
	} else {
		sec_input_set_grip_type(ts->client, ONLY_EDGE_HANDLER);

		stm_ts_set_external_noise_mode(ts, EXT_NOISE_MODE_MAX);

		if (ts->plat_data->touchable_area) {
			ret = stm_ts_set_touchable_area(ts);
			if (ret < 0)
				goto out;
		}
	}

	if (ts->plat_data->ed_enable)
		stm_ts_ear_detect_enable(ts, ts->plat_data->ed_enable);
	if (ts->plat_data->pocket_mode)
		stm_ts_pocket_mode_enable(ts, ts->plat_data->pocket_mode);
out:
	stm_ts_set_scanmode(ts, ts->scan_mode);
	
}
/*
 * don't need it in interrupt handler in reality, but, need it in vendor IC for requesting vendor IC.
 * If you are requested additional i2c protocol in interrupt handler by vendor.
 * please add it in stm_ts_external_func.
 */
static void stm_ts_external_func(struct stm_ts_data *ts)
{
	sec_input_set_temperature(ts->client, SEC_INPUT_SET_TEMPERATURE_IN_IRQ);

}

static void stm_ts_coord_parsing(struct stm_ts_data *ts, struct stm_ts_event_coordinate *p_event_coord, u8 t_id)
{
	ts->plat_data->coord[t_id].id = t_id;
	ts->plat_data->coord[t_id].action = p_event_coord->tchsta;
	ts->plat_data->coord[t_id].x = (p_event_coord->x_11_4 << 4) | (p_event_coord->x_3_0);
	ts->plat_data->coord[t_id].y = (p_event_coord->y_11_4 << 4) | (p_event_coord->y_3_0);
	ts->plat_data->coord[t_id].z = p_event_coord->z & 0x3F;
	ts->plat_data->coord[t_id].ttype = p_event_coord->ttype_3_2 << 2 | p_event_coord->ttype_1_0 << 0;
	ts->plat_data->coord[t_id].major = p_event_coord->major;
	ts->plat_data->coord[t_id].minor = p_event_coord->minor;

	if (!ts->plat_data->coord[t_id].palm && (ts->plat_data->coord[t_id].ttype == STM_TS_TOUCHTYPE_PALM))
		ts->plat_data->coord[t_id].palm_count++;

	ts->plat_data->coord[t_id].palm = (ts->plat_data->coord[t_id].ttype == STM_TS_TOUCHTYPE_PALM);
	if (ts->plat_data->coord[t_id].palm)
		ts->plat_data->palm_flag |= (1 << t_id);
	else
		ts->plat_data->palm_flag &= ~(1 << t_id);

	ts->plat_data->coord[t_id].left_event = p_event_coord->left_event;

	ts->plat_data->coord[t_id].noise_level = max(ts->plat_data->coord[t_id].noise_level,
							p_event_coord->noise_level);
	ts->plat_data->coord[t_id].max_strength = max(ts->plat_data->coord[t_id].max_strength,
							p_event_coord->max_strength);
	ts->plat_data->coord[t_id].hover_id_num = max(ts->plat_data->coord[t_id].hover_id_num,
							(u8)p_event_coord->hover_id_num);

	if (ts->plat_data->coord[t_id].z <= 0)
		ts->plat_data->coord[t_id].z = 1;
}
static void stm_ts_fod_vi_event(struct stm_ts_data *ts)
{
	int ret = 0;

	ts->plat_data->fod_data.vi_data[0] = SEC_TS_CMD_SPONGE_FOD_POSITION;
	ret = ts->stm_ts_read_sponge(ts, ts->plat_data->fod_data.vi_data, ts->plat_data->fod_data.vi_size);
	if (ret < 0)
		input_info(true, &ts->client->dev, "%s: failed read fod vi\n", __func__);
}

static void stm_ts_gesture_event(struct stm_ts_data *ts, u8 *event_buff)
{
	struct stm_ts_gesture_status *p_gesture_status;
	int ret = 0;
	int x, y;

	p_gesture_status = (struct stm_ts_gesture_status *)event_buff;

	x = (p_gesture_status->gesture_data_1 << 4) | (p_gesture_status->gesture_data_3 >> 4);
	y = (p_gesture_status->gesture_data_2 << 4) | (p_gesture_status->gesture_data_3 & 0x0F);

	if (p_gesture_status->stype == STM_TS_SPONGE_EVENT_SWIPE_UP) {
		sec_input_gesture_report(ts->client, SPONGE_EVENT_TYPE_SPAY, 0, 0);
	} else if (p_gesture_status->stype == STM_TS_GESTURE_CODE_DOUBLE_TAP) {
		if (p_gesture_status->gesture_id == STM_TS_SPONGE_EVENT_GESTURE_ID_AOD) {
			sec_input_gesture_report(ts->client, SPONGE_EVENT_TYPE_AOD_DOUBLETAB, x, y);
		} else if (p_gesture_status->gesture_id == STM_TS_SPONGE_EVENT_GESTURE_ID_DOUBLETAP_TO_WAKEUP) {
			input_info(true, &ts->client->dev, "%s: AOT\n", __func__);
			input_report_key(ts->plat_data->input_dev, KEY_WAKEUP, 1);
			input_sync(ts->plat_data->input_dev);
			input_report_key(ts->plat_data->input_dev, KEY_WAKEUP, 0);
			input_sync(ts->plat_data->input_dev);
		}
	} else if (p_gesture_status->stype  == STM_TS_SPONGE_EVENT_SINGLETAP) {
		sec_input_gesture_report(ts->client, SPONGE_EVENT_TYPE_SINGLE_TAP, x, y);
	} else if (p_gesture_status->stype  == STM_TS_SPONGE_EVENT_PRESS) {
		if (p_gesture_status->gesture_id == STM_TS_SPONGE_EVENT_GESTURE_ID_FOD_LONG ||
			p_gesture_status->gesture_id == STM_TS_SPONGE_EVENT_GESTURE_ID_FOD_NORMAL) {
			sec_input_gesture_report(ts->client, SPONGE_EVENT_TYPE_FOD_PRESS, x, y);
			input_info(true, &ts->client->dev, "%s: FOD %sPRESS\n",
					__func__, p_gesture_status->gesture_id ? "" : "LONG");
		} else if (p_gesture_status->gesture_id == STM_TS_SPONGE_EVENT_GESTURE_ID_FOD_RELEASE) {
			sec_input_gesture_report(ts->client, SPONGE_EVENT_TYPE_FOD_RELEASE, x, y);
			input_info(true, &ts->client->dev, "%s: FOD RELEASE\n", __func__);
			memset(ts->plat_data->fod_data.vi_data, 0x0, ts->plat_data->fod_data.vi_size);
		} else if (p_gesture_status->gesture_id == STM_TS_SPONGE_EVENT_GESTURE_ID_FOD_OUT) {
			sec_input_gesture_report(ts->client, SPONGE_EVENT_TYPE_FOD_OUT, x, y);
			input_info(true, &ts->client->dev, "%s: FOD OUT\n", __func__);
		} else if (p_gesture_status->gesture_id == STM_TS_SPONGE_EVENT_GESTURE_ID_FOD_VI) {
			if ((ts->plat_data->lowpower_mode & SEC_TS_MODE_SPONGE_PRESS) && ts->plat_data->support_fod_lp_mode)
				stm_ts_fod_vi_event(ts);
		} else {
			input_info(true, &ts->client->dev, "%s: invalid id %d\n",
					__func__, p_gesture_status->gesture_id);
		}
	} else if (p_gesture_status->stype  == STM_TS_GESTURE_CODE_DUMPFLUSH) {
#if IS_ENABLED(CONFIG_TOUCHSCREEN_DUMP_MODE)
		if (ts->sponge_inf_dump) {
			if (ts->plat_data->power_state == SEC_INPUT_STATE_LPM) {
				if (p_gesture_status->gesture_id == STM_TS_SPONGE_DUMP_0)
					stm_ts_sponge_dump_flush(ts, STM_TS_SPONGE_DUMP_0);
				if (p_gesture_status->gesture_id == STM_TS_SPONGE_DUMP_1)
					stm_ts_sponge_dump_flush(ts, STM_TS_SPONGE_DUMP_1);
			} else {
				ts->sponge_dump_delayed_flag = true;
				ts->sponge_dump_delayed_area = p_gesture_status->gesture_id;
			}
		}
	}
#endif
}

static void stm_ts_coordinate_event(struct stm_ts_data *ts, u8 *event_buff)
{
	struct stm_ts_event_coordinate *p_event_coord;
	u8 t_id = 0;

	if (ts->plat_data->power_state != SEC_INPUT_STATE_POWER_ON) {
		input_err(true, &ts->client->dev,
				"%s: device is closed %x %x %x %x %x %x %x %x\n", __func__,
				event_buff[0], event_buff[1], event_buff[2],
				event_buff[3], event_buff[4], event_buff[5],
				event_buff[6], event_buff[7]);
		return;
	}

	p_event_coord = (struct stm_ts_event_coordinate *)event_buff;

	t_id = p_event_coord->tid;

	if (t_id < SEC_TS_SUPPORT_TOUCH_COUNT) {
		ts->plat_data->prev_coord[t_id] = ts->plat_data->coord[t_id];
		stm_ts_coord_parsing(ts, p_event_coord, t_id);

		if ((ts->plat_data->coord[t_id].ttype == STM_TS_TOUCHTYPE_NORMAL)
				|| (ts->plat_data->coord[t_id].ttype == STM_TS_TOUCHTYPE_PALM)
				|| (ts->plat_data->coord[t_id].ttype == STM_TS_TOUCHTYPE_WET)
				|| (ts->plat_data->coord[t_id].ttype == STM_TS_TOUCHTYPE_GLOVE)) {
			sec_input_coord_event(ts->client, t_id);
		} else {
			input_err(true, &ts->client->dev,
					"%s: do not support coordinate type(%d)\n",
					__func__, ts->plat_data->coord[t_id].ttype);
		}
	} else {
		input_err(true, &ts->client->dev, "%s: tid(%d) is out of range\n", __func__, t_id);
	}
}

static void stm_ts_status_event(struct stm_ts_data *ts, u8 *event_buff)
{
	struct stm_ts_event_status *p_event_status;

	p_event_status = (struct stm_ts_event_status *)event_buff;

	if (p_event_status->stype > 0)
		input_info(true, &ts->client->dev, "%s: STATUS %x %x %x %x %x %x %x %x\n", __func__,
				event_buff[0], event_buff[1], event_buff[2],
				event_buff[3], event_buff[4], event_buff[5],
				event_buff[6], event_buff[7]);

	if (p_event_status->stype == STM_TS_EVENT_STATUSTYPE_ERROR) {
		if (p_event_status->status_id == STM_TS_ERR_EVENT_QUEUE_FULL) {
			input_err(true, &ts->client->dev, "%s: IC Event Queue is full\n", __func__);
			stm_ts_release_all_finger(ts);
		} else if (p_event_status->status_id == STM_TS_ERR_EVENT_ESD) {
			input_err(true, &ts->client->dev, "%s: ESD detected\n", __func__);
			if (!ts->reset_is_on_going)
				schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(10));
		}
	} else if (p_event_status->stype == STM_TS_EVENT_STATUSTYPE_INFO) {
		if (p_event_status->status_id == STM_TS_INFO_READY_STATUS) {
			if (p_event_status->status_data_1 == 0x10) {
				input_err(true, &ts->client->dev, "%s: IC Reset\n", __func__);
				if (!ts->reset_is_on_going)
					schedule_delayed_work(&ts->reset_work, msecs_to_jiffies(10));
			}
		} else if (p_event_status->status_id == STM_TS_INFO_WET_MODE) {
			ts->plat_data->wet_mode = p_event_status->status_data_1;
			input_info(true, &ts->client->dev, "%s: water wet mode %d\n",
				__func__, ts->plat_data->wet_mode);
			if (ts->plat_data->wet_mode)
				ts->plat_data->hw_param.wet_count++;
		} else if (p_event_status->status_id == STM_TS_INFO_NOISE_MODE) {
			ts->plat_data->touch_noise_status = (p_event_status->status_data_1 >> 4);

			input_info(true, &ts->client->dev, "%s: NOISE MODE %s[%02X]\n",
					__func__, ts->plat_data->touch_noise_status == 0 ? "OFF" : "ON",
					p_event_status->status_data_1);

			if (ts->plat_data->touch_noise_status)
				ts->plat_data->hw_param.noise_count++;
		}
	} else if (p_event_status->stype == STM_TS_EVENT_STATUSTYPE_VENDORINFO) {
		if (ts->plat_data->support_ear_detect) {
			if (p_event_status->status_id == 0x6A) {
				ts->hover_event = p_event_status->status_data_1;
				input_report_abs(ts->plat_data->input_dev_proximity, ABS_MT_CUSTOM, p_event_status->status_data_1);
				input_sync(ts->plat_data->input_dev_proximity);
				input_info(true, &ts->client->dev, "%s: proximity: %d\n", __func__, p_event_status->status_data_1);
			}
		}
	}
}

static int stm_ts_get_event(struct stm_ts_data *ts, u8 *data, int *remain_event_count)
{
	int ret = 0;
	u8 address = 0;

	address = STM_TS_READ_ONE_EVENT;
	ret = ts->stm_ts_i2c_read(ts, &address, 1, (u8 *)data, STM_TS_EVENT_BUFF_SIZE);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: i2c read one event failed\n", __func__);
		return ret;
	}

	if (ts->debug_flag & SEC_TS_DEBUG_PRINT_ONEEVENT)
		input_info(true, &ts->client->dev, "ONE: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1],
				data[2], data[3],
				data[4], data[5],
				data[6], data[7]);

	if (data[0] == 0) {
		input_info(true, &ts->client->dev, "%s: event buffer is empty\n", __func__);
		return SEC_ERROR;
	}

	*remain_event_count = data[7] & 0x1F;

	if (*remain_event_count > MAX_EVENT_COUNT - 1) {
		input_err(true, &ts->client->dev, "%s: event buffer overflow\n", __func__);
		address = STM_TS_CMD_CLEAR_ALL_EVENT;
		ret = ts->stm_ts_i2c_write(ts, &address, 1, NULL, 0); //guide
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: i2c write clear event failed\n", __func__);

		stm_ts_release_all_finger(ts);

		return SEC_ERROR;
	}

	if (*remain_event_count > 0) {
		address = STM_TS_READ_ALL_EVENT;
		ret = ts->stm_ts_i2c_read(ts, &address, 1, &data[1 * STM_TS_EVENT_BUFF_SIZE],
				 (STM_TS_EVENT_BUFF_SIZE) * (*remain_event_count));
		if (ret < 0) {
			input_err(true, &ts->client->dev, "%s: i2c read one event failed\n", __func__);
			return ret;
		}
	}

	return SEC_SUCCESS;
}

irqreturn_t stm_ts_irq_thread(int irq, void *ptr)
{
	struct stm_ts_data *ts = (struct stm_ts_data *)ptr;
	int ret;
	u8 event_id;
	u8 read_event_buff[MAX_EVENT_COUNT * STM_TS_EVENT_BUFF_SIZE] = {0};
	u8 *event_buff;
	int curr_pos;
	int remain_event_count;
	ret = event_id = curr_pos = remain_event_count = 0;

#if IS_ENABLED(CONFIG_INPUT_SEC_SECURE_TOUCH)
	if (secure_filter_interrupt(ts) == IRQ_HANDLED) {
		wait_for_completion_interruptible_timeout(&ts->secure_interrupt,
				msecs_to_jiffies(5 * MSEC_PER_SEC));

		input_info(true, &ts->client->dev,
				"%s: secure interrupt handled\n", __func__);

		return IRQ_HANDLED;
	}
#endif
#if IS_ENABLED(CONFIG_SAMSUNG_TUI)
	if (STUI_MODE_TOUCH_SEC & stui_get_mode())
		return IRQ_HANDLED;
#endif

	ret = sec_input_handler_start(ts->client);
	if (ret < 0)
		return IRQ_HANDLED;

	ret = stm_ts_get_event(ts, read_event_buff, &remain_event_count);
	if (ret < 0)
		return IRQ_HANDLED;

	mutex_lock(&ts->eventlock);

	do {
		event_buff = &read_event_buff[curr_pos * STM_TS_EVENT_BUFF_SIZE];
		event_id = event_buff[0] & 0x3;
		if (ts->debug_flag & SEC_TS_DEBUG_PRINT_ALLEVENT)
			input_info(true, &ts->client->dev, "ALL: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					event_buff[0], event_buff[1], event_buff[2], event_buff[3],
					event_buff[4], event_buff[5], event_buff[6], event_buff[7]);

		if (event_id == STM_TS_STATUS_EVENT)
			stm_ts_status_event(ts, event_buff);
		else if (event_id == STM_TS_COORDINATE_EVENT)
			stm_ts_coordinate_event(ts, event_buff);
		else if (event_id == STM_TS_GESTURE_EVENT)
			stm_ts_gesture_event(ts, event_buff);
		else if (event_id == STM_TS_VENDOR_EVENT)
			input_info(true, &ts->client->dev,
				"%s: %s event %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
				__func__, (event_buff[1] == 0x01 ? "echo": ""), event_buff[0], event_buff[1], event_buff[2], event_buff[3], event_buff[4], event_buff[5],
				event_buff[6], event_buff[7], event_buff[8], event_buff[9], event_buff[10], event_buff[11],
				event_buff[12], event_buff[13], event_buff[14], event_buff[15]);
		else
			input_info(true, &ts->client->dev,
					"%s: unknown event %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
						__func__, event_buff[0], event_buff[1], event_buff[2], event_buff[3], event_buff[4], event_buff[5],
						event_buff[6], event_buff[7], event_buff[8], event_buff[9], event_buff[10], event_buff[11],
						event_buff[12], event_buff[13], event_buff[14], event_buff[15]);
		curr_pos++;
		remain_event_count--;
	} while (remain_event_count >= 0);

	stm_ts_external_func(ts);

	mutex_unlock(&ts->eventlock);

	return IRQ_HANDLED;
}

int stm_ts_input_open(struct input_dev *dev)
{
	struct stm_ts_data *ts = input_get_drvdata(dev);
	int ret;

	if (!ts->info_work_done) {
		input_err(true, &ts->client->dev, "%s not finished info work\n", __func__);
		return 0;
	}

	mutex_lock(&ts->modechange);

	ts->plat_data->enabled = true;
	ts->plat_data->prox_power_off = 0;

#if IS_ENABLED(CONFIG_INPUT_SEC_SECURE_TOUCH)
	secure_touch_stop(ts, 0);
#endif

	if (ts->plat_data->power_state == SEC_INPUT_STATE_LPM) {
		ts->plat_data->lpmode(ts, TO_TOUCH_MODE);
		sec_input_set_grip_type(ts->client, ONLY_EDGE_HANDLER);
	} else {
		ret = ts->plat_data->start_device(ts);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to start device\n", __func__);
	}

	if (ts->fix_active_mode) 
		stm_ts_fix_active_mode(ts, true);

	sec_input_set_temperature(ts->client, SEC_INPUT_SET_TEMPERATURE_FORCE);

	mutex_unlock(&ts->modechange);

	cancel_delayed_work(&ts->work_print_info);
	ts->plat_data->print_info_cnt_open = 0;
	ts->plat_data->print_info_cnt_release = 0;
	if (!ts->plat_data->shutdown_called)
		schedule_work(&ts->work_print_info.work);
	return 0;
}

void stm_ts_input_close(struct input_dev *dev)
{
	struct stm_ts_data *ts = input_get_drvdata(dev);

	if (!ts->info_work_done) {
		input_err(true, &ts->client->dev, "%s not finished info work\n", __func__);
		return;
	}
	if (ts->plat_data->shutdown_called) {
		input_err(true, &ts->client->dev, "%s shutdown was called\n", __func__);
		return;
	}

	mutex_lock(&ts->modechange);

	ts->plat_data->enabled = false;

#ifdef TCLM_CONCEPT
	sec_tclm_debug_info(ts->tdata);
#endif
	cancel_delayed_work(&ts->work_print_info);
	sec_input_print_info(ts->client, ts->tdata);
#if IS_ENABLED(CONFIG_INPUT_SEC_SECURE_TOUCH)
	secure_touch_stop(ts, 1);
#endif
#if IS_ENABLED(CONFIG_SAMSUNG_TUI)
	stui_cancel_session();
#endif

	cancel_delayed_work(&ts->reset_work);

	if (ts->plat_data->lowpower_mode || ts->plat_data->ed_enable || ts->plat_data->pocket_mode || ts->plat_data->fod_lp_mode)
		ts->plat_data->lpmode(ts, TO_LOWPOWER_MODE);
	else
		ts->plat_data->stop_device(ts);

	mutex_unlock(&ts->modechange);
}

int stm_ts_stop_device(void *data)
{
	struct stm_ts_data *ts = (struct stm_ts_data *)data;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	mutex_lock(&ts->device_mutex);

	if (ts->plat_data->power_state == SEC_INPUT_STATE_POWER_OFF) {
		input_err(true, &ts->client->dev, "%s: already power off\n", __func__);
		goto out;
	}

	disable_irq(ts->irq);

	ts->plat_data->power_state = SEC_INPUT_STATE_POWER_OFF;

	stm_ts_locked_release_all_finger(ts);

	ts->plat_data->power(ts->client, false);
	ts->plat_data->pinctrl_configure(ts->client, false);

out:
	mutex_unlock(&ts->device_mutex);
	return 0;
}

int stm_ts_start_device(void *data)
{
	struct stm_ts_data *ts = (struct stm_ts_data *)data;
	int ret = -1;
	u8 address = 0;

	input_info(true, &ts->client->dev, "%s\n", __func__);

	ts->plat_data->pinctrl_configure(ts->client, true);

	mutex_lock(&ts->device_mutex);

	if (ts->plat_data->power_state == SEC_INPUT_STATE_POWER_ON) {
		input_err(true, &ts->client->dev, "%s: already power on\n", __func__);
		goto out;
	}

	stm_ts_locked_release_all_finger(ts);

	ts->plat_data->power(ts->client, true);

	sec_delay(20);

	ts->plat_data->power_state = SEC_INPUT_STATE_POWER_ON;
	ts->plat_data->touch_noise_status = 0;

	ret = stm_ts_wait_for_ready(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev,
				"%s: Failed to wait_for_ready\n", __func__);
		goto err;
	}

	ret = stm_ts_read_chip_id(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: Failed to read chip id\n", __func__);
		return ret;
	}

	ts->plat_data->init(ts);

err:
	/* Sense_on */
	address = STM_TS_CMD_SENSE_ON;
	ret = ts->stm_ts_i2c_write(ts, &address, 1, NULL, 0);
	if (ret < 0)
		input_err(true, &ts->client->dev, "%s: fail to write Sense_on\n", __func__);

	enable_irq(ts->irq);
out:
	mutex_unlock(&ts->device_mutex);
	return ret;
}

static int stm_ts_hw_init(struct i2c_client *client)
{
	struct stm_ts_data *ts = i2c_get_clientdata(client);
	int ret = 0;
	int retry = 3;
	u8 reg[3] = { 0 };
	u8 data[STM_TS_EVENT_BUFF_SIZE] = { 0 };
	ts->plat_data->pinctrl_configure(ts->client, true);

	ts->plat_data->power(ts->client, true);
	if (!ts->plat_data->regulator_boot_on)
		sec_delay(TOUCH_POWER_ON_DWORK_TIME);

	ts->plat_data->power_state = SEC_INPUT_STATE_POWER_ON;

	ret = ts->stm_ts_i2c_read(ts, &reg[0], 1, data, STM_TS_EVENT_BUFF_SIZE);
	if (ret == -ENOTCONN) {
		return ret;
	}
	
	do {
		ret = stm_ts_fw_corruption_check(ts);
		if (ret == -STM_TS_ERROR_FW_CORRUPTION) {
			ts->plat_data->hw_param.checksum_result = 1;
			break;
		} else if (ret < 0) {
			if (ret == -STM_TS_ERROR_BROKEN_OSC_TRIM) {
				break;
			} else if (ts->plat_data->hw_param.checksum_result) {
				break;
			} else if (ret == -STM_TS_ERROR_TIMEOUT_ZERO) {
				ret = stm_ts_read_chip_id_hw(ts);
				if (ret == STM_TS_NOT_ERROR) {
					ts->plat_data->hw_param.checksum_result = 1;
					input_err(true, &ts->client->dev, "%s: config corruption\n", __func__);
					break;
				}
			}
			stm_ts_systemreset(ts, 20);
		} else {
			break;
		}
	} while (--retry);

	stm_ts_get_version_info(ts);

	if (ret == -STM_TS_ERROR_BROKEN_OSC_TRIM) {
		ret = stm_ts_osc_trim_recovery(ts);
		if (ret < 0)
			input_err(true, &ts->client->dev, "%s: Failed to recover osc trim\n", __func__);
	}

	if (ts->plat_data->hw_param.checksum_result) {
		ts->fw_version_of_ic = 0;
		ts->config_version_of_ic = 0;
		ts->fw_main_version_of_ic = 0;
	}

	ret = stm_ts_read_chip_id(ts);
	if (ret < 0) {
		stm_ts_systemreset(ts, 500);	/* Delay to discharge the IC from ESD or On-state.*/
		input_err(true, &ts->client->dev, "%s: Reset caused by chip id error\n", __func__);
		stm_ts_read_chip_id(ts);
	}

	ret = stm_ts_fw_update_on_probe(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: Failed to firmware update\n",
				__func__);
		return STM_TS_ERROR_FW_UPDATE_FAIL;
	}

	ret = stm_ts_get_channel_info(ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: read failed rc = %d\n", __func__, ret);
		return 1;
	}

	ts->pFrame = kzalloc(ts->rx_count * ts->tx_count* 2 + 1, GFP_KERNEL);
	if (!ts->pFrame)
		return 1;

	ts->cx_data = kzalloc(ts->rx_count * ts->tx_count+ 1, GFP_KERNEL);
	if (!ts->cx_data) {
		kfree(ts->pFrame);
		return 1;
	}

	ts->ito_result = kzalloc(STM_TS_ITO_RESULT_PRINT_SIZE, GFP_KERNEL);
	if (!ts->ito_result) {
		kfree(ts->cx_data);
		kfree(ts->pFrame);
		return 1;
	}

	/* fts driver set functional feature */
	ts->plat_data->touch_count = 0;
	ts->touch_opmode = STM_TS_OPMODE_NORMAL;
	ts->charger_mode = STM_TS_BIT_CHARGER_MODE_NORMAL;

#ifdef TCLM_CONCEPT
	ts->tdata->external_factory = false;
#endif

	ts->plat_data->touch_functions = STM_TS_TOUCHTYPE_DEFAULT_ENABLE;
	stm_ts_set_touch_function(ts);
	sec_delay(10);

	stm_ts_command(ts, STM_TS_CMD_FORCE_CALIBRATION, true);
	stm_ts_command(ts, STM_TS_CMD_CLEAR_ALL_EVENT, true);
	ts->scan_mode = STM_TS_SCAN_MODE_DEFAULT;
	stm_ts_set_scanmode(ts, ts->scan_mode);

	input_info(true, &ts->client->dev, "%s: Initialized\n", __func__);

	stm_ts_init_proc(ts);

	return ret;
}

static int stm_ts_init(struct i2c_client *client)
{
	struct stm_ts_data *ts;
	struct sec_ts_plat_data *pdata;
	struct sec_tclm_data *tdata = NULL;
	int ret = 0;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct sec_ts_plat_data), GFP_KERNEL);

		if (!pdata) {
			ret = -ENOMEM;
			goto error_allocate_pdata;
		}

		client->dev.platform_data = pdata;

		ret = sec_input_parse_dt(client);
		if (ret) {
			input_err(true, &client->dev, "%s: Failed to parse dt\n", __func__);
			goto error_allocate_mem;
		}
		tdata = devm_kzalloc(&client->dev,
				sizeof(struct sec_tclm_data), GFP_KERNEL);
		if (!tdata) {
			ret = -ENOMEM;
			goto error_allocate_tdata;
		}

#ifdef TCLM_CONCEPT
		sec_tclm_parse_dt(client, tdata);
#endif
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			ret = -ENOMEM;
			input_err(true, &client->dev, "%s: No platform data found\n", __func__);
			goto error_allocate_pdata;
		}
	}

	pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(pdata->pinctrl))
		input_err(true, &client->dev, "%s: could not get pinctrl\n", __func__);

	ts = devm_kzalloc(&client->dev, sizeof(struct stm_ts_data), GFP_KERNEL);
	if (!ts) {
		ret = -ENOMEM;
		goto error_allocate_mem;
	}

	ts->client = client;
	ts->plat_data = pdata;
	ts->irq = client->irq;
	ts->stm_ts_i2c_read = stm_ts_i2c_read;
	ts->stm_ts_i2c_write = stm_ts_i2c_write;
	ts->stm_ts_read_sponge = stm_ts_read_from_sponge;
	ts->stm_ts_write_sponge = stm_ts_write_to_sponge;
	ts->stm_ts_systemreset = stm_ts_systemreset;
	ts->stm_ts_command = stm_ts_command;

	ts->plat_data->pinctrl_configure = sec_input_pinctrl_configure;
	ts->plat_data->power = sec_input_power;
	ts->plat_data->start_device = stm_ts_start_device;
	ts->plat_data->stop_device = stm_ts_stop_device;
	ts->plat_data->init = stm_ts_reinit;
	ts->plat_data->lpmode = stm_ts_set_lowpowermode;
	ts->plat_data->set_grip_data = set_grip_data_to_ic;
	ts->plat_data->set_temperature = stm_ts_set_temperature;

	ptsp = &client->dev;

	ts->tdata = tdata;
	if (!ts->tdata) {
		ret = -ENOMEM;
		goto err_null_tdata;
	}

#ifdef TCLM_CONCEPT
	sec_tclm_initialize(ts->tdata);
	ts->tdata->client = ts->client;
	ts->tdata->tclm_read = sec_tclm_data_read;
	ts->tdata->tclm_write = sec_tclm_data_write;
	ts->tdata->tclm_execute_force_calibration = stm_ts_tclm_execute_force_calibration;
	ts->tdata->tclm_parse_dt = sec_tclm_parse_dt;
#endif

	INIT_DELAYED_WORK(&ts->reset_work, stm_ts_reset_work);
	INIT_DELAYED_WORK(&ts->work_read_info, stm_ts_read_info_work);
	INIT_DELAYED_WORK(&ts->work_print_info, stm_ts_print_info_work);
	INIT_DELAYED_WORK(&ts->work_read_functions, stm_ts_get_touch_function);
	mutex_init(&ts->device_mutex);
	mutex_init(&ts->i2c_mutex);
	mutex_init(&ts->eventlock);
	mutex_init(&ts->modechange);
	mutex_init(&ts->sponge_mutex);
	mutex_init(&ts->fn_mutex);

	ts->plat_data->sec_ws = wakeup_source_register(&ts->client->dev, "tsp");
	device_init_wakeup(&client->dev, true);

	init_completion(&ts->plat_data->resume_done);
	complete_all(&ts->plat_data->resume_done);

	i2c_set_clientdata(client, ts);

	ret = sec_input_device_register(client, ts);
	if (ret) {
		input_err(true, &ts->client->dev, "failed to register input device, %d\n", ret);
		goto err_register_input_device;
	}

	ts->plat_data->input_dev->open = stm_ts_input_open;
	ts->plat_data->input_dev->close = stm_ts_input_close;

	ret = stm_ts_fn_init(ts);
	if (ret) {
		input_err(true, &ts->client->dev, "%s: fail to init fn\n", __func__);
		goto err_fn_init;
	}

#if IS_ENABLED(CONFIG_INPUT_SEC_SECURE_TOUCH)
	if (sysfs_create_group(&ts->plat_data->input_dev->dev.kobj, &secure_attr_group) < 0)
		input_err(true, &ts->client->dev, "%s: do not make secure group\n", __func__);
	else
		secure_touch_init(ts);

	sec_secure_touch_register(ts, ts->plat_data->ss_touch_num, &ts->plat_data->input_dev->dev.kobj);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_DUMP_MODE)
	dump_callbacks.inform_dump = dump_tsp_log;
	INIT_DELAYED_WORK(&ts->check_rawdata, stm_ts_check_rawdata);
#endif
	input_info(true, &client->dev, "%s: init resource\n", __func__);

	return 0;

err_fn_init:
err_register_input_device:
	wakeup_source_unregister(ts->plat_data->sec_ws);
err_null_tdata:
error_allocate_mem:
	regulator_put(pdata->dvdd);
	regulator_put(pdata->avdd);
error_allocate_tdata:
error_allocate_pdata:
	input_err(true, &client->dev, "%s: failed(%d)\n", __func__, ret);
	input_log_fix();
	return ret;
}

void stm_ts_release(struct i2c_client *client)
{
	struct stm_ts_data *ts = i2c_get_clientdata(client);

	input_info(true, &ts->client->dev, "%s\n", __func__);

	cancel_delayed_work_sync(&ts->work_read_info);
	cancel_delayed_work_sync(&ts->work_print_info);
	cancel_delayed_work_sync(&ts->work_read_functions);
	cancel_delayed_work_sync(&ts->reset_work);
	flush_delayed_work(&ts->reset_work);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_DUMP_MODE)
	cancel_delayed_work_sync(&ts->check_rawdata);
	dump_callbacks.inform_dump = NULL;
#endif
	stm_ts_fn_remove(ts);

	device_init_wakeup(&client->dev, false);
	wakeup_source_unregister(ts->plat_data->sec_ws);

	ts->plat_data->lowpower_mode = false;
	ts->probe_done = false;

	ts->plat_data->power(ts->client, false);

	regulator_put(ts->plat_data->dvdd);
	regulator_put(ts->plat_data->avdd);
}


int stm_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stm_ts_data *ts;
	int ret = 0;

#if IS_ENABLED(CONFIG_BATTERY_SAMSUNG)
	if (lpcharge == 1) {
		input_info(true, &client->dev,
			"%s: Do not load driver due to : lpm %d\n",	__func__, lpcharge);
		return -ENODEV;
	}
#endif

	input_info(true, &client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		input_err(true, &client->dev, "%s: EIO err!\n", __func__);
		return -EIO;
	}

	ret = stm_ts_init(client);
	if (ret < 0) {
		input_err(true, &client->dev, "%s: fail to init resource\n", __func__);
		return ret;
	}

	ts = i2c_get_clientdata(client);

	ret = stm_ts_hw_init(client);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: fail to init hw\n", __func__);
		stm_ts_release(client);
		return ret;
	}

	stm_ts_get_custom_library(ts);
	stm_ts_set_custom_library(ts);

	input_info(true, &ts->client->dev, "%s: request_irq = %d\n", __func__, client->irq);
	ret = request_threaded_irq(client->irq, NULL, stm_ts_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, STM_TS_I2C_NAME, ts);
	if (ret < 0) {
		input_err(true, &ts->client->dev, "%s: Unable to request threaded irq\n", __func__);
		stm_ts_release(client);
		return ret;
	}

	ts->probe_done = true;
	ts->plat_data->enabled = true;


	input_err(true, &ts->client->dev, "%s: done\n", __func__);
	input_log_fix();

	if (!ts->plat_data->shutdown_called)
		schedule_delayed_work(&ts->work_read_info, msecs_to_jiffies(50));

	return 0;
}

int stm_ts_remove(struct i2c_client *client)
{
	struct stm_ts_data *ts = i2c_get_clientdata(client);

	input_info(true, &ts->client->dev, "%s\n", __func__);

	mutex_lock(&ts->modechange);
	ts->plat_data->shutdown_called = true;
	mutex_unlock(&ts->modechange);

	disable_irq_nosync(ts->client->irq);
	free_irq(ts->client->irq, ts);

	stm_ts_release(client);

	return 0;
}

void stm_ts_shutdown(struct i2c_client *client)
{
	struct stm_ts_data *ts = i2c_get_clientdata(client);

	input_info(true, &ts->client->dev, "%s\n", __func__);

	stm_ts_remove(client);
}


#if IS_ENABLED(CONFIG_PM)
static int stm_ts_pm_suspend(struct device *dev)
{
	struct stm_ts_data *ts = dev_get_drvdata(dev);

	reinit_completion(&ts->plat_data->resume_done);

	return 0;
}

static int stm_ts_pm_resume(struct device *dev)
{
	struct stm_ts_data *ts = dev_get_drvdata(dev);

	complete_all(&ts->plat_data->resume_done);

	return 0;
}
#endif


static const struct i2c_device_id stm_ts_id[] = {
	{ STM_TS_I2C_NAME, 0 },
	{ },
};

#if IS_ENABLED(CONFIG_PM)
static const struct dev_pm_ops stm_ts_dev_pm_ops = {
	.suspend = stm_ts_pm_suspend,
	.resume = stm_ts_pm_resume,
};
#endif

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id stm_ts_match_table[] = {
	{ .compatible = "stm,stm_ts",},
	{ },
};
#else
#define stm_ts_match_table NULL
#endif

static struct i2c_driver stm_ts_driver = {
	.probe		= stm_ts_probe,
	.remove		= stm_ts_remove,
	.shutdown	= stm_ts_shutdown,
	.id_table	= stm_ts_id,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= STM_TS_I2C_NAME,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = stm_ts_match_table,
#endif
#if IS_ENABLED(CONFIG_PM)
		.pm = &stm_ts_dev_pm_ops,
#endif
	},
};

module_i2c_driver(stm_ts_driver);

MODULE_SOFTDEP("pre: acpm-mfd-bus");
MODULE_DESCRIPTION("stm TouchScreen driver");
MODULE_LICENSE("GPL");
