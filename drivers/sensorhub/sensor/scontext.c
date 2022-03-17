#include <linux/slab.h>

#include "../comm/shub_comm.h"
#include "../comm/shub_iio.h"
#include "../utility/shub_utility.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../sensormanager/shub_sensor_type.h"
#include "../sensorhub/shub_device.h"
#include "scontext_cmd.h"

#define BIG_DATA_SIZE       256
#define PRINT_TRUNCATE      6

#define SCONTEXT_NAME_MAX 30

#define SCONTEXT_DATA_LEN   56
#define SCONTEXT_HEADER_LEN 8

#define RESET_REASON_KERNEL_RESET  0x01
#define RESET_REASON_MCU_CRASHED   0x02
#define RESET_REASON_SYSFS_REQUEST 0x03
#define RESET_REASON_HUB_REQUEST   0x04

void shub_scontext_log(const char *func_name, const char *data, int length)
{
	char buf[6];
	char *log_str;
	int log_size, i;

	if (likely(length <= BIG_DATA_SIZE)) {
		log_size = length;
	} else {
		log_size = PRINT_TRUNCATE * 2 + 1;
	}

	log_size = sizeof(buf) * log_size + 1;
	log_str = kzalloc(log_size, GFP_ATOMIC);
	if (unlikely(!log_str)) {
		shub_errf("allocate memory for data log err");
		return;
	}

	for (i = 0; i < length; i++) {
		if (length < BIG_DATA_SIZE || i < PRINT_TRUNCATE || i >= length - PRINT_TRUNCATE) {
			snprintf(buf, sizeof(buf), "0x%x", (unsigned char)data[i]);
			strlcat(log_str, buf, log_size);

			if (i < length - 1) {
				strlcat(log_str, ", ", log_size);
			}
		}
		if (length > BIG_DATA_SIZE && i == PRINT_TRUNCATE) {
			strlcat(log_str, "..., ", log_size);
		}
	}

	shub_info("%s(%d): %s", func_name, length, log_str);
	kfree(log_str);
}

void shub_report_scontext_data(char *data, u32 data_len)
{
	u16 start, end;
	u64 timestamp;
	char type;
	char buf[SCONTEXT_HEADER_LEN + SCONTEXT_DATA_LEN] = {0,};
	struct shub_sensor *sensor;

	shub_scontext_log(__func__, data, data_len);

	start = 0;
	memcpy(buf, &data_len, sizeof(u32));
	timestamp = get_current_timestamp();

	type = data[2];
	sensor = get_sensor((int)(type + SENSOR_TYPE_SS_BASE));
	if (sensor)
		sensor->event_buffer.timestamp = timestamp;

	while (start < data_len) {
		if (start + SCONTEXT_DATA_LEN < data_len) {
			end = start + SCONTEXT_DATA_LEN - 1;
		} else {
			memset(buf + SCONTEXT_HEADER_LEN, 0, SCONTEXT_DATA_LEN);
			end = data_len - 1;
		}

		memcpy(buf + sizeof(data_len), &start, sizeof(u16));
		memcpy(buf + sizeof(data_len) + sizeof(start), &end, sizeof(u16));
		memcpy(buf + SCONTEXT_HEADER_LEN, data + start, end - start + 1);

		shub_report_sensordata(SENSOR_TYPE_SCONTEXT, timestamp, buf,
				       get_sensor(SENSOR_TYPE_SCONTEXT)->report_event_size);

		start = end + 1;
	}
}

void shub_report_scontext_notice_data(char notice)
{
	char notice_buf[4] = {0x02, 0x01, 0x00, 0x00};
	int len = 3;
	struct shub_data_t *shub_data = get_shub_data();

	notice_buf[2] = notice;

	if (notice == SCONTEXT_AP_STATUS_RESET) {
		len = 4;
		if (shub_data->reset_type == RESET_TYPE_KERNEL_SYSFS)
			notice_buf[3] = RESET_REASON_SYSFS_REQUEST;
		else if (shub_data->reset_type == RESET_TYPE_KERNEL_NO_EVENT)
			notice_buf[3] = RESET_REASON_KERNEL_RESET;
		else if (shub_data->reset_type == RESET_TYPE_KERNEL_COM_FAIL)
			notice_buf[3] = RESET_REASON_KERNEL_RESET;
		else if (shub_data->reset_type == RESET_TYPE_HUB_CRASHED)
			notice_buf[3] = RESET_REASON_MCU_CRASHED;
		else if (shub_data->reset_type == RESET_TYPE_HUB_NO_EVENT)
			notice_buf[3] = RESET_REASON_HUB_REQUEST;
	}

	shub_report_scontext_data(notice_buf, len);

	if (notice == SCONTEXT_AP_STATUS_WAKEUP) {
		shub_infof("wake up");
	} else if (notice == SCONTEXT_AP_STATUS_SLEEP) {
		shub_infof("sleep");
	} else if (notice == SCONTEXT_AP_STATUS_RESET) {
		shub_infof("reset");
	} else {
		shub_errf("invalid notice(0x%x)", notice);
	}
}

int convert_ap_status(int command)
{
	int ret = -1;

	switch (command) {
	case SCONTEXT_AP_STATUS_SHUTDOWN:
		ret = AP_SHUTDOWN;
		break;
	case SCONTEXT_AP_STATUS_WAKEUP:
		ret = LCD_ON;
		break;
	case SCONTEXT_AP_STATUS_SLEEP:
		ret = LCD_OFF;
		break;
	case SCONTEXT_AP_STATUS_RESUME:
		ret = AP_RESUME;
		break;
	case SCONTEXT_AP_STATUS_SUSPEND:
		ret = AP_SUSPEND;
		break;
	case SCONTEXT_AP_STATUS_POW_CONNECTED:
		ret = POW_CONNECTED;
		break;
	case SCONTEXT_AP_STATUS_POW_DISCONNECTED:
		ret = POW_DISCONNECTED;
		break;
	case SCONTEXT_AP_STATUS_CALL_IDLE:
		ret = CALL_IDLE;
		break;
	case SCONTEXT_AP_STATUS_CALL_ACTIVE:
		ret = CALL_ACTIVE;
		break;
	}

	return ret;
}

int shub_scontext_send_cmd(const char *buf, int count)
{
	int ret = 0;
	int convert_status = 0;
	struct shub_data_t* shub_data = get_shub_data();

	if (buf[2] < SCONTEXT_AP_STATUS_WAKEUP || buf[2] > SCONTEXT_AP_STATUS_CALL_ACTIVE) {
		shub_errf("INST_LIB_NOTI err(%d)", buf[2]);
		return -EINVAL;
	}

	convert_status = convert_ap_status(buf[2]);
	ret = shub_send_status(convert_status);

	if (buf[2] == SCONTEXT_AP_STATUS_WAKEUP || buf[2] == SCONTEXT_AP_STATUS_SLEEP) {
		shub_data->lcd_status = convert_status;
	}

	if (buf[2] == SCONTEXT_AP_STATUS_SUSPEND || buf[2] == SCONTEXT_AP_STATUS_RESUME) {
		shub_data->ap_status = convert_status;
	}

	return ret;
}

#define SCONTEXT_VALUE_CURRENTSYSTEMTIME              0x0E
#define SCONTEXT_VALUE_PEDOMETER_USERHEIGHT           0x12
#define SCONTEXT_VALUE_PEDOMETER_USERWEIGHT           0x13
#define SCONTEXT_VALUE_PEDOMETER_USERGENDER           0x14
#define SCONTEXT_VALUE_PEDOMETER_INFOUPDATETIME       0x15

int convert_scontext_putvalue_subcmd(int subcmd)
{
	int ret = -1;
	switch (subcmd) {
	case SCONTEXT_VALUE_CURRENTSYSTEMTIME :
		ret = CURRENT_SYSTEM_TIME;
		break;
	case SCONTEXT_VALUE_PEDOMETER_USERHEIGHT :
		ret = PEDOMETER_USERHEIGHT;
		break;
	case SCONTEXT_VALUE_PEDOMETER_USERWEIGHT:
		ret = PEDOMETER_USERWEIGHT;
		break;
	case SCONTEXT_VALUE_PEDOMETER_USERGENDER:
		ret = PEDOMETER_USERGENDER;
		break;
	case SCONTEXT_VALUE_PEDOMETER_INFOUPDATETIME:
		ret = PEDOMETER_INFOUPDATETIME;
		break;
	default:
		ret = subcmd;
	}

	return ret;
}

int convert_scontext_getvalue_subcmd(int subcmd)
{
	int ret = -1;
	switch (subcmd) {
	case SCONTEXT_VALUE_CURRENTSTATUS :
		ret = LIBRARY_CURRENTSTATUS;
		break;
	case SCONTEXT_VALUE_CURRENTSTATUS_BATCH :
		ret = LIBRARY_CURRENTSTATUS_BATCH;
		break;
	case SCONTEXT_VALUE_VERSIONINFO:
		ret = LIBRARY_VERSIONINFO;
		break;
	default:
		ret = subcmd;
	}

	return ret;
}

void get_ss_sensor_name(int type, char *buf, int buf_size)
{
        memset(buf, 0, buf_size);
        switch(type) {
        case SENSOR_TYPE_SS_PEDOMETER :
                strncpy(buf, "pedometer", buf_size);
                break;
        case SENSOR_TYPE_SS_STEP_COUNT_ALERT :
                strncpy(buf, "step count alert", buf_size);
                break;
        case SENSOR_TYPE_SS_AUTO_ROTATION :
                strncpy(buf, "auto rotation", buf_size);
                break;
        case SENSOR_TYPE_SS_SLOCATION :
                strncpy(buf, "slocation", buf_size);
                break;
        case SENSOR_TYPE_SS_MOVEMENT :
                strncpy(buf, "smart alert", buf_size);
                break;
        case SENSOR_TYPE_SS_ACTIVITY_TRACKER :
                strncpy(buf, "activity tracker", buf_size);
                break;
        case SENSOR_TYPE_SS_DPCM :
                strncpy(buf, "dpcm", buf_size);
                break;
        case SENSOR_TYPE_SS_SENSOR_STATUS_CHECK :
                strncpy(buf, "sensor status check", buf_size);
                break;
        case SENSOR_TYPE_SS_ACTIVITY_CALIBRATION :
                strncpy(buf, "activity calibration", buf_size);
                break;
        case SENSOR_TYPE_SS_DEVICE_POSITION :
                strncpy(buf, "device position", buf_size);
                break;
        case SENSOR_TYPE_SS_CHANGE_LOCATION_TRIGGER :
                strncpy(buf, "change location trigger", buf_size);
                break;
        }

        return;
}

int shub_scontext_send_instruction(const char *buf, int count)
{
	char cmd, type, sub_cmd = 0;
	char *buffer = (char *)(buf + 2);
	int length = count - 2;
	char name[SCONTEXT_NAME_MAX] = "";

	if (buf[0] == SCONTEXT_INST_LIBRARY_REMOVE) {
		cmd = CMD_REMOVE;
		type = buf[1] + SENSOR_TYPE_SS_BASE;
		if(type < SENSOR_TYPE_SS_MAX) {
			get_ss_sensor_name(type, name, sizeof(name));
			shub_infof("REMOVE LIB %s, type %d", name, type);
			return disable_sensor(type, buffer, length);
		} else {
			return -EINVAL;
		}
	} else if (buf[0] == SCONTEXT_INST_LIBRARY_ADD) {
		cmd = CMD_ADD;
		type = buf[1] + SENSOR_TYPE_SS_BASE;
		if(type < SENSOR_TYPE_SS_MAX) {
			get_ss_sensor_name(type, name, sizeof(name));
			shub_infof("ADD LIB %s, type %d", name, type);
			return enable_sensor(type, buffer, length);
		} else {
			return -EINVAL;
		}
	} else if (buf[0] == SCONTEXT_INST_LIB_SET_DATA) {
		cmd = CMD_SETVALUE;
		if (buf[1] != SCONTEXT_VALUE_LIBRARY_DATA) {
			type = TYPE_MCU;
			sub_cmd = convert_scontext_putvalue_subcmd(buf[1]);
		} else {
			type = buf[2] + SENSOR_TYPE_SS_BASE;
			sub_cmd = LIBRARY_DATA;
			length = count - 3;
			if (length > 0) {
				buffer = (char *)(buf + 3);
			} else {
				buffer = NULL;
			}
		}
	} else if (buf[0] == SCONTEXT_INST_LIB_GET_DATA) {
		cmd = CMD_GETVALUE;
		type = buf[1] + SENSOR_TYPE_SS_BASE;
		sub_cmd =  convert_scontext_getvalue_subcmd(buf[2]);
		length = count - 3;
		if (length > 0) {
			buffer = (char *)(buf + 3);
		} else {
			buffer = NULL;
		}
	} else {
		shub_errf("0x%x is not supported", buf[0]);
		return -EINVAL;
	}

	return shub_send_command(cmd, type, sub_cmd, buffer, length);
}

void init_scontext_enable_state(void)
{
	int type;

	for (type = SENSOR_TYPE_SS_BASE; type < SENSOR_TYPE_SS_MAX; type++) {
		struct shub_sensor *sensor = get_sensor(type);

		if (sensor) {
			mutex_lock(&sensor->enabled_mutex);
			sensor->enabled_cnt = 0;
			sensor->enabled = false;
			mutex_unlock(&sensor->enabled_mutex);
		}
	}
}

void print_scontext_debug(void)
{
	/* print nothing for debug */
}

void init_scontext(bool en)
{
	struct shub_sensor *sensor = get_sensor(SENSOR_TYPE_SCONTEXT);

	if (!sensor)
		return;

	if (en) {
		strcpy(sensor->name, "scontext_iio");
		sensor->receive_event_size = 0;
		sensor->report_event_size = 64;

		sensor->funcs = (struct sensor_funcs *)kzalloc(sizeof(struct sensor_funcs), GFP_KERNEL);
		sensor->funcs->print_debug = print_scontext_debug;
	}

	return;
}