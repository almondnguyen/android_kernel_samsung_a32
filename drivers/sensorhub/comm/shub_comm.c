#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "../debug/shub_debug.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../sensor/scontext.h"
#include "../sensorhub/shub_device.h"
#include "../utility/shub_utility.h"
#include "../vendor/shub_vendor.h"
#include "shub_cmd.h"

#define SHUB_CMD_SIZE        64
#define SHUB_MSG_HEADER_SIZE 13

#define SSP2AP_BYPASS_DATA	0x37
#define SSP2AP_LIBRARY_DATA 	0x01
#define SSP2AP_DEBUG_DATA 	0x03
#define SSP2AP_BIG_DATA 	0x04
#define SSP2AP_META_DATA 	0x05
#define SSP2AP_TIME_SYNC 	0x06
#define SSP2AP_NOTI_RESET 	0x07
#define SSP2AP_GYRO_CAL 	0x08
#define SSP2AP_PROX_THRESH 	0x09
#define SSP2AP_REQ_RESET 	0x0A
#define SSP2AP_MAG_CAL 		0x0B
#define SSP2AP_DUMP_DATA 	0xDD
#define SSP2AP_CALLSTACK 	0x0F
#define SSP2AP_SYSTEM_INFO 	0x31
#define SSP2AP_SENSOR_SPEC	0x41

struct shub_msg {
	u8 cmd;
	u8 type;
	u8 subcmd;
	u16 length;
	u64 timestamp;
	char *buffer;
	bool is_empty_pending_list;
	struct completion *done;
struct list_head list;
} __attribute__((__packed__));

struct mutex comm_mutex;
struct mutex pending_mutex;
struct list_head pending_list;

unsigned int cnt_timeout;
unsigned int cnt_comm_fail;

char shub_cmd_data[SHUB_CMD_SIZE];

static struct shub_msg* make_msg(u8 cmd, u8 type, u8 subcmd, char *send_buf, int send_buf_len)
{
	struct shub_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg) {
		shub_errf("kzalloc error");
		return NULL;
	}

	msg->cmd = cmd;
	msg->type = type;
	msg->subcmd = subcmd;
	msg->length = send_buf_len;
	msg->timestamp = get_current_timestamp();

	if (send_buf != NULL && send_buf_len != 0) {
		msg->buffer = kzalloc(send_buf_len, GFP_KERNEL);
		if (!msg->buffer) {
		kfree(msg);
		msg = NULL;
		shub_errf("kzalloc error");
		return NULL;
		}
		memcpy(msg->buffer, send_buf, send_buf_len);
	}

	return msg;
}

static void clean_msg(struct shub_msg *msg)
{
	if (msg->buffer != NULL) {
		kfree(msg->buffer);
	}
	kfree(msg);
}

static int comm_to_sensorhub(struct shub_msg *msg, int timeout)
{
	int ret;

	mutex_lock(&comm_mutex);
	memcpy(shub_cmd_data, msg, SHUB_MSG_HEADER_SIZE);
	if (msg->length > 0) {
		memcpy(&shub_cmd_data[SHUB_MSG_HEADER_SIZE], msg->buffer, msg->length);
	} else if (msg->length > (SHUB_CMD_SIZE - SHUB_MSG_HEADER_SIZE)) {
		shub_errf("command size is over.");
		mutex_unlock(&comm_mutex);
		return -EINVAL;
	}

	if (!is_shub_working()) {
		shub_errf("sensorhub is not working");
		mutex_unlock(&comm_mutex);
		return -EIO;
	}

	shub_infof("cmd %d type %d subcmd %d send_buf_len %d", msg->cmd, msg->type, msg->subcmd, msg->length);

	ret = sensorhub_comms_write(shub_cmd_data, SHUB_CMD_SIZE, timeout);
	mutex_unlock(&comm_mutex);

	if (ret < 0) {
		bool is_shub_shutdown = !is_shub_working();
		cnt_comm_fail += (is_shub_shutdown)? 0 : 1;
		shub_errf("comm write FAILED. cnt_comm_fail %d , shub_down %d ", cnt_comm_fail, is_shub_shutdown);
		return ret;
	}

	return ret;
}

int shub_send_command(u8 cmd, u8 type, u8 subcmd, char *send_buf, int send_buf_len)
{
	int ret = 0;
	struct shub_msg *msg = make_msg(cmd, type, subcmd, send_buf, send_buf_len);

	if (msg == NULL) return -EINVAL;

	if (comm_to_sensorhub(msg, 0) < 0) {
		shub_errf("comm_to_sensorhub FAILED.");
	}

	kfree(msg);
	return ret;
}

int shub_send_command_wait(u8 cmd, u8 type, u8 subcmd, int timeout,
                           char *send_buf, int send_buf_len, char **receive_buf, int *receive_buf_len)
{
	int ret = 0;
	DECLARE_COMPLETION_ONSTACK(done);
	struct shub_msg *msg = make_msg(cmd, type, subcmd, send_buf, send_buf_len);
	if (msg == NULL) return -EINVAL;

	msg->done = &done;

	mutex_lock(&pending_mutex);
	list_add_tail(&msg->list, &pending_list);
	mutex_unlock(&pending_mutex);

	if (comm_to_sensorhub(msg, timeout) < 0) {
		shub_errf("comm_to_sensorhub FAILED.");

		mutex_lock(&pending_mutex);
		list_del(&msg->list);
		mutex_unlock(&pending_mutex);
		return -EINVAL;
	}

	ret = wait_for_completion_timeout(msg->done, msecs_to_jiffies(timeout));

	if (msg->is_empty_pending_list) {
		shub_errf("is_empty_pending_list %d", msg->is_empty_pending_list);
		msg->is_empty_pending_list = 0;
		return -EINVAL;
	}

	/* when timeout happen */
	if (!ret) {
		bool is_shub_shutdown = !is_shub_working();

		msg->done = NULL;
		mutex_lock(&pending_mutex);
		list_del(&msg->list);
		mutex_unlock(&pending_mutex);
		cnt_timeout += (is_shub_shutdown)? 0 : 1;

		shub_errf("wait_for_completion_timeout TIMEOUT. cnt_timeout %d, shub_down %d !!", cnt_timeout, is_shub_shutdown);
		return -EINVAL;
	}

	if ((receive_buf != NULL) && (receive_buf_len != NULL) && (msg->length != 0)) {
		*receive_buf = kzalloc(msg->length, GFP_KERNEL);
		if (!(*receive_buf)) {
		shub_errf("kzalloc error");
		return -ENOMEM;
		}
		*receive_buf_len = msg->length;
		memcpy(*receive_buf, msg->buffer, msg->length);
	}

	clean_msg(msg);

	return ret;
}

void clean_pending_list()
{
	struct shub_msg *msg, *n;

	shub_infof("");

	mutex_lock(&pending_mutex);
	list_for_each_entry_safe(msg, n, &pending_list, list) {
		list_del(&msg->list);
		if (msg->done != NULL && !completion_done(msg->done)) {
		msg->is_empty_pending_list = 1;
		complete(msg->done);
		}
	}
	mutex_unlock(&pending_mutex);
}

static int parse_dataframe(char *dataframe, int frame_len)
{
	int index;
	struct sensor_event event;
	int ret = 0;
	struct shub_sensor *sensor;
	struct shub_data_t *data = get_shub_data();

	if (!is_shub_working()) {
		shub_infof("ssp shutdown, do not parse");
		return 0;
	}

	// print_dataframe(data, dataframe, frame_len);

	memset(&event, 0, sizeof(event));

	for (index = 0; index < frame_len && (ret == 0);) {
		int cmd = dataframe[index++];
		int reset_type, no_event_type;
		switch (cmd) {
		case SSP2AP_DEBUG_DATA:
			ret = print_mcu_debug(dataframe, &index, frame_len);
			break;
		case SSP2AP_BYPASS_DATA:
			ret = parsing_bypass_data(dataframe, &index, frame_len);
			break;
		case SSP2AP_META_DATA:
			ret = parsing_meta_data(dataframe, &index);
			break;
		case SSP2AP_LIBRARY_DATA:
			ret = parsing_scontext_data(dataframe, &index);
			break;
		case SSP2AP_GYRO_CAL:
			sensor = get_sensor(SENSOR_TYPE_GYROSCOPE);
			if (sensor)
				ret = sensor->funcs->parsing_data(dataframe, &index);
			break;
		case SSP2AP_MAG_CAL:
			sensor = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD);
			if (sensor)
				ret = sensor->funcs->parsing_data(dataframe, &index);
			break;
		case SSP2AP_SYSTEM_INFO:
			ret = print_system_info(dataframe + index, &index);
			break;
		case SSP2AP_SENSOR_SPEC:
			ret = parsing_sensor_spec(dataframe + index, &index);
			break;
		case SSP2AP_NOTI_RESET:
			shub_infof("Reset MSG received from MCU");
			if (data->is_probe_done == true) {
				// queue_refresh_task(data, 0);
			} else {
				shub_infof("skip reset msg");
			}
			break;
		case SSP2AP_REQ_RESET:
			reset_type = dataframe[index++];
			no_event_type = dataframe[index++];
			// if (reset_type == HUB_RESET_REQ_NO_EVENT) {
			shub_infof("Hub request reset[0x%x] No Event type %d", reset_type, no_event_type);
			reset_mcu(RESET_TYPE_HUB_NO_EVENT);
			//}
			break;
		case SSP2AP_PROX_THRESH:
			sensor = get_sensor(SENSOR_TYPE_PROXIMITY);
			if (sensor)
				sensor->funcs->parsing_data(dataframe, &index);
			break;
		default:
			shub_errf("0x%x cmd doesn't support", cmd);
			ret = -1;
			break;
		}
	}

	if (ret < 0) {
		print_dataframe(dataframe, frame_len);
		return ret;
	}

	return ret;
}

void handle_packet(char *packet, int packet_size)
{
	u16 msg_length = 0;
	u8 msg_cmd = 0, msg_subcmd = 0, msg_type = 0;
	char *buffer;

	if(packet_size < SHUB_MSG_HEADER_SIZE) {
		shub_infof("packet size is small/(%s)", packet);
		return;
	}

	msg_cmd = packet[0];
	msg_type = packet[1];
	msg_subcmd = packet[2];
	msg_length = MAKE_WORD(packet[4], packet[3]);

	/*
	shub_infof("cmd %d, type %d, sub_cmd %d, length %d(0x%x, 0x%x)",
			msg_cmd, msg_type, msg_subcmd, msg_length, packet[3], packet[4]);
	*/

	if (msg_length == 0) {
		shub_errf("lengh is zero %d %d %d", msg_cmd, msg_type, msg_subcmd);
		return;
	}

	if (msg_cmd <= CMD_GETVALUE) {
		bool found = false;
		struct shub_msg *msg, *n;

		mutex_lock(&pending_mutex);
		if (!list_empty(&pending_list)) {
			list_for_each_entry_safe(msg, n, &pending_list, list) {

				if ((msg->cmd == msg_cmd) && (msg->type == msg_type) &&
				    (msg->subcmd == msg_subcmd)) {
					list_del(&msg->list);
					found = true;
					break;
				}
			}

			if (!found) {
				shub_errf("%d %d %d - Not match error",  msg_cmd, msg_type, msg_subcmd);
				goto exit;
			}

			if (msg_cmd == CMD_GETVALUE) {
				msg->length = msg_length;
				if (msg->length != 0) {
					if (msg->buffer != NULL) {
						kfree(msg->buffer);
					}
					msg->buffer = kzalloc(msg->length, GFP_KERNEL);
					memcpy(msg->buffer, packet + SHUB_MSG_HEADER_SIZE, msg->length);
				}
			}

			if (msg->done != NULL && !completion_done(msg->done)) {
				complete(msg->done);
			}

		} else {
			shub_errf("List empty error(%d %d %d)", msg_cmd, msg_type, msg_subcmd);
		}
exit:
		mutex_unlock(&pending_mutex);
	} else if (msg_cmd == CMD_REPORT) {
		buffer = kzalloc(msg_length, GFP_KERNEL);
		memcpy(buffer, &packet[SHUB_MSG_HEADER_SIZE], msg_length);
		parse_dataframe(buffer, msg_length);
		kfree(buffer);
	} else {
		shub_infof("msg_cmd does not define. cmd is %d", msg_cmd);
	}
	return;
}

int get_cnt_comm_fail(void)
{
	return cnt_comm_fail;
}

int get_cnt_timeout(void)
{
	return cnt_timeout;
}

int init_comm_to_hub(void)
{
	mutex_init(&comm_mutex);
	mutex_init(&pending_mutex);
	INIT_LIST_HEAD(&pending_list);

	cnt_timeout = 0;
	cnt_comm_fail = 0;

	memset(shub_cmd_data, 0, SHUB_CMD_SIZE);
	return 0;
}

void exit_comm_to_hub(void)
{
	mutex_destroy(&comm_mutex);
	mutex_destroy(&pending_mutex);
	clean_pending_list();
}