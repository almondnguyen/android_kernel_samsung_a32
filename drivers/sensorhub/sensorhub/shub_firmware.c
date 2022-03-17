/*
 *  Copyright (C) 2019, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include "../utility/shub_utility.h"
#include "../comm/shub_comm.h"
#include "../sensormanager/shub_sensor_type.h"
#ifdef CONFIG_SHUB_FIRMWARE_DOWNLOAD
#include "../vendor/shub_firmware_version.h"
#endif

enum _fw_type {
	FW_TYPE_NONE,
	FW_TYPE_BIN,
	FW_TYPE_PUSH,
	FW_TYPE_SPU,
};

enum _fw_type fw_type;
char fw_name[PATH_MAX];
u32 cur_fw_version = 0;

#ifdef CONFIG_SHUB_FIRMWARE_DOWNLOAD
#define SUPPORT_SPU_FW

#define UPDATE_BIN_FILE "shub.bin"

#ifdef SUPPORT_SPU_FW
#define SPU_FW_FILE "/spu/sensorhub/shub_spu.bin"

#define FW_VER_LEN 8
extern long spu_firmware_signature_verify(const char* fw_name, const u8* fw_data, const long fw_size);

static int request_spu_firmware(u8 **fw_buf)
{
	int ret = 0;
	size_t file_size = 0, remaining;
	int offset = 0;
	unsigned int read_size = PAGE_SIZE*10;
	long fw_size = 0;
	struct file *filp = NULL;
	u8 *file_buf;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	shub_infof("");

	filp = filp_open(SPU_FW_FILE, O_RDONLY, 0);
	if(IS_ERR(filp)) {
		shub_infof("filp_open failed %d", PTR_ERR(filp));
		set_fs(old_fs);
		return 0;
	}

	file_size = filp->f_path.dentry->d_inode->i_size;
	if(file_size <= 0) {
		filp_close(filp, NULL);
		set_fs(old_fs);
		return 0;
	}

	file_buf = kvzalloc(file_size, GFP_KERNEL);
	if(file_buf == NULL) {
		shub_errf("file buf kvzalloc error");
		return 0;
	}

	remaining = file_size;

	while (remaining > 0) {
		int ret = 0;
		if (read_size > remaining) {
		read_size = remaining;
		}

		ret = (unsigned int)vfs_read(filp, (char __user *)(file_buf+offset),
					read_size, &filp->f_pos);

		if(ret != read_size) {
		shub_errf("file read fail %d", ret);
		break;
		}

		offset += read_size;
		remaining -= read_size;
		filp->f_pos = offset;
	}

	filp_close(filp, NULL);
	set_fs(old_fs);
	if (ret < 0) {
		shub_errf("file read fail %d", ret);
		kfree(file_buf);
		return 0;
	}

	//check signing
	fw_size = spu_firmware_signature_verify("SENSORHUB", file_buf, file_size);
	if(fw_size < 0) {
		shub_errf("signature verification failed %d", fw_size);
		fw_size = 0;
	} else {
		u32 fw_version = 0;
		char str_ver[9] = "";

		shub_infof("signature verification success %d", fw_size);
		if(fw_size < FW_VER_LEN) {
		shub_errf("fw size is wrong %d", fw_size);
		kfree(file_buf);
		return 0;
		}

		memcpy(str_ver, file_buf + fw_size - FW_VER_LEN, 8);

		ret = kstrtou32(str_ver, 10, &fw_version);
		shub_infof("urgent fw_version %d kernel ver %d", fw_version, SHUB_FIRMWARE_REVISION);

		if(fw_version > SHUB_FIRMWARE_REVISION) {
		fw_size -= FW_VER_LEN;
		shub_infof("use spu fw size %d", fw_size);
		*fw_buf = kvzalloc(fw_size, GFP_KERNEL);
		if(*fw_buf == NULL) {
			shub_errf("fw buf kvzalloc error");
			kfree(file_buf);
			return 0;
		}
		memcpy(*fw_buf, file_buf, fw_size);
		} else {
		fw_size = 0;
		}
	}

	kfree(file_buf);

	return (int)fw_size;
}

static void release_spu_firmware(u8 *fw_buf)
{
	kfree(fw_buf);
}
#endif

int download_sensorhub_firmware(struct device *dev, void * addr)
{
	int ret = 0;
	int fw_size;
	char* fw_buf = NULL;
	const struct firmware *entry = NULL;

	if(addr == NULL)
		return -EINVAL;

#ifdef CONFIG_SHUB_DEBUG
	shub_infof("check push bin file");
	ret = request_firmware(&entry, UPDATE_BIN_FILE, dev);
	if(!ret) {
		fw_type = FW_TYPE_PUSH;
		fw_size = (int)entry->size;
		fw_buf = (char*)entry->data;
	} else
#endif
	{
#ifdef SUPPORT_SPU_FW
		fw_size = request_spu_firmware((u8**) &fw_buf);
		if(fw_size > 0) {
		shub_infof("download spu firmware");
		fw_type = FW_TYPE_SPU;
		} else
#endif
		{
		shub_infof("download %s", fw_name);
		ret = request_firmware(&entry, fw_name, dev);
		if(ret) {
			shub_errf("request_firmware failed %d", ret);
			fw_type = FW_TYPE_NONE;
			release_firmware(entry);
			return -EINVAL;
		}

		fw_type = FW_TYPE_BIN;
		fw_size = (int)entry->size;
		fw_buf = (char*)entry->data;
		}
	}

	shub_infof("fw type %d bin(size:%d) on %lx",fw_type, (int)fw_size, (unsigned long)addr);
	cur_fw_version = 0;

	memcpy(addr, fw_buf, fw_size);
	if(entry) {
		release_firmware(entry);
	}

#ifdef SUPPORT_SPU_FW
	if(fw_type == FW_TYPE_SPU) {
		release_spu_firmware(fw_buf);
	}
#endif
	return 0;
}

int init_shub_firmware(const char *fw)
{
	memcpy(fw_name, fw, strlen(fw));
	shub_infof("fw_name %s", fw_name);
	return 0;
}

void remove_shub_firmware(void)
{
}

int get_kernel_fw_rev(void)
{
	return SHUB_FIRMWARE_REVISION;
}
#endif /* CONFIG_SHUB_FIRMWARE_DOWNLOAD */

int get_firmware_rev(void)
{
	if(!cur_fw_version) {
		int ret;
		u32 result = 0;
		char *buffer = NULL;
		int buffer_length;

		ret = shub_send_command_wait(CMD_GETVALUE, TYPE_MCU, VERSION_INFO, 1000, NULL, 0, &buffer, &buffer_length);
		if (ret < 0)
			shub_errf("transfer fail %d", ret);
		else if (buffer_length != sizeof(result))
			shub_errf("VERSION_INFO length is wrong");
		else
			memcpy(&result, buffer, buffer_length);

		if (buffer != NULL)
			kfree(buffer);

		cur_fw_version = result;
		shub_info("MCU Firm Version %8u", cur_fw_version);
	}

	return cur_fw_version;
}

int get_firmware_type(void)
{
	return fw_type;
}