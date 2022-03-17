#ifndef __SHUB_SENSOR_H_
#define __SHUB_SENSOR_H_

#include "shub_sensor_type.h"

#include <linux/rtc.h>

#define SENSOR_NAME_MAX 40

struct sensor_event {
	union {
		struct { /* accel, gyro, mag */
			s16 x;
			s16 y;
			s16 z;
			u32 gyro_dps;
		} __attribute__((__packed__));
		struct { /*calibrated mag, gyro*/
			s16 cal_x;
			s16 cal_y;
			s16 cal_z;
			u8 accuracy;
		} __attribute__((__packed__));
		struct { /*uncalibrated mag, gyro*/
			s16 uncal_x;
			s16 uncal_y;
			s16 uncal_z;
			s16 offset_x;
			s16 offset_y;
			s16 offset_z;
		} __attribute__((__packed__));
		struct { /* rotation vector */
			s32 quat_a;
			s32 quat_b;
			s32 quat_c;
			s32 quat_d;
			u8 acc_rot;
		} __attribute__((__packed__));
		struct { /* light */
			u32 lux;
			s32 cct;
			u32 raw_lux;
			u16 r;
			u16 g;
			u16 b;
			u16 w;
			u16 a_time;
			u16 a_gain;
			u32 brightness;
		} __attribute__((__packed__));
		struct { /* pressure */
			s32 pressure;
			s16 temperature;
			s32 pressure_cal;
			s32 pressure_sealevel;
		} __attribute__((__packed__));
		struct { /* step detector */
			u8 step_det;
		};
		struct { /* step counter */
			u32 step_diff;
			u64 step_total;
		} __attribute__((__packed__));
		struct { /* proximity */
			u8 prox;
			u16 prox_ex;
		} __attribute__((__packed__));
		struct { /* proximity raw */
			u16 prox_raw[4];
		};
		struct { /* proximity cal */
			u16 prox_cal[2];
		};
		struct { /* significant motion */
			u8 sig_motion;
		};
		struct { /* tilt detector */
			u8 tilt_detector;
		};
		struct { /* pickup gesture */
			u8 pickup_gesture;
		};
		struct { /* light auto brightness */
			s32 ab_lux;
			u8 ab_min_flag;
			u32 ab_brightness;
		} __attribute__((__packed__));
		struct { /* flip cover detector */
			u8 value;
			s32 magX;
			s32 stable_min_max;
			s32 uncal_mag_x;
			s32 uncal_mag_y;
			s32 uncal_mag_z;
			u8 saturation;
		} __attribute__((__packed__));
		struct meta_data_event { /* meta data */
			s32 what;
			s32 sensor;
		} __attribute__((__packed__)) meta_data;
		u8 data[20];
	};
	u64 timestamp;
} __attribute__((__packed__));

struct sensor_spec_t {
	uint8_t uid;
	uint8_t name[15];
	uint8_t vendor;
	uint16_t version;
	uint8_t is_wake_up;
	int32_t min_delay;
	uint32_t max_delay;
	uint16_t max_fifo;
	uint16_t reserved_fifo;
	float resolution;
	float max_range;
	float power;
} __attribute__((__packed__));

struct sensor_funcs {
	int (*sync_status)(void); /* this is called when sensorhub ready to work or reset */
	int (*enable)(void);
	int (*disable)(void);
	int (*batch)(int, int);
	void (*report_event)(void);
	int (*inject_additional_data)(char *, int);
	void (*print_debug)(void);
	int (*parsing_data)(char *, int *);
	int (*set_position)(int);
	int (*get_position)(void);
	int (*init_chipset)(char *);
	int (*open_calibration_file)(void);
};

struct shub_sensor {
	int type;
	char name[SENSOR_NAME_MAX];
	char chipset_name[15];

	bool enabled;
	int enabled_cnt;
	struct mutex enabled_mutex;

	uint32_t sampling_period;
	uint32_t max_report_latency;

	int receive_event_size;
	int report_event_size;

	u64 enable_timestamp;
	u64 disable_timestamp;
	struct rtc_time enable_time;
	struct rtc_time disable_time;

	struct sensor_event event_buffer;

	void *data;
	struct sensor_funcs *funcs;
};

typedef void (*init_sensor)(bool en);

#endif /* __SHUB_SENSOR_H_ */