#include "../comm/shub_comm.h"
#include "../sensormanager/shub_sensor.h"
#include "../sensormanager/shub_sensor_manager.h"
#include "../utility/shub_utility.h"

#include <linux/device.h>

#define PROX_THRESH_HIGH		0
#define PROX_THRESH_LOW			1
#define PROX_THRESH_SIZE		2

#define DEFUALT_HIGH_THRESHOLD		50
#define DEFUALT_LOW_THRESHOLD		35
#define DEFUALT_DETECT_HIGH_THRESHOLD	200
#define DEFUALT_DETECT_LOW_THRESHOLD	190

#define PROX_ADC_BITS_NUM		14

#define PDATA_MIN			0
#define PDATA_MAX			0x0FFF

struct proximity_data {
	struct proximity_chipset_funcs *chipset_funcs;
	void *threshold_data;

	unsigned int prox_raw_avg[4];
	bool is_proxraw_enabled;
	u16 prox_threshold[PROX_THRESH_SIZE];
	int prox_trim;
	bool need_compensation;
};

struct proximity_chipset_funcs {
	void (*init)(struct proximity_data *data);
	void (*init_proximity_variable)(struct proximity_data *data);
	void (*parse_dt)(struct device *dev);
	u8 (*get_proximity_threshold_mode)(void);
	void (*set_proximity_threshold_mode)(u8 mode);
	void (*sync_proximity_state)(struct proximity_data *data);
	void (*pre_enable_proximity)(struct proximity_data *data);
	void (*pre_report_event_proximity)(void);
	int (*open_calibration_file)(void);
};

struct proximity_gp2ap110s_data {
	int prox_setting_mode;
	u16 prox_setting_thresh[2];
	u16 prox_mode_thresh[PROX_THRESH_SIZE];
};

struct proximity_stk3x6x_data {
	u8 prox_cal_mode;
};

void set_proximity_threshold(void);

struct proximity_chipset_funcs *get_proximity_stk3x6x_function_pointer(char *name);
struct proximity_chipset_funcs *get_proximity_gp2ap110s_function_pointer(char *name);
struct proximity_chipset_funcs *get_proximity_stk3328_function_pointer(char *name);

/* YUM NEED TO
struct proximity_chipset_funcs *get_proximity_stk3a5x_function_pointer(char *name);
*/
