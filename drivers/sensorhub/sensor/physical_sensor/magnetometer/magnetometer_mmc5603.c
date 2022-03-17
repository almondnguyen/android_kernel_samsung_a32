#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include "../../../comm/shub_comm.h"
#include "../../../utility/shub_utility.h"
#include "../../../sensormanager/shub_sensor.h"
#include "../../../sensormanager/shub_sensor_manager.h"

#include "../../magnetometer.h"

#define MMC5603_NAME   "MMC5633"

static void init_mag_mmc5603(void)
{
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;

	data->mag_matrix_len = 27;
	data->cal_data_len = sizeof(struct calibration_data_mmc5603);
}

static void parse_dt_magnetometer_mmc5603(struct device *dev)
{
	struct magnetometer_data *data = get_sensor(SENSOR_TYPE_GEOMAGNETIC_FIELD)->data;
	struct device_node *np = dev->of_node;
	int check_mst_gpio, check_nfc_gpio;
	int value_mst = 0, value_nfc = 0;

	if (of_property_read_u32(np, "mag-mmc5603-position", &data->position)) {
		data->position = 0;
		shub_err("no mag-mmc5603-position, set as 0");
	}

	shub_info("position[%d]", data->position);

	if (of_property_read_u8_array(np, "mag-mmc5603-array", data->mag_matrix, data->mag_matrix_len))
		shub_err("no mag-mmc5603-array, set as 0");

	// check nfc/mst for mag matrix
	check_nfc_gpio = of_get_named_gpio(np, "shub-mag-check-nfc", 0);
	if (check_nfc_gpio >= 0)
		value_nfc = gpio_get_value(check_nfc_gpio);

	check_mst_gpio = of_get_named_gpio(np, "shub-mag-check-mst", 0);
	if (check_mst_gpio >= 0) {
		value_mst = gpio_get_value(check_mst_gpio);
		if (value_mst == 1) {
			shub_info("mag matrix(%d %d) nfc/mst array", value_nfc, value_mst);
			if (of_property_read_u8_array(np, "mag-mmc5603-mst-array", data->mag_matrix,
						      data->mag_matrix_len))
				shub_err("no mag-mmc5603-mst-array");
		} else if (value_nfc == 1) {
			shub_info("mag matrix(%d %d) nfc only array", value_nfc, value_mst);
			if (of_property_read_u8_array(np, "mag-mmc5603-nfc-array", data->mag_matrix,
						      data->mag_matrix_len))
				shub_err("no mag-mmc5603-nfc-array");
		}
	}
}

struct magnetometer_chipset_funcs magnetic_mmc5603_ops = {
	.init = init_mag_mmc5603,
	.parse_dt = parse_dt_magnetometer_mmc5603,
};

struct magnetometer_chipset_funcs *get_magnetic_mmc5603_function_pointer(char *name)
{
	if (strcmp(name, MMC5603_NAME) != 0)
		return NULL;

	return &magnetic_mmc5603_ops;
}
