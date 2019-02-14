/*!
 *	@brief Example shows basic application of reading accelerometer data.
 */

#include "bma423.h"
#include "stdio.h"

/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH (9.80665f)

void delay_ms(uint32_t period_ms);
uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint16_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint16_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], uint16_t rslt);
float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);

int main(void)
{
	struct bma4_dev bma;
	struct bma4_accel sens_data;
	struct bma4_accel_config accel_conf;
	uint16_t rslt;
	float x, y, z;

	bma.delay = delay_ms;

	/* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x18) & VDD for SECONDARY(0x19)) */
	bma.dev_addr = BMA4_I2C_ADDR_PRIMARY;
	bma.bus_read = i2c_reg_read;
	bma.bus_write = i2c_reg_write;
	bma.interface = BMA4_I2C_INTERFACE;

	/* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

	/*
	   bma.dev_addr = 0;
	   bma.bus_read = spi_reg_read;
	   bma.bus_write = spi_reg_write;
	   bma.interface = BMA4_SPI_INTERFACE;
	 */

	/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
	bma.read_write_len = 8;

	/* Sensor initialization */
	rslt = bma423_init(&bma);
	print_rslt("bma423_init", rslt);

	/* Upload the configuration file to enable the features of the sensor. */
	rslt = bma423_write_config_file(&bma);
	print_rslt("bma423_write_config", rslt);

	/* Enable the accelerometer */
	rslt = bma4_set_accel_enable(1, &bma);
	print_rslt("bma4_set_accel_enable status", rslt);

	/* Accelerometer configuration Setting */
	/* Output data Rate */
	accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;

	/* G range of the sensor (+/- 2G, 4G, 8G, 16G) */
	accel_conf.range = BMA4_ACCEL_RANGE_2G;

	/* Bandwidth configure number of sensor samples required to average
	 * if value = 2, then 4 samples are averaged
	 * averaged samples = 2^(val(accel bandwidth))
	 * Note1 : More info refer datasheets
	 * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
	 * since the performance power mode phase is increased, the power consumption will also rise.
	 */
	accel_conf.bandwidth = 2; // or BMA4_ACCEL_NORMAL_AVG4

	/* Enable the filter performance mode where averaging of samples
	 * will be done based on above set bandwidth and ODR.
	 * There are two modes
	 *  0 -> Averaging samples (Default)
	 *	1 -> No averaging
	 * For more info on No Averaging mode refer datasheets.
	 */
	accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

	/* Set the accel configurations */
	rslt = bma4_set_accel_config(&accel_conf, &bma);
	print_rslt("bma4_set_accel_config status", rslt);
	printf("Ax[m/s2], Ay[m/s2], Az[m/s2]\r\n");
	while (1) {
		/* Read the accel data */
		rslt = bma4_read_accel_xyz(&sens_data, &bma);

		/* Converting lsb to meters per seconds square for 12 bit accelerometer at 2G range */
		x = lsb_to_ms2(sens_data.x, 2, bma.resolution);
		y = lsb_to_ms2(sens_data.y, 2, bma.resolution);
		z = lsb_to_ms2(sens_data.z, 2, bma.resolution);

		/* Print the data in m/s2 */
		printf("%.2f, %.2f, %.2f\r\n", x, y, z);

		/* Pause for 10ms, 100Hz output data rate */
		bma.delay(10);
	}

	return 0;
}

/*!	@brief Converts raw sensor values(LSB) to meters per seconds square.
 *
 *	@param[in] val		: Raw sensor value.
 *	@param[in] g_range	: Accel Range selected (2G, 4G, 8G, 16G).
 *	@param[in] bit_width	: Resolution of the sensor.
 *
 *	@return Accel values in meters per second square.
 *
 */
float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
	float half_scale = (float)(1 << bit_width) / 2.0f;

	return GRAVITY_EARTH * val * g_range / half_scale;
}

/*!
 *	@brief Function that creates a mandatory delay required in some of the APIs such as "bma4_write_config_file",
 *		"bma4_write_regs", "bma4_set_accel_config"  and so on.
 *
 *	@param[in] period_ms  : the required wait time in milliseconds.
 *	@return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
	/* Implement the delay routine according to the target machine */
}

/*!
 *	@brief Function for writing the sensor's registers through I2C bus.
 *
 *	@param[in] i2c_addr : sensor I2C address.
 *	@param[in] reg_addr	: Register address.
 *	@param[in] reg_data	: Pointer to the data buffer whose value is to be written.
 *	@param[in] length	: No of bytes to write.
 *
 *	@return Status of execution
 *	@retval 0 -> Success
 *	@retval >0 -> Failure Info
 *
 */
uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the I2C write routine according to the target machine. */
	return 1;
}

/*!
 *	@brief Function for reading the sensor's registers through I2C bus.
 *
 *	@param[in] i2c_addr : Sensor I2C address.
 *	@param[in] reg_addr	: Register address.
 *	@param[out] reg_data	: Pointer to the data buffer to store the read data.
 *	@param[in] length	: No of bytes to read.
 *
 *	@return Status of execution
 *	@retval 0 -> Success
 *	@retval >0 -> Failure Info
 *
 */
uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the I2C read routine according to the target machine. */
	return 1;
}

/*!
 *	@brief Function for writing the sensor's registers through SPI bus.
 *
 *	@param[in] cs			: Chip select to enable the sensor.
 *	@param[in] reg_addr		: Register address.
 *	@param[in] reg_data	: Pointer to the data buffer whose data has to be written.
 *	@param[in] length		: No of bytes to write.
 *
 *	@return Status of execution
 *	@retval 0 -> Success
 *	@retval >0 -> Failure Info
 *
 */
uint16_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the SPI write routine according to the target machine. */
	return 1;
}

/*!
 *	@brief Function for reading the sensor's registers through SPI bus.
 *
 *	@param[in] cs		: Chip select to enable the sensor.
 *	@param[in] reg_addr	: Register address.
 *	@param[out] reg_data	: Pointer to the data buffer to store the read data.
 *	@param[in] length	: No of bytes to read.
 *
 *	@return Status of execution
 *	@retval 0 -> Success
 *	@retval >0 -> Failure Info
 *
 */
uint16_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

	/* Implement the SPI read routine according to the target machine. */
	return 1;
}

/*!
 *	@brief Prints the execution status of the APIs.
 *
 *	@param[in] api_name	: name of the API whose execution status has to be printed.
 *	@param[in] rslt		: error code returned by the API whose execution status has to be printed.
 *
 *	@return void.
 */
void print_rslt(const char api_name[], uint16_t rslt)
{
	if (rslt != BMA4_OK) {
		printf("%s\t", api_name);
		if (rslt & BMA4_E_NULL_PTR) {
			printf("Error [%d] : Null pointer\r\n", rslt);
		} else if (rslt & BMA4_E_CONFIG_STREAM_ERROR) {
			printf("Error [%d] : Invalid configuration stream\r\n", rslt);
		} else if (rslt & BMA4_E_SELF_TEST_FAIL) {
			printf("Error [%d] : Self test failed\r\n", rslt);
		} else if (rslt & BMA4_E_INVALID_SENSOR) {
			printf("Error [%d] : Device not found\r\n", rslt);
		} else {
			/* For more error codes refer "*_defs.h" */
			printf("Error [%d] : Unknown error code\r\n", rslt);
		}
	}
}
