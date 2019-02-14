/*!
 *	@brief Example shows basic application of reading sensor temperature.
 */

#include "bma423.h"
#include "stdio.h"

void delay_ms(uint32_t period_ms);
uint16_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint16_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint16_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
uint16_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], uint16_t rslt);

int main(void)
{
	struct bma4_dev bma;
	uint16_t rslt;
	int32_t get_temp_C = 0;
	int32_t get_temp_F = 0;
	int32_t get_temp_K = 0;
	float actual_temp = 0;

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
	print_rslt("bma423_init status", rslt);

	/* Upload the configuration file to enable the features of the sensor. */
	rslt = bma423_write_config_file(&bma);
	print_rslt("bma423_write_config status", rslt);

	/* Get temperature in degree C */
	rslt = bma4_get_temperature(&get_temp_C, BMA4_DEG, &bma);
	print_rslt("bma4_get_temperature in degree C status", rslt);

	/* Get temperature in degree F */
	rslt = bma4_get_temperature(&get_temp_F, BMA4_FAHREN, &bma);
	print_rslt("bma4_get_temperature in degree F status", rslt);

	/* Get temperature in degree K */
	rslt = bma4_get_temperature(&get_temp_K, BMA4_KELVIN, &bma);
	print_rslt("bma4_get_temperature in degree K status", rslt);

	/* Scale the output to get the actual temperature  */
	actual_temp = (float)get_temp_C / (float)BMA4_SCALE_TEMP;
	printf("Actual temperature in degree celsius is %10.2f degrees C\r\n", actual_temp);
	actual_temp = (float)get_temp_F / (float)BMA4_SCALE_TEMP;
	printf("Actual temperature in degree fahranheit is %10.2f degrees F\r\n", actual_temp);
	actual_temp = (float)get_temp_K / (float)BMA4_SCALE_TEMP;
	printf("Actual temperature in degree kelvin is %10.2f degrees K\r\n", actual_temp);

	return 0;
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
