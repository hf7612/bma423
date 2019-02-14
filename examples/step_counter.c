/*!
 *	@brief Example shows basic setup application of step counter feature.
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
	struct bma4_accel sens_data;
	struct bma4_accel_config accel_conf;
	uint16_t rslt;

	/* Variable to get the step counter output */
	uint32_t step_out = 0;

	/* Variable to get the interrupt status */
	uint16_t int_status = 0;

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

	/* Accelerometer Configuration Setting */
	/* Output data Rate */
	accel_conf.odr = BMA4_OUTPUT_DATA_RATE_100HZ;

	/* G range of the sensor (+/- 2G, 4G, 8G, 16G) */
	accel_conf.range = BMA4_ACCEL_RANGE_2G;

	/* Bandwidth configure number of sensor samples required to average
	 * if value = 2, then 4 samples are averaged
	 * averaged samples = 2^(val(accel bandwidth))
	 * Note1 : More info refer datasheets
	 * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but since the
	 * performance power mode phase is increased, the power consumption will also rise.
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

	/* Enable step counter */
	rslt = bma423_feature_enable(BMA423_STEP_CNTR, 1, &bma);
	print_rslt("bma423_feature_enable status", rslt);

	/* Map the interrupt pin with that of step counter interrupts
	 * Interrupt will  be generated when step activity is generated.
	 */
	rslt = bma423_map_interrupt(BMA4_INTR1_MAP, BMA423_STEP_CNTR_INT, 1, &bma);
	print_rslt("bma423_map_interrupt status", rslt);

	/* Set water-mark level 1 to get interrupt after 20 steps.
	 * Range of step counter interrupt is 0 to 20460(resolution of 20 steps).
	 */
	rslt = bma423_step_counter_set_watermark(1, &bma);
	print_rslt("bma423_step_counter status", rslt);
	printf("Move/perform the walk/step action with the sensor\n");
	while (1) {
		/* Read the interrupt register to check whether step counter interrupt is received. */
		rslt = bma423_read_int_status(&int_status, &bma);

		/* Check if step counter interrupt is triggered */
		if (int_status & BMA423_STEP_CNTR_INT) {
			printf("\nStep counter interrupt received\n");

			/* On interrupt, Get step counter output */
			rslt = bma423_step_counter_output(&step_out, &bma);
			printf("\nThe step counter output is %lu\r\n", step_out);
		}
		bma.delay(100);
	}

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
