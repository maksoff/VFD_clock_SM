#include "bme280_drv.h"
#include "freertos_inc.h"
#include "semphr.h"
#include "i2c.h"


struct bme280_dev dev;
uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

// seems to be update in last versions, used only for 2ms and 1ms delays
void user_delay_us(uint32_t period, void *intf_ptr)
{
    /*
     * Return control or wait,
     * for a period amount of ~milliseconds~ ms
     */
	osDelay(period/1000);
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
	xSemaphoreTake(muI2CHandle, portMAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, 1, reg_data, len, 100);
	xSemaphoreGive(muI2CHandle);

    return rslt;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    xSemaphoreTake(muI2CHandle, portMAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, 1, (uint8_t*)reg_data, len, 100);
	xSemaphoreGive(muI2CHandle);

    return rslt;
}


int8_t init_bme280_i2c(void)
{
	int8_t rslt = BME280_OK;
	dev.intf_ptr = &dev_addr;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_us = user_delay_us;

	rslt = bme280_init(&dev);
	return rslt;
}
