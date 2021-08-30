/*
 * d3231.c
 *
 *  Created on: Aug 1, 2021
 *      Author: makso
 */

#include "d3231.h"
#include "main.h"
#include "microrl_cmd.h"

#define D3231_ADDRESS (0b1101000 << 1)
uint8_t d3231_mem[19];

I2C_HandleTypeDef * hi2c;

void d3231_init(I2C_HandleTypeDef * hi2ci)
{
	hi2c = hi2ci;
}

uint8_t * d3231_get_time(void)
{
	HAL_I2C_Mem_Read(hi2c, D3231_ADDRESS, 0, 1, d3231_mem, 3, 10);
	return d3231_mem;
}

uint8_t * d3231_get_temp(void)
{
	HAL_I2C_Mem_Read(hi2c, D3231_ADDRESS, 0x11, 1, d3231_mem+0x11, 2, 10);
	return d3231_mem+0x11;
}

uint8_t * d3231_get_all(void)
{
	HAL_I2C_Mem_Read(hi2c, D3231_ADDRESS, 0, 1, d3231_mem, 19, 100);
	return d3231_mem;
}

uint8_t d3231_get_A2M2(void)
{
	return d3231_get_all()[0xB];
}

void d3231_set_A2M2(uint8_t data)
{
	HAL_I2C_Mem_Write(hi2c, D3231_ADDRESS, 0xB, 1, &data, 1, 100);
}

void d3231_set(uint8_t * arr, bool date)
{
	HAL_I2C_Mem_Write(hi2c, D3231_ADDRESS, date<<2, 1, arr, 3, 100);
}

