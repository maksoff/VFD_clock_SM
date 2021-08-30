/*
 * d3231.h
 *
 *  Created on: Aug 1, 2021
 *      Author: makso
 */

#ifndef INC_D3231_H_
#define INC_D3231_H_

#include "main.h"
void d3231_init(I2C_HandleTypeDef * hi2ci);
uint8_t * d3231_get_time(void);
uint8_t * d3231_get_all(void);
uint8_t * d3231_get_temp(void);

uint8_t d3231_get_A2M2(void);
void d3231_set_A2M2(uint8_t data);

void d3231_set(uint8_t * arr, bool date);

#endif /* INC_D3231_H_ */
