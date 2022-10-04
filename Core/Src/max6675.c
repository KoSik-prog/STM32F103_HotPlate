/*
 * max6675.c
 *
 *  Created on: Jul 12, 2022
 *      Author: KoSik
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */

#include "stm32f1xx_hal.h"
#include "main.h"

uint8_t max6675_read_temperature(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t *temperature){
	uint8_t RXbuf[2];
	uint16_t status;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 0);
	HAL_SPI_Receive(hspi, RXbuf, 2, 100);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, 1);
	*temperature = ((RXbuf[0] << 8) + RXbuf[1]) >> 5;
	status = (RXbuf[1] & 0b00000100) >> 2;
	return status;
}

uint8_t max6675_get_sensor_status(SPI_HandleTypeDef *hspi){
	uint8_t RXbuf[2];
	HAL_GPIO_WritePin(MAX6675_2_CS_GPIO_Port, MAX6675_2_CS_Pin, 0);
	HAL_SPI_Receive(hspi, RXbuf, 1, 100);
	HAL_GPIO_WritePin(MAX6675_2_CS_GPIO_Port, MAX6675_2_CS_Pin, 1);
	return (RXbuf[1] & 0b00000100) >> 2;
}
