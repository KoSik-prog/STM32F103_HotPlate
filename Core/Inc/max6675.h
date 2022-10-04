/*
 * max6675.h
 *
 *  Created on: Jul 12, 2022
 *      Author: KoSik
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */

#ifndef INC_MAX6675_H_
#define INC_MAX6675_H_

uint8_t max6675_read_temperature(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint16_t *temperature);
uint8_t max6675_get_sensor_status(SPI_HandleTypeDef *hspi);

#endif /* INC_MAX6675_H_ */
