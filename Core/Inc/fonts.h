/*
 * fonts.h
 *
 *  Created on: 13.02.2021
 *      Author: kosik
 */

#ifndef FONTS_H_
#define FONTS_H_

#include "stm32f1xx_hal.h"

typedef struct {
	const uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef;

extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_14x22;
extern FontDef Font_16x26;
extern FontDef Font_19x32;
extern FontDef Font2_19x32;

#endif /* FONTS_H_ */
