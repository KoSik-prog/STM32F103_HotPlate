/*
 * oled_ssd1306.h
 *
 *  Created on: 13.02.2021
 *      Author: KoSik
 */

#ifndef OLED_SSD1306_H_
#define OLED_SSD1306_H_

#include "stm32f1xx_hal.h"
#include "fonts.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_TOTAL_WIDTH 128

typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

void chipSelect(uint8_t lcd_nr, uint8_t param);
void oledWriteSpi(SPI_HandleTypeDef *hspi, unsigned char data);
void oledWriteCmd(SPI_HandleTypeDef *hspi, uint8_t lcd_nr, unsigned char cmd);
void oledWriteData(SPI_HandleTypeDef *hspi, uint8_t lcd_nr, unsigned char data);
void oledInit(SPI_HandleTypeDef *hspi, uint8_t lcd_nr);
void oledDisplayCls(unsigned char fill);
void oledRefresh(SPI_HandleTypeDef *hspi, uint8_t lcd_nr, uint16_t line);
void oledRefreshAll(SPI_HandleTypeDef *hspi);
void oledSetColStart(SPI_HandleTypeDef *hspi, uint8_t lcd_nr);
void oledDrawPoint(uint16_t x, uint16_t y, unsigned char p);
void oledDrawHorizontalBox(int8_t y, uint8_t height, uint8_t colour);
void oledDrawHorizontalLine(int8_t y, uint8_t colour);
char oledWriteChar(uint16_t x, uint16_t y, uint8_t ch, FontDef Font, uint8_t mode);
char oledWriteChar2(uint16_t x, uint16_t y, uint8_t ch, FontDef Font, uint8_t mode);
void oledDispTxt(uint16_t x, uint16_t y, const uint8_t *txt, FontDef Font, uint8_t mode);
void oledDispBigNumber(uint16_t x, uint16_t y, const uint8_t *txt, FontDef Font, uint8_t mode);
void drawCircleSub(int xc, int yc, int x, int y, uint8_t colour);
void oledDrawCircle(uint16_t xc, uint16_t yc, uint8_t r, uint8_t colour);
void oledDrawFillCircle(uint16_t xc, uint16_t yc, uint8_t r, uint8_t colour);
void oled_SetBrightness(SPI_HandleTypeDef *hspi, uint8_t lcd_nr, uint8_t bright);
void oled_SetAllBrightness(SPI_HandleTypeDef *hspi, uint8_t bright);
void oled_TurnOnOff(SPI_HandleTypeDef *hspi, uint8_t onoff);

#endif /* OLED_SSD1306_H_ */
