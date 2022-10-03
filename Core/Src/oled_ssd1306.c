#include "main.h"
#include "fonts.h"
#include "oled_ssd1306.h"

uint8_t act_lcd=0;
uint8_t lcd_line=0;
uint32_t licznik=0;  //testowy
uint8_t oledRefreshActiveFlag=0;


uint8_t DispBuff [SSD1306_HEIGHT/8][SSD1306_TOTAL_WIDTH];



void oledWriteSpi(SPI_HandleTypeDef *hspi, unsigned char data){  // 5 - wszystkie razem
	HAL_SPI_Transmit(hspi, &data, 1,100);
}

void chipSelect(uint8_t lcd_nr, uint8_t param){
	if (lcd_nr == 0){
		HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, param);
	} else if (lcd_nr == 5){
		HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, param);
	}
}

void oledWriteCmd(SPI_HandleTypeDef *hspi, uint8_t lcd_nr, unsigned char cmd){
	chipSelect(lcd_nr, 1);
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, 0);
	chipSelect(lcd_nr, 0);
	oledWriteSpi(hspi, cmd);
	chipSelect(lcd_nr, 1);
}

void oledWriteData(SPI_HandleTypeDef *hspi, uint8_t lcd_nr, unsigned char data){
	chipSelect(lcd_nr, 1);
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, 1);
	chipSelect(lcd_nr, 0);
	oledWriteSpi(hspi, data);
	chipSelect(lcd_nr, 1);
}

void oledInit(SPI_HandleTypeDef *hspi, uint8_t lcd_nr){
	chipSelect(lcd_nr, 1);
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, 0);
	HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, 0);
	HAL_Delay(10);
	HAL_GPIO_WritePin(OLED_RES_GPIO_Port, OLED_RES_Pin, 1);
	oledWriteCmd(hspi, lcd_nr, 0xAE); //wy��cz panel OLED
	oledWriteCmd(hspi, lcd_nr, 0x00); //adres koluny LOW
	oledWriteCmd(hspi, lcd_nr, 0x10); //adres koluny HIGH
	oledWriteCmd(hspi, lcd_nr, 0x40); //adres startu linii
	oledWriteCmd(hspi, lcd_nr, 0x20); //tryb adresowania strony
	oledWriteCmd(hspi, lcd_nr, 0x02);
	oledWriteCmd(hspi, lcd_nr, 0x81); //ustaw kontrast
	oledWriteCmd(hspi, lcd_nr, 0xCF);
	oledWriteCmd(hspi, lcd_nr, 0xA1); //ustaw remapowanie
	oledWriteCmd(hspi, lcd_nr, 0xC0); //kierunek skanowania
	oledWriteCmd(hspi, lcd_nr, 0xA6); //wy�wietlanie bez inwersji
	oledWriteCmd(hspi, lcd_nr, 0xA8); //ustaw multiplex ratio
	oledWriteCmd(hspi, lcd_nr, 0x3F); //1/64
	oledWriteCmd(hspi, lcd_nr, 0xD3); //ustaw display offset
	oledWriteCmd(hspi, lcd_nr, 0x00); //bez offsetu
	oledWriteCmd(hspi, lcd_nr, 0xD5); //ustaw divide ratio/cz�stotliwo��oscylatora
	oledWriteCmd(hspi, lcd_nr, 0x80); //100ramek/sec
	oledWriteCmd(hspi, lcd_nr, 0xD9); //ustaw okres pre charge
	oledWriteCmd(hspi, lcd_nr, 0xF1); //pre charge 15 cykli, discharge 1cykl
	oledWriteCmd(hspi, lcd_nr, 0xDA); //konfiguracja wyprowadze�sterownika
	oledWriteCmd(hspi, lcd_nr, 0x12);
	oledWriteCmd(hspi, lcd_nr, 0xDB); //ustawienie vcomh
	oledWriteCmd(hspi, lcd_nr, 0x40);
	oledWriteCmd(hspi, lcd_nr, 0x8D); //ustawienie Charge Pump
	oledWriteCmd(hspi, lcd_nr, 0x14);
	oledWriteCmd(hspi, lcd_nr, 0xA4); //�pod��czenie� zawarto�ci RAMdo panelu OLED
	oledWriteCmd(hspi, lcd_nr, 0xA6); //wy��czenie inwersji wy�wietlania
	oledWriteCmd(hspi, lcd_nr, 0xAF); //w��cza wy�wietlacz
	oledDisplayCls(0);
	chipSelect(lcd_nr, 0);
}

void oledDisplayCls(unsigned char fill){
	 uint16_t i, j;

	 for (i = 0; i < 8; i ++) {
		 for (j = 0; j < SSD1306_TOTAL_WIDTH; j ++) {
			 DispBuff[i][j] = fill;
		 }
	 }
}

void oledRefreshAll(SPI_HandleTypeDef *hspi){
	if(oledRefreshActiveFlag == 1){
		if (lcd_line > 7) {
			lcd_line = 0;
			chipSelect(act_lcd, 1);
			HAL_SPI_DMAStop(hspi);
			act_lcd++;
		}
		if (act_lcd > 1) {
			act_lcd = 0;
			licznik++;
			oledRefreshActiveFlag=0;
		}
		oledRefresh(hspi, act_lcd, lcd_line);
		lcd_line++;
	}
}

void oledRefresh(SPI_HandleTypeDef *hspi, uint8_t lcd_nr, uint16_t line){
	oledWriteCmd(hspi, lcd_nr, 0xB0 + line);
	oledSetColStart(hspi, lcd_nr);
	chipSelect(lcd_nr, lcd_nr);
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, 1);
	chipSelect(lcd_nr, 0);
	HAL_SPI_Transmit_DMA(hspi, &DispBuff[line][0+(lcd_nr*128)], 128);
}

void oledSetColStart(SPI_HandleTypeDef *hspi, uint8_t lcd_nr){
	oledWriteCmd(hspi, lcd_nr, 0x00); //low
	oledWriteCmd(hspi, lcd_nr, 0x10); //high
}

void oledDrawPoint(uint16_t x, uint16_t y, unsigned char p){
	 uint16_t chPos, chBx, chTemp = 0;

	 if (x > (5*127) || y > 63) return;
	 chPos = 7 - y / 8;
	 chBx = y % 8;
	 chTemp = 1 << (7 - chBx);
	 if (p) {
		 DispBuff[chPos][x] |= chTemp;
	 } else {
		 DispBuff[chPos][x] &= ~chTemp;
	 }
}

void drawCircleSub(int xc, int yc, int x, int y, uint8_t colour){
	oledDrawPoint(xc+x, yc+y, colour);
	oledDrawPoint(xc-x, yc+y, colour);
	oledDrawPoint(xc+x, yc-y, colour);
	oledDrawPoint(xc-x, yc-y, colour);
	oledDrawPoint(xc+y, yc+x, colour);
	oledDrawPoint(xc-y, yc+x, colour);
	oledDrawPoint(xc+y, yc-x, colour);
	oledDrawPoint(xc-y, yc-x, colour);
}

void oledDrawCircle(uint16_t xc, uint16_t yc, uint8_t r, uint8_t colour){
    int x = 0, y = r;
    int d = 3 - 2 * r;
    drawCircleSub(xc, yc, x, y, colour);
    while (y >= x){
        x++;
        if (d > 0)
        {
            y--;
            d = d + 4 * (x - y) + 10;
        }
        else
            d = d + 4 * x + 6;
        drawCircleSub(xc, yc, x, y, colour);
    }
}

void oledDrawFillCircle(uint16_t xc, uint16_t yc, uint8_t r, uint8_t colour){
    for(uint8_t i=r; i>0; i--){
    	oledDrawCircle(xc, yc, i, colour);
    }
    oledDrawPoint(xc, yc, colour);
    oledDrawPoint(xc+1, yc+1, colour);
    oledDrawPoint(xc+1, yc-1, colour);
    oledDrawPoint(xc-1, yc+1, colour);
    oledDrawPoint(xc-1, yc-1, colour);
}

char oledWriteChar(uint16_t x, uint16_t y, uint8_t ch, FontDef Font, uint8_t mode){
	uint32_t i, b, j;

	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth+1; j++)
		{
			if ((b << j) & 0x8000)
			{
				oledDrawPoint(x + j, (y + i), 1);
			}
			else
			{
				oledDrawPoint(x + j, (y + i), 0);
			}
		}
	}
	return ch;
}

char oledWriteChar2(uint16_t px, uint16_t py, uint8_t ch, FontDef Font, uint8_t mode) {
	uint32_t i, b, j;
	uint8_t x = 0, y = 0;

	for (i = 0; i < ((Font.FontHeight * Font.FontWidth) / 8); i++) {
		if (ch >= 48) {
			b = Font.data[(ch - 48) * ((Font.FontHeight * Font.FontWidth) / 8)
					+ i];
		} else {
			b = Font.data[(10) * ((Font.FontHeight * Font.FontWidth) / 8) + i]; //jesli pusty
		}
		for (j = 0; j < 8; j++) {
			if ((b << j) & 0x80) {
				oledDrawPoint(px + x, (py + y), 1);
			} else {
				oledDrawPoint(px + x, (py + y), 0);
			}
			x++;
			if (x > Font.FontWidth - 1) {
				x = 0;
				y++;
			}
		}
	}
	return ch;
}


void oledDispTxt(uint16_t x, uint16_t y, const uint8_t *txt, FontDef Font, uint8_t mode){
	 while (*txt != '\0') {
		 if (x > (SSD1306_TOTAL_WIDTH - Font.FontWidth / 2)) {
			 x = 0; y += Font.FontWidth;
			 if (y > (SSD1306_HEIGHT - Font.FontHeight)) {
				 y = x = 0;
			 }
		 }
		 oledWriteChar(x, y, *txt, Font, mode);
		 x += Font.FontWidth+1;
		 txt ++;
	 }
}

void oledDispBigNumber(uint16_t x, uint16_t y, const uint8_t *txt, FontDef Font, uint8_t mode){
	 while (*txt != '\0') {
		 if (x > (SSD1306_TOTAL_WIDTH - Font.FontWidth / 2)) {
			 x = 0; y += Font.FontWidth;
			 if (y > (SSD1306_HEIGHT - Font.FontHeight)) {
				 y = x = 0;
			 }
		 }
		 oledWriteChar2(x, y, *txt, Font, mode);
		 x += Font.FontWidth+1;
		 txt ++;
	 }
}

void oled_SetBrightness(SPI_HandleTypeDef *hspi, uint8_t lcd_nr, uint8_t bright){
	 oledWriteCmd(hspi, lcd_nr, 0x81);
	 oledWriteCmd(hspi, lcd_nr, bright);
}

void oled_SetAllBrightness(SPI_HandleTypeDef *hspi, uint8_t bright){
	/*for (uint8_t q = 0; q < 5; q++) {
		oledWriteCmd(hspi, q, 0x81);
		oledWriteCmd(hspi, q, bright);
	}*/
	oledWriteCmd(hspi, 5, 0x81);
	oledWriteCmd(hspi, 5, bright);
}

void oled_TurnOnOff(SPI_HandleTypeDef *hspi, uint8_t onoff){
	oledWriteCmd(hspi, 5, 0xAE + onoff);
}
