/*
 * buzzer.c
 *
 *  Created on: Oct 3, 2022
 *      Author: KoSik
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */

#include "main.h"

uint8_t beepFlag = 0;

uint16_t beepFrequency = 1000;
uint16_t beepTime = 0;

void beep (uint16_t frequency, uint16_t time){
	beepFlag = 1;
	beepFrequency = frequency;
	beepTime = time;
}


void beep_callback(TIM_HandleTypeDef *htim, uint32_t channel){
	if(beepFlag == 1){
		TIM3->CCR1 = 9;
		TIM3->PSC = beepFrequency;
		HAL_TIM_PWM_Start(htim, channel);
		HAL_Delay(beepTime);
		HAL_TIM_PWM_Stop(htim, channel);
		beepFlag = 0;
	}
}
