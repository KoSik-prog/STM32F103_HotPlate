/*
 * buzzer.h
 *
 *  Created on: Oct 3, 2022
 *      Author: KoSik
 *		e-mail: kosik84@gmail.com
 *		
 *	   version: 1.0
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

void beep (uint16_t frequency, uint16_t time);
void beep_callback(TIM_HandleTypeDef *htim, uint32_t channel);

#endif /* INC_BUZZER_H_ */
