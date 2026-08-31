/*
 * TIM6.h
 *
 *  Created on: 2022/08/19
 *      Author: kobayashi
 */

#ifndef HARDWARE_TIM6_H_
#define HARDWARE_TIM6_H_

void TIM6_Handler();
void TIM6_Initialize(uint16_t us);
void TIM6_Start();
void TIM6_Stop();
void TIM6_AttachEventHandler(UserEventHandlerPtr handler);

#endif /* HARDWARE_TIM6_H_ */
