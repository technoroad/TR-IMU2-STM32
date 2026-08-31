/*
 * pwm.h
 *
 *  Created on: 2025/08/12
 *      Author: techno-road
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#if 0 // old
// U5xx 156.25MHz
#define U5_2KHZ_PWM_PRES 4
#define U5_2KHZ_PWM_ARR 15624
#define U5_4KHZ_PWM_PRES 0
#define U5_4KHZ_PWM_ARR 39061
#define U5_8KHZ_PWM_PRES 0
#define U5_8KHZ_PWM_ARR 19530
#elif 1 // new
// U5xx 160MHz
#define U5_2KHZ_PWM_PRES 1
#define U5_2KHZ_PWM_ARR 39999
#define U5_4KHZ_PWM_PRES 0
#define U5_4KHZ_PWM_ARR 39999
#define U5_8KHZ_PWM_PRES 0
#define U5_8KHZ_PWM_ARR 19999
#endif


void PWM_Initialize();
void PWM_Deinitialize();
void PWM_StartOutput();



#endif /* INC_PWM_H_ */
