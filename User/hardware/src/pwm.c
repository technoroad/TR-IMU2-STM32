/*
 * pwm_f765.c
 *
 *  Created on: 2025/08/12
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Global variables ----------------------------------------------------------*/
extern System_Status gSystem;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMx TIM3

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


void PWM_Initialize(){
  uint16_t pres = 0;
  uint16_t arr = 0;

  switch(gSystem.param.conversion_rate){
    case 2000:{
      pres = U5_2KHZ_PWM_PRES;
      arr = U5_2KHZ_PWM_ARR;
      break;
    }
    case 4000:{
      pres = U5_4KHZ_PWM_PRES;
      arr = U5_4KHZ_PWM_ARR;
      break;
    }
    case 8000:{
      pres = U5_8KHZ_PWM_PRES;
      arr = U5_8KHZ_PWM_ARR;
      break;
    }
    default :{
      pres = U5_2KHZ_PWM_PRES;
      arr = U5_2KHZ_PWM_ARR;
      break;
    }
  }

  LL_TIM_SetPrescaler(TIMx,pres);
  LL_TIM_SetAutoReload(TIMx, arr);
  LL_TIM_OC_SetCompareCH3(TIMx, arr / 2);
}

void PWM_Deinitialize(){
  LL_TIM_CC_DisableChannel(TIMx,LL_TIM_CHANNEL_CH3);
  LL_TIM_DisableCounter(TIMx);
}

void PWM_StartOutput(){
  LL_TIM_CC_EnableChannel(TIMx,LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableCounter(TIMx);
}
