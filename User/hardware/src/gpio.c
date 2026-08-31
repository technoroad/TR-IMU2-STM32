/*
 * gpio.c
 *
 *  Created on: 2024/10/22
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
//#define HAL

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint32_t GPIO_PortToExtiPort(GPIO_TypeDef *port);
static uint32_t GPIO_PinToExtiLine(uint32_t pin);
static uint32_t GPIO_PinToSourceLine(uint32_t pin);

/* Global variables ----------------------------------------------------------*/

void GPIO_Write(GPIO_PinConfig *config,GPIO_PinState state){
#ifdef HAL
  HAL_GPIO_WritePin(config->port,config->pin,state);
#else
  if(state == GPIO_PIN_SET){
    LL_GPIO_SetOutputPin(config->port,config->pin);
  }else if(state == GPIO_PIN_RESET){
    LL_GPIO_ResetOutputPin(config->port,config->pin);
  }
#endif
}

bool GPIO_Read(GPIO_PinConfig *config){
#ifdef HAL
  return HAL_GPIO_ReadPin(config->port,config->pin);
#else
  return LL_GPIO_IsInputPinSet(config->port,config->pin);
#endif
}

void GPIO_DeInit(GPIO_PinConfig *config){
  LL_GPIO_SetPinMode(config->port,config->pin, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinPull(config->port,config->pin, LL_GPIO_PULL_NO);
}

void GPIO_SetInput(GPIO_PinConfig *config,uint32_t pull){
  LL_GPIO_SetPinMode(config->port, config->pin, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(config->port, config->pin, pull);
}

void GPIO_SetOutput(GPIO_PinConfig *config,
                    uint32_t outputType,
                    uint32_t speed,
                    uint32_t pull){
  /* モード：出力 */
  LL_GPIO_SetPinMode(config->port, config->pin, LL_GPIO_MODE_OUTPUT);

  /* 出力タイプ：LL_GPIO_OUTPUT_PUSHPULL / LL_GPIO_OUTPUT_OPENDRAIN */
  LL_GPIO_SetPinOutputType(config->port, config->pin, outputType);

  /* スピード：LL_GPIO_SPEED_FREQ_LOW/MEDIUM/HIGH/VERY_HIGH */
  LL_GPIO_SetPinSpeed(config->port, config->pin, speed);

  /* プル：LL_GPIO_PULL_NO / UP / DOWN */
  LL_GPIO_SetPinPull(config->port, config->pin, pull);
}


void GPIO_SetInputEXTI_NoIRQ(GPIO_PinConfig *config ,uint32_t pull,uint8_t trigger){
  uint32_t port  =GPIO_PortToExtiPort(config->port);
  uint32_t line = GPIO_PinToExtiLine(config->pin);

  /* EXTI: 割り込みは使うが、CPU割り込みは起こさずに割り込みの外でトリガフラグを取得する使い方をする */
  LL_EXTI_EnableIT_0_31(line);
  LL_EXTI_DisableEvent_0_31(line);

  // EXTIをどこのポートに割り当てるかを設定
  LL_EXTI_SetEXTISource(port,GPIO_PinToSourceLine(config->pin));

  /* Trigger */
  LL_EXTI_DisableRisingTrig_0_31(line);
  LL_EXTI_DisableFallingTrig_0_31(line);

  if (trigger & LL_EXTI_TRIGGER_RISING)
    LL_EXTI_EnableRisingTrig_0_31(line);

  if (trigger & LL_EXTI_TRIGGER_FALLING)
    LL_EXTI_EnableFallingTrig_0_31(line);

  LL_GPIO_SetPinPull(config->port, config->pin, pull);
  LL_GPIO_SetPinMode(config->port, config->pin, LL_GPIO_MODE_INPUT);

  /* 念のためクリア */
  LL_EXTI_ClearFallingFlag_0_31(line);
  LL_EXTI_ClearRisingFlag_0_31(line);
}

bool GPIO_EXTI_WasTriggered(GPIO_PinConfig *config){
  uint32_t line = GPIO_PinToExtiLine(config->pin);

  if (LL_EXTI_IsActiveRisingFlag_0_31(line)) {
    LL_EXTI_ClearRisingFlag_0_31(line);
    LL_EXTI_ClearFallingFlag_0_31(line);
    return true;
  }

  if (LL_EXTI_IsActiveFallingFlag_0_31(line)) {
    LL_EXTI_ClearFallingFlag_0_31(line);
    return true;
  }

  return false;
}

static uint32_t GPIO_PortToExtiPort(GPIO_TypeDef *port){
  if (port == GPIOA){
    return LL_EXTI_EXTI_PORTA;
  }
  if (port == GPIOB){
    return LL_EXTI_EXTI_PORTB;
  }
  if (port == GPIOC){
    return LL_EXTI_EXTI_PORTC;
  }
//  if (port == GPIOD) return LL_EXTI_EXTI_PORTD;
//  if (port == GPIOE) return LL_EXTI_EXTI_PORTE;
//  if (port == GPIOF) return LL_EXTI_EXTI_PORTF;
//  if (port == GPIOG) return LL_EXTI_EXTI_PORTG;
//  if (port == GPIOH) return LL_EXTI_EXTI_PORTH;
  // U535で未使用でも将来用
  return LL_EXTI_EXTI_PORTA;
}

static uint32_t GPIO_PinToSourceLine(uint32_t pin)
{
  switch (pin) {
    case LL_GPIO_PIN_0:  return LL_EXTI_EXTI_LINE0;
    case LL_GPIO_PIN_1:  return LL_EXTI_EXTI_LINE1;
    case LL_GPIO_PIN_2:  return LL_EXTI_EXTI_LINE2;
    case LL_GPIO_PIN_3:  return LL_EXTI_EXTI_LINE3;
    case LL_GPIO_PIN_4:  return LL_EXTI_EXTI_LINE4;
    case LL_GPIO_PIN_5:  return LL_EXTI_EXTI_LINE5;
    case LL_GPIO_PIN_6:  return LL_EXTI_EXTI_LINE6;
    case LL_GPIO_PIN_7:  return LL_EXTI_EXTI_LINE7;
    case LL_GPIO_PIN_8:  return LL_EXTI_EXTI_LINE8;
    case LL_GPIO_PIN_9:  return LL_EXTI_EXTI_LINE9;
    case LL_GPIO_PIN_10: return LL_EXTI_EXTI_LINE10;
    case LL_GPIO_PIN_11: return LL_EXTI_EXTI_LINE11;
    case LL_GPIO_PIN_12: return LL_EXTI_EXTI_LINE12;
    case LL_GPIO_PIN_13: return LL_EXTI_EXTI_LINE13;
    case LL_GPIO_PIN_14: return LL_EXTI_EXTI_LINE14;
    case LL_GPIO_PIN_15: return LL_EXTI_EXTI_LINE15;
    default: return 0;
  }
}

static uint32_t GPIO_PinToExtiLine(uint32_t pin)
{
  switch (pin) {
    case LL_GPIO_PIN_0:  return LL_EXTI_LINE_0;
    case LL_GPIO_PIN_1:  return LL_EXTI_LINE_1;
    case LL_GPIO_PIN_2:  return LL_EXTI_LINE_2;
    case LL_GPIO_PIN_3:  return LL_EXTI_LINE_3;
    case LL_GPIO_PIN_4:  return LL_EXTI_LINE_4;
    case LL_GPIO_PIN_5:  return LL_EXTI_LINE_5;
    case LL_GPIO_PIN_6:  return LL_EXTI_LINE_6;
    case LL_GPIO_PIN_7:  return LL_EXTI_LINE_7;
    case LL_GPIO_PIN_8:  return LL_EXTI_LINE_8;
    case LL_GPIO_PIN_9:  return LL_EXTI_LINE_9;
    case LL_GPIO_PIN_10: return LL_EXTI_LINE_10;
    case LL_GPIO_PIN_11: return LL_EXTI_LINE_11;
    case LL_GPIO_PIN_12: return LL_EXTI_LINE_12;
    case LL_GPIO_PIN_13: return LL_EXTI_LINE_13;
    case LL_GPIO_PIN_14: return LL_EXTI_LINE_14;
    case LL_GPIO_PIN_15: return LL_EXTI_LINE_15;
    default: return 0;
  }
}

