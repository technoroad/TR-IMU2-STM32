/*
 The MIT License (MIT)

 Copyright (c) 2023 Techno Road Inc.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

typedef struct{
  GPIO_TypeDef * port;
  uint16_t pin;
}GPIO_PinConfig;

#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET

#define IMU_RESET_ON LL_GPIO_ResetOutputPin(ADIS_nRST_GPIO_Port, ADIS_nRST_Pin)
#define IMU_RESET_OFF LL_GPIO_SetOutputPin(ADIS_nRST_GPIO_Port, ADIS_nRST_Pin)
#define GREEN_ON LL_GPIO_ResetOutputPin(LD2_GPIO_Port,LD2_Pin)
#define GREEN_OFF LL_GPIO_SetOutputPin(LD2_GPIO_Port,LD2_Pin)
#define DEBUG0_PIN_HIGH LL_GPIO_SetOutputPin(DEBUG_PIN0_GPIO_Port,DEBUG_PIN0_Pin)
#define DEBUG0_PIN_LOW LL_GPIO_ResetOutputPin(DEBUG_PIN0_GPIO_Port,DEBUG_PIN0_Pin)
#define TP1_H LL_GPIO_SetOutputPin(TP1_GPIO_Port,TP1_Pin)
#define TP1_L LL_GPIO_ResetOutputPin(TP1_GPIO_Port,TP1_Pin)
#define TP2_H LL_GPIO_SetOutputPin(TP2_GPIO_Port,TP2_Pin)
#define TP2_L LL_GPIO_ResetOutputPin(TP2_GPIO_Port,TP2_Pin)

void GPIO_Write(GPIO_PinConfig *config,GPIO_PinState state);
bool GPIO_Read(GPIO_PinConfig *config);
void GPIO_DeInit(GPIO_PinConfig *config);

void GPIO_SetInput(GPIO_PinConfig *config,uint32_t pull);
void GPIO_SetOutput(GPIO_PinConfig *config, uint32_t outputType,uint32_t speed,uint32_t pull);
void GPIO_SetInputEXTI_NoIRQ(GPIO_PinConfig *config ,uint32_t pull,uint8_t trigger);
bool GPIO_EXTI_WasTriggered(GPIO_PinConfig *config);

#endif /* INC_GPIO_F765_H_ */
