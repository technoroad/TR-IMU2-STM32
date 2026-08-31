/*
 The MIT License (MIT)

 Copyright (c) 2026 Techno Road Inc.

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
 
/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;
extern System_Status gSystem;

/**
 * @brief  Set the board from the value of the configuration switch.
 */
void CheckConfigrationSwitch() {
  GPIO_PinConfig DP1 = {DP1_GPIO_Port,DP1_Pin};
  GPIO_PinConfig BRD = {BRD_GPIO_Port,BRD_Pin};

  gSystem.dip.DP1 = !GPIO_Read(&DP1);
  gSystem.dip.BRD = GPIO_Read(&BRD);

  if (gSystem.dip.DP1) {
  }
  if (gSystem.dip.BRD) {
  }

  GPIO_DeInit(&DP1);
  GPIO_DeInit(&BRD);
}
