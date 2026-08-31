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

/* Global variables ----------------------------------------------------------*/
extern System_Status gSystem;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static Filter_Function *filter = &gSystem.filter;

/**
 * @brief  何もしないがACKは欲しい時に送るコマンド
 */
ReturnTypeDef CmdGetTelemetryFunc(BinaryPacket *packet) {
  ReturnTypeDef ret = RET_OK;
  return ret;
}

ReturnTypeDef CmdStartPeriodicSendFunc(BinaryPacket *packet) {
  ReturnTypeDef ret = RET_OK;

  CMD_PeriodicSendEnable();
  return ret;
}

ReturnTypeDef CmdStopPeriodicSendFunc(BinaryPacket *packet) {
  ReturnTypeDef ret = RET_OK;

  CMD_PeriodicSendDisable();
  return ret;
}

ReturnTypeDef CmdResetPoseEstinateFunc(BinaryPacket *packet) {
  ReturnTypeDef ret = RET_OK;

  if(filter->Reset != NULL){
    filter->Reset();
  }
  return ret;
}

ReturnTypeDef CmdSystemResetFunc(BinaryPacket *packet) {
  ReturnTypeDef ret = RET_OK;

  HAL_NVIC_SystemReset();
  return ret;
}

ReturnTypeDef CmdJumpDfuFunc(BinaryPacket *packet) {
  ReturnTypeDef ret = RET_OK;
  if (packet->data[0] != 0x12) {
    return RET_OUTRANGE;
  }
  if (packet->data[1] != 0x34) {
    return RET_OUTRANGE;
  }
  if (packet->data[2] != 0x56) {
    return RET_OUTRANGE;
  }
  if (packet->data[3] != 0x78) {
    return RET_OUTRANGE;
  }

  JumpToBootloader();
  return ret;
}
