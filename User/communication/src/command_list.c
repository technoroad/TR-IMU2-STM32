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
extern System_Status gSystem;
extern CommunicationErrorFlag gCommError;

/**
 * @brief  受信したコマンドごとの処理を実行します
 */
void AcceptReceivedCommand(BinaryPacket *packet) {
  ReturnTypeDef ret = RET_OK;

  // コマンドによって分岐
  switch (packet->command) {
    case CmdGetTelemetry: {
      ret = CmdGetTelemetryFunc(packet);
      break;
    }
    case CmdStartPeriodicSend: {
      ret = CmdStartPeriodicSendFunc(packet);
      break;
    }
    case CmdStopPeriodicSend: {
      ret = CmdStopPeriodicSendFunc(packet);
      break;
    }
    case CmdResetPoseEstinate: {
      ret = CmdResetPoseEstinateFunc(packet);
      break;
    }

    case CmdGetConfigure:{
      ret = CmdGetConfigureFunc(packet);
      break;
    }
    case CmdSaveConfigure: {
      ret = CmdSaveConfigureFunc(packet);
      break;
    }
    case CmdDefaultConfigure: {
      ret = CmdDefaultConfigureFunc(packet);
      break;
    }
    case CmdPeripheralEnable: {
      ret = CmdPeripheralEnableFunc(packet);
      break;
    }
    case CmdImuConfigure:{
      ret = CmdImuConfigureFunc(packet);
      break;
    }
    case CmdFilterSelect:{
      ret = CmdFilterSelectFunc(packet);
      break;
    }
    case CmdGravityCorrConfigure:{
      ret = CmdGravityCorrConfigureFunc(packet);
      break;
    }
    case CmdIn0Configure:{
      ret = CmdIn0ConfigureFunc(packet);
      break;
    }

    case CmdSystemReset: {
      ret = CmdSystemResetFunc(packet);
      break;
    }
    case CmdJumpDfu:{
      ret = CmdJumpDfuFunc(packet);
      break;
    }
    default: {
      ret = RET_NO_CMD;
      break;
    }
  }

  //フラグを立てる
  CheckCommandErrorFlag(packet, ret);
}
