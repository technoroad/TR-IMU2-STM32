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
static IMU_PersistConfig *conf = &gSystem.conf;

/* Private function prototypes -----------------------------------------------*/

ReturnTypeDef CmdGetConfigureFunc(BinaryPacket *packet) {
  return RET_OK;
}

ReturnTypeDef CmdSaveConfigureFunc(BinaryPacket *packet) {
  if (packet->data[0] != 0x12 || packet->data[1] != 0x34) {
    return RET_OUTRANGE;
  }

  SetFlashImuSetting(&gSystem.conf);

  if (SaveSettingsToFlash() == false) {
    return RET_FLASH_ERR;
  }

  return RET_OK;
}

ReturnTypeDef CmdDefaultConfigureFunc(BinaryPacket *packet) {
  // trueでflashの設定値を強制的に初期化します。
  LoadSettingsFromFlash(true);

  return RET_OK;
}

ReturnTypeDef CmdPeripheralEnableFunc(BinaryPacket *packet) {
  ReturnTypeDef ret = RET_OK;

  conf->usb_enable = (packet->data[0] & 0x01) ? true : false;
  conf->fdcan_enable = (packet->data[0] & 0x02) ? true : false;
  conf->uart4_enable = (packet->data[0] & 0x04) ? true : false;
  return ret;
}

ReturnTypeDef CmdImuConfigureFunc(BinaryPacket *packet) {
  conf->read32bit_enable = (packet->data[0] & 0x01) ? true : false;
  return RET_OK;
}

ReturnTypeDef CmdFilterSelectFunc(BinaryPacket *packet) {
  conf->filter_select = packet->data[0];
  return RET_OK;
}

ReturnTypeDef CmdGravityCorrConfigureFunc(BinaryPacket *packet) {
  conf->gravity_corr_enable = packet->data[0];
  return RET_OK;
}

ReturnTypeDef CmdIn0ConfigureFunc(BinaryPacket *packet){
  uint8_t pupd = packet->data[0];
  uint8_t trigger = packet->data[1];

  if(pupd > 2)return RET_OUTRANGE;
  if(trigger > 2)return RET_OUTRANGE;

  conf->in0_pupd = pupd;
  conf->in0_trigger = trigger;

  return RET_OK;
}
