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
#define USBUART_MAX_DATA_LENGTH 16U

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void ResetPacketStruct(BinaryPacket *packet);
static void BinaryCommandParse(BinaryPacket *packet, uint8_t c);
static ReturnTypeDef ValidateBinaryPayload(const BinaryPacket *packet);

static ReturnTypeDef ValidateBinaryPayload(const BinaryPacket *packet) {
  if (packet->data_length < 1 || packet->data_length > USBUART_MAX_DATA_LENGTH) {
    return RET_OUTRANGE;
  }

  switch (packet->command) {
    case CmdGetTelemetry:
    case CmdStartPeriodicSend:
    case CmdStopPeriodicSend:
    case CmdResetPoseEstinate:
    case CmdGetConfigure:
    case CmdDefaultConfigure:
    case CmdSystemReset: {
      if (packet->data_length < 1 || packet->data[0] != 0x00) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdSaveConfigure: {
      if (packet->data_length < 2 || packet->data[0] != 0x12
          || packet->data[1] != 0x34) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdPeripheralEnable: {
      if (packet->data_length < 1
          || (packet->data[0] & (uint8_t) ~0x07U) != 0) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdImuConfigure:
    case CmdGravityCorrConfigure: {
      if (packet->data_length < 1 || packet->data[0] > 1) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdFilterSelect: {
      if (packet->data_length < 1 || packet->data[0] > 5) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdIn0Configure: {
      if (packet->data_length < 2 || packet->data[0] > 2
          || packet->data[1] > 2) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdJumpDfu: {
      if (packet->data_length < 4 || packet->data[0] != 0x12
          || packet->data[1] != 0x34 || packet->data[2] != 0x56
          || packet->data[3] != 0x78) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    default: {
      return RET_OK;
    }
  }
}

/* Global variables ----------------------------------------------------------*/

static void BinaryCommandParse(BinaryPacket *packet, uint8_t c) {
  switch (packet->status) {  // パケットの受信
    case kCheckHeader1: {  // ヘッダ受信
      if (c == HEADER1) {
        // ヘッダを受信したらパケット構造体を初期化します
        ResetPacketStruct(packet);
        packet->status = kCheckHeader2;
        packet->recv_counter = 0;
      }
      break;
    }
    case kCheckHeader2: {  // ヘッダ受信
      if (c == HEADER2) {
        packet->status = kGetCommand;
      }else{
        packet->status = kCheckHeader1;
      }
      break;
    }
    case kGetCommand: {  // コマンド受信
      packet->command = c;
      packet->sum += c;
      packet->status = kGetDataLength;
      break;
    }
    case kGetDataLength: {  // データ長受信
      packet->data_length = c;
      packet->sum += c;

      if (packet->data_length > MAX_DATA_SIZE) {
        ResetPacketStruct(packet);
      } else if (packet->data_length == 0) {
        packet->status = kCompareChecksum1;
      } else {
        packet->status = kGetData;
      }
      break;
    }
    case kGetData: {  // データ受信
      // 64byteまで受信可能
      if (packet->recv_counter < MAX_DATA_SIZE) {
        packet->data[packet->recv_counter++] = c;
        packet->sum += c;

        // データ長を満たしたらチェックサムへ
        if (packet->data_length <= packet->recv_counter) {
          packet->status = kCompareChecksum1;
        }
      } else {
        // 容量を超えたらリセットする
        ResetPacketStruct(packet);
      }
      break;
    }
    case kCompareChecksum1: {  // チェックサム下位バイト受信
      packet->status = kCompareChecksum2;
      packet->sum += c;
      break;
    }
    case kCompareChecksum2: {  // チェックサム上位バイト受信
      ReturnTypeDef ret = RET_OK;
      packet->sum += c << 8;

      // チェックサムの比較
      if (CompareChecksum(packet->sum)) {
        ret = ValidateBinaryPayload(packet);
        if (ret == RET_OK) {
          AcceptReceivedCommand(packet);
        } else {
          CheckCommandErrorFlag(packet, ret);
        }
        MakeCommandError(packet);

        // Ackの送信許可
        SetAcknowledgeCmd(packet);

        // エラービットリセット
        packet->error.bits = 0;
      }
      // パケットリセット
      ResetPacketStruct(packet);
      break;
    }
    default:{
      ResetPacketStruct(packet);
      break;
    }
  }
}

/**
 * @brief  パケット用の構造体を初期化します
 */
static void ResetPacketStruct(BinaryPacket *packet) {
  packet->status = kCheckHeader1;
  packet->command = 0;
  packet->data_length = 0;
  packet->recv_counter = 0;
  packet->sum = 0;
  packet->error.bits = 0;
  memset(packet->data, 0, sizeof(packet->data));
}

void CMD_CheckReceiveData() {
  static BinaryPacket usb_packet={0};
  static BinaryPacket uart_packet={0};

  // USB通信のデータ受信と解析処理
  while (USB_GetDataCount() != 0) {
    uint8_t c = USB_GetChar();

    BinaryCommandParse(&usb_packet, c);
  }

  // UART4通信のデータ受信と解析処理
  while (UART4_GetDataCount() != 0) {
    uint8_t c = UART4_GetChar();

    BinaryCommandParse(&uart_packet, c);
  }

  //CANFD通信のデータ受信と解析処理
  CANFD_ProcessReceivedPackets();
}

void CheckCommandErrorFlag(BinaryPacket *packet, ReturnTypeDef ret){
  // エラーフラグを立てる
  switch (ret) {
    case RET_OUTRANGE: {
      packet->error.out_of_range = true;
      break;
    }

    case RET_NO_CMD: {
      packet->error.no_command = true;
      break;
    }
    case RET_FLASH_ERR: {
      packet->error.flash_error = true;
      break;
    }

    case RET_OK: {
      break;
    }
    default: {
      break;
    }
  }
}

/**
 * @brief  通信エラーフラグを上位PCへ送信する形式に成形
 */
void MakeCommandError(BinaryPacket *packet) {
  if(GetSystemStatus() == FatalError_Status ){
    packet->error.no_imu_error = 1;
  }

  if(IWDG_CheckResetFlag()){
    packet->error.iwdg_error = 1;
  }

  if (packet->error.flash_error) {
    packet->error.flash_error = 1;
  }

  if (packet->error.out_of_range) {
    packet->error.out_of_range = 1;
  }

  if (packet->error.no_command) {
    packet->error.no_command = 1;
  }
}
