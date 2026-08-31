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
extern FDCAN_HandleTypeDef hfdcan1;

/* Private typedef -----------------------------------------------------------*/
typedef struct {
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[64];
} CANFD_RxStoredFrame;

/* Private define ------------------------------------------------------------*/
#define ID_MASK 0x0FF
#define REQUEST_ID_MIN 0x000
#define REQUEST_ID_MAX 0x0FF
#define REQUEST_MAX_DLC 16

#define CANFD_RX_QUEUE_SIZE 8U

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static BinaryPacket packet = { 0 };

static CANFD_RxStoredFrame rx_queue[CANFD_RX_QUEUE_SIZE];
static volatile uint8_t rx_queue_write_index = 0;
static volatile uint8_t rx_queue_read_index = 0;
static volatile uint8_t rx_queue_count = 0;

/* Private function prototypes -----------------------------------------------*/
static ReturnTypeDef CANFD_ValidateFrame(const FDCAN_RxHeaderTypeDef *hdr,
                                         const uint8_t *data);
static ReturnTypeDef CANFD_ValidatePayload(uint8_t command,
                                           uint8_t len,
                                           const uint8_t *data);

static void CANFD_StoreReceivedFrame(const FDCAN_RxHeaderTypeDef *hdr,
                                     const uint8_t *data);
static uint8_t CANFD_PopReceivedFrame(CANFD_RxStoredFrame *frame);
static void CANFD_ExecuteReceivedFrame(const CANFD_RxStoredFrame *frame);

/* Private functions ---------------------------------------------------------*/
static ReturnTypeDef CANFD_ValidateFrame(const FDCAN_RxHeaderTypeDef *hdr,
                                         const uint8_t *data) {
  uint8_t len = FDCAN_GetDataLength(hdr);

  if (hdr->IdType != FDCAN_STANDARD_ID) {
    return RET_OUTRANGE;
  }

  if (hdr->Identifier < REQUEST_ID_MIN || hdr->Identifier > REQUEST_ID_MAX) {
    return RET_OUTRANGE;
  }

  if (hdr->RxFrameType != FDCAN_DATA_FRAME) {
    return RET_OUTRANGE;
  }

  if (hdr->FDFormat != FDCAN_FD_CAN) {
    return RET_OUTRANGE;
  }

  if (hdr->BitRateSwitch != FDCAN_BRS_ON) {
    return RET_OUTRANGE;
  }

  if (len > REQUEST_MAX_DLC) {
    return RET_OUTRANGE;
  }

  return CANFD_ValidatePayload((uint8_t)(hdr->Identifier & ID_MASK), len, data);
}

static ReturnTypeDef CANFD_ValidatePayload(uint8_t command,
                                           uint8_t len,
                                           const uint8_t *data) {
  switch (command) {
    case CmdGetTelemetry:
    case CmdStartPeriodicSend:
    case CmdStopPeriodicSend:
    case CmdResetPoseEstinate:
    case CmdGetConfigure:
    case CmdDefaultConfigure:
    case CmdSystemReset: {
      if (len < 1 || data[0] != 0x00) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdSaveConfigure: {
      if (len < 2 || data[0] != 0x12 || data[1] != 0x34) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdPeripheralEnable: {
      if (len < 1 || (data[0] & (uint8_t)~0x07U) != 0) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdImuConfigure:
    case CmdGravityCorrConfigure: {
      if (len < 1 || data[0] > 1) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdFilterSelect: {
      if (len < 1 || data[0] > 5) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdIn0Configure: {
      if (len < 2 || data[0] > 2 || data[1] > 2) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    case CmdJumpDfu: {
      if (len < 4 || data[0] != 0x12 || data[1] != 0x34 || data[2] != 0x56
          || data[3] != 0x78) {
        return RET_OUTRANGE;
      }
      return RET_OK;
    }

    default: {
      return RET_OK;
    }
  }
}

static void CANFD_StoreReceivedFrame(const FDCAN_RxHeaderTypeDef *hdr,
                                     const uint8_t *data) {
  uint8_t len = FDCAN_GetDataLength(hdr);

  if (len > sizeof(rx_queue[0].data)) {
    return;
  }

  /*
   * キューが満杯の場合は新規受信データを破棄する。
   * 必要であれば、ここで overflow error 用のフラグを立てる。
   */
  if (rx_queue_count >= CANFD_RX_QUEUE_SIZE) {
    return;
  }

  rx_queue[rx_queue_write_index].header = *hdr;
  memset(rx_queue[rx_queue_write_index].data, 0,
         sizeof(rx_queue[rx_queue_write_index].data));
  memcpy(rx_queue[rx_queue_write_index].data, data, len);

  rx_queue_write_index++;
  if (rx_queue_write_index >= CANFD_RX_QUEUE_SIZE) {
    rx_queue_write_index = 0;
  }

  rx_queue_count++;
}

static uint8_t CANFD_PopReceivedFrame(CANFD_RxStoredFrame *frame) {
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  if (rx_queue_count == 0) {
    if (primask == 0U) {
      __enable_irq();
    }
    return 0;
  }

  *frame = rx_queue[rx_queue_read_index];

  rx_queue_read_index++;
  if (rx_queue_read_index >= CANFD_RX_QUEUE_SIZE) {
    rx_queue_read_index = 0;
  }

  rx_queue_count--;

  if (primask == 0U) {
    __enable_irq();
  }

  return 1;
}

static void CANFD_ExecuteReceivedFrame(const CANFD_RxStoredFrame *frame) {
  ReturnTypeDef ret = RET_OK;
  uint8_t command = 0;
  uint8_t len = 0;

  command = (uint8_t)(frame->header.Identifier & ID_MASK);
  len = FDCAN_GetDataLength(&frame->header);

  if (len <= MAX_DATA_SIZE) {
    packet.command = command;
    packet.data_length = len;
    packet.error.bits = 0;
    memset(packet.data, 0, sizeof(packet.data));
    memcpy(packet.data, frame->data, len);

    ret = CANFD_ValidateFrame(&frame->header, frame->data);
    if (ret == RET_OK) {
      AcceptReceivedCommand(&packet);
    } else {
      CheckCommandErrorFlag(&packet, ret);
    }

    MakeCommandError(&packet);
    SetAcknowledgeCmd(&packet);
    packet.error.bits = 0;
  }
}

/* Public functions ----------------------------------------------------------*/
void CANFD_ProcessReceivedPackets(void) {
  CANFD_RxStoredFrame frame;

  while (CANFD_PopReceivedFrame(&frame) != 0U) {
    CANFD_ExecuteReceivedFrame(&frame);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[64];

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                 &rx_header, rx_data) == HAL_OK) {
        CANFD_StoreReceivedFrame(&rx_header, rx_data);
      } else {
        break;
      }
    }
  }
}
