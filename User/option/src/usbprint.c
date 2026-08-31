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

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SEND_BUFFER_SIZE 4096

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// 送信バッファ
static uint8_t send_buf[SEND_BUFFER_SIZE] = { 0 };
static uint8_t tx_snapshot[SEND_BUFFER_SIZE] = { 0 };
// 送信バッファのライトインデックス
static uint32_t write_idx = 0;

/* Private function prototypes -----------------------------------------------*/
static inline bool PushWriteBuffer(uint8_t *buf, uint32_t length);
static inline bool IsTransmitFree();
static inline void TransmitStart();

/* Global variables ----------------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceFS;

bool USB_IsBufferEmpty() {
  return (write_idx == 0);
}

void USB_SendIfReady() {
  if (IsTransmitFree()) {
    TransmitStart();
  }
}

bool USB_puts(uint8_t *buf) {
  uint32_t length = strlen((char*) buf);
  return PushWriteBuffer(buf, length);
}

bool USB_putd(uint8_t *buf, uint32_t length) {
  return PushWriteBuffer(buf, length);
}

void USB_putn(int32_t num, uint8_t len) {
  char *s = str_putn(num, len);
  USB_puts((uint8_t*) s);
}

void USB_putx(uint32_t num, uint8_t len) {
  char *s = str_putx(num, len);
  USB_puts((uint8_t*) s);
}

void USB_putf(float val, int bp, int ap) {
  USB_puts((uint8_t*) str_putf(val, bp, ap));
}

uint8_t USB_GetChar() {
  return USB_PullChar();
}

static inline void TransmitStart() {
  uint32_t len = write_idx;
  if (len > 0) {
    memcpy(tx_snapshot, send_buf, len);
    write_idx = 0;

    uint8_t ret = CDC_Transmit_FS(tx_snapshot, len);
    if (ret != USBD_OK) {
      if (len <= (SEND_BUFFER_SIZE - write_idx)) {
        memcpy(&send_buf[write_idx], tx_snapshot, len);
        write_idx += len;
      } else {
        // 量が多すぎたら諦める
      }
    }
  }
}

static inline bool IsTransmitFree() {
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*) hUsbDeviceFS
        .pClassData;
    if (hcdc && hcdc->TxState == 0) {
      return true;
    }
  }
  return false;

}

static inline bool PushWriteBuffer(uint8_t *buf, uint32_t length) {
  // データ数チェック
  if ((length + write_idx) > SEND_BUFFER_SIZE) {
    return false;
  }

  // 送信バッファへ入力
  for (int i = 0; i < length; i++) {
    send_buf[write_idx + i] = buf[i];
  }
  write_idx += length;

  return true;
}
