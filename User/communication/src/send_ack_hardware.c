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
#define SEND_BUF_SIZE TLMT_DATA_SIZE + INFO_SIZE

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t send_buf[SEND_BUF_SIZE];
static bool periodic = false;
static bool can_select = false;
static bool usb_select = false;
static bool uart_select = false;

/* Private function prototypes -----------------------------------------------*/
static inline void checksum(AcknowledgePacket *packet);

void CMD_SetPeriodicSend(){
  if(periodic){
    SetPeriodicCmd();
  }
}

void CMD_PeriodicSendEnable(){
  periodic = true;
}

void CMD_PeriodicSendDisable(){
  periodic = false;
}

void CMD_canfd_selector(){
  can_select = true;
}

void CMD_usb_selector(){
  usb_select = true;
}

void CMD_uart_selector(){
  uart_select = true;
}

/**
 * @brief  ACKパケットを送信
 * @note  送信直前にcsumを更新するので他の処理でセットする必要はない
 */
void TransmitAcknowledge(AcknowledgePacket *packet) {
  uint8_t count = 0;

  // csum計算
  checksum(packet);

  // 送信データの作成
  send_buf[count++] = HEADER1;
  send_buf[count++] = HEADER2;
  send_buf[count++] = packet->command;
  send_buf[count++] = packet->data_length;
  for (int i = 0; i < packet->data_length; i++) {
    send_buf[count++] = packet->data[i];
  }
  count += U16ToByte(packet->checksum, &send_buf[count]);

  // 配列の容量に問題なければ送信
  if (count <= SEND_BUF_SIZE) {
    if(usb_select){
      USB_putd(send_buf, count);
    }

    if(can_select){
      CANFD_SendMessage(packet);
    }
    if(uart_select){
      UART4_putd(send_buf, count);
    }
  }
}

/**
 * @brief  ACKパケットのチェックサムを計算します
 */
static inline void checksum(AcknowledgePacket *packet) {
  // データ部分をすべて足す
  uint32_t sum = 0;
  sum += packet->command;
  sum += packet->data_length;
  for (int i = 0; i < packet->data_length; i++) {
    sum += packet->data[i];
  }

  // チェックサムの作成
  packet->checksum = MakeChecksum(sum);
}
