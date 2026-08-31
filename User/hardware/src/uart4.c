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
#include "linked_list.h"

/* Private typedef -----------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
extern DMA_QListTypeDef UART4Node;
extern DMA_HandleTypeDef handle_GPDMA1_Channel2;
extern UART_HandleTypeDef huart4;

/* Private define ------------------------------------------------------------*/
// DMAの送信完了が起きなかった時に強制的に送信終了させるタイムアウト
#define SEND_TIMOUT_TICK 1000
// 受信バッファのサイズ
#define RECV_BUFFER_SIZE 4096
#define SEND_BUFFER_SIZE 4096

/* Private variables ---------------------------------------------------------*/
// 受信リングバッファ
static uint8_t read_buf[RECV_BUFFER_SIZE] = { 0 };
// 受信バッファのリードインデックス
static int32_t read_idx = 0;
// 送信バッファ
static uint8_t send_buf[SEND_BUFFER_SIZE] = { 0 };
static uint8_t double_buf[SEND_BUFFER_SIZE] = { 0 };
// 送信バッファのライトインデックス
static int32_t write_idx = 0;
// 送信状態のステータス
static bool busy = false;
// タイムアウトのカウンタ
static uint32_t timeout_cnt = 0;
// uartのhalインスタンス
static UART_HandleTypeDef *huart = &huart4;
// RS485用のDE線
static GPIO_PinConfig DE = {RS485_DE_GPIO_Port,RS485_DE_Pin};

/* Private macro -------------------------------------------------------------*/
// 使用するUSART
#define USARTx  huart4.Instance
// 受信用のDMAノード
#define DMA_NODE UART4Node
// 受信用のDMAハンドラ
#define DMA_HANDLE handle_GPDMA1_Channel2
// DMA用のノードを設定する関数
#define NODE_CONFIG MX_UART4Node_Config()

#define RS485_DE_ENABLE GPIO_Write(&DE,HIGH)
#define RS485_DE_DISABLE GPIO_Write(&DE,LOW)

/* Private function prototypes -----------------------------------------------*/
static inline bool PushWriteBuffer(uint8_t *buf, uint32_t length);
static inline void Abort();
static inline void TransmitStart();
static inline uint8_t ErrorCheck();
static inline void BusyTimeout();

void UART4_TxComplete_Handler(UART_HandleTypeDef *huart) {
  if (huart->Instance == USARTx) {  // 自分が使っているUARTをチェック
    // 送信中フラグをクリア
    busy = false;
    RS485_DE_DISABLE;
  }
}

/***
 * @brief  Start receiving data via DMA
 * @note   This function is executed when the microcontroller is powered on
 */
void UART4_Initialize() {
  // Initialize variables
  memset(read_buf, 0, RECV_BUFFER_SIZE);
  memset(send_buf, 0, SEND_BUFFER_SIZE);
  memset(double_buf, 0, SEND_BUFFER_SIZE);
  // Initialize read pointer
  read_idx = 0;
  write_idx = 0;
  // Disable DMA channel
  NODE_CONFIG;
  __HAL_LINKDMA(huart, hdmarx, DMA_HANDLE);
  HAL_DMAEx_List_LinkQ(&DMA_HANDLE, &DMA_NODE);

  HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*) read_buf, RECV_BUFFER_SIZE);

  busy = false;
  RS485_DE_DISABLE;
}

void UART4_Deinitialize(){
  HAL_UART_DMAStop(huart);
  HAL_UART_DeInit(huart);

  GPIO_DeInit(&DE);
}

bool UART4_SendIfReady() {
  BusyTimeout();

  if(write_idx != 0){
    if (!busy && (ErrorCheck() == 0)) {
      TransmitStart();
      return true;
    }
  }

  return false;
}

bool UART4_IsBufferEmpty() {
  return (write_idx == 0);
}

/**
 * @brief  USARTとDMAによる文字列の送信
 * @param[in]  buf : 文字列の先頭ポインタ
 * @return  送信の可否
 */
bool UART4_puts(uint8_t *buf) {
  uint32_t length = strlen((char*) buf);
  return PushWriteBuffer(buf, length);
}

/**
 * @brief  USARTとDMAによる指定データ数の送信
 * @param[in]  buf : 送信データの先頭ポインタ
 * @param[in]  length : データ長
 * @return  送信の可否
 */
bool UART4_putd(uint8_t *buf, uint32_t length) {
  return PushWriteBuffer(buf, length);
}

void UART4_Putn(int32_t num, uint8_t len) {
  char *s = str_putn(num, len);
  UART4_puts((uint8_t*) s);
}

void UART4_Putx(uint32_t num, uint8_t len) {
  char *s = str_putx(num, len);
  UART4_puts((uint8_t*) s);
}

void UART4_Putf(float val, int bp, int ap) {
  char *s = str_putf(val, bp, ap);
  UART4_puts((uint8_t*) s);
}

/**
 * @brief  USART受信バッファからの1文字受信
 * @return  char
 */
uint8_t UART4_GetChar(void) {
  if (UART4_GetDataCount() != 0) {
    uint8_t c = read_buf[read_idx];
    read_idx = (read_idx + 1) % RECV_BUFFER_SIZE;
    return c;
  }
  return -1;
}

uint32_t UART4_GetDataCount(void) {
  uint32_t dma_counter;

  // DMA残り転送数を取得（DMAが進んでいる途中でも安全）
  if (huart->hdmarx != NULL) {
    dma_counter = __HAL_DMA_GET_COUNTER(huart->hdmarx);
  } else {
    return 0;
  }

  // DMAはバッファの最後から逆にカウントしているので変換
  uint32_t new_index = RECV_BUFFER_SIZE - dma_counter;

  // ローカルコピーで競合回避
  uint32_t read_index = read_idx;

  // 未読数を計算（リングバッファ対応）
  if (new_index >= read_index) {
    return new_index - read_index;
  } else {
    return (RECV_BUFFER_SIZE - read_index) + new_index;
  }
}

static inline bool PushWriteBuffer(uint8_t *buf, uint32_t length) {
  // データ数チェック
  if ((write_idx + length) > SEND_BUFFER_SIZE) {
    return false;
  }

  // 送信バッファへ入力
  for (int i = 0; i < length; i++) {
    send_buf[write_idx + i] = buf[i];
  }
  write_idx += length;

  return true;
}

/**
 * @brief  USARTを停止させる関数
 */
static inline void Abort() {
  CLEAR_BIT(
      USARTx->CR1,
      USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE | USART_CR1_TCIE);

  CLEAR_BIT(USARTx->CR3, USART_CR3_EIE);
  CLEAR_BIT(USARTx->CR3, USART_CR3_DMAT);

  // エラーフラグのリセット
  WRITE_REG(USARTx->ICR,
            USART_ICR_PECF|USART_ICR_FECF| USART_ICR_NECF | USART_ICR_ORECF);
}

/**
 * @brief  USARTとDMAの送信設定と送信バッファに格納する関数
 */
static inline void TransmitStart() {
  // 送信状態へ遷移
  busy = true;
  timeout_cnt = 0;
  uint32_t len = write_idx;

  // DE線オン
  RS485_DE_ENABLE;

  // ダブルバッファへ送信するデータをコピー
  memcpy(double_buf, send_buf, len);
  write_idx = 0;

  // 送信完了フラグのリセット
  __HAL_DMA_CLEAR_FLAG(huart->hdmatx, DMA_FLAG_TC);
  HAL_UART_Transmit_DMA(huart, double_buf, len);
}

/***
 * @brief  USARTのエラーチェック関数
 * @note  送信の直前にこの関数を置いてください。
 */
static inline uint8_t ErrorCheck() {
  //ORE,NE,FE,PEエラーフラグの監視
  if (USARTx->ISR & 0xF) {
    uint8_t ret = USARTx->ISR & 0xF;

    Abort();
    UART4_Initialize();
    return ret;
  }
  return 0;
}

/**
 * @brief   DMAの送信状態のタイムアウトの監視
 */
static inline void BusyTimeout() {
  //送信状態の時判定
  if (busy) {
    // 一定時間で送信完了割り込みが起こらなかったら強制的にリセットを行う
    if (timeout_cnt < SEND_TIMOUT_TICK) {
      timeout_cnt++;
    } else {
      timeout_cnt = 0;
      busy = false;
      RS485_DE_DISABLE;
      __HAL_DMA_CLEAR_FLAG(huart->hdmatx, DMA_FLAG_TC);
    }
  } else {
    timeout_cnt = 0;
  }
}
