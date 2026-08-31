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
typedef struct {
  uint8_t command;
  uint8_t error;
} AckRequest;

/* Private define ------------------------------------------------------------*/
#define ACK_QUEUE_SIZE 16

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static ImuComputationData *cmpt = &gSystem.cmpt;
static ImuRawData *imu = &gSystem.raw;
static IMU_PersistConfig *conf = &gSystem.conf;
static IMU_Function *func = &gSystem.func;
static IMU_Parameter *param = &gSystem.param;
static uint32_t send_cnt = 0;

static AckRequest s_ack_q[ACK_QUEUE_SIZE];
static uint8_t s_ack_head = 0;
static uint8_t s_ack_tail = 0;

/* Private function prototypes -----------------------------------------------*/
static inline bool ack_queue_is_empty(void);
static inline bool ack_queue_is_full(void);
static bool ack_queue_push(AckRequest r);
static bool ack_queue_pop(AckRequest *out);
static inline void TelemetryDataSet(AcknowledgePacket *packet);
static inline void ConfigureDataSet(AcknowledgePacket *packet);
uint32_t FloatToByte(float x, uint8_t *buf);

/**
 * @brief  ACKを返信します。
 * @note  返信する場合はSendAckEnable()を実行してください
 */
void CMD_SendAcknowledge() {
  static AcknowledgePacket packet = { 0 };
  static AckRequest r;

  while (ack_queue_pop(&r)) {
    packet.command = r.command;
    packet.error = r.error;
    packet.data_length = 0;
    packet.checksum = 0;
    memset(packet.data, 0, sizeof(packet.data));

    if (packet.command < CmdGetConfigure) {
      TelemetryDataSet(&packet);
    } else {
      ConfigureDataSet(&packet);
    }

    TransmitAcknowledge(&packet);
  }
}

/**
 * @brief  ACKパケットの設定をします。
 * @param[in] command : ACKのコマンド種別(エコーバックのために必要)
 */
void SetAcknowledgeCmd(BinaryPacket *recv) {
  AckRequest r = {
      .command = recv->command,
      .error   = recv->error.bits
  };
  ack_queue_push(r);
}

/**
 * @brief  定期送信コマンドをセットします。
 */
void SetPeriodicCmd() {
  AckRequest r = {
      .command = PeriodicSend,
      .error   = 0
  };
  ack_queue_push(r);
}

/**
 * @brief  送信バッファへデータをセットします
 */
static inline void TelemetryDataSet(AcknowledgePacket *packet) {
  uint8_t *buf = packet->data;
  uint32_t counter = 0;

  buf[counter++] = packet->error;
  counter += U32ToByte(send_cnt++, &buf[counter]);

  if(GetSystemStatus() != FatalError_Status){
    for (int i = 0; i < 4; i++) {
      counter += FloatToByte(cmpt->quat[i], &buf[counter]);
    }

    for (int i = 0; i < 3; i++) {
      counter += U32ToByte(imu->accl.raw[i], &buf[counter]);
    }

    for (int i = 0; i < 3; i++) {
      counter += U32ToByte(imu->gyro.raw[i], &buf[counter]);
    }

    int16_t temp = (int16_t) roundf((float) imu->temperature * 10.0f);
    counter += U16ToByte(temp, &buf[counter]);

    counter += U16ToByte(imu->data_cnt, &buf[counter]);

    counter += U16ToByte(gSystem.cmpt.dropped, &buf[counter]);

    uint32_t cmpt = gSystem.cmpt.computation_time;
    if (cmpt > 65535) {
      cmpt = 65535;
    }
    counter += U16ToByte(cmpt, &buf[counter]);

    uint32_t spi = SPI_GetCommTime();
    if (spi > 65535) {
      spi = 65535;
    }
    counter += U16ToByte(spi, &buf[counter]);
    buf[counter++] = gSystem.cmpt.in0;

    counter += U64ToByte(gSystem.cmpt.time_us, &buf[counter]);
  }

  packet->data_length = TLMT_DATA_SIZE;  // 固定値
}

/**
 * @brief  送信バッファへデータをセットします
 */
static inline void ConfigureDataSet(AcknowledgePacket *packet) {
  uint8_t *buf = packet->data;
  uint32_t counter = 0;

  buf[counter++] = packet->error;
  counter += U32ToByte(send_cnt++, &buf[counter]);
  counter += U32ToByte(GetBuildDateInt(), &buf[counter]);

  {
    uint8_t flag = (conf->usb_enable ? 0x01 : 0);
    flag |= (conf->fdcan_enable ? 0x02 : 0);
    flag |= (conf->uart4_enable ? 0x04 : 0);
    buf[counter++] = flag;
  }

  {
    uint8_t flag = (conf->read32bit_enable ? 0x01 : 0);
    buf[counter++] = flag;
  }

  buf[counter++] = conf->filter_select;

  double g_sen = 0, a_sen = 0;
  func->GetSensitivity(&g_sen, &a_sen);
  counter += U64ToByte((uint64_t) (a_sen * 1e6), &buf[counter]);
  counter += U64ToByte((uint64_t) (g_sen * 1e6), &buf[counter]);

  counter += U16ToByte(param->sample_rate, &buf[counter]);
  buf[counter++] = param->maker;
  counter += U16ToByte(param->product_id, &buf[counter]);
  buf[counter++] = param->model;

  buf[counter++] = gSystem.dip.BRD;
  buf[counter++] = (conf->gravity_corr_enable ? 0x01 : 0);
  buf[counter++] = conf->in0_pupd;
  buf[counter++] = conf->in0_trigger;

  packet->data_length = CONF_DATA_SIZE;  // 固定値
}

static inline float clampf(float x) {
  if (x > 1.0f)
    return 1.0f;
  if (x < -1.0f)
    return -1.0f;
  return x;
}

/*  float (-1 ~ +1) -> int16_t（-32767..+32767） */
static inline int16_t quant16(float x) {
  x = clampf(x);
  int32_t v = (int32_t) roundf(x * 32767.0f);
  if (v > 32767)
    v = 32767;
  if (v < -32767)
    v = -32767;
  return (int16_t) v;
}

/* 復元 */
static inline float dequant16(int16_t q) {
  return ((float) q) / 32767.0f;
}

// float (-1 ~ +1)変数をバイト列に変換
uint32_t FloatToByte(float x, uint8_t *buf) {
  uint16_t q = quant16(x);

  buf[0] = q & 0xFF;
  buf[1] = (q >> 8) & 0xFF;

  return 2;  // バイト数を戻す
}

static inline bool ack_queue_is_empty(void) {
  return s_ack_head == s_ack_tail;
}
static inline bool ack_queue_is_full(void) {
  return (uint8_t) ((s_ack_head + 1U) % ACK_QUEUE_SIZE) == s_ack_tail;
}
static bool ack_queue_push(AckRequest r) {
  if (ack_queue_is_full())
    return false;
  s_ack_q[s_ack_head] = r;
  s_ack_head = (s_ack_head + 1) & (ACK_QUEUE_SIZE - 1);
  return true;
}
static bool ack_queue_pop(AckRequest *out) {
  if (ack_queue_is_empty())
    return false;
  *out = s_ack_q[s_ack_tail];
  s_ack_tail = (s_ack_tail + 1U) % ACK_QUEUE_SIZE;
  return true;
}
