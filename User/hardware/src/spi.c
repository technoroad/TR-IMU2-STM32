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
typedef struct{
  SPI_TypeDef *instance;
  GPIO_PinConfig nCS;
  uint32_t data_width;
  uint32_t cpol;
  uint32_t cpha;
}SPI_IMUStructure;

/* Private define ------------------------------------------------------------*/
// SPI
#define TIMEOUT (1000-1)  // us
#define BUF_SIZE 32

/* Private variables ---------------------------------------------------------*/
static SPI_IMUStructure spi[2];
static uint32_t use_spi = 0;
static uint32_t timeout_cnt = 0;
static uint8_t stall = 16;
static uint32_t *pwrite = NULL;
static uint32_t *pread = NULL;
static int write_length = 0;
static int progress_index = 0;
static bool busy = false;
static bool brf = false;
static Stopwatch sw ={0};
static uint32_t spi_comm_time = 0;

/* Private macro -------------------------------------------------------------*/
#define SPI_ENABLE_CHIPSELECT GPIO_Write(&spi[use_spi].nCS,LOW)
#define SPI_DISABLE_CHIPSELECT GPIO_Write(&spi[use_spi].nCS,HIGH)

/* Private function prototypes -----------------------------------------------*/
static bool SPI_TimeoutReporter();
void SPI_ReceiveEnable(uint16_t *rb,uint16_t len);
void SPI_WriteData(uint32_t data);
uint32_t SPI_ReadData();

/* Global variables ----------------------------------------------------------*/

void SPI_Handler(){
  // 1フレームの送信完了して受信割り込みがあった時
  if (LL_SPI_IsActiveFlag_RXP(spi[use_spi].instance)){

    if(write_length > progress_index){
      pread[progress_index] = SPI_ReadData();
      progress_index++;

      if(brf){
        if(write_length > progress_index){
          SPI_WriteData(pwrite[progress_index]);
        }else{
          // タイマでTSTALLの待機
          SPI_DISABLE_CHIPSELECT;
          TIM6_Start();
        }
      }else{
        // タイマでTSTALLの待機
        SPI_DISABLE_CHIPSELECT;
        TIM6_Start();
      }
    }else{
      // 余計に割り込みがあった時は割り込みを解消するためにデータを読み出す。
      SPI_ReadData();
    }
  }
}

void TIM_WaitCompleteCallback(void){
  if(write_length <= progress_index){
    // 送受信が完了した。
    busy = false;
    timeout_cnt = 0;
    LL_SPI_DisableIT_RXP(spi[use_spi].instance);

    spi_comm_time = ElapsedMicroseconds(&sw);
  }else{
    // 通信中
    SPI_ENABLE_CHIPSELECT;
    SPI_WriteData(pwrite[progress_index]);
  }
}

/**
 * @brief  Sets the SPI speed.
 */
void SPI_SetPrescaler(uint32_t ll_baud) {
  LL_SPI_Disable(spi[use_spi].instance);
  LL_SPI_SetBaudRatePrescaler(spi[use_spi].instance, ll_baud);
  LL_SPI_Enable(spi[use_spi].instance);
  LL_SPI_StartMasterTransfer(spi[use_spi].instance);
}

void SPI_SetTStall(uint8_t t){
  stall = t;
}

bool SPI_IsFree(){
  return !busy;
}

static inline void change_bit_width(){
  LL_SPI_Disable(spi[use_spi].instance);
  LL_SPI_SetDataWidth(spi[use_spi].instance, spi[use_spi].data_width);
  LL_SPI_Enable(spi[use_spi].instance);
  LL_SPI_StartMasterTransfer(spi[use_spi].instance);
}

void SPI_ChangeBitWidth_32bit(){
  spi[use_spi].data_width = LL_SPI_DATAWIDTH_32BIT;
  change_bit_width();
}

void SPI_ChangeBitWidth_16bit(){
  spi[use_spi].data_width = LL_SPI_DATAWIDTH_16BIT;
  change_bit_width();
}

void SPI_ChangeBitWidth_8bit(){
  spi[use_spi].data_width = LL_SPI_DATAWIDTH_8BIT;
  change_bit_width();
}

static inline void change_mode(){
  LL_SPI_Disable(spi[use_spi].instance);
  LL_SPI_SetClockPolarity(spi[use_spi].instance, spi[use_spi].cpol);
  LL_SPI_SetClockPhase(spi[use_spi].instance, spi[use_spi].cpha);
  LL_SPI_Enable(spi[use_spi].instance);
  LL_SPI_StartMasterTransfer(spi[use_spi].instance);
}

void SPI_ChangeMode_0(){
  spi[use_spi].cpol = LL_SPI_POLARITY_LOW;
  spi[use_spi].cpha = LL_SPI_PHASE_1EDGE;
  change_mode();
}

void SPI_ChangeMode_3(){
  spi[use_spi].cpol = LL_SPI_POLARITY_HIGH;
  spi[use_spi].cpha = LL_SPI_PHASE_2EDGE;
  change_mode();
}

void SPI_SwitchToSPI2(){
  use_spi = 1;
}

void SPI_SelectAndDisableUnused(uint8_t unuse){
  if(unuse > UNUSE_SPI2){
    return;
  }

  if (unuse == UNUSE_SPI1) {
    LL_SPI_Disable(SPI1);
    LL_SPI_DisableIT_RXP(SPI1);
    LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI1);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);
    NVIC_DisableIRQ(SPI1_IRQn);
  }else if (unuse == UNUSE_SPI2) {
    LL_SPI_Disable(SPI2);
    LL_SPI_DisableIT_RXP(SPI2);
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI2);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_15, LL_GPIO_MODE_ANALOG);
    NVIC_DisableIRQ(SPI2_IRQn);
  }
}

/**
 *
 */
void SPI_Initialize() {
  LL_SPI_Enable(SPI1);
  LL_SPI_StartMasterTransfer(SPI1);
  LL_SPI_Enable(SPI2);
  LL_SPI_StartMasterTransfer(SPI2);

  spi[0].instance = SPI1;
  spi[0].nCS.port = GPIOA;
  spi[0].nCS.pin = LL_GPIO_PIN_4;
  spi[0].data_width = LL_SPI_DATAWIDTH_16BIT;
  spi[1].instance = SPI2;
  spi[1].nCS.port = GPIOB;
  spi[1].nCS.pin = LL_GPIO_PIN_12;
  spi[1].data_width = LL_SPI_DATAWIDTH_16BIT;
}

void SPI_Enable_IT(){
  TIM6_Initialize(stall);
  TIM6_AttachEventHandler(TIM_WaitCompleteCallback);
}

/**
 * @brief  Write and read at the same time with SPI.
 */
bool SPI_WriteReceive(uint32_t *wb, uint32_t *rb, int length) {
  if(busy){
    return false;
  }

  ElapsedMicroseconds(&sw);

  for (int i = 0; i < length; i++) {
    SPI_ENABLE_CHIPSELECT;
    SPI_WriteData(wb[i]);
    timeout_cnt = 0;
    while (!LL_SPI_IsActiveFlag_TXC(spi[use_spi].instance)) {
      if (SPI_TimeoutReporter()) {
        SetSystemError(kSpiTimeoutError);
        break;
      }
    }
    rb[i] = SPI_ReadData();

    SPI_DISABLE_CHIPSELECT;
    DelayMicrosecond(stall);
  }

  SPI_DISABLE_CHIPSELECT;
  DelayMicrosecond(stall);

  spi_comm_time = ElapsedMicroseconds(&sw);

  return true;
}

/**
 * @brief  Write and read at the same time with SPI.
 */
bool SPI_WriteReceive_IT(uint32_t *wb, uint32_t *rb, int length) {
  if(busy){
    if(SPI_TimeoutReporter()){
      SetSystemError(kSpiTimeoutError);
      busy = false;
      LL_SPI_DisableIT_RXP(spi[use_spi].instance);
    }
    return false;
  }

  ElapsedMicroseconds(&sw);

  if(length > BUF_SIZE){
    length =BUF_SIZE;
  }

  brf = false;
  pwrite = wb;
  pread = rb;
  write_length = length;
  progress_index = 0;
  busy = true;

  LL_SPI_EnableIT_RXP(spi[use_spi].instance);
  TIM6_Start();

  return true;
}

/**
 * @brief  Write and read at the same time with SPI.
 */
bool SPI_WriteReceiveBurstRead(uint32_t *wb, uint32_t *rb, int length) {
  if(busy){
    return false;
  }

  ElapsedMicroseconds(&sw);

  SPI_ENABLE_CHIPSELECT;
  for (int i = 0; i < length; i++) {
    SPI_WriteData(wb[i]);
    timeout_cnt = 0;
    while (!LL_SPI_IsActiveFlag_TXC(spi[use_spi].instance)) {
      if (SPI_TimeoutReporter()) {
        SetSystemError(kSpiTimeoutError);
        break;
      }
    }
    rb[i] = SPI_ReadData();
  }
  SPI_DISABLE_CHIPSELECT;
  DelayMicrosecond(stall);

  spi_comm_time = ElapsedMicroseconds(&sw);

  return true;
}

/**
 * @brief  Write and read at the same time with SPI.
 */
bool SPI_WriteReceiveBRF_IT(uint32_t *wb, uint32_t *rb, int length) {
  if(busy){
    if(SPI_TimeoutReporter()){
      SetSystemError(kSpiTimeoutError);
      busy = false;
      LL_SPI_DisableIT_RXP(spi[use_spi].instance);
    }
    return false;
  }

  ElapsedMicroseconds(&sw);

  if(length > BUF_SIZE){
    length =BUF_SIZE;
  }

  brf = true;
  pwrite = wb;
  pread = rb;
  write_length = length;
  progress_index = 0;
  busy = true;

  LL_SPI_EnableIT_RXP(spi[use_spi].instance);
  TIM6_Start();

  return true;
}

uint32_t SPI_GetCommTime(){
  return spi_comm_time;
}

/**
 * @brief  Counts the SPI wait time and returns the timeout flag.
 */
bool SPI_TimeoutReporter() {
  if (timeout_cnt > TIMEOUT) {
    timeout_cnt = 0;
    return true;
  } else {
    DelayMicrosecond(1);
    timeout_cnt++;
    return false;
  }
}

void SPI_WriteData(uint32_t data){
  switch(spi[use_spi].data_width){
    case LL_SPI_DATAWIDTH_32BIT:{
      LL_SPI_TransmitData32(spi[use_spi].instance, data);
      break;
    }
    case LL_SPI_DATAWIDTH_16BIT:{
      LL_SPI_TransmitData16(spi[use_spi].instance, (uint16_t)data);
      break;
    }
    case LL_SPI_DATAWIDTH_8BIT:{
      LL_SPI_TransmitData8(spi[use_spi].instance, (uint8_t)data);
      break;
    }
    default :{
      break;
    }
  }
}

uint32_t SPI_ReadData(){
  uint32_t ret = 0;

  switch(spi[use_spi].data_width){
    case LL_SPI_DATAWIDTH_32BIT:{
      ret = LL_SPI_ReceiveData32(spi[use_spi].instance);
      break;
    }
    case LL_SPI_DATAWIDTH_16BIT:{
      ret = LL_SPI_ReceiveData16(spi[use_spi].instance);
      break;
    }
    case LL_SPI_DATAWIDTH_8BIT:{
      ret = LL_SPI_ReceiveData8(spi[use_spi].instance);
      break;
    }
    default :{
      break;
    }
  }
  return ret;
}
