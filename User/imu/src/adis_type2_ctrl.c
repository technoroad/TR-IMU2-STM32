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
#define SPI_BUF_SIZE 32

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t compatible_model[] = {16495};
static uint32_t any_send_buf[SPI_BUF_SIZE] = { 0 };
static uint32_t cmd_buf[SPI_BUF_SIZE] = { 0 };
static uint32_t read_buf[SPI_BUF_SIZE] = { 0 };
static uint32_t copy[SPI_BUF_SIZE] = {0};
static IMU_Parameter *param = &gSystem.param;
static IMU_Function *func = &gSystem.func;
static ImuRawData imu ={0};
static bool bit32 = false;
static bool brf_enable = false;
static int cmd_buf_len = 0;

/* Private function prototypes -----------------------------------------------*/
static void IMU_Configration();
static void IMU_Read32bitEnable();
static void IMU_SetDecimationRate();
static void IMU_EnableBurstRead();
static void IMU_SetHardwareFilter();
static void IMU_SetDirectSyncMode();
static uint16_t IMU_GetDiagnosticFlag();
static uint16_t IMU_GetProductId();
static uint16_t IMU_GetRangeModel();
static void IMU_GetSensitivity(double *gyro_sens, double *accl_sens);
static void IMU_WriteData(uint8_t page, uint8_t addr, uint16_t data);
static uint16_t IMU_ReadData(uint8_t page, uint16_t addr);
static void IMU_Update6AxisData();
static void IMU_CopyRawData(ImuRawData *dst);

static inline void IMU_Update6AxisDataWithSingleRead();
static inline void IMU_Update6AxisDataWithBurstRead();
static inline uint32_t make_read_cmd(uint32_t reg);
static inline void kConvert32bitDataWithSingleRead(uint32_t *data);
static inline void kConvert32bitDataWithBurstRead(uint32_t *data);
static inline void kConvert16bitDataWithSingleRead(uint32_t *data);
static inline void kConvert16bitDataWithBurstRead(uint32_t *data);
static uint32_t kAddWriteCurrentPage(uint32_t *buf, uint8_t page);

/* Global variables ----------------------------------------------------------*/

void IMU_RegisterType2Functions(){
  // 初期化
  func->Configration              = NULL;
  func->Read32bitEnable           = NULL;
  func->SetDecimationRate         = NULL;
  func->EnableBurstRead           = NULL;
  func->SetHardwareFilter         = NULL;
  func->SetDirectSyncMode         = NULL;
  func->GetDiagnosticFlag         = NULL;
  func->GetProductId              = NULL;
  func->GetRangeModel             = NULL;
  func->GetSensitivity            = NULL;
  func->WriteData                 = NULL;
  func->ReadData                  = NULL;
  func->Update6AxisData           = NULL;
  func->CopyRawData               = NULL;

  // 関数ポインタを設定
  func->Configration              = IMU_Configration;
  func->Read32bitEnable           = IMU_Read32bitEnable;
  func->SetDecimationRate         = IMU_SetDecimationRate;
  func->EnableBurstRead           = IMU_EnableBurstRead;
  func->SetHardwareFilter         = IMU_SetHardwareFilter;
  func->SetDirectSyncMode         = IMU_SetDirectSyncMode;
  func->GetDiagnosticFlag         = IMU_GetDiagnosticFlag;
  func->GetProductId              = IMU_GetProductId;
  func->GetRangeModel             = IMU_GetRangeModel;
  func->GetSensitivity            = IMU_GetSensitivity;
  func->WriteData                 = IMU_WriteData;
  func->ReadData                  = IMU_ReadData;
  func->Update6AxisData           = IMU_Update6AxisData;
  func->CopyRawData               = IMU_CopyRawData;
}

static void IMU_Read32bitEnable(){
  bit32 = true;
}

static void IMU_EnableBurstRead(){
  // MSC_CTRL does not exist in the ADIS1649X series.
  Crc32Initialize();  //CRC32 initialize

  brf_enable = true;
}

static void IMU_SetDecimationRate(){
  IMU_WriteData(PAGE3,TYPE2_DEC_RATE, param->dec_rate);
}

/**
 * @brief  Hard filter selection.
 * @note  See the data sheet for details.
 */
static void IMU_SetHardwareFilter() {
  uint8_t f0 = param->filter;
  uint16_t filter0 = 0;
  uint16_t filter1 = 0;
  int bank0_arr[5] = { f0, f0, f0, f0, f0 };

  //FILTR_BNK_0
  for (int i = 0; i < 5; i++) {
    if ((bank0_arr[i] <= FILT_BANK_D) && (bank0_arr[i] >= FILT_BANK_A)) {
      filter0 |= (uint16_t) ((bank0_arr[i] | 0x0004) << (i * 3));
    } else {
      filter0 |= (uint16_t) ((bank0_arr[i] & 0x0003) << (i * 3));
    }
  }

  //FILTR_BNK_1
  if ((f0 <= FILT_BANK_D) && (f0 >= FILT_BANK_A)) {
    filter1 |= (uint16_t) (f0 | 0x0004);
  } else {
    filter1 |= (uint16_t) (f0 & 0x0003);
  }

  IMU_WriteData(PAGE3, TYPE2_FILTR_BNK_0, filter0);
  DelayMillisecond(1);
  IMU_WriteData(PAGE3, TYPE2_FILTR_BNK_1, filter1);
  DelayMillisecond(1);

  param->filter = f0;
}

static void IMU_SetDirectSyncMode(){
  uint16_t reg = IMU_ReadData(PAGE3,TYPE2_FNCTIO_CTRL);
  reg |= 0xC0;
  IMU_WriteData(PAGE3,TYPE2_FNCTIO_CTRL, reg);
}

static uint16_t IMU_GetDiagnosticFlag(){
  return IMU_ReadData(PAGE0,TYPE2_SYS_E_FLAG);
}

static uint16_t IMU_GetProductId(){
  uint16_t id = IMU_ReadData(0,TYPE2_PROD_CMD);
  uint16_t len = sizeof(compatible_model) / sizeof(uint16_t);

  for(int i=0;i<len;i++){
    if(compatible_model[i] == id){
      return compatible_model[i];
    }
  }

  return 0;
}

static uint16_t IMU_GetRangeModel(){
  return IMU_ReadData(PAGE3,TYPE2_RANG_MDL);
}

static void IMU_Configration(){
  IMU_SetHardwareFilter();
  IMU_SetDirectSyncMode();
  IMU_SetDecimationRate();
}

static void IMU_GetSensitivity(double *gyro_sens, double *accl_sens){
  if(bit32){
    *gyro_sens = param->g_sens_32b;
    *accl_sens = param->a_sens_32b;
  }else{
    *gyro_sens = param->g_sens_16b;
    *accl_sens = param->a_sens_16b;
  }
}

/**
 * @brief  Change the current page number.
 */
static uint32_t kAddWriteCurrentPage(uint32_t *buf, uint8_t page){
  // page write
  buf[0] = (0x80 << 8) | (page & 0xFF);
  buf[1] = (0x81 << 8) | ((page >> 8) & 0xFF);
  return 2;
}

/**
 * @brief  Write to any address
 * @note   The page works only for the ADIS1649X series.
 */
static void IMU_WriteData(uint8_t page,uint8_t addr, uint16_t data) {
  int len = 0;
  len += kAddWriteCurrentPage(&any_send_buf[len],page);
  any_send_buf[len++] = ((addr | 0x80) << 8) | (data & 0xFF);
  any_send_buf[len++] = ((addr | 0x81) << 8) | ((data >> 8) & 0xFF);
  len += kAddWriteCurrentPage(&any_send_buf[len],PAGE0);

  SPI_WriteReceive(any_send_buf, read_buf, len);
}

/**
 * @brief  Read any address
 * @note   The page works only for the ADIS1649X series.
 */
static uint16_t IMU_ReadData(uint8_t page,uint16_t addr) {
  // Burst command can not be read
  if (addr == param->burst_cmd) {
    return 0;
  }

  int len = 0;
  len += kAddWriteCurrentPage(&any_send_buf[len],page);
  any_send_buf[len++] = make_read_cmd(addr);
  any_send_buf[len++] = 0x00;
  len += kAddWriteCurrentPage(&any_send_buf[len],PAGE0);

  SPI_WriteReceive(any_send_buf, read_buf, len);
  return read_buf[3];
}

/**
 * @brief  A function to switch between 32bit burst read and single read.
 */
static void IMU_Update6AxisData() {
  if (brf_enable) {
    IMU_Update6AxisDataWithBurstRead();
  } else {
    IMU_Update6AxisDataWithSingleRead();
  }
}

static void IMU_CopyRawData(ImuRawData *dst){
  memcpy(dst,&imu,sizeof(imu));
}

static inline uint32_t make_read_cmd(uint32_t reg){
  return (reg << 8) & 0x0000FF00;
}

static inline void make_send_cmd_brf(){
  memset(cmd_buf,0,sizeof(cmd_buf));
  cmd_buf[0] = param->burst_cmd << 8;

  if(bit32){
    cmd_buf_len = param->brf_size_32b;
  }else{
    cmd_buf_len = param->brf_size_16b;
  }
}

static inline void make_send_cmd_single_32b(){
  int len = 0;
  memset(cmd_buf,0,sizeof(cmd_buf));

  cmd_buf[len++] = make_read_cmd(TYPE2_SYS_E_FLAG);
  for (int i = TYPE2_X_GYRO_LOW; i <= TYPE2_Z_ACCL_OUT; i += 2) {
    cmd_buf[len++] = make_read_cmd(i);
  }
  cmd_buf[len++] = make_read_cmd(TYPE2_TEMP_OUT);
  cmd_buf[len++] = make_read_cmd(TYPE2_DATA_CNT);
  cmd_buf[len++] = 0x0000;
  cmd_buf_len = len;
}

static inline void make_send_cmd_single_16b(){
  int len = 0;
  memset(cmd_buf,0,sizeof(cmd_buf));

  cmd_buf[len++] =make_read_cmd(TYPE2_SYS_E_FLAG);
  for (int i = TYPE2_X_GYRO_OUT; i <= TYPE2_Z_ACCL_OUT; i += 4) {
    cmd_buf[len++] = make_read_cmd(i);
  }
  cmd_buf[len++] = make_read_cmd(TYPE2_TEMP_OUT);
  cmd_buf[len++] = make_read_cmd(TYPE2_DATA_CNT);
  cmd_buf[len++] = 0x0000;
  cmd_buf_len = len;
}

/**
 * @brief  Updating 6-axis data with a burst read
 */
static void IMU_Update6AxisDataWithBurstRead() {
  memcpy(copy,read_buf,sizeof(read_buf));

  SPI_WriteReceiveBRF_IT(cmd_buf, read_buf, cmd_buf_len);

  if(bit32){
    kConvert32bitDataWithBurstRead( &copy[1]);
  }else{
    kConvert16bitDataWithBurstRead( &copy[1]);
  }
}

/**
 * @brief  Updating 6-axis data with a single read.
 */
static inline void IMU_Update6AxisDataWithSingleRead() {
  memcpy(copy,read_buf,sizeof(read_buf));

  SPI_WriteReceive_IT(cmd_buf, read_buf, cmd_buf_len);

  if(bit32){
    kConvert32bitDataWithSingleRead( &copy[1]);
  }else{
    kConvert16bitDataWithSingleRead( &copy[1]);
  }
}

/**
 * @brief  Store the data read by
 *         IMU_Update6AxisDataWithSingleRead() in the IMU data structure.
 */
static inline void kConvert32bitDataWithSingleRead(uint32_t *data) {
  imu.diag_stat = data[0];
  for (int i = 0; i < 3; i++) {
    imu.gyro.raw[i] = ((data[i * 2 + 2] << 16) | data[i * 2 + 1]);
    imu.accl.raw[i] = ((data[i * 2 + 8] << 16) | data[i * 2 + 7]);

    imu.gyro.fpp[i] =  //
        (int32_t) imu.gyro.raw[i] / param->g_sens_32b * M_PI / 180.0;

    imu.accl.fpp[i] = (int32_t) imu.accl.raw[i] / param->a_sens_32b;
  }
  imu.temperature = 25.0 + (double) ((int16_t) data[13]) / 80.0;
  imu.data_cnt = data[14];
}

/**
 * @brief  Store the data read by
 *         IMU_Update6AxisDataWithSingleRead() in the IMU data structure.
 */
static void kConvert32bitDataWithBurstRead(uint32_t *data) {
  // Since the data rarely deviates, it is corrected.
  if (data[1] == 0xA5A5) {
    data = &data[1];
  }
  // once again.
  if (data[1] == 0xA5A5) {
    data = &data[1];
  }

  imu.diag_stat = data[1];
  for (int i = 0; i < 3; i++) {
    imu.gyro.raw[i] = ((data[i * 2 + 4] << 16) | data[i * 2 + 3]);
    imu.accl.raw[i] = ((data[i * 2 + 10] << 16) | data[i * 2 + 9]);

    imu.gyro.fpp[i] =  //
        (int32_t) imu.gyro.raw[i] / param->g_sens_32b * M_PI / 180.0;

    imu.accl.fpp[i] = (int32_t) imu.accl.raw[i] / param->a_sens_32b;
  }
  imu.temperature = 25.0 + (double) ((int16_t) data[2]) / 80.0;
  imu.data_cnt = data[15];
  imu.recv_crc32 = ((data[17] << 16) | data[16]);
  imu.calc_crc32 = ~(MakeCrc32(&data[1],15));
}

/**
 * @brief  Store the data read by
 *         IMU_Update6AxisDataWithSingleRead() in the IMU data structure.
 */
static void kConvert16bitDataWithSingleRead(uint32_t *data) {
  imu.diag_stat = data[0];
  for (int i = 0; i < 3; i++) {
    imu.gyro.raw[i] = data[i + 1];
    imu.accl.raw[i] = data[i + 4];

    // 符号拡張
    imu.accl.raw[i] = (int32_t)SIGN_EXT16(imu.accl.raw[i]);
    imu.gyro.raw[i] = (int32_t)SIGN_EXT16(imu.gyro.raw[i]);

    imu.gyro.fpp[i] =  //
        (int32_t) imu.gyro.raw[i] / param->g_sens_16b * M_PI / 180.0;

    imu.accl.fpp[i] = (int32_t) imu.accl.raw[i] / param->a_sens_16b;
  }
  imu.temperature = 25.0 + (double) ((int16_t) data[7]) / 80.0;
  imu.data_cnt = data[8];
}

/**
 * @brief  Store the data read by
 *         IMU_Update6AxisDataWithSingleRead() in the IMU data structure.
 */
static void kConvert16bitDataWithBurstRead(uint32_t *data) {
}
