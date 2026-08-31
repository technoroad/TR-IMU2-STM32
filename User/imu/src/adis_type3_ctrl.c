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
static uint16_t compatible_id[] = {0x6000};
static uint16_t compatible_model[] = {16607};
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
static inline void make_send_cmd_brf();
static inline void make_send_cmd_single_32b();
static inline void make_send_cmd_single_16b();

static inline uint32_t make_read_cmd(uint32_t reg);
static inline void kConvert32bitDataWithSingleRead(uint32_t *data);
static inline void kConvert32bitDataWithBurstRead(uint32_t *data);
static inline void kConvert16bitDataWithSingleRead(uint32_t *data);
static inline void kConvert16bitDataWithBurstRead(uint32_t *data);
static inline uint8_t calculate_crc4(uint32_t data);

/* Global variables ----------------------------------------------------------*/

void IMU_RegisterType3Functions(){
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
  brf_enable = true;
}

static void IMU_SetDecimationRate(){
  IMU_WriteData(0,TYPE3_DEC_RATE, param->dec_rate);
}

/**
 * @brief  Hard filter selection.
 * @note  See the data sheet for details.
 */
static void IMU_SetHardwareFilter() {
  // Set digital filter
//  IMU_WriteData(0,TYPE3_FILT_CTRL, kSet->filter);
}

static void IMU_SetDirectSyncMode(){
  uint16_t reg = IMU_ReadData(0,TYPE3_MSC_CTRL);
  reg |= 1 << 2;
  IMU_WriteData(0,TYPE3_MSC_CTRL, reg);
}

static uint16_t IMU_GetDiagnosticFlag(){
  return 0;
}

static uint16_t IMU_GetProductId(){
  uint16_t id = IMU_ReadData(0,TYPE3_PROD_CMD);
  uint16_t len = sizeof(compatible_model) / sizeof(uint16_t);

  if(len != (sizeof(compatible_id) / sizeof(uint16_t))){
    return 0;
  }

  for(int i=0;i<len;i++){
    if(compatible_id[i] == id){
      return compatible_model[i];
    }
  }

  return 0;
}

static uint16_t IMU_GetRangeModel(){
  return IMU_ReadData(0,TYPE3_RANG_MDL);
}

static void IMU_Configration(){
  IMU_WriteData(0,TYPE3_SPI_FULL_DPX, 0xB3B3);
  IMU_ReadData(0, TYPE3_DIAG_STAT);
  IMU_ReadData(0, TYPE3_DIGI_STATUS);
  IMU_WriteData(0,TYPE3_USER_GPIO_CFG1, 0);
  IMU_WriteData(0,TYPE3_USER_GPIO_CFG1, 0x0249);
  IMU_WriteData(0,TYPE3_DEC_RATE, 0);
  IMU_WriteData(0,TYPE3_DEC_RATE, param->dec_rate);
  IMU_WriteData(0,TYPE3_WRITE_LOCK, 0x5555);
  IMU_WriteData(0,TYPE3_WRITE_LOCK, 0xAAAA);
  IMU_ReadData(0, TYPE3_WRITE_LOCK);
  IMU_ReadData(0, TYPE3_DIAG_STAT);
  IMU_ReadData(0, TYPE3_DIAG_STAT);

  if(brf_enable){
    make_send_cmd_brf();
  }else{
    if(bit32){
      make_send_cmd_single_32b();
    }else{
      make_send_cmd_single_16b();
    }
  }
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
 * @brief  Write to any address
 * @note   The page works only for the ADIS1649X series.
 */
static void IMU_WriteData(uint8_t page,uint8_t addr, uint16_t data) {
  int len = 0;
  uint32_t buf = (data << 16) | (addr << 8) | 0x80;
  buf |= calculate_crc4(buf);

#if 0
  any_send_buf[len++] = (buf >> 16) & 0x0000FFFF;
  any_send_buf[len++] = buf & 0x0000FFFF;
#else
  any_send_buf[len++] = buf;
#endif

  SPI_WriteReceive(any_send_buf,read_buf, len);
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
  any_send_buf[len++] = make_read_cmd(addr);
  any_send_buf[len++] = 0;

  SPI_WriteReceive(any_send_buf, read_buf, len);
  return read_buf[1] >> 16;
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

static inline uint8_t calculate_crc4(uint32_t data) {
    uint8_t crc = 0b1010;  //
    uint8_t poly = 0b10001; //

    // [31:4] の 28ビットに対して計算
    for (int i = 31; i >= 4; i--) {
        uint8_t bit = (data >> i) & 1;  // 上位ビットを取得
        crc = ((crc << 1) | bit) & 0x1F; // 左シフトして新しいビットを追加

        // MSBが1なら多項式を適用 (XOR演算)
        if (crc & 0b10000) {
            crc ^= poly;
        }
    }

    return crc & 0xF; // 計算結果を返す
}

static inline uint32_t make_read_cmd(uint32_t reg){
  uint32_t data = reg << 8;
  data &= 0xFFFF7F8F;  // read select
  data |= calculate_crc4(data);

  return data;
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

  cmd_buf[len++] =make_read_cmd(TYPE3_DIAG_STAT);
  for (int i = TYPE3_X_ACCL_HIGH; i <= TYPE3_Z_GYRO_LOW; i += 1) {
    cmd_buf[len++] = make_read_cmd(i);
  }
  cmd_buf[len++] = make_read_cmd(TYPE3_TEMP_OUT);
  cmd_buf[len++] = make_read_cmd(TYPE3_DATA_CNT);
  cmd_buf[len++] = 0x0000;
  cmd_buf_len = len;
}

static inline void make_send_cmd_single_16b(){
  int len = 0;
  memset(cmd_buf,0,sizeof(cmd_buf));

  cmd_buf[len++] =make_read_cmd(TYPE3_DIAG_STAT);
  for (int i = TYPE3_X_ACCL_HIGH; i <= TYPE3_Z_GYRO_HIGH; i += 2) {
    cmd_buf[len++] = make_read_cmd(i);
  }
  cmd_buf[len++] = make_read_cmd(TYPE3_TEMP_OUT);
  cmd_buf[len++] = make_read_cmd(TYPE3_DATA_CNT);
  cmd_buf[len++] = 0x0000;
  cmd_buf_len = len;
}

/**
 * @brief  Updating 6-axis data with a burst read
 */
static inline void IMU_Update6AxisDataWithBurstRead() {

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

static inline void kConvert32bitDataWithBurstRead(uint32_t *data) {

}

/**
 * @brief  Store the data read by
 *         IMU_Update6AxisDataWithSingleRead() in the IMU data structure.
 */
static inline void kConvert32bitDataWithSingleRead(uint32_t *data) {
  imu.diag_stat = (data[0] >> 16);
  for (int i = 0; i < 3; i++) {
    imu.accl.raw[i] = ((data[i * 2 + 1] & 0xFFFF0000) | (data[i * 2 + 2] >> 16)) >> 8;
    imu.gyro.raw[i] = ((data[i * 2 + 7] & 0xFFFF0000) | (data[i * 2 + 8] >> 16)) >> 8;

    // 符号拡張
    imu.accl.raw[i] = (int32_t)SIGN_EXT24(imu.accl.raw[i]);
    imu.gyro.raw[i] = (int32_t)SIGN_EXT24(imu.gyro.raw[i]);

    imu.gyro.fpp[i] =  //
        (int32_t) imu.gyro.raw[i] / param->g_sens_32b * M_PI / 180.0;

    imu.accl.fpp[i] = (double)((int32_t) imu.accl.raw[i]) / param->a_sens_32b;
    imu.accl.fpp[i] /= param->g;
  }
  imu.temperature = (double)((int16_t)(data[13] >> 16)) / 200.0 + 25.0;
  imu.data_cnt = (data[14] >> 16);
}

/**
 * @brief  Store the data read by
 *         IMU_Update6AxisDataWithSingleRead() in the IMU data structure.
 */
static inline void kConvert16bitDataWithSingleRead(uint32_t *data) {
  imu.diag_stat = (data[0] >> 16);
  for (int i = 0; i < 3; i++) {
    imu.accl.raw[i] = data[i + 1] >> 16;
    imu.gyro.raw[i] = data[i + 4] >> 16;

    // 符号拡張
    imu.accl.raw[i] = (int32_t)SIGN_EXT16(imu.accl.raw[i]);
    imu.gyro.raw[i] = (int32_t)SIGN_EXT16(imu.gyro.raw[i]);

    imu.gyro.fpp[i] =  //
        (int32_t) imu.gyro.raw[i] / param->g_sens_16b * M_PI / 180.0;

    imu.accl.fpp[i] = (double)((int32_t) imu.accl.raw[i]) / param->a_sens_16b;
    imu.accl.fpp[i] /= param->g;
  }
  imu.temperature = (double)((int16_t)(data[7] >> 16)) / 200.0 + 25.0;
  imu.data_cnt = (data[8] >> 16);
}

static inline void kConvert16bitDataWithBurstRead(uint32_t *data) {

}
