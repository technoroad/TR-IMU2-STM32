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

#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

typedef void (*UserEventHandlerPtr)(void);
typedef volatile uint8_t bool;

#define AXIS 3

#define true 1
#define false 0

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)

//Standard Gravity Acceleration( Tokyo )
#define STANDARD_GRAVITY 9.797464

#define ADIS1647X_CLOCKERROR 0x0080
#define ADIS1647X_MEMORYERROR 0x0040
#define ADIS1647X_SENSORFAILURE 0x0020
#define ADIS1647X_STANDBYMODE 0x0010
#define ADIS1647X_SPICOMMERROR 0x0008
#define ADIS1647X_MEMORYUPDATEFAILURE 0x0004
#define ADIS1647X_DATAPATHOVERRUN 0x0002

// List of system status
typedef enum {
  Initialize_Status,
  WaitWakeup_Status,
  BiasSetup_Status,
  UpdatePose_Status,
  FatalError_Status,
} SystemStatus;

typedef enum {
  IMU_Type1 = 1,
  IMU_Type2 = 2,
  IMU_Type3 = 3,
} IMU_Type;

typedef struct{
  IMU_Type type;
  uint8_t tstall;
  uint16_t sample_rate;
  uint16_t conversion_rate;
  uint8_t dec_rate;
  double dt;
  uint8_t filter;
  uint8_t maker;
  uint16_t product_id;
  uint8_t model;
  double g;

  uint32_t burst_cmd;
  double g_sens_32b;
  double a_sens_32b;
  uint8_t brf_size_32b;
  double g_sens_16b;
  double a_sens_16b;
  uint8_t brf_size_16b;
  uint8_t exti_trigger;
}IMU_Parameter;

// Struct of parameters to be saved in flash
typedef struct __attribute__((aligned(16), packed)){
  bool usb_enable;
  bool fdcan_enable;
  bool uart4_enable;
  bool read32bit_enable;
  uint8_t filter_select;
  bool gravity_corr_enable;
  uint8_t in0_pupd;
  uint8_t in0_trigger;
} IMU_PersistConfig;

typedef struct {
  uint32_t raw[3];
  double fpp[3];
} ImuVector3;

typedef struct {
  float quat[4];
  uint32_t computation_time;
  uint16_t dropped;
  uint8_t in0;
  uint64_t time_us;
} ImuComputationData;

typedef struct {
  uint16_t diag_stat;
  ImuVector3 gyro;
  ImuVector3 accl;
  double temperature;
  uint16_t data_cnt;
  uint16_t csum;
  uint16_t sum;
  uint32_t recv_crc32;
  uint32_t calc_crc32;
} ImuRawData;

typedef struct{
  bool DP1;
  bool BRD;
}ConfigSwitch;

typedef struct {
  void (*Configration)(void);
  void (*Read32bitEnable)(void);
  void (*SetDecimationRate)(void);
  void (*EnableBurstRead)(void);
  void (*SetHardwareFilter)();
  void (*SetDirectSyncMode)(void);
  uint16_t (*GetDiagnosticFlag)(void);
  uint16_t (*GetProductId)(void);
  uint16_t (*GetRangeModel)(void);
  void (*GetSensitivity)(double *gyro_sens, double *accl_sens);
  void (*WriteData)(uint8_t page, uint8_t addr, uint16_t data);
  uint16_t (*ReadData)(uint8_t page, uint16_t addr);
  void (*Update6AxisData)(void);
  void (*CopyRawData)(ImuRawData *dst);
} IMU_Function;

typedef struct {
  void (*Initialize)(double dt);
  void (*Reset)(void);
  void (*Update)(void);
  bool (*IsRest)(void);
} Filter_Function;

typedef struct {
  void (*Initialize)(double dt);
  void (*Reset)(void);
  void (*UpdateGyro)(double *gyro);
  void (*UpdateAccl)(double *g);
  void (*GetQuat3D)(float *quat);
  void (*GetQuat6D)(float *quat);
  bool (*GetRestDetected)(void);
} VQF_Function;

typedef struct {
  ConfigSwitch dip;
  IMU_PersistConfig conf;
  IMU_Parameter param;
  IMU_Function func;
  Filter_Function filter;

  ImuRawData raw;
  ImuComputationData cmpt;
} System_Status;

typedef struct __attribute__((aligned(16), packed)){
  IMU_PersistConfig conf;
  uint32_t build_date;
  uint8_t padding1[12];  // 埋め
  uint32_t build_time;
  uint8_t padding2[12];  // 埋め
  uint16_t csum;
  uint8_t padding3[14];  // 埋め
} Flash_Settings;

typedef enum {
  RET_OK = 0,
  RET_ERROR = 1,
  RET_OUTRANGE = 2,
  RET_NO_CMD = 3,
  RET_FLASH_ERR = 4,
} ReturnTypeDef;

#endif /* INC_DEFINES_H_ */
