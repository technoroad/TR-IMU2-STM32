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

/* Global variables ----------------------------------------------------------*/
extern System_Status gSystem;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
//#define MAHONY
#define MASUYA

#ifdef MAHONY
#elif defined(MASUYA)
#define FREQ_ACC 3.0
#define FERQ_GYRO 10.0
#define FREQ_RELAX 90.0
#endif
#define GYRO_B 0.003  // The value for converting noise density to a threshold.
#define LPF_a 0.008
#define BIAS_GYRO_CUTOFF 10.0
#define BIAS_ACC_CUTOFF 3.0

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#ifdef MAHONY
mkAE_IMU mkae_Mah;
static double Kp_Mah = 1.0, Ki_Mah = 0.1;
static double wA_Mah = 1.0;
static double gyro_mah[3], acc_mah[3];
#elif defined(MASUYA)
static mkAE_IMU_MS2016 mkae_ms;
static mkAE_IMU_MS2016 my_filter;
#endif
static IMU_Parameter *param = &gSystem.param;
static ImuRawData *imu = &gSystem.raw;
static ImuComputationData *cmpt = &gSystem.cmpt;
static Filter_Function *func = &gSystem.filter;
static double *gyro = gSystem.raw.gyro.fpp;
static double *acc = gSystem.raw.accl.fpp;
static double accl_f[AXIS] = { 0 };
static double gyro_f[AXIS] = { 0 };
static double smooth_gyro[AXIS] = { 0 };
static double gyro_est[AXIS] = { 0 };
static double bias_est[AXIS] = { 0 };
static bool is_rest = false;
static double DT = 0;
static double off_wA[] = {0,0,0};
static double on_wA[]  = {1,1,0};
static bool gravity_correnction_enable = true;

/* Private function prototypes -----------------------------------------------*/
static void MKAE_MS2016_Initialize(double dt);
static void MKAE_MS2016_Update();
static void MKAE_MS2016_FilterReset();
static bool IsRest();

static bool UpdateGyroBiasOnRest();
static bool UpdateBiasEstination();
static void BiasEstination_Initialize();
static bool MKAE_RestDetected();
static void MKAE_MS2016_SetParam(mkAE_IMU_MS2016 *mkae_ms);
static void MyFilter_SetParam(mkAE_IMU_MS2016 *filter);

void MKAE_RegisterFunctions(){
  func->Initialize                 = NULL;
  func->Reset                      = NULL;
  func->Update                     = NULL;
  func->IsRest                     = NULL;

  func->Initialize                 = MKAE_MS2016_Initialize;
  func->Reset                      = MKAE_MS2016_FilterReset;
  func->Update                     = MKAE_MS2016_Update;
  func->IsRest                     = IsRest;
}

/**
 * @brief  Filter initialization
 */
static void MKAE_MS2016_Initialize(double dt) {
  DT = dt;
  gravity_correnction_enable = gSystem.conf.gravity_corr_enable;

#ifdef MAHONY
  mkAE_IMU_Initialize(&mkae_Mah, acc);
#elif defined(MASUYA)
  // Filter initialization
  mkAE_IMU_MS2016_Initialize(&mkae_ms, gyro, acc, 1, 1, 1);
  MKAE_MS2016_SetParam(&mkae_ms);

  BiasEstination_Initialize();
#endif
}

static void MKAE_MS2016_Update() {
  is_rest = UpdateBiasEstination();

  double *wA = off_wA;

  // 重力補正オン
  if(gSystem.conf.gravity_corr_enable){
    wA = on_wA;
  }

  // Output of quat
  mkAE_IMU_MS2016_CF_Quat(&mkae_ms, gyro_est, acc, wA, 1,
                          0, DT);

  for (int i = 0; i < 4; i++) {
    cmpt->quat[i] = (float) mkae_ms.mkae.q_est[i];
  }

  SystemError error = DetectSensorError(imu->diag_stat);
  if (error != kNoneError) {
    SetSystemError(error);
  }
}

/**
 * @brief  Set the output values of yaw, pitch and roll to 0.
 */
static void MKAE_MS2016_FilterReset() {
  double g[3], a[3];

#ifdef MAHONY
  for (int i = 0; i < 3; i++) {
    gyro[i] = gyro_mah[i];
    acc[i] = acc_mah[i];
  }

  mkAE_IMU_Reset(&mkae_Mah, gyro, acc);
#elif defined(MASUYA)
  for (int i = 0; i < 3; i++) {
    g[i] = *mkae_ms._gyro_mes[i];
    a[i] = *mkae_ms._acc_mes[i];
  }

  mkAE_IMU_MS2016_Reset(&mkae_ms, g, a);
#endif
}

static bool IsRest(){
  return is_rest;
}

static void BiasEstination_Initialize() {
  mkAE_IMU_MS2016_Initialize(&my_filter, gyro, acc, 1, 1, 1);
  MyFilter_SetParam(&my_filter);

  for (int i = 0; i < AXIS; i++) {
    bias_est[i] = 0;
  }
}

static bool UpdateBiasEstination() {
  // LPフィルタ
  mkAE_IMU_MS2016_Cmpt_MesgyroFilter(&my_filter, gyro, gyro_f);
  mkAE_IMU_MS2016_Cmpt_MesAccFilter(&my_filter, acc, accl_f);

  bool ret = UpdateGyroBiasOnRest();

  for (int i = 0; i < 3; i++) {
    gyro_est[i] = gyro[i] + bias_est[i];
  }
  return ret;
}

static bool MKAE_RestDetected() {
  const double gyro_th = 1.0 * DEG_TO_RAD;           // deg/s
  const double acc_th = 0.05;           // g variation
  const int required_samples = 1000;

  // staticで関数内状態を保持
  static int stable_count = 0;
  double gx = smooth_gyro[0];
  double gy = smooth_gyro[1];
  double gz = smooth_gyro[2];
  double ax = accl_f[0];
  double ay = accl_f[1];
  double az = accl_f[2];

  double acc_norm = sqrt(ax * ax + ay * ay + az * az) - 1.0;

  bool gyro_ok = (fabs(gx) < gyro_th) && (fabs(gy) < gyro_th)
      && (fabs(gz) < gyro_th);

  bool acc_ok = (fabs(acc_norm) < acc_th);

  if (gyro_ok && acc_ok) {
    stable_count++;
  } else {
    stable_count = 0;
  }

  return stable_count > required_samples;
}

static bool UpdateGyroBiasOnRest() {
  static double sum_bias[AXIS] = { 0 };
  static double lpf_bias[AXIS] = { 0 };
  static int sum_count = 0;
  static double bias_alpha = 0.001;        // バイアス学習率
  static int sum_samples = 500;

  bool ret = false;
  if (MKAE_RestDetected()) {
    for (int i = 0; i < AXIS; i++) {
      // バイアス推定更新
      lpf_bias[i] += bias_alpha * (smooth_gyro[i] - lpf_bias[i]);
      sum_bias[i] += lpf_bias[i];
    }
    sum_count++;

    if (sum_count >= sum_samples) {
      // 平均バイアス補正の蓄積
      for (int i = 0; i < AXIS; i++) {
        bias_est[i] -= sum_bias[i] / sum_count;
        sum_bias[i] = 0.0;
      }
      sum_count = 0;
    }
    ret = true;
  } else {
    // 動いている時はリセット
    sum_count = 0;
    for (int i = 0; i < AXIS; i++) {
      sum_bias[i] = 0.0;
    }
  }

  for (int i = 0; i < 3; i++) {
    smooth_gyro[i] = gyro_f[i] + bias_est[i];
  }

  return ret;
}

/* Sensor dynamics */
static void MKAE_MS2016_SetParam(mkAE_IMU_MS2016 *mkae_ms) {
#ifdef MASUYA
  int i;
  /* Filter for the estimate of the acceleration */
  for (i = 0; i < 3; i++) {
    mk_Set_Param_1stOrderFilter_LP(mkae_ms->param_EstAcc0Fil[i],  //
        1.0, FREQ_ACC, param->dt);
  }

  /* Filter for the accelerometer's measurement */
  for (i = 0; i < 3; i++) {
    mk_Set_Param_1stOrderFilter_LP(mkae_ms->param_MesAccFil[i],  //
        1.0, FREQ_RELAX, param->dt);
  }

  /* Filter for the gyroscope's measurement */
  for (i = 0; i < 3; i++) {
    mk_Set_Param_1stOrderFilter_PC(mkae_ms->param_MesgyroFil[i],  //
        1.0, FERQ_GYRO, FREQ_RELAX, param->dt);
  }
#endif
}

/* Sensor dynamics */
static void MyFilter_SetParam(mkAE_IMU_MS2016 *filter) {
#ifdef MASUYA
  int i;

  /* Filter for the accelerometer's measurement */
  for (i = 0; i < 3; i++) {
    mk_Set_Param_1stOrderFilter_LP(filter->param_MesAccFil[i],  //
        1.0, BIAS_ACC_CUTOFF, param->dt);
  }

  /* Filter for the gyroscope's measurement */
  for (i = 0; i < 3; i++) {
    mk_Set_Param_1stOrderFilter_LP(filter->param_MesgyroFil[i],  //
        1.0, BIAS_GYRO_CUTOFF, param->dt);
  }
#endif
}
