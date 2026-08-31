/*
 * fusion_filter.c
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Techno Road Inc.
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"
#include "fusion_imu.h"

/* Global variables ----------------------------------------------------------*/
extern System_Status gSystem;

/* Private variables ---------------------------------------------------------*/
static ImuComputationData *cmpt = &gSystem.cmpt;
static Filter_Function *func = &gSystem.filter;
static double *gyro = gSystem.raw.gyro.fpp;
static double *acc = gSystem.raw.accl.fpp;
static bool gravityCorrectionEnabled = true;

/* Private function prototypes -----------------------------------------------*/
static void initialize(double dt);
static void filter_reset(void);
static void update(void);
static bool IsRest(void);

void FUSION_RegisterFunctions(void) {
  func->Initialize = initialize;
  func->Reset = filter_reset;
  func->Update = update;
  func->IsRest = IsRest;
}

static void initialize(double dt) {
  FusionImu_Initialize((float) dt);
  gravityCorrectionEnabled = gSystem.conf.gravity_corr_enable;
}

static void filter_reset(void) {
  FusionImu_ResetOrientation();
}

static void update(void) {
  const float gx = (float) gyro[0];
  const float gy = (float) gyro[1];
  const float gz = (float) gyro[2];
  const float ax = (float) acc[0];
  const float ay = (float) acc[1];
  const float az = (float) acc[2];

  if (gravityCorrectionEnabled) {
    FusionImu_Update6Axis(gx, gy, gz, ax, ay, az);
  } else {
    FusionImu_UpdateGyroscope(gx, gy, gz);
  }
  FusionImu_GetQuaternion(cmpt->quat);
}

static bool IsRest(void) {
  return FusionImu_IsRest() ? true : false;
}
