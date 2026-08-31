/*
 * vqf_filter.c
 *
 *  Created on: 2025/10/31
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"
#include "vqf_double.h"
#include "vqf_mixed.h"
#include "vqf_float.h"

/* Global variables ----------------------------------------------------------*/
extern System_Status gSystem;
VQF_Function vqf_func = {0};

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
static ImuRawData *imu = &gSystem.raw;
static ImuComputationData *cmpt = &gSystem.cmpt;
static Filter_Function *func = &gSystem.filter;
static bool gravity_correnction_enable = true;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void VQF_Initialize(double dt);
static void VQF_ResetFilter();
static void VQF_Update();
static bool IsRest();

void VQF_RegisterFunctions(FilterList f){
  func->Initialize                 = NULL;
  func->Reset                      = NULL;
  func->Update                     = NULL;
  func->IsRest                     = NULL;

  func->Initialize                 = VQF_Initialize;
  func->Reset                      = VQF_ResetFilter;
  func->Update                     = VQF_Update;
  func->IsRest                     = IsRest;

  switch(f){
    case List_VQF_Double:{
      VQF_Double_RegisterFunctions();
      break;
    }
    case List_VQF_Mixed:{
      VQF_Mixed_RegisterFunctions();
      break;
    }
    case List_VQF_Float:{
      VQF_Float_RegisterFunctions();
      break;
    }
    /* ここにVQF以外は追加しないよう注意 */
    default :{
      VQF_Double_RegisterFunctions();
      break;
    }
  }
}

static void VQF_Initialize(double dt){
  vqf_func.Initialize(dt);
  gravity_correnction_enable = gSystem.conf.gravity_corr_enable;
}

static void VQF_ResetFilter(){
  vqf_func.Reset();
}

static void VQF_Update(){
  vqf_func.UpdateGyro(gSystem.raw.gyro.fpp);
  vqf_func.UpdateAccl(gSystem.raw.accl.fpp);

  if(gravity_correnction_enable){
    vqf_func.GetQuat6D(cmpt->quat);
  }else{
    vqf_func.GetQuat3D(cmpt->quat);
  }

  SystemError error = DetectSensorError(imu->diag_stat);
  if (error != kNoneError) {
    SetSystemError(error);
  }
}

static bool IsRest(){
  return vqf_func.GetRestDetected();
}
