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

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static Filter_Function *func = &gSystem.filter;
static IMU_PersistConfig *conf = &gSystem.conf;
static ImuComputationData *cmpt = &gSystem.cmpt;

/* Private function prototypes -----------------------------------------------*/
static void NOP_RegisterFunctions();

void Filter_Initialize(){
  cmpt->quat[0] = 1.0;
  cmpt->quat[1] = 0.0;
  cmpt->quat[2] = 0.0;
  cmpt->quat[3] = 0.0;

  switch(conf->filter_select){
    case List_NOP_Filter:{
      NOP_RegisterFunctions();
      break;
    }
    case List_MKAE:{
      MKAE_RegisterFunctions();
      break;
    }
    case List_VQF_Double:{
      VQF_RegisterFunctions(conf->filter_select);
      break;
    }
    case List_VQF_Mixed:{
      VQF_RegisterFunctions(conf->filter_select);
      break;
    }
    case List_VQF_Float:{
      VQF_RegisterFunctions(conf->filter_select);
      break;
    }
    case List_Fusion:{
      FUSION_RegisterFunctions();
      break;
    }

    default:{
      NOP_RegisterFunctions();
      break;
    }
  }
}

static void NOP_func1(double dt){/* nothing to do */}
static void NOP_func2(){/* nothing to do */}
static void NOP_func3(){/* nothing to do */}
static bool NOP_func4(){return 0;}

static void NOP_RegisterFunctions(){
  func->Initialize                 = NULL;
  func->Reset                      = NULL;
  func->Update                     = NULL;
  func->IsRest                     = NULL;

  func->Initialize                 = NOP_func1;
  func->Reset                      = NOP_func2;
  func->Update                     = NOP_func3;
  func->IsRest                     = NOP_func4;
}
