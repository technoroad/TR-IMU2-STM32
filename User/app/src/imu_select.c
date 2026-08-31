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
System_Status gSystem = { 0 };

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PROD_ID_ERROR (id == 0)
#define ADIS_CTRL_FREQ 1000

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static IMU_Parameter *param = &gSystem.param;
static IMU_Function *func = &gSystem.func;
static IMU_PersistConfig *conf = &gSystem.conf;

/* Private function prototypes -----------------------------------------------*/
static bool IMU_PorductSelector();

/**
 * @brief  Configure IMU related settings.
 */
bool IMU_Initialize(void) {
  // Hardware reset
  IMU_RESET_ON;
  DelayMillisecond(500);
  IMU_RESET_OFF;
  DelayMillisecond(500);

  // SPI1で発見できたらSPI2を止める
  if(IMU_PorductSelector()){
    SPI_SelectAndDisableUnused(UNUSE_SPI2);
  }else{
    // SPI1で発見できなければSPI2に切り替える
    SPI_SwitchToSPI2();

    // SPI2で発見できたらSPI1を止める
    if(IMU_PorductSelector()){
      SPI_SelectAndDisableUnused(UNUSE_SPI1);
    }else{
      // SPI1,SPI2にIMUが繋がっていなければfalseを返す
      return false;
    }
  }

  func->Configration();
  DelayMillisecond(500);
  GREEN_ON;

  return true;
}

bool IMU_PorductSelector() {
  SPI_SetPrescaler(SPI_1MHZ_PRES);
  IMU_RegisterType1Functions();
  uint16_t id = func->GetProductId();

  param->type = IMU_Type1;
  param->g_sens_32b = 0;
  param->tstall = 16;
  param->sample_rate = ADIS_CTRL_FREQ;
  param->conversion_rate = 2000;
  param->dec_rate = param->conversion_rate / param->sample_rate - 1 ;
  param->filter = 1;
  param->burst_cmd = TYPE1_BURST_CMD;
  param->brf_size_32b = 17;
  param->brf_size_16b = 10;
  param->g = 1.0;
  param->dt = 1.0 / param->sample_rate;
  param->maker = 1;
  param->exti_trigger = LL_EXTI_TRIGGER_FALLING;

  if (PROD_ID_ERROR) {
    IMU_RegisterType2Functions();
    id = func->GetProductId();
    if (PROD_ID_ERROR) {
      SPI_ChangeBitWidth_32bit();
      IMU_RegisterType3Functions();
      id = func->GetProductId();
    }
  }

  if(PROD_ID_ERROR){
    return false;
  }

  uint16_t model = func->GetRangeModel();

  if(conf->read32bit_enable){
    func->Read32bitEnable();
  }

  if (id == 16470) {
    // case by ADIS16470
    param->g_sens_32b = 655360.0;
    param->a_sens_32b = 52428800.0;
    param->g_sens_16b = param->g_sens_32b / 65536.0;
    param->a_sens_16b = param->a_sens_32b / 65536.0;
    model = 0;

  } else if (id == 16475) {
    // case by ADIS16475
    if(model == MDL_1BMLZ){
      param->g_sens_32b = 10485760.0;
    }else if(model == MDL_2BMLZ){
      param->g_sens_32b = 2621440.0;
    }else if(model == MDL_3BMLZ){
      param->g_sens_32b = 655360.0;
    }
    param->a_sens_32b = 262144000.0;
    param->g_sens_16b = param->g_sens_32b / 65536.0;
    param->a_sens_16b = param->a_sens_32b / 65536.0;

  } else if (id == 16477) {
    // case by ADIS16477-2
    if(model == MDL_1BMLZ){
      param->g_sens_32b = 10485760.0;
    }else if(model == MDL_2BMLZ){
      param->g_sens_32b = 2621440.0;
    }else if(model == MDL_3BMLZ){
      param->g_sens_32b = 655360.0;
    }
    param->a_sens_32b = 52428800.0;
    param->g_sens_16b = param->g_sens_32b / 65536.0;
    param->a_sens_16b = param->a_sens_32b / 65536.0;
    if (func->ReadData(0,TYPE1_FIRM_REV) >= 0x134) {
      // enable 32bit burst read
      func->EnableBurstRead();
    }
  } else if (id == 16500) {
    // case by ADIS16500
    param->g_sens_32b = 655360.0;
    param->a_sens_32b = 5351254.0;
    param->g_sens_16b = param->g_sens_32b / 65536.0;
    param->a_sens_16b = param->a_sens_32b / 65536.0;
    param->brf_size_32b = 18;
    param->brf_size_16b = 12;
    param->g = STANDARD_GRAVITY;
    model = 0;

    // enable 32bit burst read
    func->EnableBurstRead();

  } else if (id == 16505) {
    // case by ADIS16505
    if(model == MDL_1BMLZ){
      param->g_sens_32b = 10485760.0;
    }else if(model == MDL_2BMLZ){
      param->g_sens_32b = 2621440.0;
    }else if(model == MDL_3BMLZ){
      param->g_sens_32b = 655360.0;
    }
    param->a_sens_32b = 26756268.0;
    param->g_sens_16b = param->g_sens_32b / 65536.0;
    param->a_sens_16b = param->a_sens_32b / 65536.0;

    param->brf_size_32b = 18;
    param->brf_size_16b = 12;
    param->g = STANDARD_GRAVITY;

    // enable 32bit burst read
    func->EnableBurstRead();

  } else if (id == 16575) {
    // case by ADIS16575
    if(model == MDL_1BMLZ){
      /* none model */
    }else if(model == MDL_2BMLZ){
      param->g_sens_32b = 2621440.0;
    }else if(model == MDL_3BMLZ){
      param->g_sens_32b = 655360.0;
    }
    param->a_sens_32b = 262144000.0;
    param->g_sens_16b = param->g_sens_32b / 65536.0;
    param->a_sens_16b = param->a_sens_32b / 65536.0;

    param->brf_size_32b = 18;
    param->brf_size_16b = 11;
    param->tstall = 5;
    param->conversion_rate = 2000;
    param->dec_rate = param->conversion_rate / param->sample_rate - 1 ;
    param->filter = 2;

    // enable 32bit burst read
    func->EnableBurstRead();
  }else if (id == 16495) {
    param->type = IMU_Type2;
    param->tstall = 5;
    param->conversion_rate = 4000;
    param->dec_rate = param->conversion_rate / param->sample_rate - 1 ;
    param->filter = FILT_BANK_C;
    param->burst_cmd = TYPE2_BURST_CMD;
    param->brf_size_32b = 21;
    param->brf_size_16b = 20;

    if(model == MDL_1BMLZ){
      param->g_sens_32b = 10485760.0;
    }else if(model == MDL_2BMLZ){
      param->g_sens_32b = 2621440.0;
    }else if(model == MDL_3BMLZ){
      param->g_sens_32b = 655360.0;
    }
    param->a_sens_32b = 262144000.0;
    param->g_sens_16b = param->g_sens_32b / 65536.0;
    param->a_sens_16b = param->a_sens_32b / 65536.0;

    func->EnableBurstRead();
  }else if (id == 16607) {
    param->type = IMU_Type3;
    param->tstall = 1;
    param->conversion_rate = 8000;
    param->dec_rate = param->conversion_rate / param->sample_rate - 1 ;

    param->g_sens_32b = 16000.0;

    if(model == TYPE3_MDL_1BMLZ){
      model = 3;
      param->g_sens_32b = 64000.0;
    }else if(model == TYPE3_MDL_2BMLZ){
      model = 7;
      param->g_sens_32b = 16000.0;
    }else if(model == TYPE3_MDL_3BMLZ){
      model = 15;
      param->g_sens_32b = 4000.0;
    }else{
      model = 0;
    }

    param->a_sens_32b = 200000.0;
    param->g_sens_16b = param->g_sens_32b / 256.0;
    param->a_sens_16b = param->a_sens_32b / 256.0;

    param->exti_trigger = LL_EXTI_TRIGGER_RISING;
  }

  param->product_id = id;
  param->model = model;

  //TSTALLの設定
  SPI_SetTStall(param->tstall);

  if(param->type == IMU_Type1){
    SPI_SetPrescaler(SPI_1MHZ_PRES);
  }else if(param->type == IMU_Type2){
    SPI_SetPrescaler(SPI_2MHZ_PRES);
  }else if(param->type == IMU_Type3){
    SPI_SetPrescaler(SPI_2MHZ_PRES);
  }

  // センサの設定が読み込めたかチェック
  if (param->g_sens_32b != 0.0) {
    return true;
  } else {
    return false;
  }
}
