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
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static SystemStatus status = Initialize_Status;
static IMU_PersistConfig *conf = &gSystem.conf;
static Filter_Function *filter = &gSystem.filter;
static bool data_ready = false;
// IMUのDR線
static GPIO_PinConfig DR = {ADIS_DR_GPIO_Port,ADIS_DR_Pin};
static GPIO_PinConfig IN0 = {IN0_GPIO_Port,IN0_Pin};

/* Private function prototypes -----------------------------------------------*/
void StartupFunc();
void UpdatePoseFunc();
void SystemErrorFunc();
void StartDataAcquisition();
bool CheckUpdateDataCounter();
bool Update6AxisData();
void UpdateInputPort();

bool System_Initialize(){
  StartupFunc();

  if(status == FatalError_Status){
//    SystemErrorFunc();
    return false;
  }

  return true;
}

/**

 */
void System_ImuManager() {
  if (GPIO_EXTI_WasTriggered(&DR)) {
    static Stopwatch calc_sw;

    data_ready = false;

    // watchdog refresh
    IWDG_Refresh();

    UpdateInputPort();

    gSystem.cmpt.time_us =micros64();

    switch (status) {
      case WaitWakeup_Status: {
        if(CheckUpdateDataCounter()){
          status = BiasSetup_Status;
        }
        break;
      }

      case BiasSetup_Status: {
        Update6AxisData();
        filter->Initialize(gSystem.param.dt);
        status = UpdatePose_Status;
        break;
      }

      case UpdatePose_Status: {
        ElapsedMicroseconds(&calc_sw);
        if(!Update6AxisData()){
          gSystem.cmpt.dropped++;
        }

        filter->Update();
        CMD_SetPeriodicSend();
        CMD_SendAcknowledge();
        gSystem.cmpt.computation_time = ElapsedMicroseconds(&calc_sw);
        break;
      }

      default: {
        SetSystemError(kUnknownError);
        break;
      }
    }
  }else if(status == FatalError_Status){
    // watchdog refresh
    IWDG_Refresh();
  }
}


/**
 * @brief  Transitions at startup and performs initial settings.
 * @note  When completed, transition to "kStartupWait".
 *  If the sensor ID is incorrect at this time, it transitions to "kFatalError".
 */
void StartupFunc() {
  // Acquisition of CONF switch information
  CheckConfigrationSwitch();

  if (gSystem.dip.DP1) {
    // DIPスイッチ1がオンの時は強制的にUSBオン
    CMD_usb_selector();
  }else{
    // DIPスイッチ1がオフの時は設定次第
    if(conf->usb_enable){
      CMD_usb_selector();
    }else{
      HAL_PCD_Stop(&hpcd_USB_DRD_FS);
      HAL_PCD_DeInit(&hpcd_USB_DRD_FS);
    }
  }

  if(conf->fdcan_enable){
    FDCAN_Initialize();
    CMD_canfd_selector();
  }else{
    FDCAN_Deinitialize();
  }

  if(conf->uart4_enable){
    UART4_Initialize();
    CMD_uart_selector();
  }else{
    UART4_Deinitialize();
  }

  // UART1は未使用
  UART1_Deinitialize();

  // IMU initialization
  if (IMU_Initialize()) {
    Filter_Initialize();

    SPI_Enable_IT();

    // Start pwm output for direct input sync
    PWM_Initialize();
    PWM_StartOutput();

    // ADIS_DRピンの設定
    GPIO_SetInputEXTI_NoIRQ(&DR,
                            LL_GPIO_PULL_NO,
                            gSystem.param.exti_trigger);

    uint32_t pupd = 0;
    uint32_t trigger = 0;

    if(gSystem.conf.in0_pupd == 0){
      pupd = LL_GPIO_PULL_NO;
    }else if(gSystem.conf.in0_pupd == 1){
      pupd = LL_GPIO_PULL_UP;
    }else if(gSystem.conf.in0_pupd == 2){
      pupd = LL_GPIO_PULL_DOWN;
    }

    if(gSystem.conf.in0_trigger == 0){
      trigger = LL_EXTI_TRIGGER_RISING;
    }else if(gSystem.conf.in0_trigger == 1){
      trigger = LL_EXTI_TRIGGER_FALLING;
    }else if(gSystem.conf.in0_trigger == 2){
      trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    }

    GPIO_SetInputEXTI_NoIRQ(&IN0,pupd,trigger);

    status = WaitWakeup_Status;
  } else {
    // An error occurs if the device ID is not a compatible model
    SetSystemError(kIdIncorrectError);
  }

  // Watchdog setting
  IWDG_Enable();
}

/**
 * @brief  Acquire data from sensors and estimate posture.
 * @note  There is no event to transition to "kFatalError" in the current version.
 */
void UpdatePoseFunc() {
}

/**
 * @brief  kFatalError : A function to stop when a fatal error occurs.
 * @note  In the current version, it only transitions from "kStartup".
 */
void SystemErrorFunc() {
  char str[128] = {0};

  DelayMillisecond(1000);
  IWDG_Refresh();

  str_concat(str, "Failed to communicate with the sensor.\r\n");

  USB_puts((uint8_t*)str);
  UART4_puts((uint8_t*)str);
  USB_SendIfReady();
  UART4_SendIfReady();
  DelayMillisecond(500);
//  HAL_NVIC_SystemReset();
}

/**
 * @brief  A function to execute when the DR pin is sensed.
 */
void ExtiInterrupt() {
  data_ready = true;
}

/**
 * @brief  Enter fatal error status in system manager.
 */
void SetSystemFatalError() {
  status = FatalError_Status;
}

/**
 * @brief Returns the state of the system manager.
 */
SystemStatus GetSystemStatus() {
  return status;
}

/**
 * @brief  Reset the system manager to
 *         the initial state and redo the settings.
 */
bool ResetSystemStatus() {
  status = Initialize_Status;
  return true;
}

bool CheckUpdateDataCounter(){
  static int count = 0;

  if(Update6AxisData()){
    count++;
  }else{
    count = 0;
  }

  if(count > 10){
    count = 0;
    return true;
  }

  return false;
}

/**
 * @brief
 */
bool Update6AxisData() {
  static uint16_t p_count = 0;
  bool ret = false;

  gSystem.func.Update6AxisData();
  gSystem.func.CopyRawData(&gSystem.raw);

  if (((p_count + 1)%65536) == gSystem.raw.data_cnt) {
    ret = true;
  } else {
    ret = false;
  }
  p_count = gSystem.raw.data_cnt;

  return ret;
}

void UpdateInputPort(){
  if(GPIO_EXTI_WasTriggered(&IN0)){
    gSystem.cmpt.in0 |= 0x01;
  }else{
    gSystem.cmpt.in0 &= ~0x01;
  }

  if(GPIO_Read(&IN0)){
    gSystem.cmpt.in0 |= 0x02;
  }else{
    gSystem.cmpt.in0 &= ~0x02;
  }
}
