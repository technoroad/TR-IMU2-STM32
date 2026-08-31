/*
 * flash_ctrl.c
 *
 *  Created on: May 7, 2024
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BACKUP_ADDR (uint32_t)0x08002000
#define CSUM_SIZE 28

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static Flash_Settings kFlash = { 0 };
static uint8_t *kPtr = (uint8_t*) &kFlash;

/* Private function prototypes -----------------------------------------------*/
uint32_t kMakeSum(uint8_t *data, uint32_t size);

/* Global variables ----------------------------------------------------------*/
extern System_Status gSystem;

/**
 * @brief フラッシュメモリから設定を読み込む関数
 *
 * この関数は、マイコンの起動時にフラッシュメモリから設定を読み込みます。
 *
 * @param init_flg true 強制的に設定値を初期化します
 * @return 読み込んだ設定が正常に完了した場合はtrue、それ以外の場合はfalseを返します。
 */
void LoadSettingsFromFlash(bool init_flg) {
  uint32_t size = sizeof(kFlash);

  // フラッシュメモリからkFlashを読み込む
  FLASH_Load(BACKUP_ADDR, kPtr, size);

  // csum生成
  uint32_t sum = kMakeSum(kPtr, size - CSUM_SIZE);
  sum += kFlash.csum;  // csumだけ分解しないで足す

  bool same_date = false;

  // ビルド日時が同じかチェック
  if(kFlash.build_date == GetBuildDateInt()){
    if(kFlash.build_time == GetBuildTimeInt()){
      same_date = true;
    }
  }

  // csumが相違するか、ビルド日時が一致しなければkFlashを初期化したのちフラッシュを上書きする
  if (!CompareChecksum(sum) || !same_date || init_flg) {
    SetInitializeSettingsToStruct();
    SaveSettingsToFlash();
  }

  // 上位PCへ送信用の構造体にも反映
  memcpy(&gSystem.conf, &kFlash.conf, sizeof(kFlash.conf));
}

/**
 * @brief  kFlashをフラッシュメモリに書き込みます
 */
bool SaveSettingsToFlash() {
  //csumを生成
  uint32_t sum = kMakeSum(kPtr, sizeof(kFlash) - CSUM_SIZE);
  kFlash.csum = MakeChecksum(sum);

  // Flashに書き込む
  return FLASH_Store(BACKUP_ADDR, kPtr, sizeof(kFlash));
}

/**
 * @brief  kFlashを初期化します
 */
void SetInitializeSettingsToStruct() {
  memset(&kFlash, 0, sizeof(kFlash));  // 全体をゼロ初期化

  kFlash.conf.usb_enable = USB_PERIODIC_INIT;
  kFlash.conf.fdcan_enable = FDCAN_ENABLE_INIT;
  kFlash.conf.uart4_enable = UART4_ENABLE_INIT;
  kFlash.conf.read32bit_enable = READ32BIT_ENABLE_INIT;
  kFlash.conf.filter_select = FILTER_SEL_INIT;
  kFlash.conf.gravity_corr_enable = GRAV_CORR_EN_INIT;
  kFlash.conf.in0_pupd = IN0_PUPD_INIT;
  kFlash.conf.in0_trigger = IN0_TRIGGER_INIT;
  kFlash.build_date = GetBuildDateInt();
  kFlash.build_time = GetBuildTimeInt();

  // csum生成
  uint32_t sum = kMakeSum(kPtr, sizeof(kFlash) - CSUM_SIZE);
  kFlash.csum = MakeChecksum(sum);
}

/**
 * @brief  フラッシュに保存する構造体のポインタを返します。
 */
Flash_Settings* GetFlashSettingPtr() {
  return &kFlash;
}

/**
 * @brief  フラッシュに保存する設定を更新します。
 */
void SetFlashImuSetting(IMU_PersistConfig *config) {
  memcpy(&kFlash.conf, config, sizeof(*config));
}

/**
 * @brief  合計値の算出
 */
uint32_t kMakeSum(uint8_t *data, uint32_t size) {
  uint32_t sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return sum;
}
