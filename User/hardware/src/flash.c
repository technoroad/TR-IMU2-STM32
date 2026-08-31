/*
 * flash.c
 *
 *  Created on: May 7, 2024
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Private typedef -----------------------------------------------------------*/
#define BACKUP_FLASH_SECTOR_NUM   1
#define DOUBLEWORD_ONLY
#define ALIGN 16

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/**
 * @brief フラッシュメモリをクリアする関数
 *
 * この関数は、指定されたセクターのフラッシュメモリをクリアします。
 *
 * @return フラッシュメモリのクリアが成功したかどうかを表すブール値。成功した場合はtrue、失敗した場合はfalseを返します。
 */
bool FLASH_Clear() {
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page = BACKUP_FLASH_SECTOR_NUM;  // 対象のページ番号
  EraseInitStruct.NbPages = 1;                     // 消去するページ数
  EraseInitStruct.Banks = FLASH_BANK_1;   // 対象のバンク（FLASH_BANK_1またはFLASH_BANK_2）

  // エラーが発生したページ番号がerror_pageに入る
  uint32_t error_page;
  HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_page);

  HAL_FLASH_Lock();

  return result == HAL_OK && error_page == 0xFFFFFFFF;
}

/**
 * @brief フラッシュメモリからデータを読み込む関数
 *
 * この関数は、指定されたアドレスから指定されたサイズのデータを読み込みます。
 *
 * @param address 読み込みを開始するアドレス
 * @param data 読み込んだデータを格納するバッファへのポインタ
 * @param size 読み込むデータのサイズ（バイト単位）
 */
void FLASH_Load(uint32_t address, uint8_t *data, uint32_t size) {
  memcpy(data, (uint8_t*) address, size);
}

/**
 * @brief フラッシュメモリにデータを書き込む関数（STM32U535用）
 *
 * 指定されたアドレスに、クアッドワード（128bit）単位でデータを書き込みます。
 *
 * @param address 書き込みを開始するアドレス（16バイトアライン必須）
 * @param data 書き込むデータ（16バイトアライン）へのポインタ
 * @param size 書き込みサイズ（バイト単位、8の倍数であること）
 * @return 成功時true、失敗時false
 */
bool FLASH_Store(uint32_t address, uint8_t *data, uint32_t size) {
  // 書き込みは128bitアライン必須（16バイト単位）
  if ((address % ALIGN != 0) || (size % ALIGN != 0)) {
    return false;
  }

  if (!FLASH_Clear()) {
    return false;
  }

  HAL_FLASH_Unlock();

  HAL_StatusTypeDef result = HAL_OK;

  for (uint32_t i = 0; i < size; i += ALIGN) {
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address + i, (uint32_t)data + i);
    if (result != HAL_OK) {
      break;
    }
  }

  HAL_FLASH_Lock();

  return result == HAL_OK;
}
