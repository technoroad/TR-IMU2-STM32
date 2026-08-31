/*
 * canfd.c
 *
 *  Created on: 2025/10/27
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Global variables ----------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static FDCAN_TxHeaderTypeDef TxHeader;
static GPIO_PinConfig STB = {CAN_nSTBY_GPIO_Port,CAN_nSTBY_Pin};
static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/* Private function prototypes -----------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CAN_CHIP_ENABLE GPIO_Write(&STB,LOW)
#define CAN_CHIP_DISABLE GPIO_Write(&STB,HIGH)
#define REQUEST_ID_BASE 0x000
#define RESPONSE_ID_BASE 0x100

void FDCAN_Initialize(void) {
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  // FDCANの0x001 ~ 0x0FFまでのIDのコマンドしか受け付けない
  sFilterConfig.FilterID1 = REQUEST_ID_BASE | 0x01;
  sFilterConfig.FilterID2 = REQUEST_ID_BASE | 0xFF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
      != HAL_OK) {
    Error_Handler();
  }

  /* Prepare Tx Header for CAN FD */
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_64;  // 最大64バイト
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;     // データフェーズ高速化
  TxHeader.FDFormat = FDCAN_FD_CAN;          // CAN FDモード
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  CAN_CHIP_ENABLE;
}

void FDCAN_Deinitialize(){
  HAL_FDCAN_Stop(&hfdcan1);
  HAL_FDCAN_DeInit(&hfdcan1);
  CAN_CHIP_DISABLE;
  GPIO_DeInit(&STB);
}

HAL_StatusTypeDef CANFD_SendMessage(AcknowledgePacket *packet) {
  if (packet->data_length > 64) {
    return HAL_ERROR;  // 長さチェック
  }

  TxHeader.Identifier = RESPONSE_ID_BASE | packet->command;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, packet->data);
}

uint8_t FDCAN_GetDataLength(const FDCAN_RxHeaderTypeDef *hdr){
    return DLCtoBytes[hdr->DataLength & 0x0F];
}
