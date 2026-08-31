/*
 * uart_ctrl.c
 *
 *  Created on: 2025/10/28
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  UART1_TxComplete_Handler(huart);
  UART4_TxComplete_Handler(huart);
}
