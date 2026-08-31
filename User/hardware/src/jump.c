/*
 * jump.c
 *
 *  Created on: 2024/11/01
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"
#include "usb_device.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum{C0, F030x8, F030xC, F03xx, F05, F07, F09, F10xx, F105, F107, F10XL, F2, F3, F4, F7, G0, G4, H503, H563, H573, H7x, H7A, H7B, L0, L1, L4, L5, WBA, WBX, WL, U5};

#define MCU     U5   //Define here the MCU being used

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;
extern UART_HandleTypeDef huart4;

void JumpToBootloader (void)
{
  uint32_t i=0;
  void (*SysMemBootJump)(void);


  /* Set a vector addressed with STM32 Microcontrollers names */
  /* Each vector position contains an address to the boot loader entry point */

  volatile uint32_t BootAddr[33];

  BootAddr[C0]     = 0x1FFF0000;
  BootAddr[F030x8] = 0x1FFFEC00;
  BootAddr[F030xC] = 0x1FFFD800;
  BootAddr[F03xx]  = 0x1FFFEC00;
  BootAddr[F05]    = 0x1FFFEC00;
  BootAddr[F07]    = 0x1FFFC800;
  BootAddr[F09]    = 0x1FFFD800;
  BootAddr[F10xx]  = 0x1FFFF000;
  BootAddr[F105]   = 0x1FFFB000;
  BootAddr[F107]   = 0x1FFFB000;
  BootAddr[F10XL]  = 0x1FFFE000;
  BootAddr[F2]     = 0x1FFF0000;
  BootAddr[F3]     = 0x1FFFD800;
  BootAddr[F4]     = 0x1FFF0000;
  BootAddr[F7]     = 0x1FF00000;
  BootAddr[G0]     = 0x1FFF0000;
  BootAddr[G4]     = 0x1FFF0000;
  BootAddr[H503]   = 0x0BF87000;
  BootAddr[H563]   = 0x0BF97000;
  BootAddr[H573]   = 0x0BF97000;
  BootAddr[H7x]    = 0x1FF09800;
  BootAddr[H7A]    = 0x1FF0A800;
  BootAddr[H7B]    = 0x1FF0A000;
  BootAddr[L0]     = 0x1FF00000;
  BootAddr[L1]     = 0x1FF00000;
  BootAddr[L4]     = 0x1FFF0000;
  BootAddr[L5]     = 0x0BF90000;
  BootAddr[WBA]    = 0x0BF88000;
  BootAddr[WBX]    = 0x1FFF0000;
  BootAddr[WL]     = 0x1FFF0000;
  BootAddr[U5]     = 0x0BF90000;

  /* Disable all interrupts */
  __disable_irq();

  /* Disable Systick timer */
  SysTick->CTRL = 0;

  TIM6_Stop();
  HAL_PCD_Stop(&hpcd_USB_DRD_FS);
  HAL_PCD_DeInit(&hpcd_USB_DRD_FS);
  SPI_SelectAndDisableUnused(UNUSE_SPI1);
  SPI_SelectAndDisableUnused(UNUSE_SPI2);
  PWM_Deinitialize();
  HAL_UART_DMAStop(&huart4);
  HAL_UART_DeInit(&huart4);

  /* Set the clock to the default state */
  HAL_RCC_DeInit();

  /* Clear Interrupt Enable Register & Interrupt Pending Register */
  for (i=0;i<5;i++)
  {
    NVIC->ICER[i]=0xFFFFFFFF;
    NVIC->ICPR[i]=0xFFFFFFFF;
  }

  /* Re-enable all interrupts */
  // __enable_irq();

  /* Set up the jump to boot loader address + 4 */
  SysMemBootJump = (void (*)(void)) (*((uint32_t *) ((BootAddr[MCU] + 4))));

  /* Set the main stack pointer to the boot loader stack */
  __set_MSP(*(uint32_t *)BootAddr[MCU]);

  /* Call the function to jump to boot loader location */
  SysMemBootJump();

  /* Jump is done successfully */
  while (1)
  {
    // ウォッチドックリセット
    IWDG_Refresh();
    /* Code should never reach this loop */
  }
}
