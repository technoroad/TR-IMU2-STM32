/*
 * build_date.c
 *
 *  Created on: 2025/12/23
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
// 月略称 → 月番号（1-12）
#define BUILD_MONTH \
( __DATE__[0]=='J' && __DATE__[1]=='a' ? 1  : \
  __DATE__[0]=='F'                     ? 2  : \
  __DATE__[0]=='M' && __DATE__[2]=='r' ? 3  : \
  __DATE__[0]=='A' && __DATE__[1]=='p' ? 4  : \
  __DATE__[0]=='M' && __DATE__[2]=='y' ? 5  : \
  __DATE__[0]=='J' && __DATE__[2]=='n' ? 6  : \
  __DATE__[0]=='J' && __DATE__[2]=='l' ? 7  : \
  __DATE__[0]=='A' && __DATE__[2]=='g' ? 8  : \
  __DATE__[0]=='S'                     ? 9  : \
  __DATE__[0]=='O'                     ? 10 : \
  __DATE__[0]=='N'                     ? 11 : \
  __DATE__[0]=='D'                     ? 12 : \
  0 )

// 日（"dd"の左がスペースの可能性あり）
#define BUILD_DAY \
( (__DATE__[4]==' ' ? 0 : (__DATE__[4]-'0'))*10 + (__DATE__[5]-'0') )

// 年（"yyyy"）
#define BUILD_YEAR \
( (__DATE__[7]-'0')*1000 + (__DATE__[8]-'0')*100 + (__DATE__[9]-'0')*10 + (__DATE__[10]-'0') )

// 例：整数化（YYYYMMDD）
#define BUILD_DATE_INT (BUILD_YEAR*10000 + BUILD_MONTH*100 + BUILD_DAY)

// 時刻（"hh:mm:ss"）
#define BUILD_HOUR   ( (__TIME__[0]-'0')*10 + (__TIME__[1]-'0') )
#define BUILD_MINUTE ( (__TIME__[3]-'0')*10 + (__TIME__[4]-'0') )
#define BUILD_SECOND ( (__TIME__[6]-'0')*10 + (__TIME__[7]-'0') )

#define BUILD_TIME_INT (BUILD_HOUR*10000 + BUILD_MINUTE*100 + BUILD_SECOND)


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint32_t bdi = 0;
static uint32_t dti = 0;

/* Private function prototypes -----------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

// プログラム全体でビルド日時情報を統一するためにマイコン起動時に取得する。
void BuildDate_Initialize(){
  bdi = BUILD_DATE_INT;
  dti = BUILD_TIME_INT;
}


uint32_t GetBuildDateInt(){
  return bdi;
}


uint32_t GetBuildTimeInt(){
  return dti;
}
