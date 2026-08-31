/*
 * TIM6.c
 *
 *  Created on: 2022/08/19
 *      Author: kobayashi
 */
#include "libraries.h"

static UserEventHandlerPtr UserHandler = NULL;

// 使用するタイマーを選択してください
static TIM_TypeDef *TIMx = TIM6;

/**
 * @brief  TIMの割り込みハンドラ
 * @note  この関数はの"TIM_IRQHandler"関数で呼び出されます。
 */
void TIM6_Handler() {
  if (LL_TIM_IsActiveFlag_UPDATE(TIMx)) {
    LL_TIM_ClearFlag_UPDATE(TIMx);
    LL_TIM_DisableCounter(TIMx);  // 追加

    // 関数ポインタを使用して指定された関数を起動
    if (UserHandler != NULL) {
      (UserHandler)();
    }
  }
}

void TIM6_Initialize(uint16_t us){
  if(us == 0){
    us = 1;
  }

  // arrは155で固定
    LL_TIM_SetPrescaler(TIM6, us - 1);
//  LL_TIM_SetAutoReload(TIMx, 155);

  /* One Pulse Mode (1回だけ) */
  LL_TIM_SetOnePulseMode(TIMx, LL_TIM_ONEPULSEMODE_SINGLE);
}

/**
 * @brief  TIMのスタート関数
 */
void TIM6_Start() {
  LL_TIM_ClearFlag_UPDATE(TIMx);
  LL_TIM_SetCounter(TIMx,0);
  LL_TIM_EnableIT_UPDATE(TIMx);
  LL_TIM_EnableCounter(TIMx);
}

void TIM6_Stop(){
  LL_TIM_DisableIT_UPDATE(TIMx);
  LL_TIM_DisableCounter(TIMx);
}

/**
 * @brief  TIMの割り込み時に呼び出す関数を指定できます。
 * @param  handler : 関数ポインタ
 * @note
 */
void TIM6_AttachEventHandler(UserEventHandlerPtr handler) {
  //ユーザハンドラの設定
  UserHandler = handler;
}
