/*
 * filter_select.h
 *
 *  Created on: 2025/11/18
 *      Author: techno-road
 */

#ifndef APP_INC_FILTER_SELECT_H_
#define APP_INC_FILTER_SELECT_H_

typedef enum {
  List_NOP_Filter = 0,
  List_MKAE,
  List_VQF_Double,
  List_VQF_Mixed,
  List_VQF_Float,
  List_Fusion,
} FilterList;

void Filter_Initialize();

#endif /* APP_INC_FILTER_SELECT_H_ */
