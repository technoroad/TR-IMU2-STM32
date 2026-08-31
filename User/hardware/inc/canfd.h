/*
 * canfd.h
 *
 *  Created on: 2025/10/27
 *      Author: techno-road
 */

#ifndef HARDWARE_INC_CANFD_H_
#define HARDWARE_INC_CANFD_H_

#include "libraries.h"

void FDCAN_Initialize(void);
void FDCAN_Deinitialize();
HAL_StatusTypeDef CANFD_SendMessage(AcknowledgePacket *packet);
uint8_t FDCAN_GetDataLength(const FDCAN_RxHeaderTypeDef *hdr);

#endif /* HARDWARE_INC_CANFD_H_ */
