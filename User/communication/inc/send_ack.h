/*
 * send_ack.h
 *
 *  Created on: 2022/11/22
 *      Author: techno-road
 */

#ifndef INC_SEND_ACK_H_
#define INC_SEND_ACK_H_

#include "libraries.h"

void CMD_SendAcknowledge();
void SetAcknowledgeCmd(BinaryPacket *recv);
void SetPeriodicCmd();

#endif /* INC_SEND_ACK_H_ */
