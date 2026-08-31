/*
 * send_ack_hardware.h
 *
 *  Created on: 2025/11/10
 *      Author: techno-road
 */

#ifndef COMMUNICATION_INC_SEND_ACK_HARDWARE_H_
#define COMMUNICATION_INC_SEND_ACK_HARDWARE_H_

#define TLMT_DATA_SIZE 64
#define CONF_DATA_SIZE 64
#define ACK_DATA_SIZE TLMT_DATA_SIZE

typedef struct {
  uint8_t command;
  uint8_t data_length;
  uint8_t data[ACK_DATA_SIZE];
  uint16_t checksum;
  uint8_t error;
} AcknowledgePacket;

void TransmitAcknowledge(AcknowledgePacket *packet) ;
void CMD_SetPeriodicSend();
void CMD_PeriodicSendEnable();
void CMD_PeriodicSendDisable();

void CMD_canfd_selector();
void CMD_usb_selector();
void CMD_uart_selector();

#endif /* COMMUNICATION_INC_SEND_ACK_HARDWARE_H_ */
