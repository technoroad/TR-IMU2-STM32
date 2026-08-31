/*
 * binary_parser.h
 *
 *  Created on: 2020/06/08
 *      Author: techno-road
 */

#ifndef BINARY_PARSER_H_
#define BINARY_PARSER_H_

#include "defines.h"

#define MAX_DATA_SIZE 64

typedef enum {
  kCheckHeader1,
  kCheckHeader2,
  kGetCommand,
  kGetDataLength,
  kGetData,
  kCompareChecksum1,
  kCompareChecksum2,
} BinaryCmdParserStatus;

typedef union {
  uint8_t bits;
  struct {
    uint8_t out_of_range: 1;
    uint8_t no_command  : 1;
    uint8_t flash_error : 1;
    uint8_t iwdg_error  : 1;
    uint8_t no_imu_error : 1;
    uint8_t reserved    : 3;
  };
} CommunicationErrorFlag;

typedef struct {
  BinaryCmdParserStatus status;
  uint8_t command;
  uint8_t data_length;
  uint8_t data[MAX_DATA_SIZE];
  uint8_t recv_counter;  // 内部処理専用のカウンタ
  CommunicationErrorFlag error;
  uint32_t sum;
} BinaryPacket;

void CMD_CheckReceiveData();
void CheckCommandErrorFlag(BinaryPacket *packet, ReturnTypeDef ret);
void MakeCommandError(BinaryPacket *packet);

#endif /* BINARY_PARSER_H_ */
