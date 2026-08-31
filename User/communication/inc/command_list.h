/*
 * command_list.h
 *
 *  Created on: 2022/11/28
 *      Author: techno-road
 */

#ifndef INC_COMMAND_LIST_H_
#define INC_COMMAND_LIST_H_

#include <binary_parser.h>

#define HEADER1  (0xAA)
#define HEADER2  (0xAA)

#define INFO_SIZE 6  // パケットの情報部分のサイズ

typedef enum {
  // 定期送信
  PeriodicSend = 0x20,

  // テレメトリのコマンド
  CmdGetTelemetry = 0x30,
  CmdStartPeriodicSend = 0x31,
  CmdStopPeriodicSend  = 0x32,
  CmdResetPoseEstinate = 0x33,

  // 設定コマンド
  CmdGetConfigure = 0x70,
  CmdSaveConfigure = 0x71,
  CmdDefaultConfigure = 0x72,
  CmdPeripheralEnable = 0x73,
  CmdImuConfigure = 0x74,
  CmdFilterSelect = 0x75,
  CmdGravityCorrConfigure = 0x76,
  CmdIn0Configure = 0x77,

  // オプションコマンド
  CmdSystemReset = 0xB0,
  CmdJumpDfu = 0xB1,
} CommandList;

void AcceptReceivedCommand(BinaryPacket *packet);

#endif /* INC_COMMAND_LIST_H_ */
