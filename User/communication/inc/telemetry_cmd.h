/*
 * telemetry_cmd.h
 *
 *  Created on: 2025/08/04
 *      Author: techno-road
 */

#ifndef COMMUNICATION_INC_TELEMETRY_CMD_H_
#define COMMUNICATION_INC_TELEMETRY_CMD_H_

ReturnTypeDef CmdGetTelemetryFunc(BinaryPacket *packet);
ReturnTypeDef CmdStartPeriodicSendFunc(BinaryPacket *packet);
ReturnTypeDef CmdStopPeriodicSendFunc(BinaryPacket *packet);
ReturnTypeDef CmdResetPoseEstinateFunc(BinaryPacket *packet);
ReturnTypeDef CmdSystemResetFunc(BinaryPacket *packet);
ReturnTypeDef CmdJumpDfuFunc(BinaryPacket *packet);

#endif /* COMMUNICATION_INC_TELEMETRY_CMD_H_ */
