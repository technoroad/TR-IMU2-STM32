/*
 * configure_cmd.h
 *
 *  Created on: 2025/08/04
 *      Author: techno-road
 */

#ifndef COMMUNICATION_INC_CONFIGURE_CMD_H_
#define COMMUNICATION_INC_CONFIGURE_CMD_H_

ReturnTypeDef CmdGetConfigureFunc(BinaryPacket *packet);
ReturnTypeDef CmdSaveConfigureFunc(BinaryPacket *packet);
ReturnTypeDef CmdDefaultConfigureFunc(BinaryPacket *packet);

ReturnTypeDef CmdPeripheralEnableFunc(BinaryPacket *packet);
ReturnTypeDef CmdImuConfigureFunc(BinaryPacket *packet);
ReturnTypeDef CmdFilterSelectFunc(BinaryPacket *packet);
ReturnTypeDef CmdGravityCorrConfigureFunc(BinaryPacket *packet);
ReturnTypeDef CmdIn0ConfigureFunc(BinaryPacket *packet);

#endif /* COMMUNICATION_INC_CONFIGURE_CMD_H_ */
