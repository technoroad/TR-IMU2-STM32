/*
 * byte_convert.h
 *
 *  Created on: 2022/11/28
 *      Author: techno-road
 */

#ifndef INC_BYTE_CONVERT_H_
#define INC_BYTE_CONVERT_H_

uint32_t ByteToU32(const uint8_t *dat);
uint16_t ByteToU16(const uint8_t *dat);
uint32_t U32ToByte(uint32_t dat, uint8_t *buf);
uint32_t U16ToByte(uint16_t dat, uint8_t *buf);
uint32_t U64ToByte(uint64_t dat, uint8_t *buf);

#endif /* INC_BYTE_CONVERT_H_ */
