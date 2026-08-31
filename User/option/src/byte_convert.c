/*
 * byte_convert.c
 *
 *  Created on: 2022/11/28
 *      Author: techno-road
 */

/* Includes ------------------------------------------------------------------*/
#include "libraries.h"

// バイト列を32bit変数に変換
uint32_t ByteToU32(const uint8_t *dat) {
  return (dat[3] | dat[2] << 8 | dat[1] << 16 | dat[0] << 24);
}

// バイト列を16bit変数に変換
uint16_t ByteToU16(const uint8_t *dat) {
  return (dat[1] | dat[0] << 8);
}

// 32bit変数をバイト列に変換
uint32_t U32ToByte(uint32_t dat, uint8_t *buf) {
  buf[0] = dat & 0xFF;
  buf[1] = (dat >> 8) & 0xFF;
  buf[2] = (dat >> 16) & 0xFF;
  buf[3] = (dat >> 24) & 0xFF;

  return 4;  // バイト数を戻す
}

// 16bit変数をバイト列に変換
uint32_t U16ToByte(uint16_t dat, uint8_t *buf) {
  buf[0] = dat & 0xFF;
  buf[1] = (dat >> 8) & 0xFF;

  return 2;  // バイト数を戻す
}

// 64bit変数をバイト列に変換
uint32_t U64ToByte(uint64_t dat, uint8_t *buf) {
  buf[0] = dat & 0xFF;
  buf[1] = (dat >> 8) & 0xFF;
  buf[2] = (dat >> 16) & 0xFF;
  buf[3] = (dat >> 24) & 0xFF;
  buf[4] = (dat >> 32) & 0xFF;
  buf[5] = (dat >> 40) & 0xFF;
  buf[6] = (dat >> 48) & 0xFF;
  buf[7] = (dat >> 56) & 0xFF;

  return 8;  // バイト数を戻す
}
