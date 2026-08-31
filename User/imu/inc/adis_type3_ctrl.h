/*
 The MIT License (MIT)

 Copyright (c) 2023 Techno Road Inc.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

#ifndef INC_ADIS_TYPE3_CTRL_H_
#define INC_ADIS_TYPE3_CTRL_H_

// 24bitデータの符号拡張マクロ
#define SIGN_EXT24(x)  (((x) & 0x00800000) ? ((x) | 0xFF000000) : ((x) & 0x00FFFFFF))

// ADIS16607
#define TYPE3_DIAG_STAT     0x05
#define TYPE3_X_ACCL_HIGH   0x06
#define TYPE3_X_ACCL_LOW    0x07
#define TYPE3_Y_ACCL_HIGH   0x08
#define TYPE3_Y_ACCL_LOW    0x09
#define TYPE3_Z_ACCL_HIGH   0x0A
#define TYPE3_Z_ACCL_LOW    0x0B
#define TYPE3_X_GYRO_HIGH   0x0C
#define TYPE3_X_GYRO_LOW    0x0D
#define TYPE3_Y_GYRO_HIGH   0x0E
#define TYPE3_Y_GYRO_LOW    0x0F
#define TYPE3_Z_GYRO_HIGH   0x10
#define TYPE3_Z_GYRO_LOW    0x11

#define TYPE3_TEMP_OUT      0x20
#define TYPE3_DATA_CNT      0x22
#define TYPE3_SPI_FULL_DPX  0x31
#define TYPE3_USER_GPIO_CFG1  0x2F
#define TYPE3_USER_DATA_CFG 0x34
#define TYPE3_DEC_RATE      0x3A
#define TYPE3_MSC_CTRL      0x39
#define TYPE3_WRITE_LOCK    0x2D
#define TYPE3_PROD_CMD      0x00
#define TYPE3_DIGI_STATUS   0x4E
#define TYPE3_RANG_MDL      0x41

#define TYPE3_BURST_CMD     0x68

#define TYPE3_MDL_1BMLZ  6071
#define TYPE3_MDL_2BMLZ  6072
#define TYPE3_MDL_3BMLZ  6073

void IMU_RegisterType3Functions();

#endif /* INC_ADIS_TYPE3_CTRL_H_ */
