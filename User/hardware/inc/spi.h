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

#ifndef HARDWARE_INC_SPI_H_
#define HARDWARE_INC_SPI_H_

#define UNUSE_SPI1 0
#define UNUSE_SPI2 1

void SPI_Handler();
void SPI_Enable_IT();
bool SPI_WriteReceive_IT(uint32_t *wb, uint32_t *rb, int length);
void SPI_ChangeBitWidth_32bit();
void SPI_ChangeBitWidth_16bit();
void SPI_ChangeBitWidth_8bit();
void SPI_ChangeMode_0();
void SPI_ChangeMode_3();

void SPI_Initialize();
void SPI_SwitchToSPI2();
void SPI_SelectAndDisableUnused(uint8_t use);
void SPI_SetPrescaler(uint32_t ll_baud);
void SPI_SetTStall(uint8_t t);
bool SPI_IsFree();
void SPI_ReceiveDmaHandler(void);
bool SPI_WriteReceive(uint32_t *wb, uint32_t *rb, int length);
bool SPI_WriteReceiveBurstRead(uint32_t *wb, uint32_t *rb, int length);
bool SPI_WriteReceiveBRF_IT(uint32_t *wb, uint32_t *rb, int length);
uint32_t SPI_GetCommTime();

#endif /* HARDWARE_INC_SPI_H_ */
