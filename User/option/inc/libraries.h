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

// base file
#include "main.h"
#include "main_app.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "float.h"

// data defines
#include "defines.h"

// peripheral
#include "gpio.h"
#include "flash.h"
#include "flash_ctrl.h"
#include "iwdg.h"
#include "spi.h"
#include "crc32.h"
#include "checksum.h"
#include "pwm.h"
#include "tim6.h"
#include "canfd.h"
#include "uart1.h"
#include "uart4.h"
#include "uart_ctrl.h"

// option
#include "delay.h"
#include "str_edit.h"
#include "byte_convert.h"
#include "jump.h"
#include "build_date.h"

// sensors
#include <adis_type1_ctrl.h>
#include <adis_type2_ctrl.h>
#include <adis_type3_ctrl.h>
#include "imu_select.h"

// filter
#include "mkae_filter.h"
#include "vqf_filter.h"
#include "fusion_filter.h"
#include "filter_select.h"

// usb
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbprint.h"

// communication
#include "send_ack_hardware.h"
#include "send_ack.h"
#include "binary_parser.h"
#include "telemetry_cmd.h"
#include "configure_cmd.h"
#include "command_list.h"
#include "canfd_receive.h"

// management
#include "board.h"
#include "system_manager.h"
#include "system_error.h"

