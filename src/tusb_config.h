/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020, Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Adapted from the TinyUSB examples (examples/device/usbtmc).
 */

#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      0
#endif

#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED   OPT_MODE_FULL_SPEED
#endif

#ifndef CFG_TUSB_RHPORT0_MODE
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_TUD_MAX_SPEED)
#endif

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#define CFG_TUSB_OS            OPT_OS_NONE
#define CFG_TUSB_DEBUG         0

#define CFG_TUD_ENABLED        1
#define CFG_TUH_ENABLED        0

#define CFG_TUD_MAX_SPEED      BOARD_TUD_MAX_SPEED

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN     __attribute__ ((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC            0
#define CFG_TUD_MSC            0
#define CFG_TUD_HID            0
#define CFG_TUD_MIDI           0
#define CFG_TUD_AUDIO          0
#define CFG_TUD_VIDEO          0
#define CFG_TUD_DFU            0
#define CFG_TUD_DFU_RUNTIME    0
#define CFG_TUD_NET            0
#define CFG_TUD_BTH            0
#define CFG_TUD_ECM_RNDIS      0
#define CFG_TUD_VENDOR         0
#define CFG_TUD_USBTMC         1

#define CFG_TUD_USBTMC_ENABLE_488     1
#define CFG_TUD_USBTMC_ENABLE_INT_EP  1

#ifdef __cplusplus
}
#endif

#endif
