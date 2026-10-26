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

#include <string.h>
#include "tusb.h"
#include "usb_descriptors.h"
#include "chameleon_config.h"

enum { ITF_NUM_TMC = 0, ITF_NUM_TOTAL };

#define CHAM_CONFIG_TOTAL_LEN \
  (TUD_CONFIG_DESC_LEN + TUD_USBTMC_IF_DESCRIPTOR_LEN + \
   TUD_USBTMC_BULK_DESCRIPTORS_LEN + TUD_USBTMC_INT_DESCRIPTOR_LEN)

static tusb_desc_device_t desc_device;

uint8_t const *tud_descriptor_device_cb(void) {
  memset(&desc_device, 0, sizeof(desc_device));
  desc_device.bLength = sizeof(tusb_desc_device_t);
  desc_device.bDescriptorType = TUSB_DESC_DEVICE;
  desc_device.bcdUSB = g_desc.bcd_usb;
  desc_device.bDeviceClass = g_desc.dev_class;
  desc_device.bDeviceSubClass = g_desc.dev_subclass;
  desc_device.bDeviceProtocol = g_desc.dev_protocol;
  desc_device.bMaxPacketSize0 = g_desc.max_packet0;
  desc_device.idVendor = g_desc.vid;
  desc_device.idProduct = g_desc.pid;
  desc_device.bcdDevice = g_desc.bcd_device;
  desc_device.iManufacturer = CHAM_STRID_MANUF;
  desc_device.iProduct = CHAM_STRID_PRODUCT;
  desc_device.iSerialNumber = CHAM_STRID_SERIAL;
  desc_device.bNumConfigurations = 1;
  return (uint8_t const *) &desc_device;
}

static uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CHAM_CONFIG_TOTAL_LEN, 0x80, 100),
    TUD_USBTMC_IF_DESCRIPTOR(ITF_NUM_TMC, 3, 0, TUD_USBTMC_PROTOCOL_USB488),
    TUD_USBTMC_BULK_DESCRIPTORS(CHAM_EPNUM_TMC_OUT, CHAM_EPNUM_TMC_IN, CHAM_TMC_BULK_EPSIZE),
    TUD_USBTMC_INT_DESCRIPTOR(CHAM_EPNUM_TMC_INT, CHAM_TMC_INT_EPSIZE, CHAM_TMC_INT_INTERVAL),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index;
  return desc_configuration;
}

static uint16_t _desc_str[1 + CHAM_STR_MAX];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t chr_count;

  if (index == CHAM_STRID_LANGID) {
    _desc_str[1] = 0x0409;
    chr_count = 1;
  } else {
    char const *str;
    switch (index) {
      case CHAM_STRID_MANUF:   str = g_desc.mfr;    break;
      case CHAM_STRID_PRODUCT: str = g_desc.prod;   break;
      case CHAM_STRID_SERIAL:  str = g_desc.serial; break;
      default: return NULL;
    }

    chr_count = strlen(str);
    if (chr_count > CHAM_STR_MAX - 1) {
      chr_count = CHAM_STR_MAX - 1;
    }
    for (size_t i = 0; i < chr_count; i++) {
      _desc_str[1 + i] = (uint16_t) (uint8_t) str[i];
    }
  }

  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}
