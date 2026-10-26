#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

#include <stdint.h>

#define CHAM_EPNUM_TMC_OUT     0x01u
#define CHAM_EPNUM_TMC_IN      0x81u
#define CHAM_EPNUM_TMC_INT     0x82u
#define CHAM_TMC_BULK_EPSIZE   64u
#define CHAM_TMC_INT_EPSIZE    8u
#define CHAM_TMC_INT_INTERVAL  16u

#define CHAM_STRID_LANGID  0
#define CHAM_STRID_MANUF   1
#define CHAM_STRID_PRODUCT 2
#define CHAM_STRID_SERIAL  3

#endif
