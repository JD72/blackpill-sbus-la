#ifndef USB_EVENTS_H_
#define USB_EVENTS_H_

#include <stdbool.h>

extern volatile bool g_usb_configured;

extern volatile bool g_usb_suspended;

void usb_events_poll(void);

#endif
