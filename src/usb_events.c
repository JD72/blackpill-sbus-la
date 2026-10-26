#include <stdio.h>
#include "tusb.h"
#include "class/usbtmc/usbtmc.h"
#include "device/dcd.h"
#include "log_ring.h"
#include "usb_events.h"

volatile bool g_usb_configured = false;
volatile bool g_usb_suspended = false;

enum { CHAM_EVT_BUS_RESET = 1, CHAM_EVT_UNPLUG };

#define EVTQ_SIZE 8u
static volatile uint8_t evtq[EVTQ_SIZE];
static volatile uint8_t evtq_head = 0;
static volatile uint8_t evtq_tail = 0;

static void evtq_push(uint8_t e) {
  uint8_t next = (uint8_t) ((evtq_head + 1u) % EVTQ_SIZE);
  if (next != evtq_tail) {
    evtq[evtq_head] = e;
    evtq_head = next;
  }
}

void usb_events_poll(void) {
  while (evtq_tail != evtq_head) {
    uint8_t e = evtq[evtq_tail];
    evtq_tail = (uint8_t) ((evtq_tail + 1u) % EVTQ_SIZE);
    if (e == CHAM_EVT_BUS_RESET) {
      log_printf("[t=%u ms][USB] bus reset\r\n", (unsigned) tusb_time_millis_api());
      g_usb_configured = false;
      g_usb_suspended = false;
    } else if (e == CHAM_EVT_UNPLUG) {
      log_printf("[t=%u ms][USB] unplugged\r\n", (unsigned) tusb_time_millis_api());
      g_usb_configured = false;
    }
  }
}

void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr) {
  (void) rhport;
  (void) in_isr;
  if (eventid == DCD_EVENT_BUS_RESET_END) {
    evtq_push(CHAM_EVT_BUS_RESET);
  } else if (eventid == DCD_EVENT_UNPLUGGED) {
    evtq_push(CHAM_EVT_UNPLUG);
  }
}

void tud_mount_cb(void) {
  g_usb_configured = true;
  g_usb_suspended = false;
  log_printf("[t=%u ms][USB] SET_CONFIGURATION -> configured\r\n", (unsigned) tusb_time_millis_api());
}

void tud_umount_cb(void) {
  g_usb_configured = false;
  log_printf("[t=%u ms][USB] unmounted\r\n", (unsigned) tusb_time_millis_api());
}

void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
  g_usb_suspended = true;
  log_printf("[t=%u ms][USB] suspend\r\n", (unsigned) tusb_time_millis_api());
}

void tud_resume_cb(void) {
  g_usb_suspended = false;
  log_printf("[t=%u ms][USB] resume\r\n", (unsigned) tusb_time_millis_api());
}

static const char *usbtmc_class_request_name(uint8_t bRequest) {
  switch (bRequest) {
    case USBTMC_bREQUEST_INITIATE_ABORT_BULK_OUT: return "INITIATE_ABORT_BULK_OUT";
    case USBTMC_bREQUEST_CHECK_ABORT_BULK_OUT_STATUS: return "CHECK_ABORT_BULK_OUT_STATUS";
    case USBTMC_bREQUEST_INITIATE_ABORT_BULK_IN: return "INITIATE_ABORT_BULK_IN";
    case USBTMC_bREQUEST_CHECK_ABORT_BULK_IN_STATUS: return "CHECK_ABORT_BULK_IN_STATUS";
    case USBTMC_bREQUEST_INITIATE_CLEAR: return "INITIATE_CLEAR";
    case USBTMC_bREQUEST_CHECK_CLEAR_STATUS: return "CHECK_CLEAR_STATUS";
    case USBTMC_bREQUEST_GET_CAPABILITIES: return "GET_CAPABILITIES";
    case USBTMC_bREQUEST_INDICATOR_PULSE: return "INDICATOR_PULSE";
    case USB488_bREQUEST_READ_STATUS_BYTE: return "READ_STATUS_BYTE";
    case USB488_bREQUEST_REN_CONTROL: return "REN_CONTROL";
    case USB488_bREQUEST_GO_TO_LOCAL: return "GO_TO_LOCAL";
    case USB488_bREQUEST_LOCAL_LOCKOUT: return "LOCAL_LOCKOUT";
    default: return NULL;
  }
}

void tud_setup_received_cb(uint8_t rhport, tusb_control_request_t const *request) {
  (void) rhport;
  log_printf("[t=%u ms][SETUP] bmRequestType=0x%02X bRequest=0x%02X wValue=0x%04X wIndex=0x%04X wLength=%u\r\n",
             (unsigned) tusb_time_millis_api(), request->bmRequestType, request->bRequest,
             request->wValue, request->wIndex, request->wLength);

  if (request->bmRequestType == 0x00 && request->bRequest == 0x05) {
    log_printf("[t=%u ms][SETUP] SET_ADDRESS -> %u\r\n",
               (unsigned) tusb_time_millis_api(), (unsigned) request->wValue);
  }

  if ((request->bmRequestType & 0x60u) == 0x20u && (request->bmRequestType & 0x1Fu) == 0x01u) {
    const char *name = usbtmc_class_request_name(request->bRequest);
    if (name != NULL) {
      log_printf("[t=%u ms][SETUP] USBTMC class request: %s\r\n", (unsigned) tusb_time_millis_api(), name);
    }
  }
}
