#include <stdbool.h>
#include "tusb.h"
#include "bsp/board_api.h"
#include "uart1.h"
#include "cli.h"
#include "chameleon_config.h"
#include "usb_events.h"
#include "log_ring.h"
#include "scpi_emul.h"
#include "sampler.h"

#define LED_BLINK_PERIOD_MS 500u

static void led_task(void) {
  static uint32_t last_toggle_ms = 0;
  static bool led_state = false;

  if (g_usb_configured) {
    board_led_write(true);
    return;
  }

  uint32_t now = tusb_time_millis_api();
  if (now - last_toggle_ms >= LED_BLINK_PERIOD_MS) {
    last_toggle_ms = now;
    led_state = !led_state;
    board_led_write(led_state);
  }
}

int main(void) {
  cli_reset_cause_init();
  board_init();
  uart1_init(115200);
  log_ring_init();
  scpi_emul_init();
  sampler_init();
  chameleon_config_load();

  tusb_init();
  cli_init();

  while (1) {
    tud_task();
    usb_events_poll();
    cli_poll();
    led_task();
    log_poll();
  }
}
