#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "uart1.h"
#include "log_ring.h"

#define LOG_RING_SIZE 8192u
#define LOG_RING_MASK (LOG_RING_SIZE - 1u)

static uint8_t ring[LOG_RING_SIZE];
static volatile uint16_t head = 0;
static volatile uint16_t tail = 0;
static volatile uint32_t dropped_count = 0;

static char drop_msg[32];
static uint8_t drop_len = 0;
static uint8_t drop_pos = 0;

volatile uint8_t g_log_level = 1;

void log_ring_init(void) {
  head = 0;
  tail = 0;
  dropped_count = 0;
  drop_len = 0;
  drop_pos = 0;
  g_log_level = 1;
}

static uint16_t ring_free(void) {
  uint16_t used = (uint16_t) (head - tail);
  return (uint16_t) (LOG_RING_SIZE - used);
}

void log_write(const char *data, size_t len) {
  if (len == 0 || len > LOG_RING_SIZE) {
    if (len > LOG_RING_SIZE) dropped_count++;
    return;
  }
  if (ring_free() < len) {
    dropped_count++;
    return;
  }
  for (size_t i = 0; i < len; i++) {
    ring[head & LOG_RING_MASK] = (uint8_t) data[i];
    head++;
  }
}

void log_puts(const char *s) {
  log_write(s, strlen(s));
}

void log_printf(const char *fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) {
    size_t len = (size_t) n;
    if (len > sizeof(buf) - 1) len = sizeof(buf) - 1;
    log_write(buf, len);
  }
}

static bool log_pump(void) {
  if (drop_pos < drop_len) {
    drop_pos = (uint8_t) (drop_pos + uart1_write_nb(drop_msg + drop_pos, (size_t) (drop_len - drop_pos)));
    if (drop_pos < drop_len) return true;
  }
  while (tail != head) {
    uint16_t t = (uint16_t) (tail & LOG_RING_MASK);
    uint16_t used = (uint16_t) (head - tail);
    uint16_t run = (uint16_t) (LOG_RING_SIZE - t);
    if (run > used) run = used;
    uint16_t sent = (uint16_t) uart1_write_nb(&ring[t], run);
    tail = (uint16_t) (tail + sent);
    if (sent < run) return true;
  }
  if (dropped_count > 0) {
    int n = snprintf(drop_msg, sizeof(drop_msg), "[log] dropped %lu\r\n", (unsigned long) dropped_count);
    dropped_count = 0;
    if (n > 0) {
      drop_len = (uint8_t) n;
      drop_pos = (uint8_t) uart1_write_nb(drop_msg, (size_t) n);
      return drop_pos < drop_len;
    }
  }
  return false;
}

void log_poll(void) {
  (void) log_pump();
}

void log_drain(void) {
  while (log_pump()) {
  }
}
