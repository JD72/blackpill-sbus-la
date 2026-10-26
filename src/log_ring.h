#ifndef LOG_RING_H_
#define LOG_RING_H_

#include <stdint.h>
#include <stddef.h>

void log_ring_init(void);

extern volatile uint8_t g_log_level;

void log_write(const char *data, size_t len);
void log_puts(const char *s);
void log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

void log_poll(void);

void log_drain(void);

#endif
