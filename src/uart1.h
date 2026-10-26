#ifndef UART1_H_
#define UART1_H_

#include <stdint.h>
#include <stddef.h>

void uart1_init(uint32_t baudrate);

void uart1_write(const void *buf, size_t len);
void uart1_puts(const char *s);

size_t uart1_write_nb(const void *buf, size_t len);

void uart1_wait_tx_complete(void);

size_t uart1_read(uint8_t *buf, size_t maxlen);

#endif
