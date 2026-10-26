#ifndef UART_GEN_H_
#define UART_GEN_H_

#include <stdbool.h>
#include <stdint.h>

#define UGEN_BYTES_MAX   16u
#define UGEN_BAUD_MIN    1200u
#define UGEN_BAUD_MAX    19200u
#define UGEN_GAP_DEFAULT 60u

typedef struct {
  bool on;
  uint32_t baud;
  uint32_t arr;
  unsigned n;
  uint8_t b[UGEN_BYTES_MAX];
  unsigned gap;
  uint32_t sent;
} ugen_t;

void ugen_start(uint32_t baud, const uint8_t *b, unsigned n, unsigned gap);
void ugen_stop(void);
ugen_t ugen_get(void);

#endif
