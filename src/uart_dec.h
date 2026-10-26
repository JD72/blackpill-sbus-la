#ifndef UART_DEC_H_
#define UART_DEC_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UDEC_FRAMES_MAX 64u
#define UDEC_LIST_MAX   7u
#define UDEC_TEXT_MAX   16u
#define UDEC_TIME_MAX   20u
#define UDEC_BUSES      2u
#define UDEC_RX         0u
#define UDEC_TX         1u

enum { UDEC_PAR_NONE, UDEC_PAR_ODD, UDEC_PAR_EVEN, UDEC_PAR_MARK, UDEC_PAR_SPACE };
enum { UDEC_FMT_BIN, UDEC_FMT_DEC, UDEC_FMT_HEX, UDEC_FMT_ASCII };
#define UDEC_ERR_STOP   3u
#define UDEC_ERR_PARITY 4u

typedef struct {
  bool on;
  bool uart;
  bool dcut;
  bool show;
  char rx[8], tx[8];
  int rx_ch;
  int tx_ch;
  uint32_t baud;
  uint8_t par;
  int stop;
  bool pol_high;
  bool msb;
  int dlen;
  uint8_t fmt;
} udec_cfg_t;

typedef struct {
  uint16_t start, end;
  uint16_t val;
  uint8_t err;
} udec_frame_t;

typedef struct {
  bool done;
  const char *off;
  uint32_t fs_mhz;
  uint32_t first;
  int32_t col;
  uint32_t baud;
  uint8_t fmt;
  uint8_t dlen;
  unsigned n;
  unsigned errors;
  uint32_t cycles;

  udec_frame_t f[UDEC_FRAMES_MAX];
} udec_res_t;

extern udec_cfg_t g_udec_cfg;
extern udec_cfg_t g_udec_cfg2;
extern udec_res_t g_udec;
extern udec_res_t g_udec_tx, g_udec2, g_udec2_tx;
extern unsigned g_udec_list;
extern const char *const udec_par_name[5];
extern const char *const udec_fmt_name[4];

udec_cfg_t *udec_cfg_of(unsigned bus);
udec_res_t *udec_res_of(unsigned bus, unsigned dir);

void udec_run(const uint8_t *blocks, uint32_t fs_mhz, uint32_t first, int32_t col);

bool udec_peek(const uint8_t *blocks, uint32_t fs_mhz, uint32_t first, int32_t col, unsigned ch, uint16_t *val,
               uint16_t *start);

void udec_fill(unsigned tag, uint8_t *buf, uint32_t at, uint32_t len);

bool udec_line(unsigned bus, unsigned k, unsigned *dir, unsigned *i);

void udec_text(unsigned k, char out[UDEC_TEXT_MAX]);
void udec_time(unsigned k, char out[UDEC_TIME_MAX]);
void udec_text_of(const udec_res_t *r, unsigned k, char out[UDEC_TEXT_MAX]);
void udec_time_of(const udec_res_t *r, unsigned k, char out[UDEC_TIME_MAX]);
void udec_fmt_time(int64_t t_ps, char out[UDEC_TIME_MAX]);

void udec_cfg_str(unsigned bus, char *buf, size_t n);
void udec_res_str(char *buf, size_t n);
void udec_buses_str(char *buf, size_t n);

#endif
