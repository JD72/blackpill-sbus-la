#ifndef SCPI_EMUL_H_
#define SCPI_EMUL_H_

#include <stdbool.h>
#include <stdint.h>

#define DCVL_TOTAL   614408u
#define DCLR_HEAD_TOTAL 25u

#define DCVL_HALF2   307204u
#define DCVL_COUNTS  12u
#define DCLR_HEAD_COUNTS 16u
#define ZPAT_MAX     8u
#define ZPAT_LEN_MAX 16u
#define ZPAT_DCVL    0u
#define ZPAT_HEAD    1u
#define ZPAT_DATA    2u

typedef struct {
  uint32_t off;
  uint8_t  len;
  uint8_t  cmd;
  uint8_t  b[ZPAT_LEN_MAX];
} zpat_t;

extern zpat_t   g_zpat[ZPAT_MAX];
extern unsigned g_zpat_n;

bool scpi_emul_tx_busy(void);

bool scpi_emul_busy(void);

typedef struct {
  bool hism, hcomp, acq_set;
  int32_t c_acq;
  uint32_t corr;
} scpi_hism_t;
scpi_hism_t scpi_hism_get(void);
void scpi_hcomp_set(bool on);

typedef struct {
  bool on;
  uint8_t ch, mode;
  uint8_t uart_mode, uart_ch;
  uint16_t uart_idle;
  uint32_t ltv_max, ltv_hit_max;
} scpi_dtrig_t;
scpi_dtrig_t scpi_dtrig_get(void);

void scpi_trig_text(char *b, unsigned n, uint8_t mode, uint8_t ch, uint16_t idle);

#define SCPI_PAVA_OFF    0u
#define SCPI_PAVA_CALC   1u
#define SCPI_PAVA_PROBE  2u
#define SCPI_PAVA_PROBEI 3u
#define SCPI_PAVA_ITEMS  5u
#define SCPI_PAVA_HDR_WORDS 16u
#define SCPI_PAVA_BODY_LEN 344u

typedef struct {
  uint8_t  mode;
  bool     hdr_set;
  bool     duty_frac;
  int8_t   measc;
  uint32_t hdr[SCPI_PAVA_HDR_WORDS];
  uint8_t  n;
  uint32_t item[SCPI_PAVA_ITEMS][3];
} scpi_pava_t;
extern scpi_pava_t g_pava;

const uint8_t *scpi_pava_last(uint8_t *mode);

void scpi_dbl_text(char *b, unsigned n, double v);

void scpi_emul_init(void);

#endif
