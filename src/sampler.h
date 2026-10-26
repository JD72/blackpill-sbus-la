#ifndef SAMPLER_H
#define SAMPLER_H

#include <stdbool.h>
#include <stdint.h>

#define SAMPLER_CH        16u
#define SAMPLER_COLS      1000u
#define SAMPLER_BLK       (SAMPLER_COLS / 8u)
#define SAMPLER_FRAME_LEN (SAMPLER_CH * SAMPLER_BLK)
#define SAMPLER_RING_LEN  8192u

void sampler_init(void);

typedef struct {
  uint32_t psc;
  uint32_t arr;
  uint32_t fs_mhz;
  int limit;
  bool changed;
} sampler_rate_t;

sampler_rate_t sampler_set_rate(uint64_t fs_uhz);

bool sampler_snapshot(uint8_t *blocks, int32_t col);

void sampler_snap_info(uint32_t *fs_mhz, uint32_t *first_col, int32_t *col);

void sampler_trdl_set(int64_t trdl_ps);
int64_t sampler_trdl_get(void);
int32_t sampler_trdl_col(void);

uint64_t sampler_req_fs_get(void);

#define SAMPLER_TRIG_OFF  0u
#define SAMPLER_TRIG_RISE 1u
#define SAMPLER_TRIG_FALL 2u

typedef struct {
  uint8_t mode;
  uint8_t channel;
  uint16_t idle;
  uint32_t found;
  uint32_t missed;
  uint32_t held;

  uint32_t copy_max;
  uint32_t adv_max;
  uint32_t isr_max;
} sampler_trig_t;

void sampler_trig_set(uint8_t mode, uint8_t channel, uint16_t idle);
sampler_trig_t sampler_trig_get(void);

uint32_t sampler_head_get(void);
bool sampler_snapshot_since(uint8_t *blocks, int32_t col, uint32_t mark);

#endif
