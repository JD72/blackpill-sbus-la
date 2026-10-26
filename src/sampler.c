#include <string.h>
#include "stm32f4xx_hal.h"
#include "sampler.h"

#define SAMPLER_FS_HZ 200000u

#define SAMPLER_PRIO_TIM 0u
#define SAMPLER_PRIO_USB 1u

#define SAMPLER_SNAP_TRIES 8u

_Static_assert((SAMPLER_RING_LEN & (SAMPLER_RING_LEN - 1u)) == 0u, "ring size must be a power of two");
_Static_assert(SAMPLER_RING_LEN > SAMPLER_COLS, "ring must hold more than one frame");
_Static_assert(SAMPLER_COLS == 8u * SAMPLER_BLK, "frame is 8 columns per byte");

static volatile uint16_t s_ring[SAMPLER_RING_LEN];
static volatile uint32_t s_head;
static volatile uint32_t s_valid;
static volatile uint32_t s_isr_max;

static uint32_t s_cfg_psc, s_cfg_arr;
static uint32_t s_cfg_fs_mhz;
static volatile uint32_t s_req_psc, s_req_arr;
static volatile uint32_t s_req_pending;

static uint64_t s_req_fs_uhz = (uint64_t) SAMPLER_FS_HZ * 1000000u;
static int64_t s_trdl_ps;

#define SAMPLER_APPLY_GUARD 32u

static void pins_input_pulldown(GPIO_TypeDef *port, uint32_t first, uint32_t last) {
  for (uint32_t pin = first; pin <= last; pin++) {
    port->PUPDR = (port->PUPDR & ~(3u << (2u * pin))) | (2u << (2u * pin));
    port->MODER &= ~(3u << (2u * pin));
  }
}

static uint32_t tim3_clock_hz(void) {
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE1) >= RCC_CFGR_PPRE1_DIV2) ? 2u * pclk1 : pclk1;
}

#define SAMPLER_FS_MAX_HZ 500000u
#define SAMPLER_TICKS_MAX 4294967296ull

static int sampler_calc_div(uint32_t clk_hz, uint64_t fs_uhz, uint32_t *psc, uint32_t *arr) {
  uint64_t ticks_min = ((uint64_t) clk_hz + SAMPLER_FS_MAX_HZ - 1u) / SAMPLER_FS_MAX_HZ;
  uint64_t ticks;
  int limit = 0;
  if (fs_uhz > (uint64_t) SAMPLER_FS_MAX_HZ * 1000000u) {
    ticks = ticks_min;
    limit = 1;
  } else if (fs_uhz == 0u) {
    ticks = SAMPLER_TICKS_MAX;
    limit = -1;
  } else {
    ticks = ((uint64_t) clk_hz * 1000000u + fs_uhz / 2u) / fs_uhz;
    if (ticks < ticks_min) {
      ticks = ticks_min;
    }
    if (ticks > SAMPLER_TICKS_MAX) {
      ticks = SAMPLER_TICKS_MAX;
      limit = -1;
    }
  }
  uint64_t p = 1u;
  if (ticks > 65536u) {
    p = (ticks + 65535u) / 65536u;
  }
  uint64_t a = (ticks + p / 2u) / p;
  *psc = (uint32_t) (p - 1u);
  *arr = (uint32_t) (a - 1u);
  return limit;
}

void sampler_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  (void) RCC->APB1ENR;

  pins_input_pulldown(GPIOA, 1u, 8u);
  pins_input_pulldown(GPIOB, 3u, 10u);

  TIM3->CR1 = TIM_CR1_URS | TIM_CR1_ARPE;
  sampler_calc_div(tim3_clock_hz(), (uint64_t) SAMPLER_FS_HZ * 1000000u, &s_cfg_psc, &s_cfg_arr);
  s_cfg_fs_mhz = (uint32_t) ((uint64_t) tim3_clock_hz() * 1000u / (((uint64_t) s_cfg_psc + 1u) * ((uint64_t) s_cfg_arr + 1u)));
  TIM3->PSC = s_cfg_psc;
  TIM3->ARR = s_cfg_arr;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = 0u;
  TIM3->DIER = TIM_DIER_UIE;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  NVIC_SetPriority(TIM3_IRQn, SAMPLER_PRIO_TIM);
  NVIC_SetPriority(OTG_FS_IRQn, SAMPLER_PRIO_USB);
  NVIC_EnableIRQ(TIM3_IRQn);

  TIM3->CR1 |= TIM_CR1_CEN;
}

void TIM3_IRQHandler(void) {
  uint32_t t0 = DWT->CYCCNT;
  uint32_t a = GPIOA->IDR;
  uint32_t b = GPIOB->IDR;
  TIM3->SR = ~TIM_SR_UIF;
  uint32_t h = s_head;
  s_ring[h & (SAMPLER_RING_LEN - 1u)] = (uint16_t) (((a >> 1) & 0x00FFu) | ((b << 5) & 0xFF00u));
  s_head = h + 1u;
  uint32_t v = s_valid;
  if (v < SAMPLER_RING_LEN) {
    s_valid = v + 1u;
  }

  if (s_req_pending != 0u && TIM3->CNT + SAMPLER_APPLY_GUARD <= TIM3->ARR) {
    TIM3->PSC = s_req_psc;
    TIM3->ARR = s_req_arr;
    s_req_pending = 0u;
    s_valid = 0u;
  }
  uint32_t d = DWT->CYCCNT - t0;
  if (d > s_isr_max) {
    s_isr_max = d;
  }
}

static void sampler_hold_drop(void);

sampler_rate_t sampler_set_rate(uint64_t fs_uhz) {
  sampler_rate_t r;
  s_req_fs_uhz = fs_uhz;
  uint32_t clk = tim3_clock_hz();
  r.limit = sampler_calc_div(clk, fs_uhz, &r.psc, &r.arr);
  r.fs_mhz = (uint32_t) ((uint64_t) clk * 1000u / (((uint64_t) r.psc + 1u) * ((uint64_t) r.arr + 1u)));
  r.changed = (r.psc != s_cfg_psc) || (r.arr != s_cfg_arr);
  if (r.changed) {
    s_cfg_psc = r.psc;
    s_cfg_arr = r.arr;
    s_cfg_fs_mhz = r.fs_mhz;
    s_req_pending = 0u;
    s_req_psc = r.psc;
    s_req_arr = r.arr;
    s_req_pending = 1u;
    s_valid = 0u;
    sampler_hold_drop();
  }
  return r;
}

static void sampler_copy(const volatile uint16_t *ring, uint32_t head, uint32_t valid, uint32_t len, uint16_t *cols) {
  uint32_t zeros = (valid >= len) ? 0u : len - valid;
  uint32_t idx = (head - len + zeros) & (SAMPLER_RING_LEN - 1u);
  uint32_t c = 0;
  for (; c + 4u <= zeros; c += 4u) {
    cols[c] = 0u;
    cols[c + 1u] = 0u;
    cols[c + 2u] = 0u;
    cols[c + 3u] = 0u;
  }
  for (; c < zeros; c++) {
    cols[c] = 0u;
  }
  while (c < len) {
    uint32_t n = SAMPLER_RING_LEN - idx;
    if (n > len - c) {
      n = len - c;
    }
    const volatile uint16_t *src = ring + idx;
    uint16_t *dst = cols + c;
    uint32_t m = n;
    for (; m >= 4u; m -= 4u) {
      dst[0] = src[0];
      dst[1] = src[1];
      dst[2] = src[2];
      dst[3] = src[3];
      dst += 4;
      src += 4;
    }
    for (; m != 0u; m--) {
      *dst++ = *src++;
    }
    c += n;
    idx = 0u;
  }
}

static void sampler_pack(const uint16_t *cols, uint8_t *blocks) {
  for (unsigned j = 0; j < SAMPLER_BLK; j++) {
    for (unsigned k = 0; k < SAMPLER_CH; k++) {
      unsigned v = 0;
      for (unsigned i = 0; i < 8u; i++) {
        v |= ((unsigned) (cols[8u * j + i] >> k) & 1u) << i;
      }
      blocks[k * SAMPLER_BLK + j] = (uint8_t) v;
    }
  }
}

#define SAMPLER_COL_NMAX 16384u

static int32_t sampler_col(int64_t trdl_ps, uint64_t fs_uhz) {
  uint64_t step = 1000000000000000000ull / (fs_uhz != 0u ? fs_uhz : 1u);
  uint64_t a = (uint64_t) (trdl_ps < 0 ? -trdl_ps : trdl_ps);
  uint64_t n = (a + step / 2u) / step;
  if (n > SAMPLER_COL_NMAX) {
    n = SAMPLER_COL_NMAX;
  }
  return (int32_t) (SAMPLER_COLS / 2u) + (trdl_ps < 0 ? -(int32_t) n : (int32_t) n);
}

#define SAMPLER_TRIG_SPAN 7168u

static int sampler_trig_start(const uint16_t *s, uint32_t valid, unsigned ch, int rise, uint32_t idle, int32_t col,
                              uint32_t *start) {
  uint32_t real = (valid >= SAMPLER_TRIG_SPAN) ? 0u : SAMPLER_TRIG_SPAN - valid;
  int64_t need = (col > 1) ? col : 1;
  if (!rise && idle > (uint64_t) need) {
    need = idle;
  }
  int64_t lo = (int64_t) real + need;
  int64_t hi = SAMPLER_TRIG_SPAN - 1;
  if (hi > (int64_t) SAMPLER_TRIG_SPAN - SAMPLER_COLS + col) {
    hi = (int64_t) SAMPLER_TRIG_SPAN - SAMPLER_COLS + col;
  }
  *start = SAMPLER_TRIG_SPAN - SAMPLER_COLS;
  if (hi < lo) {
    return 0;
  }
  for (uint32_t e = (uint32_t) hi + 1u; e-- > (uint32_t) lo;) {
    unsigned before = (s[e - 1u] >> ch) & 1u;
    unsigned after = (s[e] >> ch) & 1u;
    if (rise ? (before == 0u && after == 1u) : (before == 1u && after == 0u)) {
      uint32_t k = 1u;
      while (!rise && k <= idle && ((s[e - k] >> ch) & 1u) != 0u) {
        k++;
      }
      if (rise || k > idle) {
        *start = (uint32_t) ((int64_t) e - col);
        return 1;
      }
      e -= k - 1u;
    }
  }
  return 0;
}

#define SAMPLER_HOLD_MS 2500u

static uint8_t s_hold[SAMPLER_FRAME_LEN];
static uint32_t s_hold_fs_mhz, s_hold_first, s_hold_ms;
static int32_t s_hold_col;
static int s_hold_ok;

static void sampler_hold_drop(void) {
  s_hold_ok = 0;
}

static int sampler_hold(int found, const uint16_t *s, uint32_t start, uint32_t now_ms, uint8_t *blocks, uint32_t *fs_mhz,
                        uint32_t *first, int32_t col) {
  if (!found && s_hold_ok && s_hold_col == col && now_ms - s_hold_ms <= SAMPLER_HOLD_MS) {
    memcpy(blocks, s_hold, SAMPLER_FRAME_LEN);
    *fs_mhz = s_hold_fs_mhz;
    *first = s_hold_first;
    return 1;
  }
  sampler_pack(s + start, blocks);
  if (found) {
    memcpy(s_hold, blocks, SAMPLER_FRAME_LEN);
    s_hold_fs_mhz = *fs_mhz;
    s_hold_first = *first;
    s_hold_col = col;
    s_hold_ms = now_ms;
    s_hold_ok = 1;
  }
  return 0;
}

_Static_assert(SAMPLER_TRIG_SPAN < SAMPLER_RING_LEN, "ring must hold more than the trigger span");
_Static_assert(SAMPLER_TRIG_SPAN >= SAMPLER_COLS, "trigger span must cover a frame");

static uint16_t s_snap[SAMPLER_TRIG_SPAN];

static uint8_t s_trig_mode = SAMPLER_TRIG_OFF;
static uint8_t s_trig_ch;
static uint16_t s_trig_idle;
static uint32_t s_trig_found, s_trig_missed, s_trig_held;
static uint32_t s_copy_max, s_adv_max;
static uint32_t s_snap_fs_mhz, s_snap_first;
static int32_t s_snap_col;

void sampler_trig_set(uint8_t mode, uint8_t channel, uint16_t idle) {
  if (mode != s_trig_mode || channel != s_trig_ch || idle != s_trig_idle) {
    s_trig_mode = mode;
    s_trig_ch = channel;
    s_trig_idle = idle;
    s_trig_found = 0u;
    s_trig_missed = 0u;
    s_trig_held = 0u;
    s_copy_max = 0u;
    s_adv_max = 0u;
    s_isr_max = 0u;
    sampler_hold_drop();
  }
}

sampler_trig_t sampler_trig_get(void) {
  sampler_trig_t t = {s_trig_mode, s_trig_ch, s_trig_idle, s_trig_found, s_trig_missed, s_trig_held, s_copy_max, s_adv_max, s_isr_max};
  return t;
}

bool sampler_snapshot(uint8_t *blocks, int32_t col) {

  bool ok = false;
  bool trig = (s_trig_mode != SAMPLER_TRIG_OFF);
  uint32_t len = trig ? SAMPLER_TRIG_SPAN : SAMPLER_COLS;
  uint32_t v0 = 0u;
  for (unsigned t = 0; t < SAMPLER_SNAP_TRIES && !ok; t++) {
    uint32_t h0;
    unsigned r = 0;
    do {
      h0 = s_head;
      v0 = s_valid;
    } while (s_head != h0 && ++r < SAMPLER_SNAP_TRIES);
    uint32_t c0 = DWT->CYCCNT;
    sampler_copy(s_ring, h0, v0, len, s_snap);
    uint32_t cyc = DWT->CYCCNT - c0;
    uint32_t adv = (uint32_t) (s_head - h0);
    ok = adv <= SAMPLER_RING_LEN - len;
    if (cyc > s_copy_max) {
      s_copy_max = cyc;
    }
    if (adv > s_adv_max) {
      s_adv_max = adv;
    }
  }
  uint32_t start = 0u;
  int found = 0;
  if (trig) {
    found = sampler_trig_start(s_snap, v0, s_trig_ch, s_trig_mode == SAMPLER_TRIG_RISE, s_trig_idle, col, &start);
    if (found) {
      s_trig_found++;
    } else {
      s_trig_missed++;
    }
  }

  uint32_t zeros = (v0 >= len) ? 0u : len - v0;
  s_snap_first = (zeros > start) ? zeros - start : 0u;
  s_snap_fs_mhz = s_cfg_fs_mhz;
  s_snap_col = col;

  if (sampler_hold(found, s_snap, start, HAL_GetTick(), blocks, &s_snap_fs_mhz, &s_snap_first, col)) {
    s_trig_held++;
  }
  return ok;
}

uint32_t sampler_head_get(void) {
  return s_head;
}

static uint32_t s_pf_mark, s_pf_next, s_pf_edge;
static uint8_t s_pf_mode, s_pf_ch;
static bool s_pf_set, s_pf_have;

static bool sampler_since_prefilter(uint32_t mark) {
  uint32_t h = s_head;
  if (!s_pf_set || mark != s_pf_mark || s_pf_mode != s_trig_mode || s_pf_ch != s_trig_ch) {
    s_pf_set = true;
    s_pf_mark = mark;
    s_pf_mode = s_trig_mode;
    s_pf_ch = s_trig_ch;
    s_pf_next = mark;
    s_pf_have = false;
  }
  if (h - s_pf_next > SAMPLER_TRIG_SPAN - 1u) {
    s_pf_next = h - (SAMPLER_TRIG_SPAN - 1u);
  }
  unsigned ch = s_trig_ch;
  unsigned want = (s_trig_mode == SAMPLER_TRIG_RISE) ? 1u : 0u;
  unsigned prev = ((unsigned) s_ring[(s_pf_next - 1u) & (SAMPLER_RING_LEN - 1u)] >> ch) & 1u;
  for (uint32_t e = s_pf_next; e != h; e++) {
    unsigned v = ((unsigned) s_ring[e & (SAMPLER_RING_LEN - 1u)] >> ch) & 1u;
    if (v != prev && v == want) {
      s_pf_edge = e;
      s_pf_have = true;
    }
    prev = v;
  }
  s_pf_next = h;
  if (s_pf_have && h - s_pf_edge > SAMPLER_TRIG_SPAN - 1u) {
    s_pf_have = false;
  }
  return s_pf_have;
}

bool sampler_snapshot_since(uint8_t *blocks, int32_t col, uint32_t mark) {
  if (s_trig_mode == SAMPLER_TRIG_OFF || !sampler_since_prefilter(mark)) {
    return false;
  }
  bool ok = false;
  uint32_t h0 = 0u;
  uint32_t v0 = 0u;
  for (unsigned t = 0; t < SAMPLER_SNAP_TRIES && !ok; t++) {
    unsigned r = 0;
    do {
      h0 = s_head;
      v0 = s_valid;
    } while (s_head != h0 && ++r < SAMPLER_SNAP_TRIES);
    uint32_t c0 = DWT->CYCCNT;
    sampler_copy(s_ring, h0, v0, SAMPLER_TRIG_SPAN, s_snap);
    uint32_t cyc = DWT->CYCCNT - c0;
    uint32_t adv = (uint32_t) (s_head - h0);
    ok = adv <= SAMPLER_RING_LEN - SAMPLER_TRIG_SPAN;
    if (cyc > s_copy_max) {
      s_copy_max = cyc;
    }
    if (adv > s_adv_max) {
      s_adv_max = adv;
    }
  }
  uint32_t start = 0u;
  if (!ok || !sampler_trig_start(s_snap, v0, s_trig_ch, s_trig_mode == SAMPLER_TRIG_RISE, s_trig_idle, col, &start) ||
      SAMPLER_TRIG_SPAN - (uint32_t) ((int64_t) start + col) > h0 - mark) {
    return false;
  }
  s_trig_found++;
  uint32_t zeros = (v0 >= SAMPLER_TRIG_SPAN) ? 0u : SAMPLER_TRIG_SPAN - v0;
  s_snap_first = (zeros > start) ? zeros - start : 0u;
  s_snap_fs_mhz = s_cfg_fs_mhz;
  s_snap_col = col;
  sampler_hold(1, s_snap, start, HAL_GetTick(), blocks, &s_snap_fs_mhz, &s_snap_first, col);
  return true;
}

void sampler_snap_info(uint32_t *fs_mhz, uint32_t *first_col, int32_t *col) {
  *fs_mhz = s_snap_fs_mhz;
  *first_col = s_snap_first;
  *col = s_snap_col;
}

void sampler_trdl_set(int64_t trdl_ps) {
  s_trdl_ps = trdl_ps;
}

int64_t sampler_trdl_get(void) {
  return s_trdl_ps;
}

int32_t sampler_trdl_col(void) {
  return sampler_col(s_trdl_ps, s_req_fs_uhz);
}

uint64_t sampler_req_fs_get(void) {
  return s_req_fs_uhz;
}
