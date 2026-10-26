#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sampler.h"
#include "scpi_emul.h"
#include "uart_dec.h"

#define DCVL_RX_OFF    80u
#define DCVL_TX_OFF    96080u
#define DCVL_LIST_OFF  192040u
#define DCVL_REC       96u
#define DCVL_TAIL1     307200u
#define DCLR_REC       64u
_Static_assert(DCVL_RX_OFF + DCVL_REC * UDEC_FRAMES_MAX <= DCVL_TX_OFF, "RX records reach the TX records");
_Static_assert(DCVL_TX_OFF + DCVL_REC * UDEC_FRAMES_MAX <= DCVL_LIST_OFF, "TX records reach the list");
_Static_assert(DCVL_LIST_OFF + DCVL_REC * UDEC_LIST_MAX <= DCVL_TAIL1, "list lines reach the tail");
_Static_assert(UDEC_LIST_MAX <= 7u, "the scope's list table holds 7 lines");
_Static_assert(DCVL_HALF2 + DCVL_TAIL1 + 4u == DCVL_TOTAL, "half 2 and its tail end the DCVL? reply");
#define PROTO_UART     2u
#define CAND_MAX       8u
#define SCORE_FRAMES   4u

udec_cfg_t g_udec_cfg = {.rx_ch = -1, .tx_ch = -1, .pol_high = true, .dlen = 8, .fmt = UDEC_FMT_ASCII};
udec_cfg_t g_udec_cfg2 = {.rx_ch = -1, .tx_ch = -1, .pol_high = true, .dlen = 8, .fmt = UDEC_FMT_ASCII};
udec_res_t g_udec, g_udec_tx, g_udec2, g_udec2_tx;
unsigned g_udec_list = 0;
static udec_cfg_t *const s_cfg[UDEC_BUSES] = {&g_udec_cfg, &g_udec_cfg2};
static udec_res_t *const s_res[UDEC_BUSES][2] = {{&g_udec, &g_udec_tx}, {&g_udec2, &g_udec2_tx}};

udec_cfg_t *udec_cfg_of(unsigned bus) {
  return s_cfg[bus & 1u];
}

udec_res_t *udec_res_of(unsigned bus, unsigned dir) {
  return s_res[bus & 1u][dir & 1u];
}

static const char s_ctrl[32][6] = {
    "[NUL]", "[SOH]", "[STX]", "[ETX]", "[EOT]", "[ENQ]", "[ACK]", "[BEL]", "[BS]",  "[HT]",  "[LF]",
    "[VT]",  "[FF]",  "[CR]",  "[SO]",  "[SI]",  "[DLE]", "[DC1]", "[DC2]", "[DC3]", "[DC4]", "[NAK]",
    "[SYN]", "[ETB]", "[CAN]", "[EM]",  "[SUB]", "[ESC]", "[FS]",  "[GS]",  "[RS]",  "[US]"};

static int chain(const udec_cfg_t *u, const uint8_t *blk, unsigned idle, uint32_t c, uint64_t step, uint64_t len,
                 unsigned maxf, udec_res_t *r) {
#define MARK(c) ((((unsigned) blk[(c) >> 3] >> ((c) & 7u)) & 1u) == idle)
  const unsigned dlen = (unsigned) u->dlen;
  const unsigned bits = 2u + dlen + (u->par != UDEC_PAR_NONE ? 1u : 0u);
  int score = 0;
  unsigned n = 0;
  uint32_t run = 1;
  for (; c < SAMPLER_COLS && n < maxf; c++) {
    if (MARK(c)) {
      run++;
      continue;
    }
    if (run == 0u) continue;
    run = 0;
    if ((uint64_t) c + len - 1u >= SAMPLER_COLS) break;

    const uint64_t mid0 = ((uint64_t) c << 16) + (step - 0x10000u) / 2u + 0x8000u;
#define AT(i) ((uint32_t) ((mid0 + (uint64_t) (i) * step) >> 16))
    if (MARK(AT(0u))) continue;
    unsigned val = 0, ones = 0;
    for (unsigned i = 1; i <= dlen; i++) {
      unsigned b = MARK(AT(i)) ? 1u : 0u;
      ones += b;
      val = u->msb ? (val << 1) | b : val | (b << (i - 1u));
    }
    uint8_t err = 0;
    if (u->par != UDEC_PAR_NONE) {
      unsigned p = MARK(AT(dlen + 1u)) ? 1u : 0u;
      bool ok = (u->par == UDEC_PAR_ODD) ? ((ones + p) & 1u) == 1u :
                (u->par == UDEC_PAR_EVEN) ? ((ones + p) & 1u) == 0u :
                (u->par == UDEC_PAR_MARK) ? p == 1u : p == 0u;
      if (!ok) err = UDEC_ERR_PARITY;
    }
    uint32_t stop = AT(bits - 1u);
    if (!MARK(stop)) err = UDEC_ERR_STOP;
    n++;
    score += (err != 0u) ? -1 : 1;
    if (r != NULL) {
      udec_frame_t *f = &r->f[r->n++];
      f->start = (uint16_t) c;
      f->end = (uint16_t) (c + len - 1u);
      f->val = (uint16_t) val;
      f->err = err;
      if (err != 0u) r->errors++;
    }
    c = stop;
    run = MARK(stop) ? 1u : 0u;
#undef AT
  }
#undef MARK
  return score;
}

static void run_one(unsigned bus, unsigned dir, const uint8_t *blocks, uint32_t fs_mhz, uint32_t first, int32_t col) {
  static const char *const no_proto[UDEC_BUSES] = {"bus 1 protocol not UART", "bus 2 protocol not UART"};
  static const char *const no_dcut[UDEC_BUSES] = {"no B1:DCUT settings", "no B2:DCUT settings"};
  static const char *const no_input[2] = {"RX not a D input", "TX not a D input"};
  const udec_cfg_t *u = s_cfg[bus];
  udec_res_t *r = s_res[bus][dir];
  const int ch = (dir == UDEC_TX) ? u->tx_ch : u->rx_ch;
  r->done = true;
  r->off = NULL;
  r->fs_mhz = (blocks != NULL) ? fs_mhz : 0u;
  r->first = first;
  r->col = col;
  r->baud = u->baud;
  r->fmt = u->fmt;
  r->dlen = (uint8_t) u->dlen;
  r->n = 0;
  r->errors = 0;
  r->cycles = 0;
  if (!u->on) {
    r->off = "DCST OFF";
  } else if (!u->uart) {
    r->off = no_proto[bus];
  } else if (!u->dcut) {
    r->off = no_dcut[bus];
  } else if (ch < 0) {
    r->off = no_input[dir];
  } else if (u->dlen < 5 || u->dlen > 8) {
    r->off = "DLEN not 5..8";
  } else if (blocks == NULL) {
    r->off = "no DI:WF? frame";
  } else if (u->baud == 0u || (uint64_t) u->baud * 4000u > fs_mhz) {
    r->off = "too few samples per bit";
  }
  if (r->off != NULL) return;

  const uint8_t *blk = blocks + (uint32_t) ch * SAMPLER_BLK;
  const unsigned idle = u->pol_high ? 1u : 0u;
#define MARK(c) ((((unsigned) blk[(c) >> 3] >> ((c) & 7u)) & 1u) == idle)
  const unsigned bits = 2u + (unsigned) u->dlen + (u->par != UDEC_PAR_NONE ? 1u : 0u);
  const uint64_t step = ((uint64_t) fs_mhz << 16) / ((uint64_t) u->baud * 1000u);
  const uint64_t len = ((uint64_t) bits * step) >> 16;

  uint32_t run = 0;
  uint32_t c0 = 0, best = 0;
  unsigned k = 0;
  int best_sc = 0;
  for (uint32_t c = first; c < SAMPLER_COLS; c++) {
    if (MARK(c)) {
      run++;
      continue;
    }
    if (run == 0u) continue;
    bool anchor = run >= len;
    run = 0;
    if (k == 0u) {
      c0 = c;
    } else if (c >= c0 + len) {
      break;
    }
    int sc = (k == 0u && anchor) ? 0 : chain(u, blk, idle, c, step, len, SCORE_FRAMES, NULL);
    if (k == 0u || sc > best_sc) {
      best_sc = sc;
      best = c;
    }
    if (anchor || ++k == CAND_MAX) break;
  }
#undef MARK
  if (best != 0u) chain(u, blk, idle, best, step, len, UDEC_FRAMES_MAX, r);
}

void udec_run(const uint8_t *blocks, uint32_t fs_mhz, uint32_t first, int32_t col) {
  for (unsigned bus = 0; bus < UDEC_BUSES; bus++) {
    run_one(bus, UDEC_RX, blocks, fs_mhz, first, col);
    run_one(bus, UDEC_TX, blocks, fs_mhz, first, col);
  }
}

bool udec_peek(const uint8_t *blocks, uint32_t fs_mhz, uint32_t first, int32_t col, unsigned ch, uint16_t *val,
               uint16_t *start) {
  const udec_cfg_t *u = &g_udec_cfg;
  if (col < (int32_t) first || col >= (int32_t) SAMPLER_COLS || u->dlen < 5 || u->dlen > 8 || u->baud == 0u ||
      (uint64_t) u->baud * 4000u > fs_mhz) {
    return false;
  }
  const unsigned bits = 2u + (unsigned) u->dlen + (u->par != UDEC_PAR_NONE ? 1u : 0u);
  const uint64_t step = ((uint64_t) fs_mhz << 16) / ((uint64_t) u->baud * 1000u);
  const uint64_t len = ((uint64_t) bits * step) >> 16;
  udec_res_t t;
  t.n = 0;
  t.errors = 0;
  chain(u, blocks + ch * SAMPLER_BLK, u->pol_high ? 1u : 0u, (uint32_t) col, step, len, 1u, &t);
  if (t.n == 0u) return false;
  *val = t.f[0].val;
  *start = t.f[0].start;
  return true;
}

void udec_text_of(const udec_res_t *r, unsigned k, char out[UDEC_TEXT_MAX]) {
  const udec_frame_t *f = &r->f[k];
  unsigned v = f->val;
  out[0] = 0;
  if (f->err == UDEC_ERR_STOP) return;
  switch (r->fmt) {
    case UDEC_FMT_BIN:
      for (unsigned i = 0; i < r->dlen; i++) {
        out[i] = (char) ('0' + ((v >> (r->dlen - 1u - i)) & 1u));
      }
      out[r->dlen] = 0;
      break;
    case UDEC_FMT_DEC:
      snprintf(out, UDEC_TEXT_MAX, "%u", v);
      break;
    case UDEC_FMT_HEX:
      snprintf(out, UDEC_TEXT_MAX, "0x%02X", v);
      break;
    default:
      if (v <= 0x1Fu) {
        strcpy(out, s_ctrl[v]);
      } else if (v <= 0x7Eu) {
        out[0] = (char) v;
        out[1] = 0;
      } else if (v == 0x7Fu) {
        strcpy(out, "[DEL]");
      } else {
        snprintf(out, UDEC_TEXT_MAX, "[%d]", (int) v);
      }
      break;
  }
}

void udec_fmt_time(int64_t t_ps, char out[UDEC_TIME_MAX]) {
  static const struct {
    const char *name;
    uint64_t ps;
  } unit[] = {{"s", 1000000000000ull}, {"ms", 1000000000ull}, {"us", 1000000ull}, {"ns", 1000ull}};
  uint64_t a = (t_ps < 0) ? (uint64_t) -t_ps : (uint64_t) t_ps;
  unsigned u = 0;
  while (u < 3u && a != 0u && a < unit[u].ps) u++;
  unsigned k = 1;
  for (uint64_t x = a / unit[u].ps; x >= 10u; x /= 10u) k++;
  unsigned dec = (k < 6u) ? 6u - k : 0u;
  uint64_t p10 = 1;
  for (unsigned i = 0; i < dec; i++) p10 *= 10u;
  uint64_t v = (unit[u].ps >= p10) ? (a + unit[u].ps / p10 / 2u) / (unit[u].ps / p10) : a * (p10 / unit[u].ps);
  if (dec > 0u && v >= 1000000u) {
    v /= 10u;
    p10 /= 10u;
    dec--;
    if (v / p10 >= 1000u && u > 0u) {
      u--;
      dec += 3u;
      p10 *= 1000u;
    }
  }
  const char *sign = (t_ps < 0) ? "-" : "";
  if (dec > 0u) {
    snprintf(out, UDEC_TIME_MAX, "%s%lu.%0*lu%s", sign, (unsigned long) (v / p10), (int) dec,
             (unsigned long) (v % p10), unit[u].name);
  } else {
    snprintf(out, UDEC_TIME_MAX, "%s%lu%s", sign, (unsigned long) v, unit[u].name);
  }
}

void udec_text(unsigned k, char out[UDEC_TEXT_MAX]) {
  udec_text_of(&g_udec, k, out);
}

void udec_time_of(const udec_res_t *r, unsigned k, char out[UDEC_TIME_MAX]) {
  int32_t d = (int32_t) r->f[k].start - r->col;
  uint64_t a = (uint64_t) (d < 0 ? -d : d) * 1000000000000000ull;
  int64_t t = (int64_t) ((a + r->fs_mhz / 2u) / r->fs_mhz);
  udec_fmt_time(d < 0 ? -t : t, out);
}

void udec_time(unsigned k, char out[UDEC_TIME_MAX]) {
  udec_time_of(&g_udec, k, out);
}

static void put(uint8_t *buf, uint32_t at, uint32_t len, uint32_t off, const uint8_t *src, uint32_t n) {
  uint32_t lo = (off > at) ? off : at;
  uint32_t hi = (off + n < at + len) ? off + n : at + len;
  for (uint32_t o = lo; o < hi; o++) {
    buf[o - at] = src[o - off];
  }
}

static void put_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t) v;
  p[1] = (uint8_t) (v >> 8);
  p[2] = (uint8_t) (v >> 16);
  p[3] = (uint8_t) (v >> 24);
}

static unsigned recs(const udec_res_t *r) {
  if (!r->done || r->off != NULL) return 0;
  return (r->n < UDEC_FRAMES_MAX) ? r->n : UDEC_FRAMES_MAX;
}

static bool bus_decoded(unsigned bus) {
  const udec_res_t *rx = s_res[bus][UDEC_RX], *tx = s_res[bus][UDEC_TX];
  return (rx->done && rx->off == NULL) || (tx->done && tx->off == NULL);
}

bool udec_line(unsigned bus, unsigned k, unsigned *dir, unsigned *i) {
  const udec_res_t *rx = s_res[bus & 1u][UDEC_RX], *tx = s_res[bus & 1u][UDEC_TX];
  const unsigned nr = recs(rx), nt = recs(tx);
  if (k >= nr + nt) return false;
  unsigned a = 0, b = 0;
  for (;; k--) {
    bool from_rx = a < nr && (b >= nt || rx->f[a].start <= tx->f[b].start);
    if (k == 0u) {
      *dir = from_rx ? UDEC_RX : UDEC_TX;
      *i = from_rx ? a : b;
      return true;
    }
    if (from_rx) {
      a++;
    } else {
      b++;
    }
  }
}

typedef struct {
  unsigned bus;
  const udec_res_t *r;
} rec_ctx_t;

static void rec_frame(const rec_ctx_t *x, unsigned k, uint8_t *rec) {
  const udec_frame_t *f = &x->r->f[k];
  udec_text_of(x->r, k, (char *) rec);
  if (f->err != 0u) memcpy(rec + 20, "Err", 4);
  put_u32(rec + 40, 1u);
  put_u32(rec + 48, f->start);
  put_u32(rec + 52, f->end);
}

static void rec_list(const rec_ctx_t *x, unsigned k, uint8_t *rec) {
  unsigned dir, i;
  if (!udec_line(x->bus, k, &dir, &i)) return;
  const udec_res_t *r = s_res[x->bus][dir];
  snprintf((char *) rec, 6, "%u", k + 1u);
  udec_time_of(r, i, (char *) rec + 20);
  udec_text_of(r, i, (char *) rec + 40);
  if (r->f[i].err != 0u) memcpy(rec + 60, "Err", 4);
  put_u32(rec + 76, dir);
}

static void rec_dclr(const rec_ctx_t *x, unsigned k, uint8_t *rec) {
  unsigned dir, i;
  if (!udec_line(x->bus, k, &dir, &i)) return;
  const udec_res_t *r = s_res[x->bus][dir];
  snprintf((char *) rec, 6, "%u", k + 1u);
  udec_time_of(r, i, (char *) rec + 6);
  udec_text_of(r, i, (char *) rec + 27);
  if (r->f[i].err != 0u) memcpy(rec + 43, "Err", 4);
  put_u32(rec + 60, dir);
}

static void put_recs(uint8_t *buf, uint32_t at, uint32_t len, uint32_t base, uint32_t step, unsigned count,
                     void (*build)(const rec_ctx_t *x, unsigned k, uint8_t *rec), const rec_ctx_t *x) {
  if (count == 0u || at + len <= base) return;
  uint8_t rec[DCVL_REC];
  for (unsigned k = (at > base) ? (at - base) / step : 0u; k < count && base + step * k < at + len; k++) {
    memset(rec, 0, step);
    build(x, k, rec);
    put(buf, at, len, base + step * k, rec, step);
  }
}

static void fill_half(unsigned bus, uint32_t half, uint8_t *buf, uint32_t at, uint32_t len) {
  if (!bus_decoded(bus) || at >= half + DCVL_TAIL1 + 4u || at + len <= half) return;
  const rec_ctx_t rx = {bus, s_res[bus][UDEC_RX]}, tx = {bus, s_res[bus][UDEC_TX]};
  const unsigned nr = recs(rx.r), nt = recs(tx.r);
  const unsigned lines = (nr + nt < UDEC_LIST_MAX) ? nr + nt : UDEC_LIST_MAX;
  uint8_t h[12];
  put_u32(h, lines);
  put_u32(h + 4, nr);
  put_u32(h + 8, nt);
  put(buf, at, len, half, h, sizeof(h));
  put_recs(buf, at, len, half + DCVL_RX_OFF, DCVL_REC, nr, rec_frame, &rx);
  put_recs(buf, at, len, half + DCVL_TX_OFF, DCVL_REC, nt, rec_frame, &tx);
  put_recs(buf, at, len, half + DCVL_LIST_OFF, DCVL_REC, lines, rec_list, &rx);
  uint8_t t[4];
  put_u32(t, PROTO_UART);
  put(buf, at, len, half + DCVL_TAIL1, t, sizeof(t));
}

void udec_fill(unsigned tag, uint8_t *buf, uint32_t at, uint32_t len) {
  if (tag == ZPAT_DCVL) {
    fill_half(0u, 0u, buf, at, len);
    fill_half(1u, DCVL_HALF2, buf, at, len);
    return;
  }
  const unsigned bus = g_udec_list & 1u;
  const udec_cfg_t *u = s_cfg[bus];
  if (!bus_decoded(bus) || !u->on || !u->uart) return;
  const rec_ctx_t x = {bus, NULL};
  const unsigned n = recs(s_res[bus][UDEC_RX]) + recs(s_res[bus][UDEC_TX]);
  if (tag == ZPAT_HEAD) {
    uint8_t h[16];
    put_u32(h, PROTO_UART);
    put_u32(h + 4, n);
    put_u32(h + 8, DCLR_REC);
    put_u32(h + 12, DCLR_REC * (n > 0u ? n : 1u));
    put(buf, at, len, 0u, h, sizeof(h));
  } else if (tag == ZPAT_DATA) {
    put_recs(buf, at, len, 0u, DCLR_REC, n, rec_dclr, &x);
  }
}

const char *const udec_par_name[5] = {"NONE", "ODD", "EVEN", "MARK", "SPACE"};
const char *const udec_fmt_name[4] = {"BIN", "DEC", "HEX", "ASCII"};

void udec_cfg_str(unsigned bus, char *buf, size_t n) {
  const udec_cfg_t *u = s_cfg[bus & 1u];
  snprintf(buf, n, "DIS %s, RX %s, TX %s, BAUD %lu, PAR %s, STOP %d, POL %s, BIT %s, DLEN %d", u->show ? "ON" : "OFF",
           u->rx, u->tx, (unsigned long) u->baud, udec_par_name[u->par], u->stop, u->pol_high ? "HIGH" : "LOW",
           u->msb ? "MSB" : "LSB", u->dlen);
}

void udec_res_str(char *buf, size_t n) {
  const udec_res_t *r = &g_udec;
  if (!r->done) {
    snprintf(buf, n, "none yet");
    return;
  }
  int k = snprintf(buf, n, "%s%s", r->off ? "off: " : "decoded", r->off ? r->off : "");
  if (r->fs_mhz != 0u && r->baud != 0u && k > 0 && (size_t) k < n) {
    uint32_t spb = (uint32_t) (r->fs_mhz / ((uint64_t) r->baud * 10u));
    k += snprintf(buf + k, n - (size_t) k, ", fs %lu.%03lu Hz, %lu.%02lu samples/bit, from column %lu",
                  (unsigned long) (r->fs_mhz / 1000u), (unsigned long) (r->fs_mhz % 1000u), (unsigned long) (spb / 100u),
                  (unsigned long) (spb % 100u), (unsigned long) r->first);
  }
  if (r->off == NULL && k > 0 && (size_t) k < n) {
    k += snprintf(buf + k, n - (size_t) k, ", %u frames, %u errors", r->n, r->errors);
  }
  if (k > 0 && (size_t) k < n) {
    snprintf(buf + k, n - (size_t) k, ", %lu cycles (%lu us)", (unsigned long) r->cycles,
             (unsigned long) (r->cycles / 84u));
  }
}

static void app(char *buf, size_t n, size_t *k, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
static void app(char *buf, size_t n, size_t *k, const char *fmt, ...) {
  if (*k + 1u >= n) return;
  va_list ap;
  va_start(ap, fmt);
  int m = vsnprintf(buf + *k, n - *k, fmt, ap);
  va_end(ap);
  if (m > 0) *k = (*k + (size_t) m < n) ? *k + (size_t) m : n - 1u;
}

void udec_buses_str(char *buf, size_t n) {
  size_t k = 0;
  buf[0] = 0;
  for (unsigned bus = 0; bus < UDEC_BUSES; bus++) {
    const udec_res_t *rx = s_res[bus][UDEC_RX], *tx = s_res[bus][UDEC_TX];
    app(buf, n, &k, "%sbus %u", bus ? "; " : "", bus + 1u);
    if (!rx->done) {
      app(buf, n, &k, " none yet");
    } else if (rx->off != NULL && rx->off == tx->off) {
      app(buf, n, &k, " off: %s", rx->off);
    } else {
      for (unsigned d = 0; d < 2u; d++) {
        const udec_res_t *r = d ? tx : rx;
        if (r->off != NULL) {
          app(buf, n, &k, "%s off: %s", d ? ", TX" : " RX", r->off);
        } else {
          app(buf, n, &k, "%s %u frames %u errors", d ? ", TX" : " RX", r->n, r->errors);
        }
      }
    }
  }
  app(buf, n, &k, "; list bus %u", (g_udec_list & 1u) + 1u);
}
