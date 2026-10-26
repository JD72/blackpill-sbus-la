#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "stm32f4xx_hal.h"
#include "tusb.h"
#include "chameleon_config.h"
#include "log_ring.h"
#include "scpi_emul.h"
#include "sampler.h"
#include "uart_dec.h"
#include "usb_events.h"

#define SCPI_MSG_BUF_SIZE 256u

#define CHAM_MARK_PORT GPIOB
#define CHAM_MARK_PIN  GPIO_PIN_0

static void mark_init(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = CHAM_MARK_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(CHAM_MARK_PORT, &gpio);
  HAL_GPIO_WritePin(CHAM_MARK_PORT, CHAM_MARK_PIN, GPIO_PIN_RESET);
}

void scpi_emul_init(void) {
  mark_init();
}

static uint8_t s_msg_buf[SCPI_MSG_BUF_SIZE];
static size_t  s_msg_len = 0;
static bool    s_msg_active = false;
static bool    s_msg_eom = false;
static bool    s_msg_overflow = false;

static char   s_resp_buf[CHAM_RESP_VAL_MAX + 1];
static size_t s_resp_len = 0;
static volatile bool s_tx_in_flight = false;

#define WF_HDR_LEN  6u
#define WF_RESP_LEN (WF_HDR_LEN + SAMPLER_FRAME_LEN)

static uint8_t s_wf_buf[WF_RESP_LEN];
static bool    s_resp_is_wf = false;
static bool    s_wf_ready = false;

static bool     s_wf_shown = false;
static uint32_t s_wf_fs_mhz = 0;
static uint32_t s_wf_first = 0;

static int32_t  s_wf_col = 500;

static bool     s_hism = false;
static bool     s_hcomp = true;
static bool     s_acq_set = false;
static int32_t  s_c_acq = 0;
static uint64_t s_step_acq = 0;
static uint32_t s_hcomp_n = 0;
static uint32_t s_wf_n = 0;
static bool     s_wf_held = false;
static bool     s_wf_ltv = false;

static bool     s_dt_on = false;
static uint8_t  s_dt_ch = 0;
static uint16_t s_dt_fall = 0;
static sampler_trig_t s_dt_uart;
static uint32_t s_dt_mark = 0;
static bool     s_dt_hit = false;
static uint32_t s_dt_cyc_max = 0;
static uint32_t s_dt_cyc_hit_max = 0;

#define CMPA_LEN      69u
#define PAVA_HDR_LEN  8u
#define PAVA_RESP_LEN (PAVA_HDR_LEN + SCPI_PAVA_BODY_LEN)
#define PAVA_SRC_D0   13u
#define PAVA_NONE     DBL_MAX

scpi_pava_t g_pava = {.mode = SCPI_PAVA_CALC, .measc = -1};
static uint8_t s_pava_buf[PAVA_RESP_LEN];
static bool    s_resp_is_pava = false;
static bool    s_pava_built = false;
static uint8_t s_pava_mode = SCPI_PAVA_OFF;
static double  s_pava_calc[SCPI_PAVA_ITEMS];
static double  s_pava_hcalc[8];

static const char *s_zero_cmd = NULL;
static unsigned s_zero_reads = 0;
static uint32_t s_zero_bytes = 0;
static uint32_t s_zero_req = 0;
static uint32_t s_zero_t_cmd = 0;
static uint32_t s_zero_t_req = 0;
static uint32_t s_zero_t_last = 0;
static int      s_zero_tag = -1;

zpat_t   g_zpat[ZPAT_MAX];
unsigned g_zpat_n = 0;
static zpat_t   s_zpat[ZPAT_MAX];
static unsigned s_zpat_n = 0;
static uint32_t s_zpat_sent = 0;

static bool wf_take_snapshot(const char *why, int32_t col) {
  if (s_tx_in_flight) {
    if (g_log_level >= 1) {
      log_printf("[t=%u ms][TMC] %s: frame is being sent, snapshot skipped\r\n",
                 (unsigned) tusb_time_millis_api(), why);
    }
    return false;
  }
  memcpy(s_wf_buf, "DI:WF ", WF_HDR_LEN);
  uint32_t held0 = sampler_trig_get().held;
  bool ok = sampler_snapshot(s_wf_buf + WF_HDR_LEN, col);
  s_wf_held = sampler_trig_get().held != held0;
  s_wf_ltv = false;
  sampler_snap_info(&s_wf_fs_mhz, &s_wf_first, &s_wf_col);
  s_wf_ready = true;
  s_wf_shown = false;
  if (g_log_level >= 1) {
    log_printf("[t=%u ms][TMC] %s: snapshot of %u samples%s\r\n",
               (unsigned) tusb_time_millis_api(), why, (unsigned) SAMPLER_COLS, ok ? "" : " (TORN)");
  }
  return true;
}

static void wf_drop_other_col(void) {
  if (s_wf_ready && s_wf_col != sampler_trdl_col()) {
    s_wf_ready = false;
  }
}

bool scpi_emul_tx_busy(void) {
  return s_tx_in_flight;
}

bool scpi_emul_busy(void) {
  return g_usb_configured && !g_usb_suspended && (s_tx_in_flight || s_zero_cmd != NULL);
}

static void pava_forget(void) {
  g_pava.n = 0;
  memset(g_pava.item, 0, sizeof(g_pava.item));
  g_pava.measc = -1;
}

scpi_hism_t scpi_hism_get(void) {
  scpi_hism_t h = {s_hism, s_hcomp, s_acq_set, s_c_acq, s_hcomp_n};
  return h;
}

void scpi_hcomp_set(bool on) {
  s_hcomp = on;
}

scpi_dtrig_t scpi_dtrig_get(void) {
  scpi_dtrig_t d = {s_dt_on, s_dt_ch, ((s_dt_fall >> s_dt_ch) & 1u) ? SAMPLER_TRIG_FALL : SAMPLER_TRIG_RISE,
                    s_dt_uart.mode, s_dt_uart.channel, s_dt_uart.idle, s_dt_cyc_max, s_dt_cyc_hit_max};
  return d;
}

void scpi_trig_text(char *b, unsigned n, uint8_t mode, uint8_t ch, uint16_t idle) {
  if (mode == SAMPLER_TRIG_OFF) {
    snprintf(b, n, "off");
  } else if (idle != 0u) {
    snprintf(b, n, "%u %s %u", (unsigned) ch, (mode == SAMPLER_TRIG_RISE) ? "rise" : "fall", (unsigned) idle);
  } else {
    snprintf(b, n, "%u %s", (unsigned) ch, (mode == SAMPLER_TRIG_RISE) ? "rise" : "fall");
  }
}

void scpi_dbl_text(char *b, unsigned n, double v) {
  if (v == 0.0) {
    snprintf(b, n, "0");
    return;
  }
  if (v == DBL_MAX) {
    snprintf(b, n, "max");
    return;
  }
  if (v != v || v > 1e300 || v < -1e300) {
    snprintf(b, n, v != v ? "nan" : (v > 0.0 ? "inf" : "-inf"));
    return;
  }
  double a = v < 0.0 ? -v : v;
  int e = 0;
  while (a >= 10.0) {
    a /= 10.0;
    e++;
  }
  while (a < 1.0) {
    a *= 10.0;
    e--;
  }
  uint32_t m = (uint32_t) (a * 100000.0 + 0.5);
  if (m >= 1000000u) {
    m /= 10u;
    e++;
  }
  snprintf(b, n, "%s%lu.%05lue%d", v < 0.0 ? "-" : "", (unsigned long) (m / 100000u), (unsigned long) (m % 100000u), e);
}

const uint8_t *scpi_pava_last(uint8_t *mode) {
  *mode = s_pava_mode;
  return s_pava_built ? s_pava_buf + PAVA_HDR_LEN : NULL;
}

#define LOG_FULL_MAX 160u
static void log_ascii_hex(const char *tag, const uint8_t *data, size_t len, size_t ascii_max) {
  if (g_log_level < 1) return;

  char line[384];
  size_t shown = len > ascii_max ? ascii_max : len;
  char ascii[LOG_FULL_MAX + 1u];
  if (shown > LOG_FULL_MAX) shown = LOG_FULL_MAX;
  for (size_t i = 0; i < shown; i++) {
    uint8_t c = data[i];
    ascii[i] = (c >= 0x20 && c < 0x7F) ? (char) c : '.';
  }
  ascii[shown] = 0;

  int n = snprintf(line, sizeof(line), "[t=%u ms]%s len=%u ascii=\"%s\"",
                    (unsigned) tusb_time_millis_api(), tag, (unsigned) len, ascii);
  shown = len > 48u ? 48u : len;
  if (g_log_level >= 2) {
    n += snprintf(line + n, sizeof(line) - (size_t) n, " hex=");
    for (size_t i = 0; i < shown && (size_t) n < sizeof(line) - 4; i++) {
      n += snprintf(line + n, sizeof(line) - (size_t) n, "%02X ", data[i]);
    }
    if (len > shown && (size_t) n < sizeof(line) - 4) {
      n += snprintf(line + n, sizeof(line) - (size_t) n, "...");
    }
  }
  if (n > 0 && (size_t) n < sizeof(line) - 2) {
    line[n++] = '\r';
    line[n++] = '\n';
  }
  log_write(line, (size_t) n);
}

static bool log_full_text(const uint8_t *data, size_t len) {
  static const char *const pre[] = {"B1:", "B2:", "DCPA", "DCDI", "DCST", "MEASC"};
  for (size_t i = 0; i < sizeof(pre) / sizeof(pre[0]); i++) {
    size_t k = strlen(pre[i]);
    if (len >= k && memcmp(data, pre[i], k) == 0) return true;
  }
  return false;
}

#define RX_HEX_MAX 128u
static void log_rx_hex(const uint8_t *data, size_t len, bool overflowed) {
  if (g_log_level < 1) return;

  size_t body = len;
  while (body > 0 && (data[body - 1] == '\r' || data[body - 1] == '\n')) body--;
  bool binary = false;
  for (size_t i = 0; i < body && !binary; i++) {
    binary = data[i] < 0x20 || data[i] > 0x7E;
  }
  if (!binary) return;

  char line[448];

  _Static_assert(sizeof("[t=4294967295 ms][TMC RX hex] len=4294967295+") - 1u + 3u * RX_HEX_MAX + sizeof(" ...\r\n") - 1u <= sizeof(line),
                 "line of log_rx_hex() too short");
  size_t shown = len > RX_HEX_MAX ? RX_HEX_MAX : len;
  int n = snprintf(line, sizeof(line), "[t=%u ms][TMC RX hex] len=%u%s", (unsigned) tusb_time_millis_api(), (unsigned) len,
                   overflowed ? "+" : "");
  for (size_t i = 0; i < shown; i++) {
    n += snprintf(line + n, sizeof(line) - (size_t) n, " %02X", data[i]);
  }
  if (len > shown) {
    n += snprintf(line + n, sizeof(line) - (size_t) n, " ...");
  }
  line[n++] = '\r';
  line[n++] = '\n';
  log_write(line, (size_t) n);
}

static size_t trim_len(const uint8_t *buf, size_t len) {
  while (len > 0) {
    uint8_t c = buf[len - 1];
    if (c == '\r' || c == '\n' || c == ' ') {
      len--;
    } else {
      break;
    }
  }
  return len;
}

static void set_response(const char *text) {
  size_t n = strlen(text);
  if (n > CHAM_RESP_VAL_MAX - 1) n = CHAM_RESP_VAL_MAX - 1;
  memcpy(s_resp_buf, text, n);
  if (n == 0 || s_resp_buf[n - 1] != '\n') {
    s_resp_buf[n++] = '\n';
  }
  s_resp_len = n;
}

static bool str_ieq(const char *a, const char *b) {
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'a' && ca <= 'z') ca = (char) (ca - 'a' + 'A');
    if (cb >= 'a' && cb <= 'z') cb = (char) (cb - 'a' + 'A');
    if (ca != cb) return false;
    a++;
    b++;
  }
  return *a == *b;
}

static bool str_istarts(const char *s, const char *prefix) {
  while (*prefix) {
    char cs = *s, cp = *prefix;
    if (cs >= 'a' && cs <= 'z') cs = (char) (cs - 'a' + 'A');
    if (cp >= 'a' && cp <= 'z') cp = (char) (cp - 'a' + 'A');
    if (cs != cp || cs == 0) return false;
    s++;
    prefix++;
  }
  return true;
}

static const char *zero_cmd_name(const char *cmd) {
  if (str_istarts(cmd, "DCVL?")) return "DCVL?";
  if (str_istarts(cmd, "DCLR?")) return "DCLR?";
  for (const char *p = cmd; *p; p++) {
    if (str_istarts(p, ":WF? DAT2")) return "WF? DAT2";
  }
  return NULL;
}

static void zero_stop(const char *why) {
  if (s_zero_cmd == NULL) return;
  if (g_log_level >= 1) {
    bool cut = str_ieq(s_zero_cmd, "DCVL?") && s_zero_bytes < DCVL_TOTAL;
    char ins[40] = "";
    if (s_zpat_n > 0) {
      snprintf(ins, sizeof(ins), ", inserts %u (%u bytes sent)", s_zpat_n, (unsigned) s_zpat_sent);
    }
    log_printf("[t=%u ms][TMC] zero %s end (%s): %u reads, %u bytes, %u ms from command to last byte%s%s\r\n",
               (unsigned) tusb_time_millis_api(), s_zero_cmd, why, s_zero_reads, (unsigned) s_zero_bytes,
               (unsigned) (s_zero_t_last - s_zero_t_cmd), cut ? ", INTERRUPTED (614408 expected)" : "", ins);
    if (s_zero_tag == (int) ZPAT_DCVL) {
      char d[160];
      udec_res_str(d, sizeof(d));
      log_printf("[t=%u ms][TMC] UART decode: %s\r\n", (unsigned) tusb_time_millis_api(), d);
      udec_buses_str(d, sizeof(d));
      log_printf("[t=%u ms][TMC] UART decode: %s\r\n", (unsigned) tusb_time_millis_api(), d);
    }
  }
  s_zero_cmd = NULL;
}

static char tdiv_up(char c) {
  return (c >= 'a' && c <= 'z') ? (char) (c - 'a' + 'A') : c;
}

static bool tdiv_parse(const char *s, uint64_t *fs_uhz) {
  static const char key[] = "TDIV";
  for (unsigned i = 0; i < 4u; i++) {
    if (tdiv_up(s[i]) != key[i]) return false;
  }
  s += 4;
  while (*s == ' ') s++;
  if (*s < '0' || *s > '9') return false;
  uint64_t n = 0;
  while (*s >= '0' && *s <= '9') {
    n = n * 10u + (uint64_t) (*s - '0');
    if (n > 1000000000000000ull) n = 1000000000000000ull;
    s++;
  }
  char u0 = tdiv_up(s[0]);
  char u1 = (u0 != 0) ? tdiv_up(s[1]) : 0;
  uint64_t unit_hz;
  if (u0 == 'N' && u1 == 'S') {
    unit_hz = 1000000000ull;
    s += 2;
  } else if (u0 == 'U' && u1 == 'S') {
    unit_hz = 1000000ull;
    s += 2;
  } else if (u0 == 'M' && u1 == 'S') {
    unit_hz = 1000ull;
    s += 2;
  } else if (u0 == 'S') {
    unit_hz = 1ull;
    s += 1;
  } else {
    return false;
  }
  while (*s == ' ') s++;
  if (*s != 0 || n == 0u) return false;

  *fs_uhz = (100000000ull * unit_hz + n / 2u) / n;
  return true;
}

#define TRDL_MAX_PS 1000000000000000ull

static char trdl_up(char c) {
  return (c >= 'a' && c <= 'z') ? (char) (c - 'a' + 'A') : c;
}

static bool trdl_parse(const char *s, int64_t *ps) {
  static const char key[] = "TRDL";
  for (unsigned i = 0; i < 4u; i++) {
    if (trdl_up(s[i]) != key[i]) return false;
  }
  s += 4;
  while (*s == ' ') s++;
  bool neg = false;
  if (*s == '+' || *s == '-') {
    neg = (*s == '-');
    s++;
  }
  if (*s < '0' || *s > '9') return false;
  uint64_t ip = 0;
  while (*s >= '0' && *s <= '9') {
    ip = ip * 10u + (uint64_t) (*s - '0');
    if (ip > TRDL_MAX_PS) ip = TRDL_MAX_PS;
    s++;
  }
  const char *frac = "";
  if (*s == '.') {
    frac = ++s;
    if (*s < '0' || *s > '9') return false;
    while (*s >= '0' && *s <= '9') s++;
  }
  char u0 = trdl_up(s[0]);
  char u1 = (u0 != 0) ? trdl_up(s[1]) : 0;
  unsigned e = 12;
  if (u0 == 'P' && u1 == 'S') {
    e = 0;
    s += 2;
  } else if (u0 == 'N' && u1 == 'S') {
    e = 3;
    s += 2;
  } else if (u0 == 'U' && u1 == 'S') {
    e = 6;
    s += 2;
  } else if (u0 == 'M' && u1 == 'S') {
    e = 9;
    s += 2;
  } else if (u0 == 'S') {
    e = 12;
    s += 1;
  }
  while (*s == ' ') s++;
  if (*s != 0) return false;
  uint64_t unit = 1;
  for (unsigned i = 0; i < e; i++) unit *= 10u;
  uint64_t v;
  if (ip > TRDL_MAX_PS / unit) {
    v = TRDL_MAX_PS;
  } else {
    v = ip * unit;
    uint64_t scale = unit;
    for (unsigned i = 0; frac[i] >= '0' && frac[i] <= '9'; i++) {
      if (scale >= 10u) {
        scale /= 10u;
        v += (uint64_t) (frac[i] - '0') * scale;
      } else {
        if (frac[i] >= '5') v++;
        break;
      }
    }
    if (v > TRDL_MAX_PS) v = TRDL_MAX_PS;
  }
  *ps = neg ? -(int64_t) v : (int64_t) v;
  return true;
}

static int32_t hism_col(bool hism, bool hcomp, bool trig_on, bool acq_set, int32_t c_acq, uint64_t step_acq,
                        uint64_t step_now, int32_t c_new, bool *rule) {
  *rule = hism && hcomp && trig_on && acq_set && step_now == step_acq;
  return (*rule && c_acq < c_new) ? c_acq : c_new;
}

static int d_input(const char *v) {
  if ((v[0] != 'D' && v[0] != 'd') || v[1] < '0' || v[1] > '9') return -1;
  char *end;
  long n = strtol(v + 1, &end, 10);
  return (*end == 0 && n <= 15) ? (int) n : -1;
}

static void decode_dcut(unsigned bus, udec_cfg_t *u, const char *tail) {
  char s[SCPI_MSG_BUF_SIZE];
  strcpy(s, tail);
  char *save = NULL;
  for (char *k = strtok_r(s, ",", &save); k != NULL; k = strtok_r(NULL, ",", &save)) {
    char *v = strtok_r(NULL, ",", &save);
    if (v == NULL) break;
    while (*k == ' ') k++;
    if (str_ieq(k, "DIS")) {
      u->show = str_ieq(v, "ON");
    } else if (str_ieq(k, "RX") || str_ieq(k, "TX")) {
      char *dst = str_ieq(k, "RX") ? u->rx : u->tx;
      strncpy(dst, v, sizeof(u->rx) - 1u);
      dst[sizeof(u->rx) - 1u] = 0;
      if (dst == u->rx) {
        u->rx_ch = d_input(v);
      } else {
        u->tx_ch = d_input(v);
      }
    } else if (str_ieq(k, "BAUD")) {
      u->baud = (uint32_t) strtoul(v, NULL, 10);
    } else if (str_ieq(k, "PAR")) {
      for (unsigned i = 0; i < 5u; i++) {
        if (str_ieq(v, udec_par_name[i])) u->par = (uint8_t) i;
      }
    } else if (str_ieq(k, "STOP")) {
      u->stop = (int) strtol(v, NULL, 10);
    } else if (str_ieq(k, "POL")) {
      u->pol_high = !str_ieq(v, "LOW");
    } else if (str_ieq(k, "BIT")) {
      u->msb = str_ieq(v, "MSB");
    } else if (str_ieq(k, "DLEN")) {
      u->dlen = (int) strtol(v, NULL, 10);
    }
  }
  u->dcut = true;
  if (g_log_level >= 1) {
    char d[128];
    udec_cfg_str(bus, d, sizeof(d));
    log_printf("[t=%u ms][TMC] B%u:DCUT -> %s\r\n", (unsigned) tusb_time_millis_api(), bus + 1u, d);
  }
}

static unsigned s_dcpa_bus = 0;

static void decode_settings(const char *cmd) {
  if ((cmd[0] == 'B' || cmd[0] == 'b') && (cmd[1] == '1' || cmd[1] == '2') && cmd[2] == ':') {
    const unsigned bus = (unsigned) (cmd[1] - '1');
    udec_cfg_t *u = udec_cfg_of(bus);
    const char *p = cmd + 3;
    if (str_ieq(p, "DCUT")) {
      u->uart = true;
    } else if (str_ieq(p, "DCIC") || str_ieq(p, "DCSP") || str_ieq(p, "DCCN") || str_ieq(p, "DCLN")) {
      u->uart = false;
    } else if (str_istarts(p, "DCUT ")) {
      decode_dcut(bus, u, p + 5);
    }
  } else if (str_istarts(cmd, "DCST ")) {
    const char *a = cmd + 5;
    while (*a == ' ') a++;
    for (unsigned bus = 0; bus < UDEC_BUSES; bus++) {
      if (str_ieq(a, "ON")) udec_cfg_of(bus)->on = true;
      else if (str_ieq(a, "OFF")) udec_cfg_of(bus)->on = false;
    }
  } else if (str_istarts(cmd, "DCPA FOMT,")) {
    for (unsigned i = 0; i < 4u; i++) {
      if (str_ieq(cmd + 10, udec_fmt_name[i])) udec_cfg_of(s_dcpa_bus)->fmt = (uint8_t) i;
    }
  } else if (str_istarts(cmd, "DCPA BUS,")) {
    if (str_ieq(cmd + 9, "B1")) s_dcpa_bus = 0;
    else if (str_ieq(cmd + 9, "B2")) s_dcpa_bus = 1;
  } else if (str_istarts(cmd, "DCPA LIST,")) {
    if (str_ieq(cmd + 10, "D2")) g_udec_list = 1;
    else if (str_ieq(cmd + 10, "D1") || str_ieq(cmd + 10, "OFF")) g_udec_list = 0;
  }
}

static int trsr_source(const char *cmd) {
  if (!str_istarts(cmd, "TRSR ")) return -2;
  const char *a = cmd + 5;
  while (*a == ' ') a++;
  return str_istarts(a, "EDGE,") ? d_input(a + 5) : -1;
}

static int trsl_slope(const char *cmd, int *fall) {
  if ((cmd[0] != 'D' && cmd[0] != 'd') || cmd[1] < '0' || cmd[1] > '9') return -1;
  char *end;
  long k = strtol(cmd + 1, &end, 10);
  if (k > 15 || !str_istarts(end, ":TRSL ")) return -1;
  const char *a = end + 6;
  while (*a == ' ') a++;
  *fall = str_ieq(a, "NEG") ? 1 : (str_ieq(a, "POS") ? 0 : -1);
  return (int) k;
}

static uint8_t dt_mode(uint8_t k) {
  return ((s_dt_fall >> k) & 1u) ? SAMPLER_TRIG_FALL : SAMPLER_TRIG_RISE;
}

static void dt_apply(void) {
  sampler_trig_set(dt_mode(s_dt_ch), s_dt_ch, 0u);
  s_dt_mark = sampler_head_get();
  s_dt_hit = false;
  s_wf_ready = false;
}

static void dt_source(const char *cmd, int k) {
  char u[24];
  if (k >= 0) {
    if (s_dt_on && (uint8_t) k == s_dt_ch) return;
    if (!s_dt_on) {
      s_dt_uart = sampler_trig_get();
    }
    if ((uint8_t) k != s_dt_ch) {
      s_dt_cyc_max = 0;
      s_dt_cyc_hit_max = 0;
    }
    s_dt_on = true;
    s_dt_ch = (uint8_t) k;
    dt_apply();
    if (g_log_level >= 1) {
      scpi_trig_text(u, sizeof(u), s_dt_uart.mode, s_dt_uart.channel, s_dt_uart.idle);
      log_printf("[t=%u ms][TMC] \"%.24s\" -> trigger on D on: D%u %s (CLI trig %s saved)\r\n", (unsigned) tusb_time_millis_api(),
                 cmd, (unsigned) s_dt_ch, dt_mode(s_dt_ch) == SAMPLER_TRIG_FALL ? "fall" : "rise", u);
    }
  } else if (s_dt_on) {
    s_dt_on = false;
    sampler_trig_set(s_dt_uart.mode, s_dt_uart.channel, s_dt_uart.idle);
    if (!(s_wf_ready && s_wf_ltv)) {
      s_wf_ready = false;
    }
    if (g_log_level >= 1) {
      scpi_trig_text(u, sizeof(u), s_dt_uart.mode, s_dt_uart.channel, s_dt_uart.idle);
      log_printf("[t=%u ms][TMC] \"%.24s\" -> trigger on D off (was D%u), CLI trig %s back\r\n", (unsigned) tusb_time_millis_api(),
                 cmd, (unsigned) s_dt_ch, u);
    }
  }
}

static void dt_slope(const char *cmd, int k, int fall) {
  uint16_t bit = (uint16_t) (1u << k);
  if (fall >= 0) {
    bool changed = ((s_dt_fall & bit) != 0u) != (fall != 0);
    s_dt_fall = fall ? (uint16_t) (s_dt_fall | bit) : (uint16_t) (s_dt_fall & ~bit);
    if (changed && s_dt_on && (uint8_t) k == s_dt_ch) {
      dt_apply();
    }
  }
  if (g_log_level >= (s_dt_on ? 1 : 2)) {
    log_printf("[t=%u ms][TMC] \"%.24s\" -> D%d slope %s%s%s\r\n", (unsigned) tusb_time_millis_api(), cmd, k,
               fall < 0 ? "not recognised, stays " : "", dt_mode((uint8_t) k) == SAMPLER_TRIG_FALL ? "fall" : "rise",
               (s_dt_on && (uint8_t) k == s_dt_ch) ? " (own trigger)" : "");
  }
}

static void dt_check(void) {
  if (s_tx_in_flight) return;
  int32_t col = sampler_trdl_col();
  uint32_t c0 = DWT->CYCCNT;
  bool hit = sampler_snapshot_since(s_wf_buf + WF_HDR_LEN, col, s_dt_mark);
  uint32_t cyc = DWT->CYCCNT - c0;
  uint32_t *mx = hit ? &s_dt_cyc_hit_max : &s_dt_cyc_max;
  if (cyc > *mx) {
    *mx = cyc;
  }
  if (!hit) return;
  memcpy(s_wf_buf, "DI:WF ", WF_HDR_LEN);
  s_wf_held = false;
  s_wf_ltv = true;
  sampler_snap_info(&s_wf_fs_mhz, &s_wf_first, &s_wf_col);
  s_wf_ready = true;
  s_wf_shown = false;
  s_dt_hit = true;
  if (g_log_level >= 1) {
    log_printf("[t=%u ms][TMC] DI:LTV? -> edge of D%u (%s) after DI:STTA?, frame laid out at c=%ld, %u cyc\r\n",
               (unsigned) tusb_time_millis_api(), (unsigned) s_dt_ch, dt_mode(s_dt_ch) == SAMPLER_TRIG_FALL ? "fall" : "rise",
               (long) s_wf_col, (unsigned) cyc);
  }
}

static unsigned s_cdata_calls = 0;

static void build_auto_reply(const char *cmd, size_t n, char *out, size_t out_max) {
  size_t m = (n < out_max - 2u) ? n : out_max - 2u;
  for (size_t i = 0; i < m; i++) {
    out[i] = (cmd[i] == '?') ? ' ' : cmd[i];
  }
  out[m] = '0';
  out[m + 1] = 0;
}

static uint32_t get_u32(const uint8_t *p) {
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

static void put_u32(uint8_t *p, uint32_t v) {
  for (unsigned i = 0; i < 4u; i++) p[i] = (uint8_t) (v >> (8u * i));
}

static void pava_stats(bool frame, uint32_t src1, double *per, double *pwid, double *nwid) {
  *per = *pwid = *nwid = PAVA_NONE;
  if (!frame || src1 < PAVA_SRC_D0 || src1 >= PAVA_SRC_D0 + SAMPLER_CH || s_wf_fs_mhz == 0u || s_wf_first >= SAMPLER_COLS) return;
  const uint8_t *blk = s_wf_buf + WF_HDR_LEN + (src1 - PAVA_SRC_D0) * SAMPLER_BLK;
  uint32_t rise_n = 0, rise_first = 0, rise_last = 0, hi_n = 0, hi_sum = 0, lo_n = 0, lo_sum = 0;
  uint32_t edge = 0;
  bool edge_set = false;
  unsigned prev = (blk[s_wf_first >> 3] >> (s_wf_first & 7u)) & 1u;
  for (uint32_t c = s_wf_first + 1u; c < SAMPLER_COLS; c++) {
    unsigned v = (blk[c >> 3] >> (c & 7u)) & 1u;
    if (v == prev) continue;
    if (edge_set) {
      if (v) {
        lo_sum += c - edge;
        lo_n++;
      } else {
        hi_sum += c - edge;
        hi_n++;
      }
    }
    if (v) {
      if (rise_n == 0u) rise_first = c;
      rise_last = c;
      rise_n++;
    }
    edge = c;
    edge_set = true;
    prev = v;
  }
  double step = 1000.0 / (double) s_wf_fs_mhz;
  if (rise_n >= 2u) *per = (double) (rise_last - rise_first) * step / (double) (rise_n - 1u);
  if (hi_n > 0u) *pwid = (double) hi_sum * step / (double) hi_n;
  if (lo_n > 0u) *nwid = (double) lo_sum * step / (double) lo_n;
}

static double pava_value(uint32_t type, double per, double pwid, double nwid) {
  double scale = g_pava.duty_frac ? 1.0 : 100.0;
  switch (type) {
    case 17u: return per;
    case 18u: return (per != PAVA_NONE) ? 1.0 / per : PAVA_NONE;
    case 19u: return pwid;
    case 20u: return nwid;
    case 24u: return (per != PAVA_NONE && pwid != PAVA_NONE) ? pwid / per * scale : PAVA_NONE;
    case 25u: return (per != PAVA_NONE && nwid != PAVA_NONE) ? nwid / per * scale : PAVA_NONE;
    default: return PAVA_NONE;
  }
}

static void pava_calc(bool frame) {
  static const uint32_t hdr_type[6] = {17u, 18u, 19u, 20u, 24u, 25u};
  double per = PAVA_NONE, pwid = PAVA_NONE, nwid = PAVA_NONE;
  for (unsigned i = 0; i < SCPI_PAVA_ITEMS; i++) {
    s_pava_calc[i] = 0.0;
    if (i >= g_pava.n) continue;
    uint32_t src1 = g_pava.item[i][1];
    if (i == 0u || src1 != g_pava.item[i - 1u][1]) pava_stats(frame, src1, &per, &pwid, &nwid);
    s_pava_calc[i] = pava_value(g_pava.item[i][0], per, pwid, nwid);
  }
  if (g_pava.measc >= 0) {
    pava_stats(frame, PAVA_SRC_D0 + (uint32_t) g_pava.measc, &per, &pwid, &nwid);
    for (unsigned w = 0; w < 6u; w++) {
      s_pava_hcalc[w] = pava_value(hdr_type[w], per, pwid, nwid);
    }
    s_pava_hcalc[6] = PAVA_NONE;
    s_pava_hcalc[7] = PAVA_NONE;
  }
}

static void cmpa_parse(const uint8_t *buf, size_t len) {
  if (len < CMPA_LEN || memcmp(buf, "CMPA ", 5) != 0) return;
  g_pava.n = buf[5] > SCPI_PAVA_ITEMS ? (uint8_t) SCPI_PAVA_ITEMS : buf[5];
  memset(g_pava.item, 0, sizeof(g_pava.item));
  for (unsigned i = 0; i < g_pava.n; i++) {
    for (unsigned f = 0; f < 3u; f++) {
      g_pava.item[i][f] = get_u32(buf + 9u + 12u * i + 4u * f);
    }
  }
  pava_calc(g_pava.mode == SCPI_PAVA_CALC && s_wf_shown);
  if (g_log_level >= 1) {
    char line[224];
    _Static_assert(sizeof("[t=4294967295 ms][TMC] CMPA n=5") - 1u + 5u * (sizeof(" (4294967295,4294967295,4294967295)") - 1u) + 2u <=
                   sizeof(line), "line of cmpa_parse() too short");
    int n = snprintf(line, sizeof(line), "[t=%u ms][TMC] CMPA n=%u", (unsigned) tusb_time_millis_api(), (unsigned) g_pava.n);
    for (unsigned i = 0; i < g_pava.n; i++) {
      n += snprintf(line + n, sizeof(line) - (size_t) n, " (%lu,%lu,%lu)", (unsigned long) g_pava.item[i][0],
                    (unsigned long) g_pava.item[i][1], (unsigned long) g_pava.item[i][2]);
    }
    line[n++] = '\r';
    line[n++] = '\n';
    log_write(line, (size_t) n);
  }
}

static void measc_parse(const char *cmd) {
  if (!str_istarts(cmd, "MEASC ")) return;
  const char *a = cmd + 6;
  while (*a == ' ') a++;
  g_pava.measc = (int8_t) d_input(a);
  pava_calc(g_pava.mode == SCPI_PAVA_CALC && s_wf_shown);
}

static bool pava_build(void) {
  if (s_tx_in_flight && s_pava_built) return false;
  memset(s_pava_buf, 0, sizeof(s_pava_buf));
  memcpy(s_pava_buf, "CM:PAVA ", PAVA_HDR_LEN);
  uint8_t *b = s_pava_buf + PAVA_HDR_LEN;
  if (g_pava.hdr_set) {
    for (unsigned i = 0; i < SCPI_PAVA_HDR_WORDS; i++) {
      put_u32(b + 4u * i, g_pava.hdr[i]);
    }
  } else if (g_pava.mode == SCPI_PAVA_CALC && g_pava.measc >= 0) {
    memcpy(b, s_pava_hcalc, sizeof(s_pava_hcalc));
  }
  for (unsigned i = 0; i < g_pava.n; i++) {
    uint8_t *it = b + 64u + 56u * i;
    for (unsigned j = 0; j < 5u; j++) {
      int64_t k = (int64_t) (i + 1u) * 1000 + (int64_t) (j + 1u);
      double d = (g_pava.mode == SCPI_PAVA_CALC) ? s_pava_calc[i] : (double) k;
      if (g_pava.mode == SCPI_PAVA_PROBEI) {
        memcpy(it + 8u * j, &k, 8);
      } else {
        memcpy(it + 8u * j, &d, 8);
      }
    }
    put_u32(it + 40u, g_pava.item[i][1]);
    put_u32(it + 44u, g_pava.item[i][2]);
    put_u32(it + 48u, g_pava.item[i][0]);
  }
  s_pava_built = true;
  s_pava_mode = g_pava.mode;
  return true;
}

static void scpi_process_message(const uint8_t *buf, size_t len, bool overflowed) {
  size_t n = trim_len(buf, len);
  char cmd[SCPI_MSG_BUF_SIZE];
  if (n >= sizeof(cmd)) n = sizeof(cmd) - 1;
  memcpy(cmd, buf, n);
  cmd[n] = 0;

  s_resp_len = 0;
  s_resp_is_wf = false;
  s_resp_is_pava = false;
  zero_stop("new message");

  if (overflowed && g_log_level >= 1) {
    log_printf("[t=%u ms][TMC] message overflow (> %u bytes), truncated\r\n",
               (unsigned) tusb_time_millis_api(), (unsigned) SCPI_MSG_BUF_SIZE);
  }
  if (!overflowed) {
    cmpa_parse(buf, len);
  }
  if (n == 0) {
    return;
  }

  if (str_istarts(cmd, "DI:CTEST")) {
    s_cdata_calls = 0;

    const char *arg = cmd + 8;
    while (*arg == ' ') arg++;
    bool mark_high = (*arg == '1');
    HAL_GPIO_WritePin(CHAM_MARK_PORT, CHAM_MARK_PIN, mark_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (g_log_level >= 1) {
      log_printf("[t=%u ms][TMC] DI:CTEST -> connect-test counter reset, PB0=%d\r\n",
                 (unsigned) tusb_time_millis_api(), (int) mark_high);
    }
  }

  uint64_t tdiv_fs_uhz;
  if (tdiv_parse(cmd, &tdiv_fs_uhz)) {
    sampler_rate_t r = sampler_set_rate(tdiv_fs_uhz);
    if (r.changed) {
      s_wf_ready = false;
    }
    wf_drop_other_col();
    if (g_log_level >= 1) {
      uint64_t req_hz = tdiv_fs_uhz / 1000000u;
      log_printf("[t=%u ms][TMC] \"%.24s\" -> fs req %u.%03u Hz, act %u.%03u Hz (PSC=%u ARR=%u)%s%s\r\n",
                 (unsigned) tusb_time_millis_api(), cmd,
                 (unsigned) (req_hz > 0xFFFFFFFFu ? 0xFFFFFFFFu : req_hz), (unsigned) ((tdiv_fs_uhz / 1000u) % 1000u),
                 (unsigned) (r.fs_mhz / 1000u), (unsigned) (r.fs_mhz % 1000u), (unsigned) r.psc, (unsigned) r.arr,
                 r.limit > 0 ? " [above max, clamped]" : (r.limit < 0 ? " [below min, clamped]" : ""),
                 r.changed ? ", ring cleared" : ", unchanged");
    }
  }

  int64_t trdl_ps;
  if (trdl_parse(cmd, &trdl_ps)) {
    sampler_trdl_set(trdl_ps);
    wf_drop_other_col();
    if (g_log_level >= 1) {
      log_printf("[t=%u ms][TMC] \"%.24s\" -> TRDL %ld us (whole), c=%ld\r\n", (unsigned) tusb_time_millis_api(), cmd,
                 (long) (trdl_ps / 1000000), (long) sampler_trdl_col());
    }
  }

  if (str_istarts(cmd, "DI:HISM?")) {
    const char *a = cmd + 8;
    while (*a == ' ') a++;
    if (str_ieq(a, "ON")) {
      s_hism = true;
    } else if (str_ieq(a, "OFF")) {
      s_hism = false;
    }
  }

  int trsr_k = trsr_source(cmd);
  if (trsr_k != -2) {
    dt_source(cmd, trsr_k);
  }
  int trsl_fall;
  int trsl_k = trsl_slope(cmd, &trsl_fall);
  if (trsl_k >= 0) {
    dt_slope(cmd, trsl_k, trsl_fall);
  }
  if (s_dt_on && str_ieq(cmd, "DI:STTA?")) {
    s_dt_mark = sampler_head_get();
    s_dt_hit = false;
    s_wf_ready = false;
  } else if (str_ieq(cmd, "DI:STTA?") && s_wf_ready && s_wf_ltv) {
    s_wf_ready = false;
  }

  decode_settings(cmd);
  measc_parse(cmd);

  if (str_ieq(cmd, "DI:WSR?")) {
    if (!(s_dt_on && s_wf_ready)) {
      wf_take_snapshot("DI:WSR?", sampler_trdl_col());
    }
  }

  const char *val = cham_resp_find(cmd);
  const char *zero = zero_cmd_name(cmd);
  if (val != NULL) {
    set_response(val);
  } else if (zero != NULL) {

    s_zero_cmd = zero;
    s_zero_reads = 0;
    s_zero_bytes = 0;
    s_zero_t_cmd = tusb_time_millis_api();
    s_zero_t_last = s_zero_t_cmd;

    int tag = -1;
    if (str_ieq(zero, "DCVL?")) {
      tag = ZPAT_DCVL;
    } else if (str_ieq(zero, "DCLR?")) {
      const char *arg = cmd + 5;
      while (*arg == ' ') arg++;
      if (str_istarts(arg, "HEAD")) tag = ZPAT_HEAD;
      else if (str_istarts(arg, "DATA")) tag = ZPAT_DATA;
    }
    s_zero_tag = tag;
    s_zpat_n = 0;
    for (unsigned i = 0; i < g_zpat_n; i++) {
      if (g_zpat[i].cmd == tag) s_zpat[s_zpat_n++] = g_zpat[i];
    }
    s_zpat_sent = 0;
    if (tag == (int) ZPAT_DCVL) {
      uint32_t c0 = DWT->CYCCNT;
      udec_run(s_wf_shown ? s_wf_buf + WF_HDR_LEN : NULL, s_wf_fs_mhz, s_wf_first, s_wf_col);
      g_udec.cycles = DWT->CYCCNT - c0;
    }
    if (g_log_level >= 1) {
      log_printf("[t=%u ms][TMC] %s -> zero stream (TransferSize zero bytes per read)\r\n",
                 (unsigned) s_zero_t_cmd, zero);
    }
  } else if (str_ieq(cmd, "DI:WF?")) {

    int32_t c_new = sampler_trdl_col();
    bool rule;
    int32_t c_lay = hism_col(s_hism, s_hcomp, sampler_trig_get().mode != SAMPLER_TRIG_OFF, s_acq_set, s_c_acq, s_step_acq,
                             sampler_req_fs_get(), c_new, &rule);
    if (rule && s_wf_ready && s_wf_col != c_lay) {
      s_wf_ready = false;
    }
    bool from_wsr = s_wf_ready;
    bool taken = false;
    if (!s_wf_ready) {
      taken = wf_take_snapshot("DI:WF?", c_lay);
    }
    s_wf_ready = false;
    s_wf_shown = true;
    if (!s_hism) {

      s_acq_set = sampler_trig_get().mode != SAMPLER_TRIG_OFF;
      s_c_acq = s_wf_col;
      s_step_acq = sampler_req_fs_get();
      if (from_wsr && s_wf_ltv) {
        s_acq_set = true;
      }
    }
    s_wf_n++;
    if (rule && c_lay != c_new && s_wf_col == c_lay) {
      s_hcomp_n++;
    }
    s_resp_is_wf = true;
    s_resp_len = WF_RESP_LEN;
    if (g_pava.mode == SCPI_PAVA_CALC) {
      pava_calc(true);
    }
    if (g_log_level >= 1) {
      log_printf("[t=%u ms][TMC] built-in DI:WF? -> %u-byte frame\r\n",
                 (unsigned) tusb_time_millis_api(), (unsigned) WF_RESP_LEN);

      uint32_t c0 = DWT->CYCCNT;
      char d8[32] = "-";
      char note[72] = "";
      uint16_t v, st;
      if (udec_peek(s_wf_buf + WF_HDR_LEN, s_wf_fs_mhz, s_wf_first, s_wf_col, 8u, &v, &st)) {
        snprintf(d8, sizeof(d8), "0x%02X at c%+d", (unsigned) v, (int) st - (int) s_wf_col);
      }
      if (rule && c_lay != c_new) {
        snprintf(note, sizeof(note), " (hcomp: c_new=%ld c_acq=%ld)", (long) c_new, (long) s_c_acq);
      }
      const char *src = from_wsr ? (s_wf_ltv ? "LTV?" : (s_wf_held ? "WSR?/held" : "WSR?")) : (taken ? (s_wf_held ? "held" : "snap") : "buf");
      uint32_t cyc = DWT->CYCCNT - c0;
      log_printf("[t=%u ms][TMC] WF #%u: HISM=%s c=%ld src=%s D8=%s%s label %u cyc\r\n", (unsigned) tusb_time_millis_api(),
                 (unsigned) s_wf_n, s_hism ? "ON" : "OFF", (long) s_wf_col, src, d8, note, (unsigned) cyc);
    }
  } else if (g_pava.mode != SCPI_PAVA_OFF && str_ieq(cmd, "CM:PAVA?")) {

    bool fresh = pava_build();
    s_resp_is_pava = true;
    s_resp_len = PAVA_RESP_LEN;
    if (g_log_level >= 1) {
      static const char *const mode_name[] = {"off", "calc", "probe", "probei"};
      char v[24] = "";
      if (g_pava.n > 0u) {
        const uint8_t *it = s_pava_buf + PAVA_HDR_LEN + 64u;
        char t[16];
        if (s_pava_mode == SCPI_PAVA_PROBEI) {
          int64_t k;
          memcpy(&k, it, 8);
          snprintf(t, sizeof(t), "%ld", (long) k);
        } else {
          double d;
          memcpy(&d, it, 8);
          scpi_dbl_text(t, sizeof(t), d);
        }
        snprintf(v, sizeof(v), " %s", t);
      }
      log_printf("[t=%u ms][TMC] built-in CM:PAVA? -> %s n=%u%s item0 type=%lu src1=%lu%s\r\n", (unsigned) tusb_time_millis_api(),
                 mode_name[s_pava_mode], (unsigned) g_pava.n, fresh ? "" : " (reply being sent: the last one again)",
                 (unsigned long) g_pava.item[0][0], (unsigned long) g_pava.item[0][1], v);
    }
  } else if (str_ieq(cmd, "DI:CDATA?")) {
    s_cdata_calls++;
    int v = (s_cdata_calls % 2u == 1u) ? 31 : 0;
    char reply[24];
    snprintf(reply, sizeof(reply), "DI:CDATA %d", v);
    set_response(reply);
    if (g_log_level >= 1) {
      log_printf("[t=%u ms][TMC] built-in DI:CDATA? -> %d (call #%u since reset)\r\n",
                 (unsigned) tusb_time_millis_api(), v, s_cdata_calls);
    }
  } else if (s_dt_on && str_ieq(cmd, "DI:LTV?")) {

    if (!s_dt_hit) {
      dt_check();
    }
    set_response(s_dt_hit ? "DI:LTV? 1" : "DI:LTV? 0");
  } else if (strchr(cmd, '?') != NULL) {
    if (g_desc.unknown_auto) {
      char reply[CHAM_RESP_VAL_MAX];
      build_auto_reply(cmd, n, reply, sizeof(reply));
      set_response(reply);
    } else {
      set_response(g_desc.unknown_resp);
    }
  }

}

static const usbtmc_response_capabilities_488_t s_caps = {
    .USBTMC_status = USBTMC_STATUS_SUCCESS,
    .bcdUSBTMC = USBTMC_VERSION,
    .bmIntfcCapabilities = {.listenOnly = 0, .talkOnly = 0, .supportsIndicatorPulse = 0},
    .bmDevCapabilities = {.canEndBulkInOnTermChar = 1},
    .bcdUSB488 = USBTMC_488_VERSION,
    .bmIntfcCapabilities488 = {.supportsTrigger = 0, .supportsREN_GTL_LLO = 0, .is488_2 = 0},
    .bmDevCapabilities488 = {.DT1 = 0, .RL1 = 0, .SR1 = 0, .SCPI = 1},
};

usbtmc_response_capabilities_488_t const *tud_usbtmc_get_capabilities_cb(void) {
  return &s_caps;
}

void tud_usbtmc_open_cb(uint8_t interface_id) {
  (void) interface_id;
  s_msg_active = false;
  s_msg_len = 0;
  s_msg_overflow = false;
  s_resp_len = 0;
  s_tx_in_flight = false;
  s_cdata_calls = 0;
  s_hism = false;
  s_acq_set = false;
  if (s_wf_ready && s_wf_ltv) {
    s_wf_ready = false;
    s_dt_hit = false;
  }
  pava_forget();
  log_printf("[t=%u ms][TMC] interface open\r\n", (unsigned) tusb_time_millis_api());
  zero_stop("interface open");
  tud_usbtmc_start_bus_read();
}

bool tud_usbtmc_msgBulkOut_start_cb(usbtmc_msg_request_dev_dep_out const *msgHeader) {
  if (!s_msg_active) {
    s_msg_len = 0;
    s_msg_overflow = false;
    s_msg_active = true;
  }
  s_msg_eom = (bool) msgHeader->bmTransferAttributes.EOM;
  return true;
}

bool tud_usbtmc_msg_data_cb(void *data, size_t len, bool transfer_complete) {
  size_t space = (s_msg_len < SCPI_MSG_BUF_SIZE) ? (SCPI_MSG_BUF_SIZE - s_msg_len) : 0;
  size_t copy = len > space ? space : len;
  if (copy > 0) {
    memcpy(&s_msg_buf[s_msg_len], data, copy);
    s_msg_len += copy;
  }
  if (len > copy) {
    s_msg_overflow = true;
  }

  if (transfer_complete) {
    s_msg_active = false;
    if (g_log_level >= 2) {
      log_printf("[t=%u ms][TMC] bulk-OUT transfer done, EOM=%d, accumulated=%u bytes\r\n",
                 (unsigned) tusb_time_millis_api(), (int) s_msg_eom, (unsigned) s_msg_len);
    }
    if (s_msg_eom) {
      log_ascii_hex("[TMC RX]", s_msg_buf, s_msg_len, log_full_text(s_msg_buf, s_msg_len) ? LOG_FULL_MAX : 48u);
      log_rx_hex(s_msg_buf, s_msg_len, s_msg_overflow);
      scpi_process_message(s_msg_buf, s_msg_len, s_msg_overflow);
      s_msg_len = 0;
    }
  }

  tud_usbtmc_start_bus_read();
  return true;
}

void tud_usbtmc_bulkOut_clearFeature_cb(void) {
  zero_stop("bulk-OUT clear feature");
  s_msg_active = false;
  s_msg_len = 0;
  s_msg_overflow = false;
  tud_usbtmc_start_bus_read();
}

bool tud_usbtmc_msgBulkIn_request_cb(usbtmc_msg_request_dev_dep_in const *request) {
  if (s_zero_cmd != NULL) {

    s_zero_req = request->TransferSize;
    s_zero_t_req = tusb_time_millis_api();
    s_tx_in_flight = true;
    bool ok = tud_usbtmc_transmit_dev_msg_data(NULL, s_zero_req, true, false);
    if (!ok && g_log_level >= 1) {
      log_printf("[t=%u ms][TMC] zero %s read #%u: TransferSize=%u refused by the class\r\n",
                 (unsigned) s_zero_t_req, s_zero_cmd, s_zero_reads + 1u, (unsigned) s_zero_req);
    }
    return ok;
  }

  if (g_log_level >= 2) {
    log_printf("[t=%u ms][TMC] bulk-IN request: wanted=%u termchar_en=%d termchar=0x%02X\r\n",
               (unsigned) tusb_time_millis_api(), (unsigned) request->TransferSize,
               (int) request->bmTransferAttributes.TermCharEnabled, (unsigned) request->TermChar);
  }

  const uint8_t *data;
  size_t len;
  if (s_resp_len > 0) {
    data = s_resp_is_wf ? s_wf_buf : (s_resp_is_pava ? s_pava_buf : (const uint8_t *) s_resp_buf);
    len = s_resp_len;
  } else {
    data = (const uint8_t *) "\n";
    len = 1;
  }
  size_t orig_len = len;
  if (len > request->TransferSize) {
    len = (request->TransferSize > 0) ? request->TransferSize : 1u;
  }
  if (len != orig_len && g_log_level >= 1) {
    log_printf("[t=%u ms][TMC TX] truncated %u -> %u\r\n",
               (unsigned) tusb_time_millis_api(), (unsigned) orig_len, (unsigned) len);
  }

  bool use_termchar = request->bmTransferAttributes.TermCharEnabled &&
                       len > 0 && data[len - 1] == request->TermChar;

  s_tx_in_flight = true;
  bool ok = tud_usbtmc_transmit_dev_msg_data(data, len, true, use_termchar);
  log_ascii_hex("[TMC TX]", data, len, 48u);

  s_resp_len = 0;
  return ok;
}

void tud_usbtmc_null_data_cb(uint8_t *buf, size_t offset, size_t len) {
  memset(buf, 0, len);
  uint32_t at = s_zero_bytes + (uint32_t) offset;
  if (s_zero_tag >= 0) {
    udec_fill((unsigned) s_zero_tag, buf, at, (uint32_t) len);
  }
  uint32_t end = at + (uint32_t) len;
  for (unsigned i = 0; i < s_zpat_n; i++) {
    const zpat_t *z = &s_zpat[i];
    uint32_t lo = (z->off > at) ? z->off : at;
    uint32_t hi = (z->off + z->len < end) ? z->off + z->len : end;
    for (uint32_t o = lo; o < hi; o++) {
      buf[o - at] = z->b[o - z->off];
      s_zpat_sent++;
    }
  }
}

bool tud_usbtmc_msgBulkIn_complete_cb(void) {
  s_tx_in_flight = false;
  if (s_zero_cmd != NULL) {
    s_zero_t_last = tusb_time_millis_api();
    s_zero_reads++;
    s_zero_bytes += s_zero_req;
    if (g_log_level >= 1) {
      log_printf("[t=%u ms][TMC] zero %s read #%u: %u bytes, %u ms\r\n", (unsigned) s_zero_t_last,
                 s_zero_cmd, s_zero_reads, (unsigned) s_zero_req, (unsigned) (s_zero_t_last - s_zero_t_req));
    }
    if (str_ieq(s_zero_cmd, "DCVL?") && s_zero_bytes >= DCVL_TOTAL) {
      zero_stop("614408 bytes sent");
    }
  } else if (g_log_level >= 2) {
    log_printf("[t=%u ms][TMC] bulk-IN complete\r\n", (unsigned) tusb_time_millis_api());
  }
  tud_usbtmc_start_bus_read();
  return true;
}

void tud_usbtmc_bulkIn_clearFeature_cb(void) {
  zero_stop("bulk-IN clear feature");
  s_resp_len = 0;
  s_tx_in_flight = false;
}

bool tud_usbtmc_initiate_abort_bulk_in_cb(uint8_t *tmcResult) {
  if (g_log_level >= 1) {
    log_printf("[t=%u ms][TMC] abort bulk-IN\r\n", (unsigned) tusb_time_millis_api());
  }
  zero_stop("abort bulk-IN");
  s_resp_len = 0;
  s_tx_in_flight = false;
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

bool tud_usbtmc_initiate_abort_bulk_out_cb(uint8_t *tmcResult) {
  if (g_log_level >= 1) {
    log_printf("[t=%u ms][TMC] abort bulk-OUT\r\n", (unsigned) tusb_time_millis_api());
  }
  zero_stop("abort bulk-OUT");
  s_msg_active = false;
  s_msg_len = 0;
  s_msg_overflow = false;
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

bool tud_usbtmc_initiate_clear_cb(uint8_t *tmcResult) {
  if (g_log_level >= 1) {
    log_printf("[t=%u ms][TMC] clear\r\n", (unsigned) tusb_time_millis_api());
  }
  zero_stop("clear");
  s_msg_active = false;
  s_msg_len = 0;
  s_msg_overflow = false;
  s_resp_len = 0;
  s_tx_in_flight = false;
  s_cdata_calls = 0;
  s_hism = false;
  s_acq_set = false;
  if (s_wf_ready && s_wf_ltv) {
    s_wf_ready = false;
    s_dt_hit = false;
  }
  pava_forget();
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

bool tud_usbtmc_check_abort_bulk_in_cb(usbtmc_check_abort_bulk_rsp_t *rsp) {
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmAbortBulkIn.BulkInFifoBytes = 0;
  rsp->NBYTES_RXD_TXD = 0;
  return true;
}

bool tud_usbtmc_check_abort_bulk_out_cb(usbtmc_check_abort_bulk_rsp_t *rsp) {
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmAbortBulkIn.BulkInFifoBytes = 0;
  rsp->NBYTES_RXD_TXD = 0;
  return true;
}

bool tud_usbtmc_check_clear_cb(usbtmc_get_clear_status_rsp_t *rsp) {
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmClear.BulkInFifoBytes = 0;
  return true;
}

uint8_t tud_usbtmc_get_stb_cb(uint8_t *tmcResult) {
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return 0;
}

bool tud_usbtmc_indicator_pulse_cb(tusb_control_request_t const *msg, uint8_t *tmcResult) {
  (void) msg;
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}
