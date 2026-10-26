#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "stm32f4xx_hal.h"
#include "tusb.h"
#include "uart1.h"
#include "chameleon_config.h"
#include "scpi_emul.h"
#include "log_ring.h"
#include "sampler.h"
#include "uart_dec.h"
#include "uart_gen.h"
#include "cli.h"

#define LINE_MAX 96u
#define REENUM_DEFAULT_MS 3000u

static char line_buf[LINE_MAX];
static size_t line_len = 0;

static uint32_t s_reset_csr;

void cli_reset_cause_init(void) {
  s_reset_csr = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF;
}

static void print_reset(void) {
  static const struct { uint32_t bit; const char *name; } flags[] = {
      {RCC_CSR_LPWRRSTF, "LPWR"}, {RCC_CSR_WWDGRSTF, "WWDG"}, {RCC_CSR_IWDGRSTF, "IWDG"}, {RCC_CSR_SFTRSTF, "SFT"},
      {RCC_CSR_PORRSTF, "POR"},   {RCC_CSR_PINRSTF, "PIN"},   {RCC_CSR_BORRSTF, "BOR"},
  };
  char buf[112];
  int n = snprintf(buf, sizeof(buf), "reset: csr=0x%08lX (", (unsigned long) s_reset_csr);
  bool any = false;
  for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); i++) {
    if ((s_reset_csr & flags[i].bit) != 0u) {
      n += snprintf(buf + n, sizeof(buf) - (size_t) n, "%s%s", any ? " " : "", flags[i].name);
      any = true;
    }
  }
  snprintf(buf + n, sizeof(buf) - (size_t) n, "%s) up=%lu s\r\n", any ? "" : "none", (unsigned long) (HAL_GetTick() / 1000u));
  uart1_puts(buf);
}

static void print_show(void) {
  uart1_puts("blackpill-sbus-la v1.0\r\n");
  print_reset();
  char buf[160];
  snprintf(buf, sizeof(buf),
           "VID=0x%04X PID=0x%04X bcdDevice=0x%04X bcdUSB=0x%04X "
           "class=0x%02X/0x%02X/0x%02X ep0=%u (fixed)\r\n",
           g_desc.vid, g_desc.pid, g_desc.bcd_device, g_desc.bcd_usb,
           g_desc.dev_class, g_desc.dev_subclass, g_desc.dev_protocol, g_desc.max_packet0);
  uart1_puts(buf);
  snprintf(buf, sizeof(buf), "mfr=\"%s\" prod=\"%s\" serial=\"%s\"\r\n",
           g_desc.mfr, g_desc.prod, g_desc.serial);
  uart1_puts(buf);
}

static void print_resp_table(void) {
  char buf[100];
  for (size_t i = 0; i < CHAM_RESP_MAX; i++) {
    if (g_desc.resp[i].key[0] != 0) {
      snprintf(buf, sizeof(buf), "resp \"%s\" -> \"%s\"\r\n", g_desc.resp[i].key, g_desc.resp[i].val);
      uart1_puts(buf);
    }
  }
  if (g_desc.unknown_auto) {
    uart1_puts("unknown -> auto (echo-prefix + \"0\")\r\n");
  } else {
    snprintf(buf, sizeof(buf), "unknown -> \"%s\"\r\n", g_desc.unknown_resp);
    uart1_puts(buf);
  }
}

static void do_reenum(uint32_t delay_ms) {
  tud_disconnect();
  HAL_Delay(delay_ms);
  tud_connect();
  uart1_puts("OK reenum\r\n");
}

static bool parse_u16(const char *s, uint16_t *out) {
  char *end;
  errno = 0;
  unsigned long v = strtoul(s, &end, 0);
  if (end == s || *end != '\0' || errno != 0 || v > 0xFFFFu) return false;
  *out = (uint16_t) v;
  return true;
}

static bool parse_u8(const char *s, uint8_t *out) {
  char *end;
  errno = 0;
  unsigned long v = strtoul(s, &end, 0);
  if (end == s || *end != '\0' || errno != 0 || v > 0xFFu) return false;
  *out = (uint8_t) v;
  return true;
}

static bool parse_u32(const char *s, uint32_t *out) {
  char *end;
  errno = 0;
  unsigned long v = strtoul(s, &end, 0);
  if (end == s || *end != '\0' || errno != 0 || v > 0xFFFFFFFFul) return false;
  *out = (uint32_t) v;
  return true;
}

static void handle_set(char **save) {
  char *field = strtok_r(NULL, " ", save);
  if (!field) {
    uart1_puts("ERR set: missing field\r\n");
    return;
  }

  if (strcmp(field, "vid") == 0 || strcmp(field, "pid") == 0 ||
      strcmp(field, "bcd") == 0 || strcmp(field, "usb") == 0) {
    char *v = strtok_r(NULL, " ", save);
    if (!v) {
      uart1_puts("ERR set: missing value\r\n");
      return;
    }
    uint16_t val;
    if (!parse_u16(v, &val)) {
      uart1_puts("ERR bad value\r\n");
      return;
    }
    if (strcmp(field, "vid") == 0) g_desc.vid = val;
    else if (strcmp(field, "pid") == 0) g_desc.pid = val;
    else if (strcmp(field, "bcd") == 0) g_desc.bcd_device = val;
    else g_desc.bcd_usb = val;
    uart1_puts("OK\r\n");
  } else if (strcmp(field, "class") == 0) {
    char *c = strtok_r(NULL, " ", save);
    char *sc = strtok_r(NULL, " ", save);
    char *pr = strtok_r(NULL, " ", save);
    if (!c || !sc || !pr) {
      uart1_puts("ERR set class: need 3 values (class subclass protocol)\r\n");
      return;
    }
    uint8_t class_v, subclass_v, protocol_v;
    if (!parse_u8(c, &class_v) || !parse_u8(sc, &subclass_v) || !parse_u8(pr, &protocol_v)) {
      uart1_puts("ERR bad value\r\n");
      return;
    }
    g_desc.dev_class = class_v;
    g_desc.dev_subclass = subclass_v;
    g_desc.dev_protocol = protocol_v;
    uart1_puts("OK\r\n");
  } else if (strcmp(field, "mfr") == 0 || strcmp(field, "prod") == 0 || strcmp(field, "serial") == 0) {
    char *rest = strtok_r(NULL, "", save);
    if (!rest) rest = "";
    if (strlen(rest) > CHAM_STR_MAX - 1) {
      uart1_puts("ERR too long\r\n");
      return;
    }
    char *dst = (strcmp(field, "mfr") == 0) ? g_desc.mfr :
                (strcmp(field, "prod") == 0) ? g_desc.prod : g_desc.serial;
    strncpy(dst, rest, CHAM_STR_MAX - 1);
    dst[CHAM_STR_MAX - 1] = 0;
    uart1_puts("OK\r\n");
  } else {
    uart1_puts("ERR set: unknown field\r\n");
  }
}

static bool ieq_upper(const char *s, const char *upper) {
  for (; *s && *upper; s++, upper++) {
    char c = (*s >= 'a' && *s <= 'z') ? (char) (*s - 'a' + 'A') : *s;
    if (c != *upper) return false;
  }
  return *s == *upper;
}

static bool resp_key_forbidden(const char *key) {
  static const char *const keys[] = {"DCVL?", "DCLR?", "DI:WF?", "CM:PAVA?"};
  size_t n = strlen(key);
  for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
    if (ieq_upper(key, keys[i])) return true;
  }
  return n >= 4u && ieq_upper(key + n - 4u, ":WF?");
}

static void resp_drop_forbidden(const char *from) {
  for (size_t i = 0; i < CHAM_RESP_MAX; i++) {
    if (g_desc.resp[i].key[0] != 0 && resp_key_forbidden(g_desc.resp[i].key)) {
      log_printf("[cfg] resp \"%s\" from %s ignored: the board answers it itself (binary reply)\r\n", g_desc.resp[i].key, from);
      g_desc.resp[i].key[0] = 0;
      g_desc.resp[i].val[0] = 0;
    }
  }
}

static void handle_resp(char **save) {
  char *first = strtok_r(NULL, " ", save);
  if (!first) {
    print_resp_table();
    return;
  }
  if (strcmp(first, "del") == 0) {
    char *key = strtok_r(NULL, " ", save);
    if (!key) {
      uart1_puts("ERR resp del: missing key\r\n");
      return;
    }
    uart1_puts(cham_resp_del(key) ? "OK\r\n" : "ERR not found\r\n");
    return;
  }
  char *text = strtok_r(NULL, "", save);
  if (!text || text[0] == 0) {
    uart1_puts("ERR resp: missing text\r\n");
    return;
  }
  if (resp_key_forbidden(first)) {
    uart1_puts("ERR resp: DCVL?, DCLR?, DI:WF?, CM:PAVA? and <ch>:WF? get a binary reply of the board, no table entry\r\n");
    return;
  }
  if (scpi_emul_tx_busy()) {
    uart1_puts("ERR resp: bulk-IN transfer in progress, try again\r\n");
    return;
  }
  uart1_puts(cham_resp_set(first, text) ? "OK\r\n" : "ERR resp: key/text too long or table full\r\n");
}

static void handle_unknown(char **save) {
  char *text = strtok_r(NULL, "", save);
  if (!text) {
    uart1_puts("ERR unknown: missing text\r\n");
    return;
  }
  if (scpi_emul_tx_busy()) {
    uart1_puts("ERR unknown: bulk-IN transfer in progress, try again\r\n");
    return;
  }
  if (strcmp(text, "auto") == 0) {
    g_desc.unknown_auto = 1;
    uart1_puts("OK\r\n");
    return;
  }
  if (strcmp(text, "-") == 0) text = "";
  if (strlen(text) > CHAM_RESP_VAL_MAX - 1) {
    uart1_puts("ERR too long\r\n");
    return;
  }
  g_desc.unknown_auto = 0;
  strncpy(g_desc.unknown_resp, text, CHAM_RESP_VAL_MAX - 1);
  g_desc.unknown_resp[CHAM_RESP_VAL_MAX - 1] = 0;
  uart1_puts("OK\r\n");
}

static void handle_log(char **save) {
  char *v = strtok_r(NULL, " ", save);
  if (!v) {
    char buf[16];
    snprintf(buf, sizeof(buf), "log=%u\r\n", (unsigned) g_log_level);
    uart1_puts(buf);
    return;
  }
  uint8_t level;
  if (!parse_u8(v, &level) || level > 2u) {
    uart1_puts("ERR log: expected 0, 1 or 2\r\n");
    return;
  }
  g_log_level = level;
  uart1_puts("OK\r\n");
}

static void handle_trig(char **save) {
  char *v = strtok_r(NULL, " ", save);
  if (!v) {
    sampler_trig_t t = sampler_trig_get();
    char buf[80];
    if (t.mode == SAMPLER_TRIG_OFF) {
      snprintf(buf, sizeof(buf), "trig=off found=%lu missed=%lu held=%lu\r\n", (unsigned long) t.found,
               (unsigned long) t.missed, (unsigned long) t.held);
    } else {
      char idle[8] = "";
      if (t.idle != 0u) snprintf(idle, sizeof(idle), " %u", (unsigned) t.idle);
      snprintf(buf, sizeof(buf), "trig=%u %s%s found=%lu missed=%lu held=%lu\r\n", (unsigned) t.channel,
               (t.mode == SAMPLER_TRIG_RISE) ? "rise" : "fall", idle, (unsigned long) t.found, (unsigned long) t.missed,
               (unsigned long) t.held);
    }
    uart1_puts(buf);
    snprintf(buf, sizeof(buf), "copy_max=%lu cyc adv_max=%lu isr_max=%lu cyc\r\n", (unsigned long) t.copy_max,
             (unsigned long) t.adv_max, (unsigned long) t.isr_max);
    uart1_puts(buf);

    scpi_dtrig_t d = scpi_dtrig_get();
    if (d.on) {
      char u[24];
      char line[160];
      scpi_trig_text(u, sizeof(u), d.uart_mode, d.uart_ch, d.uart_idle);
      snprintf(line, sizeof(line), "src=TRSR EDGE,D%u %s (uart trig %s comes back when TRSR leaves D) ltv_max=%lu cyc ltv_hit_max=%lu cyc\r\n",
               (unsigned) d.ch, d.mode == SAMPLER_TRIG_FALL ? "fall" : "rise", u, (unsigned long) d.ltv_max,
               (unsigned long) d.ltv_hit_max);
      uart1_puts(line);
    } else {
      uart1_puts("src=uart\r\n");
    }
    return;
  }
  if (scpi_dtrig_get().on) {
    uart1_puts("ERR trig: the scope triggers on D (TRSR EDGE,Dk); trig is restored and can be set when TRSR leaves D\r\n");
    return;
  }
  if (strcmp(v, "off") == 0) {
    sampler_trig_set(SAMPLER_TRIG_OFF, 0u, 0u);
    uart1_puts("OK\r\n");
    return;
  }
  uint8_t ch;
  uint16_t idle = 0;
  char *slope = strtok_r(NULL, " ", save);
  char *n = strtok_r(NULL, " ", save);
  if (!parse_u8(v, &ch) || ch > 15u || !slope || (strcmp(slope, "rise") != 0 && strcmp(slope, "fall") != 0) ||
      (n != NULL && (strcmp(slope, "fall") != 0 || !parse_u16(n, &idle) || idle == 0u))) {
    uart1_puts("ERR trig: expected off, or <0..15> rise|fall, or <0..15> fall <1..65535>\r\n");
    return;
  }
  sampler_trig_set((strcmp(slope, "rise") == 0) ? SAMPLER_TRIG_RISE : SAMPLER_TRIG_FALL, ch, idle);
  uart1_puts("OK\r\n");
}

static void handle_trdl(void) {
  int64_t ps = sampler_trdl_get();
  uint64_t a = (uint64_t) (ps < 0 ? -ps : ps);
  scpi_hism_t h = scpi_hism_get();
  char acq[16] = "none";
  if (h.acq_set) snprintf(acq, sizeof(acq), "%ld", (long) h.c_acq);
  char buf[128];
  snprintf(buf, sizeof(buf), "trdl=%s%lu.%06lu us c=%ld c_acq=%s HISM=%s corrected=%lu\r\n", ps < 0 ? "-" : "",
           (unsigned long) (a / 1000000u), (unsigned long) (a % 1000000u), (long) sampler_trdl_col(), acq,
           h.hism ? "ON" : "OFF", (unsigned long) h.corr);
  uart1_puts(buf);
}

static void handle_hcomp(char **save) {
  char *v = strtok_r(NULL, " ", save);
  if (!v) {
    uart1_puts(scpi_hism_get().hcomp ? "hcomp=on\r\n" : "hcomp=off\r\n");
    return;
  }
  if (strcmp(v, "on") != 0 && strcmp(v, "off") != 0) {
    uart1_puts("ERR hcomp: expected on or off\r\n");
    return;
  }
  scpi_hcomp_set(strcmp(v, "on") == 0);
  uart1_puts("OK\r\n");
}

static void handle_zpat(char **save) {
  static const char *const tag_arg[] = {"", "h ", "d "};
  static const char *const tag_name[] = {"DCVL?", "DCLR? HEAD,1", "DCLR? DATA,2"};
  char *v = strtok_r(NULL, " ", save);
  char buf[80];
  if (!v) {
    snprintf(buf, sizeof(buf), "zpat: %u of %u inserts\r\n", g_zpat_n, (unsigned) ZPAT_MAX);
    uart1_puts(buf);
    for (unsigned i = 0; i < g_zpat_n; i++) {
      int n = snprintf(buf, sizeof(buf), "zpat %s%lu ", tag_arg[g_zpat[i].cmd], (unsigned long) g_zpat[i].off);
      for (unsigned k = 0; k < g_zpat[i].len; k++) {
        n += snprintf(buf + n, sizeof(buf) - (size_t) n, "%02X", g_zpat[i].b[k]);
      }
      snprintf(buf + n, sizeof(buf) - (size_t) n, "\r\n");
      uart1_puts(buf);
    }
    return;
  }
  if (strcmp(v, "clear") == 0) {
    g_zpat_n = 0;
    uart1_puts("OK\r\n");
    return;
  }
  uint8_t cmd = ZPAT_DCVL;
  if (strcmp(v, "h") == 0 || strcmp(v, "d") == 0) {
    cmd = (v[0] == 'h') ? ZPAT_HEAD : ZPAT_DATA;
    v = strtok_r(NULL, " ", save);
    if (!v) v = "";
  }
  char *end;
  errno = 0;
  unsigned long off = strtoul(v, &end, 10);
  char *hex = strtok_r(NULL, "", save);
  bool ok = end != v && *end == '\0' && errno == 0 && hex != NULL;
  uint8_t b[ZPAT_LEN_MAX];
  unsigned nib = 0;
  for (const char *p = hex; ok && *p; p++) {
    char c = *p;
    if (c == ' ') continue;
    int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 :
            (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
    if (d < 0 || nib >= 2u * ZPAT_LEN_MAX) {
      ok = false;
    } else {
      b[nib / 2u] = (nib % 2u == 0) ? (uint8_t) (d << 4) : (uint8_t) (b[nib / 2u] | d);
      nib++;
    }
  }
  if (!ok || nib == 0 || nib % 2u != 0) {
    uart1_puts("ERR zpat: expected zpat, zpat clear or zpat [h|d] <offset> <1..16 hex bytes>\r\n");
    return;
  }
  unsigned len = nib / 2u;
  unsigned long total = (cmd == ZPAT_HEAD) ? DCLR_HEAD_TOTAL : DCVL_TOTAL;
  if (off > total - len) {
    snprintf(buf, sizeof(buf), "ERR zpat: past the end of the %s reply (%lu bytes)\r\n", tag_name[cmd], total);
    uart1_puts(buf);
    return;
  }

  bool counts = (cmd == ZPAT_DCVL && (off < DCVL_COUNTS || (off < DCVL_HALF2 + DCVL_COUNTS && off + len > DCVL_HALF2))) ||
                (cmd == ZPAT_HEAD && off < DCLR_HEAD_COUNTS);
  if (counts) {
    snprintf(buf, sizeof(buf), "ERR zpat: overlaps the counts of %s (%s)\r\n", tag_name[cmd],
             cmd == ZPAT_DCVL ? "0..11, 307204..307215" : "0..15");
    uart1_puts(buf);
    return;
  }
  if (g_zpat_n >= ZPAT_MAX) {
    uart1_puts("ERR zpat: table full (8 inserts), use zpat clear\r\n");
    return;
  }
  for (unsigned i = 0; i < g_zpat_n; i++) {
    if (g_zpat[i].cmd == cmd && off < g_zpat[i].off + g_zpat[i].len && g_zpat[i].off < off + len) {
      snprintf(buf, sizeof(buf), "ERR zpat: overlaps the insert at %s%lu\r\n", tag_arg[cmd], (unsigned long) g_zpat[i].off);
      uart1_puts(buf);
      return;
    }
  }
  zpat_t *z = &g_zpat[g_zpat_n];
  z->off = (uint32_t) off;
  z->len = (uint8_t) len;
  z->cmd = cmd;
  memcpy(z->b, b, len);
  g_zpat_n++;
  uart1_puts("OK\r\n");
}

static void handle_udec(void) {
  static const char *const err_name[] = {"", "", "", " Stop Err", " Parity Err"};
  char d[160];
  char buf[200];
  for (unsigned bus = 0; bus < UDEC_BUSES; bus++) {
    const udec_cfg_t *u = udec_cfg_of(bus);
    if (bus == 0u) {
      snprintf(buf, sizeof(buf), "udec: DCST %s, bus 1 protocol %s, FOMT %s\r\n", u->on ? "ON" : "OFF",
               u->uart ? "UART" : "not UART", udec_fmt_name[u->fmt]);
    } else {
      snprintf(buf, sizeof(buf), "bus 2 protocol %s, FOMT %s\r\n", u->uart ? "UART" : "not UART", udec_fmt_name[u->fmt]);
    }
    uart1_puts(buf);
    if (u->dcut) {
      udec_cfg_str(bus, d, sizeof(d));
    } else {
      snprintf(d, sizeof(d), "none yet");
    }
    snprintf(buf, sizeof(buf), "B%u:DCUT %s\r\n", bus + 1u, d);
    uart1_puts(buf);
    if (bus == 0u) {
      udec_res_str(d, sizeof(d));
      snprintf(buf, sizeof(buf), "last DCVL?: %s\r\n", d);
      uart1_puts(buf);
    }
  }
  udec_buses_str(d, sizeof(d));
  snprintf(buf, sizeof(buf), "buses: %s\r\n", d);
  uart1_puts(buf);
  for (unsigned bus = 0; bus < UDEC_BUSES; bus++) {
    unsigned dir, i;
    for (unsigned k = 0; k < UDEC_LIST_MAX && udec_line(bus, k, &dir, &i); k++) {
      const udec_res_t *r = udec_res_of(bus, dir);
      char tm[UDEC_TIME_MAX], tx[UDEC_TEXT_MAX];
      if (k == 0u && bus == 1u) uart1_puts("bus 2 list:\r\n");
      udec_time_of(r, i, tm);
      udec_text_of(r, i, tx);
      snprintf(buf, sizeof(buf), "%u %s \"%s\" columns %u..%u%s%s\r\n", k + 1u, tm, tx, (unsigned) r->f[i].start,
               (unsigned) r->f[i].end, err_name[r->f[i].err], dir == UDEC_TX ? " Tx" : "");
      uart1_puts(buf);
    }
  }
}

static void handle_ugen(char **save) {
  char *v = strtok_r(NULL, " ", save);
  char buf[128];
  if (!v) {
    ugen_t g = ugen_get();
    if (!g.on) {
      uart1_puts("ugen=off\r\n");
      return;
    }
    int n = snprintf(buf, sizeof(buf), "ugen=%lu", (unsigned long) g.baud);
    for (unsigned i = 0; i < g.n; i++) {
      n += snprintf(buf + n, sizeof(buf) - (size_t) n, " %02X", g.b[i]);
    }
    snprintf(buf + n, sizeof(buf) - (size_t) n, " gap %u sent=%lu (PB12, ARR=%lu)\r\n", g.gap, (unsigned long) g.sent,
             (unsigned long) g.arr);
    uart1_puts(buf);
    return;
  }
  if (strcmp(v, "off") == 0) {
    ugen_stop();
    uart1_puts("OK\r\n");
    return;
  }
  uint32_t baud;
  bool ok = parse_u32(v, &baud) && baud >= UGEN_BAUD_MIN && baud <= UGEN_BAUD_MAX;
  uint8_t b[UGEN_BYTES_MAX];
  unsigned nib = 0;
  unsigned gap = UGEN_GAP_DEFAULT;
  for (char *w = strtok_r(NULL, " ", save); ok && w != NULL; w = strtok_r(NULL, " ", save)) {
    if (strcmp(w, "gap") == 0) {
      char *g = strtok_r(NULL, " ", save);
      uint16_t gv = 0;
      ok = g != NULL && parse_u16(g, &gv) && strtok_r(NULL, " ", save) == NULL;
      gap = gv;
      break;
    }
    if (strlen(w) % 2u != 0) ok = false;
    for (const char *p = w; ok && *p; p++) {
      char c = *p;
      int d = (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10 :
              (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
      if (d < 0 || nib >= 2u * UGEN_BYTES_MAX) {
        ok = false;
      } else {
        b[nib / 2u] = (nib % 2u == 0) ? (uint8_t) (d << 4) : (uint8_t) (b[nib / 2u] | d);
        nib++;
      }
    }
  }
  if (!ok || nib == 0) {
    uart1_puts("ERR ugen: expected ugen, ugen off or ugen <1200..19200> <1..16 hex bytes> [gap <bits>]\r\n");
    return;
  }
  ugen_start(baud, b, nib / 2u, gap);
  uart1_puts("OK\r\n");
}

_Static_assert(CHAM_BOOT_UGEN_MAX == UGEN_BYTES_MAX, "boot_ugen_b holds the generator's bytes");

static void boot_capture(void) {
  ugen_t g = ugen_get();
  g_desc.boot_ugen_n = g.on ? (uint8_t) g.n : 0u;
  memcpy(g_desc.boot_ugen_b, g.b, sizeof(g_desc.boot_ugen_b));
  g_desc.boot_ugen_gap = (uint16_t) g.gap;
  g_desc.boot_ugen_baud = g.baud;
  scpi_dtrig_t d = scpi_dtrig_get();
  sampler_trig_t t = sampler_trig_get();
  g_desc.boot_trig_mode = d.on ? d.uart_mode : t.mode;
  g_desc.boot_trig_ch = d.on ? d.uart_ch : t.channel;
  g_desc.boot_trig_idle = d.on ? d.uart_idle : t.idle;
}

static void print_boot(void) {
  char buf[128];
  int n = snprintf(buf, sizeof(buf), "boot: ugen=");
  if (g_desc.boot_ugen_n == 0u) {
    n += snprintf(buf + n, sizeof(buf) - (size_t) n, "off");
  } else {
    n += snprintf(buf + n, sizeof(buf) - (size_t) n, "%lu", (unsigned long) g_desc.boot_ugen_baud);
    for (unsigned i = 0; i < g_desc.boot_ugen_n && i < CHAM_BOOT_UGEN_MAX; i++) {
      n += snprintf(buf + n, sizeof(buf) - (size_t) n, " %02X", g_desc.boot_ugen_b[i]);
    }
    n += snprintf(buf + n, sizeof(buf) - (size_t) n, " gap %u", g_desc.boot_ugen_gap);
  }
  char t[24];
  scpi_trig_text(t, sizeof(t), g_desc.boot_trig_mode, g_desc.boot_trig_ch, g_desc.boot_trig_idle);
  snprintf(buf + n, sizeof(buf) - (size_t) n, " trig=%s\r\n", t);
  uart1_puts(buf);
}

static void boot_apply(void) {
  if (g_desc.boot_ugen_n >= 1u && g_desc.boot_ugen_n <= UGEN_BYTES_MAX && g_desc.boot_ugen_baud >= UGEN_BAUD_MIN &&
      g_desc.boot_ugen_baud <= UGEN_BAUD_MAX) {
    ugen_start(g_desc.boot_ugen_baud, g_desc.boot_ugen_b, g_desc.boot_ugen_n, g_desc.boot_ugen_gap);
  }
  if ((g_desc.boot_trig_mode == SAMPLER_TRIG_RISE || g_desc.boot_trig_mode == SAMPLER_TRIG_FALL) && g_desc.boot_trig_ch <= 15u) {
    sampler_trig_set(g_desc.boot_trig_mode, g_desc.boot_trig_ch, g_desc.boot_trig_idle);
  }
}

static void handle_pava(char **save) {
  static const char *const mode_name[] = {"off", "calc", "probe", "probei"};
  char *v = strtok_r(NULL, " ", save);
  char buf[200];
  if (!v) {
    char src[8] = "-";
    if (g_pava.measc >= 0) snprintf(src, sizeof(src), "D%d", (int) g_pava.measc);
    snprintf(buf, sizeof(buf), "pava=%s duty=%s measc=%s hdr=%s\r\n", mode_name[g_pava.mode], g_pava.duty_frac ? "frac" : "pct", src,
             g_pava.hdr_set ? "words" : "auto");
    uart1_puts(buf);
    for (unsigned i = 0; i < SCPI_PAVA_HDR_WORDS; i++) {
      if (g_pava.hdr[i] != 0u) {
        snprintf(buf, sizeof(buf), "hdr %u = 0x%08lX\r\n", i, (unsigned long) g_pava.hdr[i]);
        uart1_puts(buf);
      }
    }
    int n = snprintf(buf, sizeof(buf), "CMPA n=%u", (unsigned) g_pava.n);
    for (unsigned i = 0; i < g_pava.n; i++) {
      n += snprintf(buf + n, sizeof(buf) - (size_t) n, " (%lu,%lu,%lu)", (unsigned long) g_pava.item[i][0],
                    (unsigned long) g_pava.item[i][1], (unsigned long) g_pava.item[i][2]);
    }
    snprintf(buf + n, sizeof(buf) - (size_t) n, "\r\n");
    uart1_puts(buf);
    uint8_t mode;
    const uint8_t *r = scpi_pava_last(&mode);
    if (r == NULL) {
      uart1_puts("last reply: none yet\r\n");
      return;
    }
    snprintf(buf, sizeof(buf), "last reply: %s\r\n", mode_name[mode]);
    uart1_puts(buf);
    for (unsigned i = 0; i < SCPI_PAVA_ITEMS; i++) {
      const uint8_t *it = r + 64u + 56u * i;
      uint32_t f[3];
      memcpy(f, it + 40, sizeof(f));
      if (f[0] == 0u && f[2] == 0u) continue;
      n = snprintf(buf, sizeof(buf), "item %u type=%lu src1=%lu src2=%lu:", i, (unsigned long) f[2], (unsigned long) f[0],
                   (unsigned long) f[1]);
      for (unsigned j = 0; j < 5u; j++) {
        char t[16];
        if (mode == SCPI_PAVA_PROBEI) {
          int64_t k;
          memcpy(&k, it + 8u * j, 8);
          snprintf(t, sizeof(t), "%ld", (long) k);
        } else {
          double d;
          memcpy(&d, it + 8u * j, 8);
          scpi_dbl_text(t, sizeof(t), d);
        }
        n += snprintf(buf + n, sizeof(buf) - (size_t) n, " %s", t);
      }
      snprintf(buf + n, sizeof(buf) - (size_t) n, "\r\n");
      uart1_puts(buf);
    }
    return;
  }
  for (unsigned m = 0; m < 4u; m++) {
    if (strcmp(v, mode_name[m]) == 0 && strtok_r(NULL, " ", save) == NULL) {
      g_pava.mode = (uint8_t) m;
      uart1_puts("OK\r\n");
      return;
    }
  }
  if (strcmp(v, "duty") == 0) {
    char *w = strtok_r(NULL, " ", save);
    if (w != NULL && (strcmp(w, "pct") == 0 || strcmp(w, "frac") == 0) && strtok_r(NULL, " ", save) == NULL) {
      g_pava.duty_frac = strcmp(w, "frac") == 0;
      uart1_puts("OK\r\n");
      return;
    }
  }
  if (strcmp(v, "hdr") == 0) {
    char *w = strtok_r(NULL, " ", save);
    char *x = strtok_r(NULL, " ", save);
    uint8_t i;
    uint32_t val;
    if (w != NULL && strcmp(w, "clear") == 0 && x == NULL) {
      memset(g_pava.hdr, 0, sizeof(g_pava.hdr));
      g_pava.hdr_set = false;
      uart1_puts("OK\r\n");
      return;
    }
    if (w != NULL && x != NULL && strtok_r(NULL, " ", save) == NULL && parse_u8(w, &i) && i < SCPI_PAVA_HDR_WORDS &&
        parse_u32(x, &val)) {
      g_pava.hdr[i] = val;
      g_pava.hdr_set = true;
      uart1_puts("OK\r\n");
      return;
    }
  }
  uart1_puts("ERR pava: expected pava, pava off|calc|probe|probei, pava duty pct|frac, pava hdr <0..15> <u32> or pava hdr clear\r\n");
}

static bool baud_allowed(uint32_t baud) {

  return baud == 115200u || baud == 230400u || baud == 460800u || baud == 921600u || baud == 1500000u;
}

static void handle_baud(char **save) {
  char *v = strtok_r(NULL, " ", save);
  if (!v) {
    uart1_puts("ERR baud: missing value\r\n");
    return;
  }
  uint32_t baud;
  if (!parse_u32(v, &baud) || !baud_allowed(baud)) {
    uart1_puts("ERR baud: allowed values are 115200, 230400, 460800, 921600, 1500000\r\n");
    return;
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "OK baud %lu\r\n", (unsigned long) baud);
  uart1_puts(buf);
  uart1_wait_tx_complete();
  uart1_init(baud);
}

static bool refuse_busy(const char *cmd, bool force) {
  if (force || !scpi_emul_busy()) return false;
  char buf[128];
  snprintf(buf, sizeof(buf), "ERR %s: a reply or the zero stream is in progress, try again%s\r\n", cmd,
           (strcmp(cmd, "disconnect") == 0 || strcmp(cmd, "reenum") == 0) ? " (or force)" : "");
  uart1_puts(buf);
  return true;
}

static void handle_line(char *s) {
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == '\r' || s[n - 1] == '\n')) {
    s[--n] = 0;
  }
  if (n == 0) return;

  char *save = NULL;
  char *cmd = strtok_r(s, " ", &save);
  if (!cmd) return;

  if (strcmp(cmd, "show") == 0) {
    print_show();
    print_resp_table();
    print_boot();
  } else if (strcmp(cmd, "save") == 0) {
    if (refuse_busy(cmd, false)) return;
    boot_capture();
    tud_disconnect();
    uart1_puts(chameleon_config_save() ? "saved; use connect\r\n" : "ERR save\r\n");
  } else if (strcmp(cmd, "load") == 0) {
    if (refuse_busy(cmd, false)) return;
    chameleon_config_load();
    resp_drop_forbidden("flash");
    uart1_puts("OK load\r\n");
  } else if (strcmp(cmd, "default") == 0) {
    if (refuse_busy(cmd, false)) return;
    chameleon_config_default();
    uart1_puts("OK default\r\n");
  } else if (strcmp(cmd, "connect") == 0) {
    tud_connect();
    uart1_puts("OK connect\r\n");
  } else if (strcmp(cmd, "disconnect") == 0) {
    char *arg = strtok_r(NULL, " ", &save);
    bool force = arg != NULL && strcmp(arg, "force") == 0;
    if ((arg != NULL && !force) || strtok_r(NULL, " ", &save) != NULL) {
      uart1_puts("ERR disconnect: expected disconnect or disconnect force\r\n");
      return;
    }
    if (refuse_busy(cmd, force)) return;
    tud_disconnect();
    uart1_puts("OK disconnect\r\n");
  } else if (strcmp(cmd, "reenum") == 0) {
    char *arg = strtok_r(NULL, " ", &save);
    bool force = arg != NULL && strcmp(arg, "force") == 0;
    if (force) {
      arg = strtok_r(NULL, " ", &save);
    } else if (arg != NULL) {
      const char *t = strtok_r(NULL, " ", &save);
      force = t != NULL && strcmp(t, "force") == 0;
    }
    if (refuse_busy(cmd, force)) return;
    uint32_t delay_ms = REENUM_DEFAULT_MS;
    if (arg) {
      uint16_t v;
      if (!parse_u16(arg, &v)) {
        uart1_puts("ERR bad value\r\n");
        return;
      }
      delay_ms = v;
    }
    do_reenum(delay_ms);
  } else if (strcmp(cmd, "set") == 0) {
    handle_set(&save);
  } else if (strcmp(cmd, "resp") == 0) {
    handle_resp(&save);
  } else if (strcmp(cmd, "unknown") == 0) {
    handle_unknown(&save);
  } else if (strcmp(cmd, "log") == 0) {
    handle_log(&save);
  } else if (strcmp(cmd, "baud") == 0) {
    handle_baud(&save);
  } else if (strcmp(cmd, "trig") == 0) {
    handle_trig(&save);
  } else if (strcmp(cmd, "trdl") == 0) {
    handle_trdl();
  } else if (strcmp(cmd, "hcomp") == 0) {
    handle_hcomp(&save);
  } else if (strcmp(cmd, "zpat") == 0) {
    handle_zpat(&save);
  } else if (strcmp(cmd, "udec") == 0) {
    handle_udec();
  } else if (strcmp(cmd, "ugen") == 0) {
    handle_ugen(&save);
  } else if (strcmp(cmd, "pava") == 0) {
    handle_pava(&save);
  } else {
    uart1_puts("ERR unknown command\r\n");
  }
}

void cli_init(void) {
  line_len = 0;
  uart1_puts("\r\nblackpill-sbus-la ready\r\n");
  print_reset();
  resp_drop_forbidden("flash");
  boot_apply();
  print_boot();
}

void cli_poll(void) {
  uint8_t byte;
  while (uart1_read(&byte, 1) == 1) {
    if (byte == '\n' || byte == '\r') {
      if (line_len > 0) {
        line_buf[line_len] = 0;
        handle_line(line_buf);
        line_len = 0;
      }
    } else if (line_len < LINE_MAX - 1) {
      line_buf[line_len++] = (char) byte;
    } else {
      line_len = 0;
      uart1_puts("ERR line too long\r\n");
    }
  }
}
