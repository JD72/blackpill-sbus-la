#ifndef CHAMELEON_CONFIG_H_
#define CHAMELEON_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#define CHAM_STR_MAX 32u

#define CHAM_RESP_MAX     16u
#define CHAM_RESP_KEY_MAX 24u
#define CHAM_RESP_VAL_MAX 60u
#define CHAM_BOOT_UGEN_MAX 16u

typedef struct {
  char key[CHAM_RESP_KEY_MAX];
  char val[CHAM_RESP_VAL_MAX];
} cham_resp_t;

typedef struct {
  uint32_t magic;
  uint16_t struct_version;
  uint16_t vid;
  uint16_t pid;
  uint16_t bcd_device;
  uint16_t bcd_usb;
  uint8_t  dev_class;
  uint8_t  dev_subclass;
  uint8_t  dev_protocol;
  uint8_t  max_packet0;
  char     mfr[CHAM_STR_MAX];
  char     prod[CHAM_STR_MAX];
  char     serial[CHAM_STR_MAX];
  cham_resp_t resp[CHAM_RESP_MAX];
  char     unknown_resp[CHAM_RESP_VAL_MAX];
  uint8_t  unknown_auto;

  uint8_t  boot_ugen_n;
  uint8_t  boot_ugen_b[CHAM_BOOT_UGEN_MAX];
  uint16_t boot_ugen_gap;
  uint32_t boot_ugen_baud;
  uint8_t  boot_trig_mode;
  uint8_t  boot_trig_ch;
  uint16_t boot_trig_idle;
  uint32_t crc32;
} chameleon_desc_t;

extern chameleon_desc_t g_desc;

void chameleon_config_default(void);

bool chameleon_config_load(void);

bool chameleon_config_save(void);

const char *cham_resp_find(const char *cmd);

bool cham_resp_set(const char *key, const char *val);

bool cham_resp_del(const char *key);

#endif
