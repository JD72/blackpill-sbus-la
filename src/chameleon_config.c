#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "chameleon_config.h"

#define CHAM_FLASH_SECTOR   FLASH_SECTOR_4
#ifndef CHAM_FLASH_ADDR
#define CHAM_FLASH_ADDR     0x08010000u
#endif
#define CHAM_FLASH_MAGIC    0x484D4333u
#define CHAM_CONFIG_VERSION 4u

chameleon_desc_t g_desc;

typedef struct {
  uint8_t head[offsetof(chameleon_desc_t, boot_ugen_n)];
  uint32_t crc32;
} cham_desc_v3_t;

static uint32_t crc32_compute(const void *data, size_t len) {
  const uint8_t *p = (const uint8_t *) data;
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t) (-(int32_t) (crc & 1u)));
    }
  }
  return ~crc;
}

static uint32_t desc_crc(const chameleon_desc_t *d) {

  return crc32_compute(d, offsetof(chameleon_desc_t, crc32));
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

const char *cham_resp_find(const char *cmd) {
  for (size_t i = 0; i < CHAM_RESP_MAX; i++) {
    if (g_desc.resp[i].key[0] != 0 && str_ieq(g_desc.resp[i].key, cmd)) {
      return g_desc.resp[i].val;
    }
  }
  return NULL;
}

bool cham_resp_set(const char *key, const char *val) {
  if (key == NULL || key[0] == 0 || val == NULL) return false;
  if (strlen(key) > CHAM_RESP_KEY_MAX - 1 || strlen(val) > CHAM_RESP_VAL_MAX - 1) return false;

  int free_slot = -1;
  for (size_t i = 0; i < CHAM_RESP_MAX; i++) {
    if (g_desc.resp[i].key[0] != 0 && str_ieq(g_desc.resp[i].key, key)) {
      strncpy(g_desc.resp[i].val, val, CHAM_RESP_VAL_MAX - 1);
      g_desc.resp[i].val[CHAM_RESP_VAL_MAX - 1] = 0;
      return true;
    }
    if (free_slot < 0 && g_desc.resp[i].key[0] == 0) free_slot = (int) i;
  }
  if (free_slot < 0) return false;

  strncpy(g_desc.resp[free_slot].key, key, CHAM_RESP_KEY_MAX - 1);
  g_desc.resp[free_slot].key[CHAM_RESP_KEY_MAX - 1] = 0;
  strncpy(g_desc.resp[free_slot].val, val, CHAM_RESP_VAL_MAX - 1);
  g_desc.resp[free_slot].val[CHAM_RESP_VAL_MAX - 1] = 0;
  return true;
}

bool cham_resp_del(const char *key) {
  if (key == NULL) return false;
  for (size_t i = 0; i < CHAM_RESP_MAX; i++) {
    if (g_desc.resp[i].key[0] != 0 && str_ieq(g_desc.resp[i].key, key)) {
      g_desc.resp[i].key[0] = 0;
      g_desc.resp[i].val[0] = 0;
      return true;
    }
  }
  return false;
}

void chameleon_config_default(void) {
  memset(&g_desc, 0, sizeof(g_desc));
  g_desc.magic = CHAM_FLASH_MAGIC;
  g_desc.struct_version = CHAM_CONFIG_VERSION;
  g_desc.vid = 0x1209u;
  g_desc.pid = 0x0001u;
  g_desc.bcd_device = 0x0100u;
  g_desc.bcd_usb = 0x0200u;
  g_desc.dev_class = 0x00u;
  g_desc.dev_subclass = 0x00u;
  g_desc.dev_protocol = 0x00u;
  g_desc.max_packet0 = 64u;
  strncpy(g_desc.mfr, "blackpill-sbus-la", CHAM_STR_MAX - 1);
  strncpy(g_desc.prod, "SBUS logic analyzer", CHAM_STR_MAX - 1);

  strncpy(g_desc.serial, "BPL000001", CHAM_STR_MAX - 1);

  cham_resp_set("*IDN?", "blackpill-sbus-la LA,SLA1016,BPL000001,7.0.9.0.1");
  cham_resp_set("IDN-SGLT-PRI?", "blackpill-sbus-la LA,SLA1016,BPL000001,7.0.9.0.1");
  cham_resp_set("DI:SV?", "DI:SV 7.0.9.0.1");
  cham_resp_set("DI:FV?", "DI:FV 1");
  cham_resp_set("DI:PV?", "DI:PV 1");
  cham_resp_set("DI:BV?", "DI:BV 1");
  cham_resp_set("DI:CMF?", "DI:CMF 0");
  cham_resp_set("DI:DTANF?", "DI:DTANF? 1");

  {
    char srln[CHAM_RESP_VAL_MAX];
    snprintf(srln, sizeof(srln), "%-13s%s", "SRLN", g_desc.serial);
    cham_resp_set("SRLN?", srln);
  }
  g_desc.unknown_resp[0] = 0;
  g_desc.unknown_auto = 1;

  g_desc.crc32 = desc_crc(&g_desc);
}

bool chameleon_config_load(void) {
  const chameleon_desc_t *flash_desc = (const chameleon_desc_t *) CHAM_FLASH_ADDR;
  const cham_desc_v3_t *v3 = (const cham_desc_v3_t *) CHAM_FLASH_ADDR;
  bool ok;

  if (flash_desc->magic == CHAM_FLASH_MAGIC && flash_desc->struct_version == CHAM_CONFIG_VERSION &&
      flash_desc->crc32 == desc_crc(flash_desc)) {
    memcpy(&g_desc, flash_desc, sizeof(g_desc));
    ok = true;
  } else if (flash_desc->magic == CHAM_FLASH_MAGIC && flash_desc->struct_version == 3u &&
             v3->crc32 == crc32_compute(v3, offsetof(cham_desc_v3_t, crc32))) {
    memset(&g_desc, 0, sizeof(g_desc));
    memcpy(&g_desc, v3->head, sizeof(v3->head));
    g_desc.struct_version = CHAM_CONFIG_VERSION;
    g_desc.crc32 = desc_crc(&g_desc);
    ok = true;
  } else {
    chameleon_config_default();
    ok = false;
  }

  g_desc.max_packet0 = 64u;
  return ok;
}

bool chameleon_config_save(void) {
  g_desc.magic = CHAM_FLASH_MAGIC;
  g_desc.struct_version = CHAM_CONFIG_VERSION;
  g_desc.crc32 = desc_crc(&g_desc);

  HAL_FLASH_Unlock();

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  FLASH_EraseInitTypeDef erase = {
      .TypeErase = FLASH_TYPEERASE_SECTORS,
      .Sector = CHAM_FLASH_SECTOR,
      .NbSectors = 1,
      .VoltageRange = FLASH_VOLTAGE_RANGE_3,
  };
  uint32_t sector_error = 0;
  bool ok = (HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK);

  if (ok) {
    const uint32_t *src = (const uint32_t *) &g_desc;
    uint32_t words = (sizeof(g_desc) + 3u) / 4u;
    for (uint32_t i = 0; i < words && ok; i++) {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CHAM_FLASH_ADDR + i * 4u, src[i]) != HAL_OK) {
        ok = false;
      }
    }
  }

  HAL_FLASH_Lock();
  return ok;
}
