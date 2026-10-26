#include <string.h>
#include "stm32f4xx_hal.h"
#include "uart_gen.h"

#define UGEN_PIN   12u
#define UGEN_HIGH  (1u << UGEN_PIN)
#define UGEN_LOW   (1u << (UGEN_PIN + 16u))
#define UGEN_PRIO  1u
#define UGEN_SLOTS 10u

static uint8_t  s_b[UGEN_BYTES_MAX];
static unsigned s_n, s_gap;
static uint32_t s_baud, s_arr;
static bool     s_on;

static unsigned s_slot, s_idx;
static uint32_t s_next;
static volatile uint32_t s_sent;

static uint32_t slot_bsrr(unsigned slot, uint8_t byte) {
  if (slot == 0u) return UGEN_LOW;
  if (slot <= 8u) return ((byte >> (slot - 1u)) & 1u) ? UGEN_HIGH : UGEN_LOW;
  return UGEN_HIGH;
}

void TIM2_IRQHandler(void) {
  GPIOB->BSRR = s_next;
  TIM2->SR = ~TIM_SR_UIF;
  unsigned slot = s_slot + 1u;
  unsigned idx = s_idx;
  if (slot >= UGEN_SLOTS + s_gap) {
    slot = 0u;
    idx = (idx + 1u < s_n) ? idx + 1u : 0u;
    s_sent++;
  }
  s_slot = slot;
  s_idx = idx;
  s_next = slot_bsrr(slot, s_b[idx]);
}

static uint32_t tim2_clock_hz(void) {
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  return ((RCC->CFGR & RCC_CFGR_PPRE1) >= RCC_CFGR_PPRE1_DIV2) ? 2u * pclk1 : pclk1;
}

static void timer_off(void) {
  TIM2->CR1 = 0u;
  TIM2->DIER = 0u;
  NVIC_DisableIRQ(TIM2_IRQn);
  NVIC_ClearPendingIRQ(TIM2_IRQn);
}

void ugen_start(uint32_t baud, const uint8_t *b, unsigned n, unsigned gap) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  (void) RCC->APB1ENR;
  timer_off();
  memcpy(s_b, b, n);
  s_n = n;
  s_gap = gap;
  s_baud = baud;
  s_slot = 0u;
  s_idx = 0u;
  s_sent = 0u;
  s_next = slot_bsrr(0u, s_b[0]);
  GPIOB->BSRR = UGEN_HIGH;
  GPIOB->MODER = (GPIOB->MODER & ~(3u << (2u * UGEN_PIN))) | (1u << (2u * UGEN_PIN));
  s_arr = (tim2_clock_hz() + baud / 2u) / baud - 1u;
  TIM2->CR1 = TIM_CR1_URS;
  TIM2->PSC = 0u;
  TIM2->ARR = s_arr;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0u;
  TIM2->DIER = TIM_DIER_UIE;
  NVIC_SetPriority(TIM2_IRQn, UGEN_PRIO);
  NVIC_EnableIRQ(TIM2_IRQn);
  s_on = true;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void ugen_stop(void) {
  timer_off();
  GPIOB->MODER &= ~(3u << (2u * UGEN_PIN));
  s_on = false;
}

ugen_t ugen_get(void) {
  ugen_t g = {s_on, s_baud, s_arr, s_n, {0}, s_gap, s_sent};
  memcpy(g.b, s_b, sizeof(g.b));
  return g;
}
