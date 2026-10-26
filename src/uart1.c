#include <string.h>
#include "stm32f4xx_hal.h"
#include "uart1.h"
#include "log_ring.h"

#define UART1_RXBUF_SIZE 128u

static volatile uint8_t rx_buf[UART1_RXBUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

void uart1_init(uint32_t baudrate) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t div100 = (pclk2 * 25u) / (4u * baudrate);
  uint32_t mant = div100 / 100u;
  uint32_t frac = ((div100 - mant * 100u) * 16u + 50u) / 100u;
  if (frac >= 16u) {
    frac = 0u;
    mant += 1u;
  }
  USART1->BRR = (uint16_t) ((mant << 4) | frac);

  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  NVIC_SetPriority(USART1_IRQn, 5);
  NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) {
  uint32_t sr = USART1->SR;
  if (sr & USART_SR_RXNE) {
    uint8_t byte = (uint8_t) USART1->DR;
    uint16_t next = (uint16_t) ((rx_head + 1u) % UART1_RXBUF_SIZE);
    if (next != rx_tail) {
      rx_buf[rx_head] = byte;
      rx_head = next;
    }
  } else if (sr & USART_SR_ORE) {
    (void) USART1->DR;
  }
}

size_t uart1_read(uint8_t *buf, size_t maxlen) {
  size_t n = 0;
  while (n < maxlen && rx_tail != rx_head) {
    buf[n++] = rx_buf[rx_tail];
    rx_tail = (uint16_t) ((rx_tail + 1u) % UART1_RXBUF_SIZE);
  }
  return n;
}

size_t uart1_write_nb(const void *buf, size_t len) {
  const uint8_t *p = (const uint8_t *) buf;
  size_t i = 0;
  while (i < len && (USART1->SR & USART_SR_TXE)) {
    USART1->DR = p[i++];
  }
  return i;
}

void uart1_write(const void *buf, size_t len) {
  const uint8_t *p = (const uint8_t *) buf;
  log_drain();
  for (size_t i = 0; i < len; i++) {
    while (!(USART1->SR & USART_SR_TXE)) {
    }
    USART1->DR = p[i];
  }
}

void uart1_puts(const char *s) {
  uart1_write(s, strlen(s));
}

void uart1_wait_tx_complete(void) {
  while (!(USART1->SR & USART_SR_TC)) {
  }
}
