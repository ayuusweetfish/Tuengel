#include "py32f0xx_hal.h"
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define RELEASE

#ifndef RELEASE
static uint8_t swv_buf[64];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  if (c == '\n') {
    swv_buf[swv_buf_ptr >= sizeof swv_buf ?
      (sizeof swv_buf - 1) : swv_buf_ptr] = '\0';
    swv_trap_line();
    swv_buf_ptr = 0;
  } else if (++swv_buf_ptr <= sizeof swv_buf) {
    swv_buf[swv_buf_ptr - 1] = c;
  }
}
static void swv_printf(const char *restrict fmt, ...)
{
  static char s[32];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}
#else
#define swv_printf(...)
#endif

int main(void)
{
  HAL_Init();

  // ======== Clocks ========
  HAL_RCC_OscConfig(&(RCC_OscInitTypeDef){
    // .OscillatorType = RCC_OSCILLATORTYPE_HSE,
    // .HSEState = RCC_HSE_BYPASS_ENABLE,
    .OscillatorType = RCC_OSCILLATORTYPE_HSI,
    .HSIState = RCC_HSI_ON,
    .HSIDiv = RCC_HSI_DIV1,
    .HSICalibrationValue = RCC_HSICALIBRATION_24MHz,
  });

  HAL_RCC_ClockConfig(&(RCC_ClkInitTypeDef){
    .ClockType =
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_PCLK1,
    // .SYSCLKSource = RCC_SYSCLKSOURCE_HSE, // 24 MHz
    .SYSCLKSource = RCC_SYSCLKSOURCE_HSISYS,
    .AHBCLKDivider = RCC_SYSCLK_DIV1,
    .APB1CLKDivider = RCC_HCLK_DIV1,
  }, FLASH_LATENCY_1);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PA2 - SWCLK, PB6 - SWDIO with correct pull-up/-down upon reset
  // Reference manual v. 1.0 p. 79 (zh) / 77 (en)

  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6,
    .Mode = GPIO_MODE_OUTPUT_PP,
  });
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);

  uint32_t last_tick = HAL_GetTick();
  bool parity = 0;
  while (1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, parity ^= 1);
    uint32_t cur_tick;
    while ((cur_tick = HAL_GetTick()) - last_tick < 200)
      HAL_PWR_EnterSLEEPMode(PWR_SLEEPENTRY_WFI);
    last_tick = cur_tick;
    swv_printf("%lu\n", cur_tick);
  }
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
void SVC_Handler() { while (1) { } }
void PendSV_Handler() { while (1) { } }
void SysTick_Handler()
{
  HAL_IncTick();
}
