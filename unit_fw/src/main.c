#include "py32f0xx_hal.h"
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../misc/crc32/crc32.h"

// #define RELEASE

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

static TIM_HandleTypeDef tim1;
static UART_HandleTypeDef uart1;

static uint8_t rx_len = 0;
static uint8_t rx_buf[256 + 4];

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

  // PA6 - Act LED
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6,
    .Mode = GPIO_MODE_OUTPUT_PP,
  });
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);

  while (0) {
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

    static int count = 0;
    if (++count == 6) {
      HAL_RCC_OscConfig(&(RCC_OscInitTypeDef){
        .OscillatorType = RCC_OSCILLATORTYPE_HSE,
        .HSEState = RCC_HSE_BYPASS_ENABLE,
      });
      HAL_RCC_ClockConfig(&(RCC_ClkInitTypeDef){
        .ClockType =
          RCC_CLOCKTYPE_SYSCLK |
          RCC_CLOCKTYPE_HCLK |
          RCC_CLOCKTYPE_PCLK1,
        .SYSCLKSource = RCC_SYSCLKSOURCE_HSE, // 24 MHz
        .AHBCLKDivider = RCC_SYSCLK_DIV1,
        .APB1CLKDivider = RCC_HCLK_DIV1,
      }, FLASH_LATENCY_1);
    }
  }

  // ====== (TIM1) Timer for servo control ======
  // PA7 - TIM1_CH4
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_7,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF2_TIM1,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });
  __HAL_RCC_TIM1_CLK_ENABLE();
  tim1 = (TIM_HandleTypeDef){
    .Instance = TIM1,
    .Init = {
      .Prescaler = 24 - 1,  // 1 us per count
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 20000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    },
  };
  HAL_TIM_PWM_Init(&tim1);
  TIM_OC_InitTypeDef tim1_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCPolarity = TIM_OCPOLARITY_LOW,
  };
  HAL_TIM_PWM_ConfigChannel(&tim1, &tim1_ch1_oc_init, TIM_CHANNEL_4);
  TIM1->CCR4 = 0;
  HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_4);

  // ====== (USART1) Serial communication ======
  // PA3 - USART_TX
  // PA4 - USART_RX
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_3 | GPIO_PIN_4,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF1_USART1,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });
  __HAL_RCC_USART1_CLK_ENABLE();
  uart1 = (UART_HandleTypeDef){
    .Instance = USART1,
    .Init = (UART_InitTypeDef){
      .BaudRate = 9600,
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits = UART_STOPBITS_1,
      .Parity = UART_PARITY_NONE,
      .Mode = UART_MODE_TX_RX,
      .HwFlowCtl = UART_HWCONTROL_NONE,
      .OverSampling = UART_OVERSAMPLING_16,
    },
  };
  HAL_UART_Init(&uart1);

  // PA5 - MAX487 driver enable
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_5,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&uart1, rx_buf, 1);

  while (1) {
    HAL_PWR_EnterSLEEPMode(PWR_SLEEPENTRY_WFI);
  }

/*
from math import *
N=100
print(', '.join('%d' % round(1500 + 200*(-cos(i/N*2*pi))) for i in range(N)))
*/
  static const uint16_t sin_lut[100] = {
1300, 1300, 1302, 1304, 1306, 1310, 1314, 1319, 1325, 1331, 1338, 1346, 1354, 1363, 1373, 1382, 1393, 1404, 1415, 1426, 1438, 1450, 1463, 1475, 1487, 1500, 1513, 1525, 1537, 1550, 1562, 1574, 1585, 1596, 1607, 1618, 1627, 1637, 1646, 1654, 1662, 1669, 1675, 1681, 1686, 1690, 1694, 1696, 1698, 1700, 1700, 1700, 1698, 1696, 1694, 1690, 1686, 1681, 1675, 1669, 1662, 1654, 1646, 1637, 1627, 1618, 1607, 1596, 1585, 1574, 1562, 1550, 1537, 1525, 1513, 1500, 1487, 1475, 1463, 1450, 1438, 1426, 1415, 1404, 1393, 1382, 1373, 1363, 1354, 1346, 1338, 1331, 1325, 1319, 1314, 1310, 1306, 1304, 1302, 1300
  };

  TIM1->CCR4 = sin_lut[0];

  int count = 0;
  int tick = HAL_GetTick();
  bool parity = 0;
  while (1) {
    int duty;
    duty = (count < 100 ? sin_lut[count] : sin_lut[0]);
    TIM1->CCR4 = duty;
    tick += 3;
    while (HAL_GetTick() - tick > 0x80000000)
      HAL_PWR_EnterSLEEPMode(PWR_SLEEPENTRY_WFI);
    if (++count == 300) {
      count = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, parity ^= 1);
    }
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

void USART1_IRQHandler()
{
  HAL_UART_IRQHandler(&uart1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *_uart1)
{
  if (rx_len == 0) {
    uint8_t len = rx_buf[0];
    if (len == 0) {
      // Ignore empty packet
      HAL_UART_Receive_IT(&uart1, rx_buf, 1);
    } else {
      // Receive payload
      rx_len = len;
      HAL_UART_Receive_IT(&uart1, rx_buf, (uint32_t)len + 4);
    }
  } else {
    // Packet complete!
    uint32_t s = crc32_bulk(rx_buf, (uint32_t)rx_len + 4);
    if (s == 0x2144DF1C) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
      HAL_Delay(3);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
      HAL_UART_Transmit(&uart1, (uint8_t *)"\x01\xAA\x7b\xa5\x01\xe4", 6, 1000);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
    }
    // Wait for next packet
    rx_len = 0;
    HAL_UART_Receive_IT(&uart1, rx_buf, 1);
  }
}
