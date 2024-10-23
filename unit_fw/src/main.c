#include "py32f0xx_hal.h"
#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../misc/crc32/crc32.h"

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

static TIM_HandleTypeDef tim1;
static UART_HandleTypeDef uart1;

static uint8_t rx_len = 0;
static uint8_t rx_buf[256 + 4];
static uint8_t rx_ptr = 0;
static uint8_t rx_byte;
static uint32_t rx_timestamp = 0xc0000000;  // For activity indicator

// 0 - Idle
// 1 - Strike
static volatile uint32_t op = 0;
static uint8_t strike_vel;
static inline void strike_init();
static inline void strike(uint8_t strike_vel);

static inline void spin_delay(uint32_t cycles)
{
  __asm__ volatile (
    "   cmp %[cycles], #5\n"
    "   ble 2f\n"
    "   sub %[cycles], #5\n"
    "   lsr %[cycles], #2\n"
    "1: sub %[cycles], #1\n"
    "   nop\n"
    "   bne 1b\n"   // 2 cycles if taken
    "2: \n"
    : [cycles] "+l" (cycles)
    : // No output
    : "cc"
  );
}

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
      .BaudRate = 115200,
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
  HAL_UART_Receive_IT(&uart1, &rx_byte, 1);

  strike_init();

  // Test clock frequency...???
  while (0) {
    uint32_t t1 = HAL_GetTick();
    spin_delay(24000000 * 0.57);
    uint32_t t2 = HAL_GetTick();
    swv_printf("%u\n", t2 - t1);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
  }

  while (1) {
    if (op == 0) {
      HAL_PWR_EnterSLEEPMode(PWR_SLEEPENTRY_WFI);
    } else if (op == 1) {
      strike(strike_vel);
      op = 0;
    }
  }
}

/*
from math import *
N=100
print(', '.join('%d' % round(1500 + 200*(-cos(i/N*2*pi))) for i in range(N)))
*/
static const uint16_t sin_lut[100] = {
1300, 1300, 1302, 1304, 1306, 1310, 1314, 1319, 1325, 1331, 1338, 1346, 1354, 1363, 1373, 1382, 1393, 1404, 1415, 1426, 1438, 1450, 1463, 1475, 1487, 1500, 1513, 1525, 1537, 1550, 1562, 1574, 1585, 1596, 1607, 1618, 1627, 1637, 1646, 1654, 1662, 1669, 1675, 1681, 1686, 1690, 1694, 1696, 1698, 1700, 1700, 1700, 1698, 1696, 1694, 1690, 1686, 1681, 1675, 1669, 1662, 1654, 1646, 1637, 1627, 1618, 1607, 1596, 1585, 1574, 1562, 1550, 1537, 1525, 1513, 1500, 1487, 1475, 1463, 1450, 1438, 1426, 1415, 1404, 1393, 1382, 1373, 1363, 1354, 1346, 1338, 1331, 1325, 1319, 1314, 1310, 1306, 1304, 1302, 1300
};

/*
print(', '.join('%d' % (72000*0.57 * (1.5**((128-i)/128.))) for i in range(256)))
*/
static const uint32_t vel_lut[256] = {
61560, 61365, 61171, 60977, 60784, 60592, 60401, 60209, 60019, 59829, 59640, 59451, 59263, 59076, 58889, 58703, 58517, 58332, 58148, 57964, 57780, 57598, 57415, 57234, 57053, 56872, 56693, 56513, 56335, 56156, 55979, 55802, 55625, 55449, 55274, 55099, 54925, 54751, 54578, 54405, 54233, 54062, 53891, 53720, 53550, 53381, 53212, 53044, 52876, 52709, 52542, 52376, 52210, 52045, 51881, 51717, 51553, 51390, 51227, 51065, 50904, 50743, 50582, 50423, 50263, 50104, 49946, 49788, 49630, 49473, 49317, 49161, 49005, 48850, 48696, 48542, 48388, 48235, 48083, 47931, 47779, 47628, 47477, 47327, 47177, 47028, 46879, 46731, 46583, 46436, 46289, 46143, 45997, 45851, 45706, 45562, 45418, 45274, 45131, 44988, 44846, 44704, 44563, 44422, 44281, 44141, 44002, 43862, 43724, 43585, 43448, 43310, 43173, 43037, 42900, 42765, 42630, 42495, 42360, 42226, 42093, 41960, 41827, 41695, 41563, 41431, 41300, 41170, 41040, 40910, 40780, 40651, 40523, 40395, 40267, 40139, 40013, 39886, 39760, 39634, 39509, 39384, 39259, 39135, 39011, 38888, 38765, 38642, 38520, 38398, 38277, 38156, 38035, 37915, 37795, 37675, 37556, 37437, 37319, 37201, 37083, 36966, 36849, 36733, 36616, 36501, 36385, 36270, 36155, 36041, 35927, 35813, 35700, 35587, 35475, 35362, 35251, 35139, 35028, 34917, 34807, 34697, 34587, 34478, 34369, 34260, 34151, 34043, 33936, 33828, 33721, 33615, 33509, 33403, 33297, 33192, 33087, 32982, 32878, 32774, 32670, 32567, 32464, 32361, 32259, 32157, 32055, 31954, 31852, 31752, 31651, 31551, 31451, 31352, 31253, 31154, 31055, 30957, 30859, 30762, 30664, 30567, 30471, 30374, 30278, 30183, 30087, 29992, 29897, 29803, 29708, 29614, 29521, 29427, 29334, 29241, 29149, 29057, 28965, 28873, 28782, 28691, 28600, 28510, 28420, 28330, 28240, 28151, 28062, 27973, 27884, 27796, 27708, 27621, 27533, 27446
};

#pragma GCC push_options
#pragma GCC optimize("O3")
inline void strike_init()
{
  TIM1->CCR4 = sin_lut[0];
}
inline void strike(uint8_t strike_vel)
{
  for (int i = 0; i < 100; i++) {
    TIM1->CCR4 = sin_lut[i];
    spin_delay(vel_lut[strike_vel]);
  }
  TIM1->CCR4 = sin_lut[0];
}
#pragma GCC pop_options

static inline void serial_tx(const uint8_t *buf, uint8_t len)
{
  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
  HAL_UART_Transmit(&uart1, &len, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart1, (uint8_t *)buf, len, HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart1, s8, 4, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}

static inline void serial_rx_process_byte(uint8_t c)
{
  static uint32_t last_timestamp = (uint32_t)-100;
  uint32_t t = HAL_GetTick();
  if (t - last_timestamp >= 100) {
    // Reset
    rx_len = 0;
  }
  last_timestamp = t;

  if (rx_len == 0) {
    if (c == 0) {
      // Ignore empty packet
    } else {
      // Receive payload
      rx_len = c;
      rx_ptr = 0;
    }
  } else {
    rx_buf[rx_ptr++] = c;
    if (rx_ptr == (uint32_t)rx_len + 4) {
      // Packet complete! Verify checksum
      uint32_t s = crc32_bulk(rx_buf, (uint32_t)rx_len + 4);
      if (s == 0x2144DF1C) {
        // Process command
        rx_timestamp = HAL_GetTick();
        uint8_t out_len = 0;
        uint8_t out_buf[24];
        if (rx_len >= 2 && rx_buf[0] == 0x01) {
          // Strike
          out_len = 1;
          if (op == 1) {
            // Another strike in progress, ignore
            out_buf[0] = 0xAB;
          } else {
            out_buf[0] = 0xAA;
            op = 1;
            strike_vel = rx_buf[1];
          }
        } else if (rx_len >= 1 && rx_buf[0] == 0x55) {
          out_len = 17;
          out_buf[0] = 0xAA;
          for (int i = 0; i < 16; i++)
            out_buf[i] = *((uint8_t *)UID_BASE + i);
        }

        if (out_len != 0) {
          spin_delay(12000 * 0.57); // 0.5 ms
          serial_tx(out_buf, out_len);
        }
      }
      // Ignore if checksum incorrect
      // Wait for next packet
      rx_len = 0;
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
  uint32_t t = HAL_GetTick();
  bool act_led = (t - rx_timestamp < 1000 ? (t - rx_timestamp < 300) : (t % 4096 < 100));
  GPIOA->BSRR = (GPIO_PIN_6 << (act_led ? 0 : 16));
}

void USART1_IRQHandler()
{
  HAL_UART_IRQHandler(&uart1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *_uart1)
{
  serial_rx_process_byte(rx_byte);
  HAL_UART_Receive_IT(&uart1, &rx_byte, 1);
}
