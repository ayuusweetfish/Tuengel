#include <stm32g0xx_hal.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define RELEASE
#ifndef RELEASE
#define _release_inline
static uint8_t swv_buf[256];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  // ITM_SendChar(c);
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
  char s[256];
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
#define _release_inline inline
#define swv_printf(...)
#endif

TIM_HandleTypeDef tim14;

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
static inline void delay_us(uint32_t us)
{
  spin_delay(us * 4);
}

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // ======== Clocks ========
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc_init.PLL.PLLState = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // 16 MHz
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV4;     // 4 MHz
  clk_init.APB1CLKDivider = RCC_HCLK_DIV4;      // 4 MHz
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_GPIO_Init(GPIOF, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0,
    .Mode = GPIO_MODE_OUTPUT_PP,
  });
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_2,
    .Mode = GPIO_MODE_OUTPUT_PP,
  });
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_1,
    .Alternate = GPIO_AF0_TIM14,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  });

  __HAL_RCC_TIM14_CLK_ENABLE();
  tim14 = (TIM_HandleTypeDef){
    .Instance = TIM14,
    .Init = {
      .Prescaler = 2 - 1,  // 1 us per count (XXX: Why not 4?)
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 20000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim14);
  TIM_OC_InitTypeDef tim14_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  };
  HAL_TIM_PWM_ConfigChannel(&tim14, &tim14_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim14, TIM_CHANNEL_1);

  while (0) {
    GPIOF->BSRR = (1 << 16); GPIOB->BSRR = (1 << 17); delay_us(1000000);
    GPIOF->BSRR = (1 <<  0); GPIOB->BSRR = (1 <<  1); delay_us(1000000);
  }
  GPIOA->BSRR = (1 << 18);

/*
from math import *
N=100
print(', '.join('%d' % round(1500 + 600*(1+sin(i/N*2*pi))) for i in range(N)))
*/
  static const uint16_t sin_lut[100] = {
2100, 2138, 2175, 2212, 2249, 2285, 2321, 2355, 2389, 2421, 2453, 2482, 2511, 2537, 2562, 2585, 2607, 2626, 2643, 2658, 2671, 2681, 2689, 2695, 2699, 2700, 2699, 2695, 2689, 2681, 2671, 2658, 2643, 2626, 2607, 2585, 2562, 2537, 2511, 2482, 2453, 2421, 2389, 2355, 2321, 2285, 2249, 2212, 2175, 2138, 2100, 2062, 2025, 1988, 1951, 1915, 1879, 1845, 1811, 1779, 1747, 1718, 1689, 1663, 1638, 1615, 1593, 1574, 1557, 1542, 1529, 1519, 1511, 1505, 1501, 1500, 1501, 1505, 1511, 1519, 1529, 1542, 1557, 1574, 1593, 1615, 1638, 1663, 1689, 1718, 1747, 1779, 1811, 1845, 1879, 1915, 1951, 1988, 2025, 2062
  };

  int count = 0;
  while (1) {
    GPIOF->BSRR = (1 << ((count < 50) ? 16 : 0));
    int duty;
    // duty = (count < 50 ? 500 : 2500);
    duty = sin_lut[count];
    TIM14->CCR1 = duty;
    HAL_Delay(10);
    if (++count == 100) count = 0;
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
void SVC_Handler() { while (1) { } }
void PendSV_Handler() { while (1) { } }
void WWDG_IRQHandler() { while (1) { } }
void RTC_TAMP_IRQHandler() { while (1) { } }
void FLASH_IRQHandler() { while (1) { } }
void RCC_IRQHandler() { while (1) { } }
void EXTI0_1_IRQHandler() { while (1) { } }
void EXTI2_3_IRQHandler() { while (1) { } }
void EXTI4_15_IRQHandler() { while (1) { } }
void DMA1_Channel1_IRQHandler() { while (1) { } }
void DMA1_Channel2_3_IRQHandler() { while (1) { } }
void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler() { while (1) { } }
void ADC1_IRQHandler() { while (1) { } }
void TIM1_BRK_UP_TRG_COM_IRQHandler() { while (1) { } }
void TIM1_CC_IRQHandler() { while (1) { } }
void TIM3_IRQHandler() { while (1) { } }
void TIM14_IRQHandler() { while (1) { } }
void TIM16_IRQHandler() { while (1) { } }
void TIM17_IRQHandler() { while (1) { } }
void I2C1_IRQHandler() { while (1) { } }
void I2C2_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void SPI2_IRQHandler() { while (1) { } }
void USART1_IRQHandler() { while (1) { } }
void USART2_IRQHandler() { while (1) { } }
