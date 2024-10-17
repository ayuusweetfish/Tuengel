#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#define SERIAL_PRINT 0
#define ACT_1 28
#define ACT_2 29

// ============ Debug output ============

#if SERIAL_PRINT
static inline void my_putc(uint8_t c)
{
  if (c == '\n') uart_putc_raw(uart0, '\r');
  uart_putc_raw(uart0, c);
}
static inline void my_print_init()
{
  uart_init(uart0, 115200);
  gpio_set_function(16, GPIO_FUNC_UART);  // UART0 TX
  gpio_set_function(17, GPIO_FUNC_UART);  // UART0 RX
}
#else
static uint8_t my_buf[256];
static size_t my_buf_ptr = 0;
__attribute__ ((noinline, used))
void my_trap_line()
{
  *(volatile char *)my_buf;
}
static inline void my_putc(uint8_t c)
{
  if (c == '\n') {
    my_buf[my_buf_ptr >= sizeof my_buf ?
      (sizeof my_buf - 1) : my_buf_ptr] = '\0';
    my_trap_line();
    my_buf_ptr = 0;
  } else if (++my_buf_ptr <= sizeof my_buf) {
    my_buf[my_buf_ptr - 1] = c;
  }
}
static inline void my_print_init()
{
}
#endif

int my_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) my_putc(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) my_putc('.');
    my_putc('\n');
  }
  return 0;
}

static void usb_serial_in_cb(__attribute__((unused)) void *_a)
{
}

int main()
{
  // 132 MHz
  // python pico-sdk/src/rp2_common/hardware_clocks/scripts/vcocalc.py 132
  set_sys_clock_pll(1584000000, 6, 2);

  stdio_usb_init();
  stdio_set_translate_crlf(&stdio_usb, false);

  stdio_set_chars_available_callback(usb_serial_in_cb, NULL);

  gpio_init(ACT_1);
  gpio_set_dir(ACT_1, GPIO_OUT);
  gpio_init(ACT_2);
  gpio_set_dir(ACT_2, GPIO_OUT);
  gpio_put(ACT_1, 0);

  my_print_init();
  my_printf("sys clk %u\n", clock_get_hz(clk_sys));

  while (1) {
    static bool parity = 0;
    gpio_put(ACT_1, parity ^= 1);
    gpio_put(ACT_2, stdio_usb_connected());
    printf("run%s", parity ? " " : "\r\n");
    sleep_ms(200);
  }
}
