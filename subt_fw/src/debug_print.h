#pragma once

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
uint8_t my_buf[256];
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
