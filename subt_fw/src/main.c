#include "pico/critical_section.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "uart.pio.h"

#include "../../misc/crc32/crc32.h"
#include "debug_print.h"

#define SERIAL_PRINT 0
#define ACT_1 28
#define ACT_2 29
#define ACT_1 25

static inline uint8_t pio_sm_get_8(PIO pio, uint sm)
{
  return *((io_rw_8 *)&pio->rxf[sm] + 3);
}

static bool serial_msg_parity = 0;

static inline void serial_tx(const uint8_t *buf, uint8_t len)
{
  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };

  stdio_putchar_raw(len);
  stdio_put_string(buf, len, false, false);
  stdio_put_string(s8, 4, false, false);
  stdio_flush();
}

static inline void serial_rx_process_cmd(const uint8_t *buf, uint8_t len)
{
  if (buf[0] == 0x55) {
    // Ping
    uint8_t resp[16] = { 0 };
    resp[0] = 0xAA;
    resp[1] = 't';
    resp[2] = 'e';
    resp[3] = 's';
    resp[4] = 't';
    serial_tx(resp, 16);
  }
}

static inline void serial_rx_bulk(const uint8_t *buf, uint32_t size)
{
  uint32_t i = 0;
  while (i < size) {
    uint8_t len = buf[i++];
    if (len == 0) continue; // Stray zeros, do not verify checksum
    if (i + len + 4 > size) break;  // Insufficient length
    uint32_t s = crc32_bulk(buf + i, (uint32_t)len + 4);
    if (s == 0x2144DF1C) serial_rx_process_cmd(buf + i, len);
    i += len + 4;
  }
}

static critical_section_t serial_crits;
static uint16_t serial_rx_ptr = 0;
static uint8_t serial_rx_buf[1024];
static inline void serial_rx_reset_if_timeout()
{
  static const uint64_t max_delay = 100000; // 100 ms
  static uint64_t last_rx = -max_delay;
  uint64_t t = to_us_since_boot(get_absolute_time());
  if (t - last_rx >= max_delay) serial_rx_ptr = 0;
  last_rx = t;
}
static inline void serial_rx_push_byte(uint8_t x)
{
  if (serial_rx_ptr >= sizeof serial_rx_buf) {
    // XXX: Print message?
    return;
  }
  serial_rx_buf[serial_rx_ptr++] = x;
}
static void usb_serial_in_cb(__attribute__((unused)) void *_a)
{
  critical_section_enter_blocking(&serial_crits);

  serial_rx_reset_if_timeout();
  while (true) {
    int c = stdio_getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) break;
    assert(c >= 0 && c < 255);
    serial_rx_push_byte((uint8_t)c);
  }

  critical_section_exit(&serial_crits);
}

static uint32_t sm_uart_tx = 0;
static uint32_t sm_uart_rx = 1;

static void pio0_irq0_handler()
{
  if (!pio_sm_is_rx_fifo_empty(pio0, sm_uart_rx)) {
    serial_rx_reset_if_timeout();
    while (!pio_sm_is_rx_fifo_empty(pio0, sm_uart_rx)) {
      uint8_t c = pio_sm_get_8(pio0, sm_uart_rx);
      serial_rx_push_byte(c);
      pio_sm_put_blocking(pio0, sm_uart_tx, c);
    }
  }
}

int main()
{
  // 132 MHz
  // python pico-sdk/src/rp2_common/hardware_clocks/scripts/vcocalc.py 132
  set_sys_clock_pll(1584000000, 6, 2);

  critical_section_init(&serial_crits);

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

  irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0_handler);
  irq_set_enabled(PIO0_IRQ_0, true);

  uint32_t sm_uart_tx_offset = pio_add_program(pio0, &uart_tx_program);
  uart_tx_program_init(pio0, sm_uart_tx, sm_uart_tx_offset, 3, 115200);

  uint32_t sm_uart_rx_offset = pio_add_program(pio0, &uart_rx_program);
  uart_rx_program_init(pio0, sm_uart_rx, sm_uart_rx_offset, 4, 115200);
  pio_set_irq0_source_enabled(pio0, PIO_INTR_SM1_RXNEMPTY_LSB, true);

  while (1) {
    static bool parity = 0;
    static int count = 0;
    if (++count == 100) { count = 0; parity ^= 1; }
    gpio_put(ACT_1, (parity | stdio_usb_connected()) ^ serial_msg_parity);
    // gpio_put(ACT_2, stdio_usb_connected());
    sleep_ms(2);
    critical_section_enter_blocking(&serial_crits);
    serial_rx_bulk(serial_rx_buf, serial_rx_ptr);
    serial_rx_ptr = 0;
    critical_section_exit(&serial_crits);
  }
}
