// Board is Pi Pico
#include "pico/critical_section.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // memmove

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

static const uint8_t PIN_UPSTRM_DIR = 0;
static const uint8_t PIN_UPSTRM_DATA = 1;
static uint32_t sm_uart_upstrm_tx = 0;
static uint32_t sm_uart_upstrm_rx = 1;

static inline void uart_tx_wait(PIO pio, uint32_t sm)
{
  while (!pio_sm_is_tx_fifo_empty(pio, sm)) { }
  sleep_us(3);  // 1 cycle @ (115200 bps * 8) = 1.085 us
  while (pio_interrupt_get(pio, sm + 0)) { }
}

static inline void uart_dir(PIO pio, uint32_t sm_rx, uint32_t sm_tx, uint32_t de_pin, int tx_mode)
{
  if (tx_mode == 1) {
    pio_sm_set_enabled(pio, sm_rx, false);
    gpio_put(de_pin, 1);
    pio_sm_set_enabled(pio, sm_tx, true);
  } else if (tx_mode == 0) {
    uart_tx_wait(pio, sm_tx);
    pio_sm_set_enabled(pio, sm_tx, false);
    gpio_put(de_pin, 0);
    pio_sm_set_enabled(pio, sm_rx, true);
  } else {
    pio_sm_set_enabled(pio, sm_rx, false);
    uart_tx_wait(pio, sm_tx);
    pio_sm_set_enabled(pio, sm_tx, false);
    gpio_put(de_pin, 0);
  }
}
static inline void upstrm_dir(int tx_mode)
{
  uart_dir(pio0, sm_uart_upstrm_rx, sm_uart_upstrm_tx, PIN_UPSTRM_DIR, tx_mode);
}

// Where does the last command come from?
// (This does not change during normal operation.)
static enum {
  SERIAL_CMD_SRC_USB,     // USB serial
  SERIAL_CMD_SRC_UPSTRM,  // Upstream-facing RS-422 serial
} serial_rx_last_cmd_source = SERIAL_CMD_SRC_USB;

static const uint8_t PORT_USB = 0xfe;
static const uint8_t PORT_UPSTRM = 0xff;
static inline void serial_tx(uint8_t port, const uint8_t *buf, uint8_t len)
{
  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };

  if (port == PORT_USB) {
    stdio_putchar_raw(len);
    stdio_put_string(buf, len, false, false);
    stdio_put_string(s8, 4, false, false);
    stdio_flush();
  } else if (port == PORT_UPSTRM) {
    upstrm_dir(1);
    pio_sm_put_blocking(pio0, sm_uart_upstrm_tx, len);
    for (uint32_t i = 0; i < len; i++)
      pio_sm_put_blocking(pio0, sm_uart_upstrm_tx, buf[i]);
    for (uint32_t i = 0; i < 4; i++)
      pio_sm_put_blocking(pio0, sm_uart_upstrm_tx, s8[i]);
    upstrm_dir(0);
  }
}

static inline void serial_rx_process_cmd(const uint8_t *buf, uint8_t len)
{
  uint8_t serial_cmd_port =
    (serial_rx_last_cmd_source == SERIAL_CMD_SRC_USB ? PORT_USB : PORT_UPSTRM);

  if (buf[0] == 0x55) {
    // Ping
    uint8_t resp[16] = { 0 };
    resp[0] = 0xAA;
    resp[1] = 't';
    resp[2] = 'e';
    resp[3] = 's';
    resp[4] = 't';
    serial_tx(serial_cmd_port, resp, 16);
  }
}

// Take to a separate buffer
// Returns the number of bytes processed
static inline uint32_t serial_rx_take(uint8_t *restrict buf, uint32_t size, uint8_t *restrict out_buf)
{
  uint32_t i = 0;
  while (i < size) {
    uint8_t len = buf[i++];
    if (len == 0) continue; // Stray zeros, do not verify checksum
    if (i + len + 4 > size) { i--; break; } // Insufficient length
    i += len + 4;
  }
  memmove(out_buf, buf, i);
  memmove(buf, buf + i, size - i);
  return i;
}
// Process a block of data
static inline void serial_rx_bulk(const uint8_t *buf, uint32_t size)
{
  uint32_t i = 0;
  while (i < size) {
    uint8_t len = buf[i++];
    if (len == 0) continue; // Stray zeros, do not verify checksum
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
  serial_rx_last_cmd_source = SERIAL_CMD_SRC_USB;

  critical_section_exit(&serial_crits);
}

static void pio0_irq0_handler()
{
  if (!pio_sm_is_rx_fifo_empty(pio0, sm_uart_upstrm_rx)) {
    critical_section_enter_blocking(&serial_crits);

    serial_rx_reset_if_timeout();
    while (!pio_sm_is_rx_fifo_empty(pio0, sm_uart_upstrm_rx)) {
      uint8_t c = pio_sm_get_8(pio0, sm_uart_upstrm_rx);
      serial_rx_push_byte(c);
    }
    serial_rx_last_cmd_source = SERIAL_CMD_SRC_UPSTRM;

    critical_section_exit(&serial_crits);
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

  gpio_init(PIN_UPSTRM_DIR); gpio_set_dir(PIN_UPSTRM_DIR, GPIO_OUT);
  gpio_put(PIN_UPSTRM_DIR, 0);

  irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0_handler);
  irq_set_enabled(PIO0_IRQ_0, true);

  uint32_t uart_tx_program_offset = pio_add_program(pio0, &uart_tx_program);
  uint32_t uart_rx_program_offset = pio_add_program(pio0, &uart_rx_program);

  uart_tx_program_init(pio0, sm_uart_upstrm_tx, uart_tx_program_offset, 115200);
  uart_rx_program_init(pio0, sm_uart_upstrm_rx, uart_rx_program_offset, 115200);
  pio_set_irq0_source_enabled(pio0, PIO_INTR_SM1_RXNEMPTY_LSB, true);

  uart_tx_set_pin(pio0, sm_uart_upstrm_tx, 5);
  uart_rx_set_pin(pio0, sm_uart_upstrm_rx, 6);
  upstrm_dir(0);

  while (1) {
    static bool parity = 0;
    static int count = 0;
    if (++count == 500) {
      count = 0; parity ^= 1;
      upstrm_dir(-1);
      if (parity == 0) {
        uart_tx_set_pin(pio0, sm_uart_upstrm_tx, 5);
        uart_rx_set_pin(pio0, sm_uart_upstrm_rx, 6);
      } else {
        uart_tx_set_pin(pio0, sm_uart_upstrm_tx, 7);
        uart_rx_set_pin(pio0, sm_uart_upstrm_rx, 8);
      }
      upstrm_dir(0);
    }

    gpio_put(ACT_1, (parity | stdio_usb_connected()));
    gpio_put(ACT_2, stdio_usb_connected());
    sleep_ms(2);

    // Process commands serial buffer
    // Move the bytes out to a local buffer to minimise critical section length
    // as well as enable interrupts during processing
    static uint8_t serial_rx_buf_local[sizeof serial_rx_buf];
    critical_section_enter_blocking(&serial_crits);
    uint32_t len = serial_rx_take(serial_rx_buf, serial_rx_ptr, serial_rx_buf_local);
    serial_rx_ptr -= len;
    critical_section_exit(&serial_crits);
    serial_rx_bulk(serial_rx_buf_local, len);
  }
}
