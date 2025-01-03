// Board is Rev. 2 Subtree
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

#define SERIAL_PRINT 0
#include "debug_print.h"
static const bool SERIAL_DNSTRM_INSPECT = false;

static const bool IS_L0 = true;

#define ACT_1 28
#define ACT_2 29

static inline uint8_t pio_sm_get_8(PIO pio, uint sm)
{
  return *((io_rw_8 *)&pio->rxf[sm] + 3);
}
// Returns 0x8000 on timeout
static inline uint16_t pio_sm_get_8_blocking(PIO pio, uint sm, uint64_t timeout)
{
  while (pio_sm_is_rx_fifo_empty(pio, sm))
    if (time_us_64() - timeout < (1ULL << 63)) return 0x8000;
  return (uint16_t)pio_sm_get_8(pio, sm);
}

static const uint8_t PIN_UPSTRM_DIR = 0;
static const uint8_t PIN_UPSTRM_DATA = 1;
static uint32_t sm_uart_upstrm_tx = 0;
static uint32_t sm_uart_upstrm_rx = 1;

static const uint8_t PIN_DNSTRM_DIR = 2;
static const uint8_t PIN_DNSTRM_DATA_START = 3;
static uint32_t sm_uart_dnstrm_tx = 2;
static uint32_t sm_uart_dnstrm_rx = 3;

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
    pio_sm_clear_fifos(pio, sm_rx);
    pio_sm_set_enabled(pio, sm_tx, true);
  } else if (tx_mode == 0) {
    uart_tx_wait(pio, sm_tx);
    pio_sm_set_enabled(pio, sm_tx, false);
    gpio_put(de_pin, 0);
    pio_sm_clear_fifos(pio, sm_tx);
    pio_sm_set_enabled(pio, sm_rx, true);
  } else {
    pio_sm_set_enabled(pio, sm_rx, false);
    uart_tx_wait(pio, sm_tx);
    pio_sm_set_enabled(pio, sm_tx, false);
    gpio_put(de_pin, 0);
    pio_sm_clear_fifos(pio, sm_tx);
    pio_sm_clear_fifos(pio, sm_rx);
  }
}
static inline void upstrm_dir(int tx_mode)
{
  uart_dir(pio0, sm_uart_upstrm_rx, sm_uart_upstrm_tx, PIN_UPSTRM_DIR, tx_mode);
  if (tx_mode == 1) uart_tx_set_pin(pio0, sm_uart_upstrm_tx, PIN_UPSTRM_DATA);
  if (tx_mode == 0) uart_rx_set_pin(pio0, sm_uart_upstrm_rx, PIN_UPSTRM_DATA);
}
static inline void dnstrm_dir(uint8_t port, int tx_mode)
{
  uart_dir(pio0, sm_uart_dnstrm_rx, sm_uart_dnstrm_tx, PIN_DNSTRM_DIR, tx_mode);
  if (tx_mode == 1) uart_tx_set_pin(pio0, sm_uart_dnstrm_tx, PIN_DNSTRM_DATA_START + port);
  if (tx_mode == 0) uart_rx_set_pin(pio0, sm_uart_dnstrm_rx, PIN_DNSTRM_DATA_START + port);
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
  } else if (port >= 0 && port < 17) {
    dnstrm_dir(port, 1);
    pio_sm_put_blocking(pio0, sm_uart_dnstrm_tx, len);
    for (uint32_t i = 0; i < len; i++)
      pio_sm_put_blocking(pio0, sm_uart_dnstrm_tx, buf[i]);
    for (uint32_t i = 0; i < 4; i++)
      pio_sm_put_blocking(pio0, sm_uart_dnstrm_tx, s8[i]);
    dnstrm_dir(port, 0);
  }
}

static inline bool serial_rx_blocking(uint8_t port, uint64_t timeout, bool (*f)(uint8_t, const uint8_t *))
{
  // Only for receiving response from downstream-facing ports
  if (port >= 18) return false;

  timeout += time_us_64();

  dnstrm_dir(port, 0);  // Change pin

  bool ret;

  while (true) {
    uint16_t len = pio_sm_get_8_blocking(pio0, sm_uart_dnstrm_rx, timeout);
    if (len >= 256) {
      if (SERIAL_DNSTRM_INSPECT)
        my_printf("Timeout at header\n");
      ret = false; break;
    }

    if (len > 0) {
      static uint8_t buf[256 + 4];
      for (int i = 0; i < (int)len + 4; i++) {
        uint16_t c = pio_sm_get_8_blocking(pio0, sm_uart_dnstrm_rx, timeout);
        if (c >= 256) {
          if (SERIAL_DNSTRM_INSPECT) {
            my_printf("Timeout at byte %d/%u\n", i, (unsigned)len);
            for (int j = 0; j < i; j++) my_printf(" %02x", (unsigned)buf[j]); my_printf("\n");
          }
          ret = false; break;
        }
        buf[i] = c;
      }
      uint32_t s = crc32_bulk(buf, (int)len + 4);
      if (s == 0x2144DF1C) {
        if (SERIAL_DNSTRM_INSPECT) {
          my_printf("received");
          for (int i = 0; i < (int)len + 4; i++) my_printf(" %02x", (unsigned)buf[i]); my_printf("\n");
        }
        if (f((uint8_t)len, buf)) { ret = true; break; }
      } else {
        if (SERIAL_DNSTRM_INSPECT) {
          my_printf("CRC error");
          for (int i = 0; i < (int)len + 4; i++) my_printf(" %02x", (unsigned)buf[i]); my_printf("\n");
        }
      }
    }
  }

  dnstrm_dir(port, -1);
  return ret;
}

static inline void serial_rx_process_cmd(const uint8_t *buf, uint8_t len)
{
  uint8_t serial_cmd_port =
    (serial_rx_last_cmd_source == SERIAL_CMD_SRC_USB ? PORT_USB : PORT_UPSTRM);

  uint8_t resp[256];
  uint8_t resp_len = 0;

  if (IS_L0) {
    if (buf[0] == 0x55) {
      // Ping
      resp[0] = 0xAA;
      for (int i = 0; i < 5; i++) {
        uint8_t dnstrm_msg[1] = {0x55};
        serial_tx(i, dnstrm_msg, 1);
        bool check_ack(uint8_t len, const uint8_t *buf) {
          bool valid = (len >= 1 && buf[0] == 0xAA);
          if (valid)
            for (int j = 0; j < 3; j++) resp[1 + i * 3 + j] = buf[1 + j];
          return valid;
        }
        bool result = serial_rx_blocking(i, 150000, check_ack);
        if (!result)
          for (int j = 0; j < 3; j++) resp[1 + i * 3 + j] = 0xEE;
      }
      resp_len = 16;

    } else if (len >= 2 && buf[0] <= 16) { // XXX: Only 17 ports supported for now
      uint8_t addr_l1 = buf[0];
      uint8_t addr_u = buf[1];
      uint8_t velocity = (len >= 3 ? buf[2] : 0x80);
      // Send to downstream port
      uint8_t dnstrm_msg[2] = {addr_u, velocity};
      uint8_t port = addr_l1;
      serial_tx(port, dnstrm_msg, 2);

      // Timeout 20 ms
      uint8_t resp_byte;
      bool check_ack(uint8_t len, const uint8_t *buf) {
        bool valid = len >= 1 && (buf[0] == 0xAA || buf[0] == 0xAB || buf[0] == 0xEE || buf[0] == 0xED);
        if (valid) resp_byte = (buf[0] == 0xEE ? 0xED : buf[0]);
        else resp_byte = 0xEA;  // Received invalid message?
        return valid;
      }
      bool result = serial_rx_blocking(port, 20000, check_ack);

      resp[0] = (result ? resp_byte : 0xEE);
      resp_len = 1;
    }
  } else {  // L1
    if (buf[0] == 0x55) {
      // Ping
      resp[0] = 0xAA;
      for (int i = 0; i < 3; i++) resp[1 + i] = 0;
      for (int i = 0; i < 17; i++) {
        uint8_t dnstrm_msg[1] = {0x55};
        serial_tx(i, dnstrm_msg, 1);
        bool check_ack(uint8_t len, const uint8_t *buf) {
          // XXX: Previous revision of unit firmware incorrectly returns
          // the unique ID in the first 16 bytes. Support that for now.
          return (len >= 1 && buf[0] == 0xAA) ||
            (len >= 17 && (buf[12] == 0x78 || buf[13] == 0x78));
        }
        bool result = serial_rx_blocking(i, 5000, check_ack);
        resp[1 + i / 8] |= ((uint8_t)result << (i % 8));
      }
      resp_len = 4;

    } else if (len == 2 && buf[0] <= 16) { // XXX: Only 17 ports supported for now
      uint8_t addr_u = buf[0];
      uint8_t velocity = buf[1];
      // Send to downstream port
      uint8_t dnstrm_msg[2] = {0x01, velocity};
      uint8_t port = addr_u;
      serial_tx(port, dnstrm_msg, 2);

      // Timeout 20 ms
      uint8_t resp_byte;
      bool check_ack(uint8_t len, const uint8_t *buf) {
        bool valid = len >= 1 && (buf[0] == 0xAA || buf[0] == 0xAB || buf[0] == 0xEE || buf[0] == 0xED);
        if (valid) resp_byte = (buf[0] == 0xEE ? 0xED : buf[0]);
        return valid;
      }
      bool result = serial_rx_blocking(port, 20000, check_ack);

      resp[0] = (result ? resp_byte : 0xEE);
      resp_len = 1;
    } else if (len == 2) {  // XXX: Out-of-range. TODO: Better reporting
      resp[0] = 0xEE;
      resp_len = 1;
    }
  }

  if (resp_len == 0) {
    resp[0] = 0xEF; // Unrecognised message
    resp_len = 1;
  }
  if (resp_len != 0)
    serial_tx(serial_cmd_port, resp, resp_len);
}

// Take to a separate buffer
// Returns the number of bytes processed
static inline uint32_t serial_rx_take(uint8_t *restrict buf, uint32_t size, uint8_t *restrict out_buf, bool is_usb)
{
  uint32_t i = 0;
  uint32_t checksum_len = (is_usb ? 0 : 4);
  while (i < size) {
    uint8_t len = buf[i++];
    if (len == 0) continue; // Stray zeros, do not verify checksum
    if (i + len + checksum_len > size) { i--; break; }  // Insufficient length
    i += len + checksum_len;
  }
  memmove(out_buf, buf, i);
  memmove(buf, buf + i, size - i);
  return i;
}
// Process a block of data
static inline void serial_rx_bulk(const uint8_t *buf, uint32_t size, bool is_usb)
{
  uint32_t i = 0;
  while (i < size) {
    uint8_t len = buf[i++];
    if (len == 0) continue; // Stray zeros, do not verify checksum
    if (is_usb) {
      serial_rx_process_cmd(buf + i, len);
      i += len;
    } else {
      uint32_t s = crc32_bulk(buf + i, (uint32_t)len + 4);
      if (s == 0x2144DF1C) serial_rx_process_cmd(buf + i, len);
      i += len + 4;
    }
  }
}

static critical_section_t serial_crits;
static uint16_t serial_rx_ptr = 0;
static uint8_t serial_rx_buf[1024];
static inline void serial_rx_reset_if_timeout()
{
  static const uint64_t max_delay = 100000; // 100 ms
  static uint64_t last_rx = -max_delay;
  uint64_t t = time_us_64();
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
    assert(c >= 0 && c < 256);
    serial_rx_push_byte((uint8_t)c);
  }
  serial_rx_last_cmd_source = SERIAL_CMD_SRC_USB;

  critical_section_exit(&serial_crits);
}

// PIO0 IRQ0 signals RXNE at upstream-facing port
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
  gpio_init(PIN_DNSTRM_DIR); gpio_set_dir(PIN_DNSTRM_DIR, GPIO_OUT);
  gpio_put(PIN_DNSTRM_DIR, 0);

  irq_set_exclusive_handler(PIO0_IRQ_0, pio0_irq0_handler);
  irq_set_enabled(PIO0_IRQ_0, true);

  uint32_t uart_tx_program_offset = pio_add_program(pio0, &uart_tx_program);
  uint32_t uart_rx_program_offset = pio_add_program(pio0, &uart_rx_program);

  // Upstream-facing port
  uart_tx_program_init(pio0, sm_uart_upstrm_tx, uart_tx_program_offset, 115200);
  uart_rx_program_init(pio0, sm_uart_upstrm_rx, uart_rx_program_offset, 115200);
  // PIO0 IRQ0 signals RXNE at upstream-facing port
  pio_set_irq0_source_enabled(pio0, PIO_INTR_SM1_RXNEMPTY_LSB, true);
  upstrm_dir(0);

  // Downstream-facing port
  uart_tx_program_init(pio0, sm_uart_dnstrm_tx, uart_tx_program_offset, 115200);
  uart_rx_program_init(pio0, sm_uart_dnstrm_rx, uart_rx_program_offset, 115200);
  dnstrm_dir(0, -1);

  while (1) {
    static bool parity = 0;
    static int count = 0;
    if (++count == 500) {
      count = 0; parity ^= 1;
    }

    gpio_put(ACT_1, parity);
    gpio_put(ACT_2, stdio_usb_connected());
    sleep_ms(2);

    // Process commands serial buffer
    // Move the bytes out to a local buffer to minimise critical section length
    // as well as enable interrupts during processing
    static uint8_t serial_rx_buf_local[sizeof serial_rx_buf];
    critical_section_enter_blocking(&serial_crits);
    bool is_usb = (serial_rx_last_cmd_source == SERIAL_CMD_SRC_USB);
    uint32_t len = serial_rx_take(serial_rx_buf, serial_rx_ptr, serial_rx_buf_local, is_usb);
    serial_rx_ptr -= len;
    critical_section_exit(&serial_crits);
    serial_rx_bulk(serial_rx_buf_local, len, is_usb);
  }
}
