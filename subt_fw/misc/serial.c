// Sends a packet with the format: <1 byte length> <payload> <CRC-32 little-endian>
// gcc % -O2 -Ilibserialport libserialport/.libs/libserialport.a -framework Foundation -framework IOKit

// ./a.out /dev/tty.usbmodem* 115200 55 00 12
// ./a.out /dev/tty.usbmodem* 115200 00 00 80

// NOTE: `kIOMainPortDefault` is valid for macOS 12.0+.
// For previous versions, replace it with `kIOMasterPortDefault` in `libserialport/macosx.c` and rebuild.

#include "libserialport.h"
#include "../../misc/crc32/crc32.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static int check(enum sp_return result);
static void _ensure(bool cond, const char *cond_name, const char *file, int line);
#define ensure(_cond) _ensure((_cond), #_cond, __FILE__, __LINE__)

static struct sp_port *port;

static inline void tx(const uint8_t *buf, uint8_t len)
{
  int n_tx;

  n_tx = check(sp_blocking_write(port, &len, 1, 100));
  ensure(n_tx == 1);

  n_tx = check(sp_blocking_write(port, buf, len, 100));
  ensure(n_tx == len);

  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };
  n_tx = check(sp_blocking_write(port, s8, 4, 100));
  ensure(n_tx == 4);

  printf("[%02x]", (unsigned)len);
  for (int i = 0; i < len; i++) printf(" %02x", (unsigned)buf[i]);
  printf(" |");
  for (int i = 0; i < 4; i++) printf(" %02x", (unsigned)s8[i]);
  putchar('\n');
}

static inline const uint8_t *rx(uint8_t *o_len, unsigned timeout)
{
  int n_rx;
  uint8_t len;

restart:
  n_rx = check(sp_blocking_read(port, &len, 1, timeout));
  ensure(n_rx == 1);
  if (len == 0) {
    printf("(ignoring empty packet)\n");
    goto restart;
  }

  printf("[%02x]\n", len);
  static uint8_t rx_buf[255 + 4];
  n_rx = check(sp_blocking_read(port, rx_buf, len + 4, timeout));
  for (int i = 0; i < n_rx; i++) printf(" %02x", (int)rx_buf[i]); putchar('\n');
  ensure(n_rx == len + 4);

  uint32_t s = crc32_bulk(rx_buf, len + 4);
  if (s == 0x2144DF1C) {
    *o_len = len;
    return rx_buf;
  } else {
    printf("Error: Incorrect checksum\n");
    *o_len = 0;
    return NULL;
  }
}

int main(int argc, char **argv)
{
  if (argc <= 1) {
    printf("Usage: %s <port> [<baud rate> <byte> <byte> ...]\n", argc == 0 ? "serial" : argv[0]);
    return 0;
  }

  int baud_rate = 115200;
  if (argc >= 3) {
    baud_rate = (int)strtol(argv[2], NULL, 10);
  }

  check(sp_get_port_by_name(argv[1], &port));
  check(sp_open(port, SP_MODE_READ_WRITE));
  check(sp_set_baudrate(port, baud_rate));
  check(sp_set_bits(port, 8));
  check(sp_set_parity(port, SP_PARITY_NONE));
  check(sp_set_stopbits(port, 1));
  check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

  sp_flush(port, SP_BUF_BOTH);

  int tx_len = (argc >= 4 ? argc - 3 : 1);
  uint8_t *tx_buf = (uint8_t *)malloc(tx_len);
  if (argc >= 4) {
    if (tx_len > 255) tx_len = 255;
    for (int i = 0; i < tx_len; i++) tx_buf[i] = strtol(argv[3 + i], NULL, 16);
  } else {
    tx_buf[0] = 0x55;
  }

  tx(tx_buf, tx_len);
  uint8_t resp_len;
  const uint8_t *resp = rx(&resp_len, (tx_buf[0] == 0x55 ? 1000 : 100));
  if (resp != NULL) {
    for (int i = 0; i < resp_len; i++) printf(" %02x", (int)resp[i]);
    putchar('\n');
  }

  check(sp_close(port));
  sp_free_port(port);
  return 0;
}

int check(enum sp_return result)
{
  char *error_message;
  switch (result) {
  case SP_ERR_ARG:
    printf("Error: Invalid argument.\n");
    abort();
  case SP_ERR_FAIL:
    error_message = sp_last_error_message();
    printf("Error: Failed: %s\n", error_message);
    sp_free_error_message(error_message);
    abort();
  case SP_ERR_SUPP:
    printf("Error: Not supported.\n");
    abort();
  case SP_ERR_MEM:
    printf("Error: Couldn't allocate memory.\n");
    abort();
  case SP_OK:
  default:
    return result;
  }
}
void _ensure(bool cond, const char *cond_name, const char *file, int line)
{
  if (!cond) {
    printf("Error: (%s:%d) condition \"%s\" does not hold\n", file, line, cond_name);
    abort();
  }
}
