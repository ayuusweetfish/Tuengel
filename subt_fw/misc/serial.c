// Sends a packet with the format: <1 byte length> <payload> <CRC-32 little-endian>
// gcc % -Ilibserialport libserialport/.libs/libserialport.a -framework Foundation -framework IOKit

// NOTE: `kIOMainPortDefault` is valid for macOS 12.0+.
// For previous versions, replace it with `kIOMasterPortDefault` in `libserialport/macosx.c` and rebuild.

#include "libserialport.h"
#include "../../misc/crc32/crc32.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static int check(enum sp_return result);
static void _ensure(bool cond, const char *cond_name);
#define ensure(_cond) _ensure((_cond), #_cond)

static struct sp_port *port;

static inline void tx(const uint8_t *buf, uint8_t len)
{
  int n_tx;

  n_tx = check(sp_blocking_write(port, &len, 1, 1000));
  ensure(n_tx == 1);

  n_tx = check(sp_blocking_write(port, buf, len, 1000));
  ensure(n_tx == len);

  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };
  n_tx = check(sp_blocking_write(port, s8, 4, 1000));
  ensure(n_tx == 4);
}

static inline const uint8_t *rx(uint8_t *o_len)
{
  int n_rx;
  uint8_t len;

  n_rx = check(sp_blocking_read(port, &len, 1, 1000));
  ensure(n_rx == 1);
  if (len == 0) {
    *o_len = 0;
    return NULL;
  }

  static uint8_t rx_buf[255 + 4];
  n_rx = check(sp_blocking_read(port, rx_buf, len + 4, 1000));
  ensure(n_rx == len + 4);

  uint32_t s = crc32_bulk(rx_buf, len + 4);
  if (s == 0x2144DF1C) {
    *o_len = len;
    return rx_buf;
  } else {
    *o_len = 0;
    return NULL;
  }
}

int main(int argc, char **argv)
{
  check(sp_get_port_by_name("/dev/cu.usbmodem141301", &port));
  check(sp_open(port, SP_MODE_READ_WRITE));
  check(sp_set_baudrate(port, 115200));
  check(sp_set_bits(port, 8));
  check(sp_set_parity(port, SP_PARITY_NONE));
  check(sp_set_stopbits(port, 1));
  check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

  tx((uint8_t *)"\x55", 1);
  uint8_t resp_len;
  const uint8_t *resp = rx(&resp_len);
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
void _ensure(bool cond, const char *cond_name)
{
  if (!cond) {
    printf("Error: condition \"%s\" does not hold\n", cond_name);
    abort();
  }
}
