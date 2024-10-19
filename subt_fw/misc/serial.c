// Sends a packet with the format: <1 byte length> <payload> <CRC-32 little-endian>
// gcc % -Ilibserialport libserialport/.libs/libserialport.a -framework Foundation -framework IOKit

// NOTE: `kIOMainPortDefault` is valid for macOS 12.0+.
// For previous versions, replace it with `kIOMasterPortDefault` in `libserialport/macosx.c` and rebuild.

#include "libserialport.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int check(enum sp_return result);
int main(int argc, char **argv)
{
  struct sp_port *port;
  check(sp_get_port_by_name("/dev/cu.usbmodem141301", &port));
  check(sp_open(port, SP_MODE_READ_WRITE));
  check(sp_set_baudrate(port, 115200));
  check(sp_set_bits(port, 8));
  check(sp_set_parity(port, SP_PARITY_NONE));
  check(sp_set_stopbits(port, 1));
  check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

  const char s[] = "\x00\x00\x00"
                   "\x01" "\x55" "\xf6\x4a\x03\xc9"
                   "\x02" "\x55\x01" "\x78\x8b\x12\xf1";
  int n_tx = check(sp_blocking_write(port, s, 16, 1000));
  uint8_t buf[24];
  int n_rx = check(sp_blocking_read(port, buf, sizeof s, 100));
  for (int i = 0; i < n_rx; i++) printf(" %02x", (int)buf[i]);
  putchar('\n');

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
