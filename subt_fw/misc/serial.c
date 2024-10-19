// Based on https://stackoverflow.com/a/38318768
// Sends a packet with the format: <1 byte length> <payload> <CRC-32 little-endian>

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

int set_interface_attribs(int fd)
{
  struct termios tty;

  if (tcgetattr(fd, &tty) < 0) {
    printf("Error from tcgetattr: %s\n", strerror(errno));
    return -1;
  }

  cfsetospeed(&tty, (speed_t)B115200);
  cfsetispeed(&tty, (speed_t)B115200);

  tty.c_cflag |= (CLOCAL | CREAD);  /* ignore modem controls */
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;       /* 8-bit characters */
  tty.c_cflag &= ~PARENB;   /* no parity bit */
  tty.c_cflag &= ~CSTOPB;   /* only need 1 stop bit */
  tty.c_cflag &= ~CRTSCTS;  /* no hardware flowcontrol */

  /* setup for non-canonical mode */
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  /* fetch bytes as they become available */
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    printf("Error from tcsetattr: %s\n", strerror(errno));
    return -1;
  }
  return 0;
}

static void serial_write(int fd, const void *data, size_t n)
{
  int n_written = write(fd, data, n);
  if (n_written != n) {
    printf("Error from write: %d, %d\n", n_written, errno);
  }
  tcdrain(fd);  /* delay for output */
}

int main()
{
  char *portname = "/dev/cu.usbmodem141301";
  int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf("Error opening %s: %s\n", portname, strerror(errno));
    return -1;
  }
  set_interface_attribs(fd);

  const char s[] = "\x00\x00"
                   "\x01" "\x55" "\xf6\x4a\x03\xc9"
                   "\x02" "\x55\x01" "\x78\x8b\x12\xf1";
  time_t t0 = time(NULL);

  do {
    if (time(NULL) - t0 >= 1) {
      serial_write(fd, s, sizeof(s) - 1);
      printf("[written]\n");
      t0 = time(NULL);
    }
    unsigned char buf[4096];
    int rdlen = read(fd, buf, sizeof(buf) - 1);
    if (rdlen > 0) {
      buf[rdlen] = 0;
      for (int i = 0; i < rdlen; i++) printf(" %02x", (int)buf[i]);
      putchar('\n');
    } else if (rdlen < 0) {
      printf("Error from read: %d: %s\n", rdlen, strerror(errno));
    } else {  /* rdlen == 0 */
      // Timeout
    }         
    /* repeat read to get full message */
  } while (1);
}
