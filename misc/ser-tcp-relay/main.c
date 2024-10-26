// clang main.c -DLOG_INFO -Ilibserialport libserialport/.libs/libserialport.a -framework Foundation -framework IOKit

// (cd libserialport-win && ./configure --host=x86_64-w64-mingw32 && make)
// x86_64-w64-mingw32-gcc main.c -DLOG_INFO -lws2_32 -static -Ilibserialport libserialport-win/.libs/libserialport.a -lsetupapi

#include "libserialport.h"

#include <errno.h>    // errno
#include <poll.h>     // poll
#include <stdbool.h>  // bool
#include <stdint.h>   // uint*_t
#include <stdio.h>    // fprintf
#include <stdlib.h>   // malloc
#include <string.h>   // strerror
#include <unistd.h>   // usleep, close

#include <pthread.h>
#if defined(_WIN32) || defined(WIN32)
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#endif

static struct sp_port *port;

static int _check(enum sp_return result, const char *file, int line)
{
  char *error_message;
  switch (result) {
  case SP_ERR_ARG:
    fprintf(stderr, "sp error | (%s:%d) Invalid argument.\n", file, line);
    abort();
  case SP_ERR_FAIL:
    error_message = sp_last_error_message();
    fprintf(stderr, "sp error | (%s:%d) Failed: %s\n", file, line, error_message);
    sp_free_error_message(error_message);
    abort();
  case SP_ERR_SUPP:
    fprintf(stderr, "sp error | (%s:%d) Not supported.\n", file, line);
    abort();
  case SP_ERR_MEM:
    fprintf(stderr, "sp error | (%s:%d) Couldn't allocate memory.\n", file, line);
    abort();
  case SP_OK:
  default:
    return result;
  }
}
#define check(_r) _check(_r, __FILE__, __LINE__)

#if defined(_WIN32) || defined(WIN32)
static inline int err_code() { return WSAGetLastError(); }
static inline const char *err_string(int err_code) {
  static char msg[256];
  switch (err_code) {
    case WSANOTINITIALISED: return "WSANOTINITIALISED";
    case WSAEADDRINUSE: return "WSAEADDRINUSE";
    case WSAECONNREFUSED: return "WSAECONNREFUSED";
    case WSAETIMEDOUT: return "WSAETIMEDOUT";
    default:
      snprintf(msg, sizeof msg, "code %d", err_code);
      return msg;
  }
  // Unused
  HMODULE hWinsock = LoadLibrary(TEXT("ws2_32.dll"));
  FormatMessage(
    FORMAT_MESSAGE_FROM_SYSTEM |
    FORMAT_MESSAGE_FROM_HMODULE |
    FORMAT_MESSAGE_IGNORE_INSERTS,
    hWinsock,
    err_code,
    MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
    msg,
    sizeof msg,
    NULL);
  return msg;
}
#else
static inline int err_code() { return errno; }
static inline const char *err_string(int err_code) { return strerror(err_code); }
#endif

static inline void panic(const char *msg)
{
  int e = err_code();
  fprintf(stderr, "panic | %s: errno = %d (%s)\n", msg, e, err_string(e));
  exit(1);
}
static inline void warn(const char *msg)
{
  int e = err_code();
#if defined(LOG_WARN) || defined(LOG_INFO)
  fprintf(stderr, "warn  | %s: errno = %d (%s)\n", msg, e, err_string(e));
#endif
}
static inline void info(const char *msg)
{
#ifdef LOG_INFO
  fprintf(stderr, "info  | %s\n", msg);
#endif
}

static size_t send_all(int fd, const void *buf, size_t len)
{
  ssize_t result;
  while (len != 0) {
    result = send(fd, buf, len, 0);
    if (result == -1) {
      if (errno == EAGAIN) {
        usleep(1000);
        continue;
      }
      warn("send() failed");
      return len;
    }
    buf += result;
    len -= result;
  }
  return 0;
}

static void serial_write_all(
  struct sp_port *port,
  const void *restrict buf, size_t len, unsigned timeout)
{
  while (len > 0) {
    int n_ser_tx = check(sp_blocking_write(port, buf, len, timeout));
    len -= n_ser_tx;
  }
}

static pthread_mutex_t serial_buf_mutex;
static uint8_t serial_buf[1024];
static size_t serial_buf_n = 0;

static bool dump_all = true;

static void *serve_client(void *arg)
{
  int conn_fd = *(int *)arg;
  free(arg);

  while (1) {
    uint8_t buf[1024];
    struct pollfd fd = {
      .fd = conn_fd,
      .events = POLLIN,
    };
    int poll_result = poll(&fd, 1, 20);
    if (poll_result == -1) {
      warn("poll() failed");
    } else if (poll_result == 0) {
      // Timeout
    } else {
      // POLLIN: data available
      ssize_t n_net_rx = recv(conn_fd, buf, sizeof buf, 0);
      if (n_net_rx == -1) {
        warn("recv() failed");
      } else if (n_net_rx == 0) {
        info("connection closed");
        break;
      } else {
        // Forward data to serial
        if (dump_all) {
          printf("tx |");
          for (size_t i = 0; i < n_net_rx; i++) printf(" %02x", (unsigned)buf[i]);
          putchar('\n');
        }
        serial_write_all(port, buf, n_net_rx, 100);
      }
    }
    // Check serial incoming buffer
    if (serial_buf_n > 0) {
      uint8_t local_buf[serial_buf_n];
      pthread_mutex_lock(&serial_buf_mutex);
      memcpy(local_buf, serial_buf, serial_buf_n);
      size_t local_buf_n = serial_buf_n;
      serial_buf_n = 0;
      pthread_mutex_unlock(&serial_buf_mutex);
      if (dump_all) {
        printf("rx |");
        for (size_t i = 0; i < local_buf_n; i++) printf(" %02x", (unsigned)local_buf[i]);
        putchar('\n');
      }
      send_all(conn_fd, local_buf, local_buf_n);
    }
  }

  return NULL;
}

static void list_serial_ports()
{
  struct sp_port **port_list;
  check(sp_list_ports(&port_list));
  int i;
  for (i = 0; port_list[i] != NULL; i++)
    printf("%d: %s\n", i, sp_get_port_name(port_list[i]));
  printf("%d serial port(s) total\n", i);
  sp_free_port_list(port_list);
}

static void *serial_read(void *_unused)
{
  struct sp_event_set *evtset;
  sp_new_event_set(&evtset);
  sp_add_port_events(evtset, port, SP_EVENT_RX_READY);
  while (1) {
    sp_wait(evtset, 0);
    pthread_mutex_lock(&serial_buf_mutex);
    bool full = (serial_buf_n >= sizeof serial_buf);
    if (!full) {
      serial_buf_n += sp_nonblocking_read(port,
        serial_buf + serial_buf_n, (sizeof serial_buf) - serial_buf_n);
    }
    pthread_mutex_unlock(&serial_buf_mutex);
    if (full) usleep(10000);
  }
}

static void init_serial()
{
  int baud_rate = 115200;
  check(sp_get_port_by_name("/dev/tty.usbmodem141301", &port));
  check(sp_open(port, SP_MODE_READ_WRITE));
  check(sp_set_baudrate(port, baud_rate));
  check(sp_set_bits(port, 8));
  check(sp_set_parity(port, SP_PARITY_NONE));
  check(sp_set_stopbits(port, 1));
  check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

  sp_flush(port, SP_BUF_BOTH);

  pthread_mutex_init(&serial_buf_mutex, NULL);

  pthread_t thr;
  if (pthread_create(&thr, NULL, &serial_read, NULL) != 0)
    panic("cannot crate thread");
}

int main()
{
  list_serial_ports();
  init_serial();

  int tcp_port = 8000;

#if defined(_WIN32) || defined(WIN32)
  WSADATA _wsadata;
  if (WSAStartup(0x202, &_wsadata) != 0)
    panic("WSAStartup() failed");
#endif

  // Allocate socket
  int sock_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock_fd == -1)
    panic("socket() failed");

  // Allow successive runs without waiting
  if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR,
      (void *)&(int){1}, sizeof(int)) == -1)
    panic("setsockopt() failed");

  // Bind to address and start listening
  struct sockaddr_in addr = { 0 };
  addr.sin_family = AF_INET;
  addr.sin_port = htons(tcp_port);
  addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(sock_fd, (struct sockaddr *)&addr, sizeof addr) == -1)
    panic("bind() failed");
  if (listen(sock_fd, 1024) == -1)
    panic("listen() failed");

  info("accepting connections");

  // Accept connections
  struct sockaddr_in cli_addr;
  socklen_t cli_addr_len;
  while (1) {
    cli_addr_len = sizeof cli_addr;
    int conn_fd = accept(sock_fd, (struct sockaddr *)&cli_addr, &cli_addr_len);
    if (conn_fd == -1)
      panic("accept() failed");

    pthread_t thr;
    int *conn_fd_container = malloc(sizeof(int));
    *conn_fd_container = conn_fd;
    if (pthread_create(&thr, NULL, &serve_client, conn_fd_container) != 0) {
      warn("cannot serve more clients");
      close(conn_fd);
      free(conn_fd_container);
      continue;
    }
    pthread_detach(thr);
  }

  return 0;
}
