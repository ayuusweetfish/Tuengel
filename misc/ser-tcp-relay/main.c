// clang main.c "-DBUILD_REV=\"$(git log --pretty=format:'%h - %ad' --date=iso -n 1)\"" -Ilibserialport libserialport/.libs/libserialport.a -framework Foundation -framework IOKit

// (cd libserialport-win && ./configure --host=x86_64-w64-mingw32 && make)
// x86_64-w64-mingw32-gcc "-DBUILD_REV=\"$(git log --pretty=format:'%h - %ad' --date=iso -n 1)\"" main.c -lws2_32 -static -Ilibserialport libserialport-win/.libs/libserialport.a -lsetupapi -O2 -o ser-tcp-relay.exe && x86_64-w64-mingw32-strip ser-tcp-relay.exe

#include "libserialport.h"

#if defined(_WIN32) || defined(WIN32)
#define WINDOWS 1
#define _GNU_SOURCE   // asprintf in stdio.h
#endif

#include <ctype.h>    // isspace
#include <errno.h>    // errno
#include <stdbool.h>  // bool
#include <stdint.h>   // uint*_t
#include <stdio.h>    // fprintf
#include <stdlib.h>   // malloc
#include <string.h>   // strerror
#include <unistd.h>   // usleep, close, getopt

#include <pthread.h>
#if WINDOWS
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#endif

static void my_abort();

static bool global_retry = false;
static pthread_mutex_t serial_port_mutex;
static struct sp_port *port;

static int _check(enum sp_return result, const char *file, int line)
{
  char *error_message;
  switch (result) {
  case SP_ERR_ARG:
    fprintf(stderr, "sp error | (%s:%d) Invalid argument.\n", file, line);
    my_abort();
  case SP_ERR_FAIL:
    error_message = sp_last_error_message();
    fprintf(stderr, "sp error | (%s:%d) Failed: %s\n", file, line, error_message);
    sp_free_error_message(error_message);
    my_abort();
  case SP_ERR_SUPP:
    fprintf(stderr, "sp error | (%s:%d) Not supported.\n", file, line);
    my_abort();
  case SP_ERR_MEM:
    fprintf(stderr, "sp error | (%s:%d) Couldn't allocate memory.\n", file, line);
    my_abort();
  case SP_OK:
  default:
    return result;
  }
}
#define check(_r) _check(_r, __FILE__, __LINE__)

#if WINDOWS
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

static bool log_debug = false;

static inline void panic(const char *msg)
{
  int e = err_code();
  fprintf(stderr, "panic | %s: errno = %d (%s)\n", msg, e, err_string(e));
  my_abort();
}
static inline void warn(const char *msg)
{
  int e = err_code();
  fprintf(stderr, "warn  | %s: errno = %d (%s)\n", msg, e, err_string(e));
}
static inline void warn_custom(const char *msg)
{
  fprintf(stderr, "warn  | %s\n", msg);
}
static inline void info(const char *msg)
{
  fprintf(stderr, "info  | %s\n", msg);
}
static inline void debug(const char *msg)
{
  if (log_debug)
    fprintf(stderr, "debug | %s\n", msg);
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

static bool dump_all = false;

static void locked_serial_write_all(const uint8_t *restrict buf, size_t len)
{
  if (dump_all) {
    printf("tx |");
    for (size_t i = 0; i < len; i++) printf(" %02x", (unsigned)buf[i]);
    putchar('\n');
  }
  pthread_mutex_lock(&serial_port_mutex);
  serial_write_all(port, buf, len, 100);
  pthread_mutex_unlock(&serial_port_mutex);
}

static pthread_mutex_t bpm_mutex;
static uint8_t global_bpm = 0;

static const uint8_t mapped_locations[6][2] = {
  {0x01, 0x01},
  {0x01, 0x04},
  {0x01, 0x05},
  {0x00, 0x01},
  {0x00, 0x02},
  {0x00, 0x04},
};
static uint32_t seed = 20241028;
static inline uint32_t my_rand()
{
  seed = seed * 1103515245 + 12345;
  return seed & 0x7fffffff;
}
static void *random_loop(void *_unused)
{
  while (true) {
    pthread_mutex_lock(&bpm_mutex);
    uint8_t bpm = global_bpm;
    pthread_mutex_unlock(&bpm_mutex);
    if (bpm > 0) {
      uint8_t id = my_rand() % (sizeof mapped_locations / sizeof mapped_locations[0]);
      uint8_t o_buf[3] = {0x02, mapped_locations[id][0], mapped_locations[id][1]};
      locked_serial_write_all(o_buf, 3);
      uint32_t delay_us = (30000000 + (int)my_rand() % 60000000) / bpm;
      usleep(delay_us);
    } else {
      usleep(100000); // Sleep 100 ms
    }
  }
}

static void process_and_tx(const uint8_t *restrict buf, size_t len)
{
  // XXX: Subject to change
  for (size_t i = 0; i < len; i++) {
    uint8_t byte = buf[i];
    if (byte >= 0x01 && byte <= 0x06) {
      uint8_t id = byte - 0x01;
      uint8_t o_buf[3] = {0x02, mapped_locations[id][0], mapped_locations[id][1]};
      locked_serial_write_all(o_buf, 3);
    } else if (byte >= 0x80 && byte <= 0xfe) {
      uint8_t bpm = byte - 0x80;
      pthread_mutex_lock(&bpm_mutex);
      global_bpm = bpm;
      pthread_mutex_unlock(&bpm_mutex);
      char msg[64];
      snprintf(msg, sizeof msg, "changed random BPM to %u", (unsigned)bpm);
      debug(msg);
    } else {
      char msg[64];
      snprintf(msg, sizeof msg, "unrecognised byte 0x%02x", (unsigned)byte);
      warn_custom(msg);
    }
  }
}

static pthread_mutex_t serial_buf_mutex;
static uint8_t serial_buf[1024];
static size_t serial_buf_n = 0;

static void *serve_client(void *arg)
{
  int conn_fd = *(int *)arg;
  free(arg);

  while (1) {
    uint8_t buf[1024];
    int poll_result;
#if WINDOWS
    {
      WSAPOLLFD poll_fd = {
        .fd = conn_fd,
        .events = POLLIN,
      };
      poll_result = WSAPoll(&poll_fd, 1, 20);
    }
#else
    {
      struct pollfd poll_fd = {
        .fd = conn_fd,
        .events = POLLIN,
      };
      poll_result = poll(&poll_fd, 1, 20);
    }
#endif
    if (poll_result < 0) {
      warn("poll() failed");
    } else if (poll_result == 0) {
      // Timeout
    } else {
      // POLLIN: data available
      ssize_t n_net_rx = recv(conn_fd, buf, sizeof buf, 0);
      if (n_net_rx == -1) {
        warn("recv() failed");
      } else if (n_net_rx == 0) {
        debug("connection closed");
        break;
      } else {
        // Forward data to serial
        process_and_tx(buf, n_net_rx);
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
    fprintf(stderr, "%d: %s\n", i, sp_get_port_name(port_list[i]));
  fprintf(stderr, "%d serial port(s) total\n", i);
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

static void init_serial(const char *port_name, unsigned baud_rate)
{
  check(sp_get_port_by_name(port_name, &port));
  check(sp_open(port, SP_MODE_READ_WRITE));
  check(sp_set_baudrate(port, baud_rate));
  check(sp_set_bits(port, 8));
  check(sp_set_parity(port, SP_PARITY_NONE));
  check(sp_set_stopbits(port, 1));
  check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));
  check(sp_set_dtr(port, SP_DTR_ON));

  sp_flush(port, SP_BUF_BOTH);

  pthread_mutex_init(&serial_buf_mutex, NULL);

  pthread_t thr;
  if (pthread_create(&thr, NULL, &serial_read, NULL) != 0)
    panic("cannot crate thread");
}

static void print_usage_and_exit(const char *prog_name)
{
  // Permutation of `argv` by `getopt()` is a GNU extension
  fprintf(stderr, "Usage: %s [-b <baud-rate>] [-p <tcp-port>] [-d] [-r] <serial-port>\n\n",
    prog_name ? prog_name : "<prog-name>");
  list_serial_ports();
  exit(1);
}

int main(int argc, char *argv[])
{
#ifdef BUILD_REV
  fprintf(stderr, "[Tuengel] Serial-TCP relay, build " BUILD_REV "\n");
#endif

  const char *serial_port_name = NULL;
  int baud_rate = 115200;
  int tcp_port = 8000;

  if (argc <= 1) {
    FILE *config_file = fopen("ser-tcp-relay.txt", "r");
    if (config_file != NULL) {
      int argv_cap = 8;
      char **config_argv = malloc(sizeof(char *) * argv_cap);
      int config_argc = 0;
      config_argv[config_argc++] = argv[0];
      int c;
      // Skip over leading spaces
      for (c = fgetc(config_file); isspace(c) && c != EOF; c = fgetc(config_file)) { }
      while (c != EOF) {
        int cap = 16;
        char *w = malloc(cap);
        int j;
        for (j = 0; !isspace(c) && c != EOF; j++, c = fgetc(config_file)) {
          if (j >= cap) w = realloc(w, cap <<= 1);
          w[j] = c;
        }
        w[j] = '\0';
        // Append to argument list
        if (config_argc >= argv_cap)
          config_argv = realloc(config_argv, sizeof(char *) * (argv_cap <<= 1));
        config_argv[config_argc++] = w;
        // Skip over trailing spaces
        for (c = fgetc(config_file); isspace(c) && c != EOF; c = fgetc(config_file)) { }
      }
      argc = config_argc;
      argv = config_argv;
      // Print info
      int total_len = 0;
      for (int i = 1; i < argc; i++) total_len += strlen(argv[i]) + 1;
      char *msg = malloc(total_len + 36);
      msg[0] = '\0';
      strcat(msg, "reading arguments from config file:");
      for (int i = 1; i < argc; i++) {
        strcat(msg, " ");
        strcat(msg, argv[i]);
      }
      info(msg);
      free(msg);
    }
  }

  int opt;
  while ((opt = getopt(argc, argv, "b:p:dr")) != -1) {
    switch (opt) {
    case 'b':
      baud_rate = strtol(optarg, NULL, 0);
      break;
    case 'p':
      tcp_port = strtol(optarg, NULL, 0);
      break;
    case 'd':
      log_debug = true;
      dump_all = true;
      break;
    case 'r':
      global_retry = true;
      break;
    default:
      print_usage_and_exit(argv[0]);
      break;
    }
  }
  if (optind >= argc) print_usage_and_exit(argv[0]);

  serial_port_name = argv[optind];

  if (global_retry) {
    warn_custom("global retry has not been implemented > <");
  }

  init_serial(serial_port_name, baud_rate);
  pthread_mutex_init(&serial_port_mutex, NULL);

  char *msg;
  asprintf(&msg, "open serial port %s with baud rate %d", serial_port_name, baud_rate);
  info(msg);
  free(msg);

  pthread_mutex_init(&bpm_mutex, NULL);
  pthread_t random_thr;
  if (pthread_create(&random_thr, NULL, &random_loop, NULL) != 0) {
    panic("cannot create thread for random triggers");
  }

#if WINDOWS
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

  asprintf(&msg, "accepting connections at TCP port %d", tcp_port);
  info(msg);
  free(msg);

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

void my_abort()
{
  exit(1);
}
