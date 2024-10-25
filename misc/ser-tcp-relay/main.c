// cc main.c -DLOG_INFO
// x86_64-w64-mingw32-gcc main.c -DLOG_INFO -lws2_32 -static

#include <errno.h>    // errno
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

static void *serve_client(void *arg)
{
  int conn_fd = *(int *)arg;
  free(arg);

  while (1) {
    uint8_t c;
    ssize_t result = recv(conn_fd, &c, 1, 0);
    if (result == -1) {
      warn("recv() failed");
    } else if (result == 0) {
      info("connection closed");
      break;
    } else {
      send_all(conn_fd, &c, 1);
    }
  }

  return NULL;
}

int main()
{
  int port = 8000;

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
  addr.sin_port = htons(port);
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
