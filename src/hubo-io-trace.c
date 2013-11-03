#include "hubo-io-trace.h"

#include <sys/time.h>
#include <time.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>

#define NSEC_PER_SEC 1000000000

#define SOCK_PATH "/tmp/hubo-io-trace"

int iotrace_open(int is_server) {

  int s, s2, t, len;
  struct sockaddr_un addr;
  char str[100];

  // make a datagram (UDP-like) socket
  if ((s = socket(AF_UNIX, SOCK_DGRAM, 0)) == -1) {
    perror("socket");
    exit(1);
  }

  // point the socket at the path
  addr.sun_family = AF_UNIX;
  strcpy(addr.sun_path, SOCK_PATH);
  len = strlen(addr.sun_path) + sizeof(addr.sun_family);

  // server binds, client connects
  if (is_server) {

    unlink(addr.sun_path);

    if (bind(s, (struct sockaddr *)&addr, len) == -1) {
      perror("bind");
      exit(1);
    }

  } else {

    // connect client to server_filename
    if (connect(s, (struct sockaddr *) &addr, len) == -1) {
      perror("connect");
      exit(1);
    }

  }

  return s;

}

void iotrace_write(int trace_socket, const io_trace_t* trace) {

  errno = 0;

  ssize_t n_written = send(trace_socket, trace, sizeof(io_trace_t), 0);

  if (n_written != sizeof(io_trace_t)) {
    fprintf(stderr, "Error writing to trace socket: %s\n", strerror(errno));
  }

}

int iotrace_read(int trace_socket, io_trace_t* trace) {

  errno = 0;

  fd_set set;

  FD_ZERO(&set);
  FD_SET(trace_socket, &set);
  
  struct timeval timeout;

  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  int status = select(trace_socket+1, &set, 0, 0, &timeout);

  if (status <= 0) {
    return 0;
  }

  ssize_t n_read = recv(trace_socket, trace, sizeof(io_trace_t), MSG_DONTWAIT);

  if (n_read != sizeof(io_trace_t)) {
    fprintf(stderr, "Error reading from trace socket: %s\n", strerror(errno));
  }

  return n_read == sizeof(io_trace_t);

}

void iotrace_close(int trace_socket, int do_unlink) {

  close(trace_socket);
  
  if (do_unlink) {
    unlink(SOCK_PATH);
  }

}

int64_t iotrace_gettime() {

  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);

  return (int64_t)t.tv_sec * (int64_t)NSEC_PER_SEC + (int64_t)t.tv_nsec;

}

