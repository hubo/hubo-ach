#ifndef _HUBO_IO_TRACE_H_
#define _HUBO_IO_TRACE_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct io_trace  {
  int64_t          timestamp;
  int              is_read;
  int              fd;
  int              result_errno;
  int              transmitted;
  struct can_frame frame;
} io_trace_t;

int iotrace_open(int is_server);

void iotrace_write(int trace_socket, const io_trace_t* trace);

int iotrace_read(int trace_socket, io_trace_t* trace);

void iotrace_close(int trace_socket, int do_unlink);

int64_t iotrace_gettime();

#ifdef __cplusplus
}
#endif


#endif
