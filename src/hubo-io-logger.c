#include "hubo-io-trace.h"
#include <signal.h>
#include <stdio.h>


volatile int interrupted = 0;

#define USEC_PER_MSEC 1000

void interrupt_handler(int sig) {
  interrupted = 1;
}


int main(int argc, char** argv) {

  FILE* fp = fopen("/tmp/hubo-io-trace.data", "wb");
  if (!fp) {
    perror("open log");
    exit(1);
  }

  int trace_socket = iotrace_open(1);
  printf("trace_socket = %d\n", trace_socket);
  

  signal(SIGINT, interrupt_handler);


  while (!interrupted) {
    
    io_trace_t trace;
    int read_ok = iotrace_read(trace_socket, &trace);

    if (read_ok) {
      if (!fwrite(&trace, sizeof(io_trace_t), 1, fp)) {
        perror("write log");
      }
    }

  }

  printf("closing!\n");
  iotrace_close(trace_socket, 1);

  fclose(fp);

  return 0;

}
