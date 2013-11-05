#include "hubo-io-trace.h"
#include <signal.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <ach.h>

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

  ach_create_attr_t attr;

  ach_create_attr_init( &attr );
  
  ach_status_t result = ach_create( IO_TRACE_CHAN_NAME,
                                    5000, sizeof(io_trace_t),
                                    &attr );

  if (result != ACH_OK) {
    fprintf(stderr, "ach_create failed: %s\n", 
            ach_result_to_string(result));
    exit(1);
  }

  printf("created channel %s\n", IO_TRACE_CHAN_NAME);

  ach_channel_t iotrace_chan;

  result = ach_open( &iotrace_chan, IO_TRACE_CHAN_NAME, NULL );

  if (result != ACH_OK) {
    fprintf(stderr, "ach_open failed: %s\n",
            ach_result_to_string(result));
    exit(1);
  }

  printf("opened channel\n");

  signal(SIGINT, interrupt_handler);


  while (!interrupted) {
    
    io_trace_t trace;
    size_t frame_size;

    result = ach_get( &iotrace_chan, &trace, sizeof(trace),
                      &frame_size, NULL, 0 );

    if (result == ACH_OK || result == ACH_MISSED_FRAME) {

      if (!fwrite(&trace, sizeof(io_trace_t), 1, fp)) {
        perror("write log");
      }

    } else if (result != ACH_STALE_FRAMES) {

      fprintf(stderr, "ach_get failed: %s\n",
              ach_result_to_string(result));

      interrupted = 1;

    }

  }

  fclose(fp);
  printf("closed file\n");

  result = ach_close(&iotrace_chan);
  if (result != ACH_OK) {
    fprintf(stderr, "ach_close failed: %s\n",
            ach_result_to_string(result));
    exit(1);
  }

  printf("closed channel\n");

  result = ach_unlink(IO_TRACE_CHAN_NAME);
  if (result != ACH_OK) {
    fprintf(stderr, "ach_unlink failed: %s\n",
            ach_result_to_string(result));
    exit(1);
  }

  printf("unlinked channel\n");

  return 0;

}
