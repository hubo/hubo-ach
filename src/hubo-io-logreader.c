#include "hubo-io-trace.h"
#include <stdio.h>
#include <string.h>
#include <hubo/canID.h>

void decodeFrame(const struct can_frame* f) {

  int fs = f->can_id;
  

  if (fs >= H_SENSOR_FT_BASE_RXDF &&
      fs < H_SENSOR_FT_MAX_RXDF) {

    printf("FT sensor\n");

  } else if (fs >= H_SENSOR_IMU_BASE_RXDF && 
             fs < H_SENSOR_IMU_MAX_RXDF) {

    printf("IMU sensor\n");

  } else if( fs >= H_CURRENT_BASE_RXDF && 
             fs < H_CURRENT_MAX_RXDF ) {

    printf("current sensor\n");

  } else if ( fs >= H_ENC_BASE_RXDF && fs < H_ENC_MAX_RXDF ) {

    printf("encoder\n");

  } else if( fs >= H_STAT_BASE_RXDF && fs < H_STAT_MAX_RXDF ) {

    printf("stat??\n");

  } else if (fs == CMD_TXDF) {
    
    printf("cmd\n");

  } else if (fs == REQ_SENSOR_TXDF) {

    printf("req sensor\n");

  } else if (fs >= REF_BASE_TXDF &&
             fs <= REF_BASE_TXDF + HUBO_JMC_COUNT) {

    printf("ref\n");

  } else {

    printf("*** UNKNOWN (%d) ***\n", fs);
    exit(1);
    
  }


}

int main(int argc, char** argv) {
  
  if (argc != 2 && argc != 3) {
    fprintf(stderr, "usage: hubo-io-logreader [-v] LOGFILE\n");
    exit(1);
  }
  
  const char* logfile = 0;
  int verbose = 0;

  if (argc == 2) {
    logfile = argv[1];
  } else {
    if (strcmp(argv[1], "-v") != 0) {
      fprintf(stderr, "bad argument: %s", argv[1]);
      exit(1);
    }
    verbose = 1;
    logfile = argv[2];
  }

  FILE* fp = fopen(logfile, "rb");
  if (!fp) {
    fprintf(stderr, "error opening %s for read\n", 
            logfile);
    exit(0);
  }

  io_trace_t trace;

  int attempted[2] = { 0, 0 };
  int successful[2] = { 0, 0 };

  int64_t first_time = -1;

  while (!feof(fp)) {

    int nread = fread(&trace, sizeof(io_trace_t), 1, fp);

    if (nread == 0) {
      if (feof(fp)) {
        break;
      } else {
        perror("read");
        exit(1);
      }
    }

    // have the trace now
    int which = trace.is_read ? 1 : 0;
    
    ++attempted[which];

    if (trace.transmitted == sizeof(struct can_frame)) {
      ++successful[which];
    }

    if (verbose) {
      
      if (first_time < 0) {
        first_time = trace.timestamp;
      }
      
      const char* label = trace.is_read ? "read " : "write";
      
      double t = (trace.timestamp - first_time)*1e-9;
      
      printf("%s at time %5.3f, %d transmitted\n",
             label, t, trace.transmitted);

      if (trace.transmitted == sizeof(struct can_frame)) {
        decodeFrame(&trace.frame);
      }

      printf("\n");

    }
    
  }

  int i;

  for (i=0; i<2; ++i) {
    const char* label = i ? "read " : "write";
    printf(" attempted %s: %d\n", label, attempted[i]);
    printf("successful %s: %d\n", label, successful[i]);
    printf("\n");
  }

  fclose(fp);

  return 0;

}
