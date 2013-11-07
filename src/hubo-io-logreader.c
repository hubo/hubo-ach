#include "hubo-io-trace.h"
#include <stdio.h>
#include <string.h>
#include "hubo/canID.h"
#include "hubo.h"

void decodeFrame(const struct can_frame* f) {

  int fs = f->can_id;
  

  if (fs >= H_SENSOR_FT_BASE_RXDF &&
      fs <= H_SENSOR_FT_MAX_RXDF) {

    printf("FT sensor for board %d\n", fs - H_SENSOR_FT_BASE_RXDF);

  } else if (fs >= H_SENSOR_IMU_BASE_RXDF && 
             fs <= H_SENSOR_IMU_MAX_RXDF) {

    printf("IMU sensor for board %d\n", fs - H_SENSOR_IMU_BASE_RXDF);

  } else if( fs >= H_CURRENT_BASE_RXDF && 
             fs < H_CURRENT_MAX_RXDF ) {

    printf("current sensor for jmc %d\n", fs - H_CURRENT_BASE_RXDF);

  } else if ( fs >= H_ENC_BASE_RXDF && fs < H_ENC_MAX_RXDF ) {

    printf("encoder for jmc %d\n", fs - H_ENC_BASE_RXDF);

  } else if( fs >= H_STAT_BASE_RXDF && fs < H_STAT_MAX_RXDF ) {

    printf("status for jmc %d\n", fs - H_STAT_BASE_RXDF);

  } else if (fs == CMD_TXDF) {

    const char* opcode_str = 0;
    
    switch (f->data[1]) {

    case H_BEEP: opcode_str = "beep"; break;
    case H_HOME: opcode_str = "home"; break;
    case 0x13:   opcode_str = "complimentary mode change"; break;
    case H_SET_POS_GAIN_0: opcode_str = "set pos gain 0"; break;
    case H_SET_POS_GAIN_1: opcode_str = "set pos gain 1"; break;
    case H_SET_CUR_GAIN_0: opcode_str = "set cur gain 0"; break;
    case H_SET_CUR_GAIN_1: opcode_str = "set cur gain 1"; break;
    case H_GET_CURRENT: opcode_str = "get current"; break;
    case H_ALARM: opcode_str = "alarm"; break;
    case H_OPENLOOP_PWM: opcode_str = "open loop pwm"; break;
    case H_SET_CTRL_MODE: opcode_str = "set ctrl mode"; break;
    case H_GET_ENCODER: opcode_str = "get encoder"; break;
    case H_GET_STATUS: opcode_str = "get status"; break;

    case H_SET_DEADZONE+0:
    case H_SET_DEADZONE+1:
    case H_SET_DEADZONE+2:
    case H_SET_DEADZONE+3:
      opcode_str = "set deadzone"; break;

    case H_SET_HOME_PARAM+0:
    case H_SET_HOME_PARAM+1:
    case H_SET_HOME_PARAM+2:
    case H_SET_HOME_PARAM+3:
      opcode_str = "set home param"; break;

    case H_SET_ENC_RES+0:
    case H_SET_ENC_RES+1:
    case H_SET_ENC_RES+2:
    case H_SET_ENC_RES+3:
      opcode_str = "set encoder resolution"; break;

    case H_SET_MAX_ACC_VEL+0:
    case H_SET_MAX_ACC_VEL+1:
    case H_SET_MAX_ACC_VEL+2:
    case H_SET_MAX_ACC_VEL+3:
      opcode_str = "set max acc vel"; break;

    case H_SET_LOW_POS_LIM+0:
    case H_SET_LOW_POS_LIM+1:
    case H_SET_LOW_POS_LIM+2:
    case H_SET_LOW_POS_LIM+3:
      opcode_str = "set lower pos lim"; break;

    case H_SET_UPP_POS_LIM+0:
    case H_SET_UPP_POS_LIM+1:
    case H_SET_UPP_POS_LIM+2:
    case H_SET_UPP_POS_LIM+3:
      opcode_str = "set upper pos lim"; break;

    case H_SET_HOME_VEL_ACC+0:
    case H_SET_HOME_VEL_ACC+1:
    case H_SET_HOME_VEL_ACC+2:
    case H_SET_HOME_VEL_ACC+3:
      opcode_str = "set home vel acc"; break;

    case H_GAIN_OVERRIDE:
      opcode_str = "gain override"; break;

    case H_SET_BOARD_NUM:
      opcode_str = "set board number"; break;

    case H_SET_ERR_BOUND:
      opcode_str = "set err bound"; break;

    case H_SWITCH_DRIVER:
      opcode_str = "switch driver"; break;

    case H_MOTOR_CTRL_ON:
      opcode_str = "enable motor"; break;

    case H_MOTOR_CTRL_OFF:
      opcode_str = "disable motor"; break;

    case H_SET_ENC_ZERO:
      opcode_str = "zero encoder"; break;

    case H_INIT_BOARD:
      opcode_str = "init board"; break;
      
    case H_REQ_NULL:
      opcode_str = "null sensor"; break;

    case H_REQ_PARAMS:
      opcode_str = "req params"; break;

    default:
      opcode_str = "UNKNOWN"; break;

    }
    
    
    printf("cmd for jmc %d with opcode %s (%d = 0x%X)\n", f->data[0], opcode_str,
           (int)(f->data[1]), (int)(f->data[1]));

  } else if (fs == REQ_SENSOR_TXDF) {

    printf("req sensor\n");

  } else if (fs >= REF_BASE_TXDF &&
             fs < REF_BASE_TXDF + HUBO_JMC_COUNT) {

    printf("ref for jmc %d\n", fs - REF_BASE_TXDF);

  } else {

    printf("*** UNKNOWN (%d = 0x%X) ***\n", fs, fs);
    exit(1);
    
  }

  int i;

  printf("id hex: 0x%04X, ", f->can_id);

  printf("data hex: [ ");
  for (i=1; i<=f->can_dlc; ++i) { printf("0x%02X ", f->data[i]); }
  printf("]\n");

  printf("id dec: %4d, ", f->can_id);

  printf("data dec: [ ");
  for (i=1; i<=f->can_dlc; ++i) { printf("%3d ", f->data[i]); }
  printf("]\n");



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
  double last_time = 0;

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
      
      printf("socket %d: %s at time %5.6f (dt=%5.6f), %d transmitted\n",
             trace.fd, label, t, t-last_time, trace.transmitted);

      last_time = t;

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
