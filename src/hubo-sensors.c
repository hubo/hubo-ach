#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for hubo
#include "hubo.h"
#include "hubo-daemon.h"
#include "hubo-jointparams.h"
#include "hubo-daemonID.h"
#include "hubo/canId.h"

// Check out which CAN API to use
#ifdef HUBO_CONFIG_ESD
#include "hubo/hubo-esdcan.h"
#else
#include "hubo/hubo-socketcan.h"
#endif

// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"


void hGetFT(int board, struct can_frame *f, int can);
void fGetFT(int board, struct can_frame *f);
void getFTAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f);
void fGetAcc(int board, struct can_frame *f);
void hGetAcc(int board, struct can_frame *f);
void getAccAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f);
void fGetIMU(int board, struct can_frame *f);
void hGetIMU(int board, struct can_frame *f);
void getIMUAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f);
void hGetCurrentValue(int jnt, struct hubo_param *h, struct can_frame *f);
void setRefAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f);
void hGotoLimitAndGoOffsetAll(struct hubo_ref *r, struct hubo_param *h, struct hubo_state *s, struct can_frame *f);
void hInitializeBoardAll(struct hubo_param *h, struct hubo_state *s, struct can_frame *f);
void fNullAccFTSensor(int bno, int nullType, struct can_frame *f);
void hNullFTSensor(hubo_d_param_t board, struct can_frame *f);
void hNullAccSensor(hubo_d_param_t board, struct can_frame *f);
void hNullAllFTSensors(struct can_frame *f);
void hNullAllAccSensors(struct can_frame *f);
void fNullIMUSensor( int bno, struct can_frame *f );
void hNullIMUSensor( hubo_d_param_t board, struct can_frame *f );
void hNullAllIMUSensors( struct can_frame *f );
void fInitAccFTSensor( int bno, struct can_frame *f );
void hInitAccFTSensor( hubo_d_param_t board, struct can_frame *f );
void hInitAllAccFTSensors( struct can_frame *f );
void hInitAllSensors( struct can_frame *f );



void hGetFT(int board, struct can_frame *f, int can)
{
	fGetFT(board,f);
	sendCan(hubo_socket[can],f);
	readCan(hubo_socket[can], f, HUBO_CAN_TIMEOUT_DEFAULT);
}

void fGetFT(int board, struct can_frame *f)
{
	f->can_id	= REQ_SENSOR_TXDF;

	__u8 data[2]; // I just realized that in all these frame fillers, this array is never used -__-U
			// TODO: Get rid of all these useless arrays.
	f->data[0]	= (uint8_t)board;
	f->data[1]	= H_GET_FT_SCALED;

	f->can_dlc 	= 2;
}


void getFTAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f)
{
	hGetFT(SBNO_RIGHT_FOOT_FT, f, LOWER_CAN);
	decodeFrame(s, h, f);

	hGetFT(SBNO_LEFT_FOOT_FT, f, LOWER_CAN);
	decodeFrame(s, h, f);

	hGetFT(SBNO_RIGHT_HAND_FT, f, UPPER_CAN);
	decodeFrame(s, h, f);

	hGetFT(SBNO_LEFT_HAND_FT, f, UPPER_CAN);
	decodeFrame(s, h, f);
}

void fGetAcc(int board, struct can_frame *f)
{
	f->can_id	= REQ_SENSOR_TXDF;

	f->data[0]	= (uint8_t)board;
	f->data[1]	= H_GET_ACC_SCALED;

	f->can_dlc	= 2;
}

void hGetAcc(int board, struct can_frame *f)
{
	fGetAcc(board,f);
	sendCan(hubo_socket[LOWER_CAN],f);
	readCan(hubo_socket[LOWER_CAN], f, HUBO_CAN_TIMEOUT_DEFAULT);
}

void getAccAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f)
{
	hGetAcc(SBNO_RIGHT_FOOT_FT, f);
	decodeFrame(s, h, f);

	hGetAcc(SBNO_LEFT_FOOT_FT, f);
	decodeFrame(s, h, f);

	// I have been told that there are no accelerometers in the hands
/*	hGetAcc(SBNO_RIGHT_HAND_FT, f);
	readCan(hubo_socket[LOWER_CAN], f, HUBO_CAN_TIMEOUT_DEFAULT);
	decodeFrame(s, h, f);

	hGetAcc(SBNO_LEFT_HAND_FT, f);
	readCan(hubo_socket[LOWER_CAN], f, HUBO_CAN_TIMEOUT_DEFAULT);
	decodeFrame(s, h, f);
*/
}

void fGetIMU(int board, struct can_frame *f)
{
	f->can_id	= REQ_SENSOR_TXDF;

	f->data[0]	= (uint8_t)board;
	f->data[1]	= 0x00;
	f->data[2]	= 0x01;

	f->can_dlc	= 3;
}

void hGetIMU(int board, struct can_frame *f)
{
	fGetIMU(board,f);
	sendCan(hubo_socket[LOWER_CAN],f);
	readCan(hubo_socket[LOWER_CAN], f, HUBO_CAN_TIMEOUT_DEFAULT);
}

void getIMUAllSlow(struct hubo_state *s, struct hubo_param *h, struct can_frame *f)
{
	hGetIMU(SBNO_IMU_0, f);
	decodeFrame(s, h, f);

	// I have been told that there is only one IMU,
	// so the rest of these are probably worthless.

	hGetIMU(SBNO_IMU_1, f);
	decodeFrame(s, h, f);

	hGetIMU(SBNO_IMU_2, f);
	decodeFrame(s, h, f);
}





