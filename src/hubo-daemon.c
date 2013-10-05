/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
/*
Copyright (c) 2012, Daniel M. Lofaro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may
      be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
#include "hubo/canID.h"
#include "hubo-daemonID.h"
#include "hubo-daemon.h"
#include "hubo-jointparams.h"

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


/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */



// Timing info
#define NSEC_PER_SEC    1000000000

#define slowLoopSplit   5		// slow loop is X times slower then

#define hubo_home_noRef_delay 3.0	// delay before trajectories can be sent while homeing in sec

double HUBO_BOARD_PARAM_CHECK_TIMEOUT = 1000;  // param check timeout in ms

/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
void setEncRef(int jnt, hubo_ref_t *r, hubo_param_t *h);
void setEncRefAll( hubo_ref_t *r, hubo_param_t *h);
void fSetEncRef(int jnt, hubo_state_t *s, hubo_ref_t *r, hubo_param_t *h,
                    hubo_pwm_gains_t *g, struct can_frame *f);
void fResetEncoderToZero(int jnt, hubo_param_t *h, struct can_frame *f);
void fGetCurrentValue(int jnt, hubo_param_t *h, struct can_frame *f);
void hSetBeep(int jnt, hubo_param_t *h, struct can_frame *f, double beepTime);
void fSetBeep(int jnt, hubo_param_t *h, struct can_frame *f, double beepTime);
void fGetBoardStatusAndErrorFlags(int jnt, hubo_param_t *h, struct can_frame *f);
void hGetBoardStatus(int jnt, hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void getBoardStatusAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void fInitializeBoard(int jnt, hubo_param_t *h, struct can_frame *f);
void fEnableMotorDriver(int jnt, hubo_param_t *h, struct can_frame *f);
void fDisableMotorDriver(int jnt, hubo_param_t *h, struct can_frame *f);
void fEnableFeedbackController(int jnt, hubo_param_t *h, struct can_frame *f);
void fDisableFeedbackController(int jnt, hubo_param_t *h, struct can_frame *f);
void fGotoLimitAndGoOffset(int jnt, hubo_param_t *h, struct can_frame *f);
void hInitilizeBoard(int jnt, hubo_ref_t *r, hubo_param_t *h, struct can_frame *f);
void hSetEncRef(int jnt, hubo_state_t *s, hubo_ref_t *r, hubo_param_t *h,
                    hubo_pwm_gains_t *g, struct can_frame *f);
void hSetEncRefAll(hubo_ref_t *r, hubo_param_t *h, hubo_pwm_gains_t *g, struct can_frame *f);
void hIniAll(hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s, struct can_frame *f);
void huboLoop(hubo_param_t *H_param, int vflag);
void hMotorDriverOnOff(int jnt, hubo_param_t *h, struct can_frame *f, hubo_d_param_t onOff);
void hFeedbackControllerOnOff(int jnt, hubo_ref_t *r, hubo_state_t *s, hubo_param_t *h, struct can_frame *f, hubo_d_param_t onOff);
void hResetEncoderToZero(int jnt, hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s, struct can_frame *f);
void huboMessage(hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h,
        hubo_state_t *s, hubo_board_cmd_t *c, struct can_frame *f);
void hGotoLimitAndGoOffset(int jnt, hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h,
    hubo_state_t *s, struct can_frame *f, int send);
int getEncRef(int jnt, hubo_state_t *s, hubo_param_t *h);
int ref2enc( int jnt, double ref, hubo_param_t *h );
void hInitializeBoard(int jnt, hubo_param_t *h, struct can_frame *f);
int decodeFrame(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
double enc2rad(int jnt, int enc, hubo_param_t *h);
double enc2radNkDrc(int jnt, int enc, hubo_param_t *h);
void hGetEncValue(int jnt, uint8_t encChoice, hubo_param_t *h, struct can_frame *f);
void fGetEncValue(int jnt, uint8_t encChoice, hubo_param_t *h, struct can_frame *f);
void getEncAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void getCurrentAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void hgetPowerVals(hubo_param_t *h, struct can_frame *f);
void fgetPowerVals(hubo_param_t *h, struct can_frame *f);
void getPower(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void hGetFT(int board, struct can_frame *f, int can);
void fGetFT(int board, struct can_frame *f);
void getFTAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void fGetAcc(int board, struct can_frame *f);
void hGetAcc(int board, struct can_frame *f);
void getAccAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void fGetIMU(int board, struct can_frame *f);
void hGetIMU(int board, struct can_frame *f);
void getIMUAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void hGetCurrentValue(int jnt, hubo_param_t *h, struct can_frame *f);
void setRefAll(hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s,
                hubo_pwm_gains_t *g, struct can_frame *f);
void hGotoLimitAndGoOffsetAll(hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h,
    hubo_state_t *s, struct can_frame *f);
void hInitializeBoardAll(hubo_param_t *h, hubo_state_t *s, struct can_frame *f);
void fNullAccFTSensor(int bno, int nullType, struct can_frame *f);
void hNullFTSensor(hubo_d_param_t board, hubo_param_t *h, struct can_frame *f);
void hNullAccSensor(hubo_d_param_t board, hubo_param_t *h, struct can_frame *f);
void hNullAllFTSensors(hubo_param_t *h, struct can_frame *f);
void hNullAllAccSensors(hubo_param_t *h, struct can_frame *f);
void fNullIMUSensor( int bno, struct can_frame *f );
void hNullIMUSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f );
void hNullAllIMUSensors( hubo_param_t *h, struct can_frame *f );
void fInitAccFTSensor( int bno, struct can_frame *f );
void hInitAccFTSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f );
void hInitAllAccFTSensors( hubo_param_t *h, struct can_frame *f );
void hInitAllSensors( hubo_param_t *h, struct can_frame *f );
void hNullSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f );
void hNullAllSensors( hubo_param_t *h, struct can_frame *f );
double doubleFromBytePair(uint8_t data0, uint8_t data1);
uint8_t getFingerInt(double n);

void refFilterMode(hubo_ref_t *r, int L, hubo_param_t *h, hubo_state_t *s, hubo_ref_t *f);

/*   ~~~~   Added by M.X. Grey. Auxiliary CAN functions   ~~~~   */
void hSetPosGain(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetPosGain0(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd);
void fSetPosGain1(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd);
void hSetCurGain(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetCurGain0(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd);
void fSetCurGain1(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd);
void hOpenLoopPWM(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fOpenLoopPWM_2CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int duty0, int dir1, int duty1);
void fOpenLoopPWM_3CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int dt0, int dir1, int dt1, int dir2, int dt2);
void fOpenLoopPWM_5CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int dt0, int dir1, int dt1, int dir2, int dt2,
            int dir3, int dt3, int dir4, int dt4);
void hSetControlMode(int jnt, hubo_param_t *h, struct can_frame *f, hubo_d_param_t mode);
void fSetControlMode(int jnt, hubo_param_t *h, struct can_frame *f, int mode);
void hSetAlarm(int jnt, hubo_param_t *h, struct can_frame *f, hubo_d_param_t sound);
void fSetAlarm(int jnt, hubo_param_t *h, struct can_frame *f, int sound);
void hSetDeadZone(int jnt, hubo_param_t *h, struct can_frame *f, int deadzone);
void fSetDeadZone(int jnt, hubo_param_t *h, struct can_frame *f, int deadzone);
void hSetHomeSearchParams( hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f );
void hSetHomeSearchParamsRaw( hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f );
void fSetHomeSearchParams(int jnt, hubo_param_t *h, struct can_frame *f, int limit,
                unsigned int dir, int32_t offset);
void hSetEncoderResolution(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetEncoderResolution(int jnt, hubo_param_t *h, struct can_frame *f, int res);
void hSetMaxAccVel(int jnt, hubo_param_t *h, struct can_frame *f, int maxAcc, int maxVel);
void fSetMaxAccVel(int jnt, hubo_param_t *h, struct can_frame *f, int maxAcc, int maxVel);
void hSetLowerPosLimit(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetLowerPosLimit(int jnt, hubo_param_t *h, struct can_frame *f, int enable, int update, int limit);
void hSetUpperPosLimit(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void hSetUpperPosLimitRaw(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetUpperPosLimit(int jnt, hubo_param_t *h, struct can_frame *f, int enable, int update, int limit);
void hSetHomeAccVel(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f);
void fSetHomeAccVel(int jnt, hubo_param_t *h, struct can_frame *f, float mAcc, int mVelS,
            int mVelP, int mode, int mDuty);
void hSetGainOverride(int jnt, hubo_param_t *h, struct can_frame *f, int gain0, int gain1, double dur);
void fSetGainOverride(int jnt, hubo_param_t *h, struct can_frame *f, int gain0, int gain1, int duration);
void hSetBoardNumber(int jnt, hubo_param_t *h, struct can_frame *f, int boardNum, int rate);
void fSetBoardNumber(int jnt, hubo_param_t *h, struct can_frame *f, int boardNum, int rate);
void fSetJamPwmLimits(int jnt, hubo_param_t *h, struct can_frame *f, int jamLimit, int pwmLimit,
            int lim_detection_duty, int jam_detection_duty );
void hSetErrorBound(int jnt, hubo_param_t *h, struct can_frame *f, int inputDiffErr, int maxError,
            int tempError);
void fSetErrorBound(int jnt, hubo_param_t *h, struct can_frame *f, int inputDiffErr, int maxError,
            int tempError);
void fGetBoardParamA( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamB( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamC( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamD( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamE( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamF( int jnt, int offset, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamG( int jnt, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamH( int jnt, hubo_param_t *h, struct can_frame *f );
void fGetBoardParamI( int jnt, hubo_param_t *h, struct can_frame *f );
void hGetBoardParams( int jnt, hubo_d_param_t param, hubo_param_t *h,
                        hubo_board_param_t *b, struct can_frame *f, int send );
void hGetAllBoardParams( hubo_param_t *h, hubo_state_t *s, struct can_frame *f );
char decodeParamFrame(int jnt, hubo_board_param_t *b, hubo_param_t *h, struct can_frame *f, int type);

void hSetNonComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f);
void fSetNonComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f);
void hSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f);
void fSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f);


/*
void fSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f, int the_mode);
void hSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f, int the_mode);
*/

void clearCanBuff(hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
void getStatusIterate( hubo_state_t *s, hubo_param_t *h, struct can_frame *f);
int isError( int jnt, hubo_state_t *s);
uint8_t isHands(int jnt);
uint8_t getJMC( hubo_param_t *h, int jnt ) { return (uint8_t)h->joint[jnt].jmc; }
uint8_t getCAN( hubo_param_t *h, int jnt ) { return h->joint[jnt].can; }
hubo_can_t getSocket( hubo_param_t *h, int jnt ) { return hubo_socket[h->joint[jnt].can]; }
hubo_can_t sensorSocket( hubo_param_t *h, hubo_sensor_index_t board) {return hubo_socket[h->sensor[board].can];}

uint8_t int_to_bytes(int d, int index);
uint8_t duty_to_byte(int dir, int duty);
unsigned short DrcFingerSignConvention(short h_input,unsigned char h_type);
unsigned long DrcSignConvention(long h_input);


/* Flags */
int verbose;
int debug;
uint8_t HUBO_FLAG_GET_DRC_BOARD_PARAM = ON;  // if ON will check for board params, else will not


// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_ref_neck;      // hubo-ach neck
ach_channel_t chan_hubo_board_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_gains;      // PWM Control gains
ach_channel_t chan_hubo_board_param; // hubo-board-param
ach_channel_t chan_hubo_to_sim;    // hubo-ach-to-sim
ach_channel_t chan_hubo_from_sim;    // hubo-ach-from-sim

//int hubo_ver_can = 0;
/* time for the ref not to be sent while a joint is being moved */
//double hubo_noRefTime[HUBO_JOINT_NUM];
//double hubo_noRefTimeAll = 0.0;
double hubo_noRefTimeAll = HUBO_STARTUP_SEND_REF_DELAY;
int slowLoop  = 0;
int slowLoopi = 0;
int statusJnt = 0;
int statusJntItt = 5;
int statusJnti = 0;
int readBuffi = 3;
int hubo_type = HUBO_ROBOT_TYPE_HUBO_PLUS;

void huboLoop(hubo_param_t *H_param, int vflag) {
    int i = 0;  // iterator
    // get initial values for hubo
    hubo_ref_t H_ref;
    hubo_ref_t H_ref_neck;
    hubo_ref_t H_ref_filter;
    hubo_pwm_gains_t H_gains;
    hubo_board_cmd_t H_cmd;
    hubo_state_t H_state;
    hubo_virtual_t H_virtual;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_ref_neck,   0, sizeof(H_ref_neck));
    memset( &H_ref_filter, 0, sizeof(H_ref_filter) );
    memset( &H_cmd,  0, sizeof(H_cmd));
    memset( &H_state, 0, sizeof(H_state));
    memset( &H_virtual, 0, sizeof(H_virtual));
    memset( &H_gains, 0, sizeof(H_gains) );

    size_t fs;
    int r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != r) {fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}
    hubo_assert( sizeof(H_ref) == fs, __LINE__ );

    r = ach_get( &chan_hubo_ref_neck, &H_ref_neck, sizeof(H_ref_neck), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != r) {fprintf(stderr, "Ref_Neck r = %s\n",ach_result_to_string(r));}
    hubo_assert( sizeof(H_ref_neck) == fs, __LINE__ );

    r = ach_get( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != r) {fprintf(stderr, "CMD r = %s\n",ach_result_to_string(r));}
    hubo_assert( sizeof(H_cmd) == fs, __LINE__ );

//    r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
//    if(ACH_OK != r) {fprintf(stderr, "State r = %s\n",ach_result_to_string(r));}
//    hubo_assert( sizeof(H_state) == fs, __LINE__ );

    // set joint parameters for Hubo
    setJointParams(H_param, &H_state, &H_gains);
    setSensorDefaults(H_param);

    /* Create CAN Frame */
    struct can_frame frame;
//   Is this really how the frame should be initialized?
    sprintf( frame.data, "1234578" );
    frame.can_dlc = strlen( frame.data );


    if(ON == HUBO_FLAG_GET_DRC_BOARD_PARAM) hGetAllBoardParams( H_param, &H_state, &frame );

    ach_put(&chan_hubo_gains, &H_gains, sizeof(H_gains));

    

    /* initilization process */
    
    /* get encoder values */
    getEncAllSlow(&H_state, H_param, &frame);

    /* set encoder values to ref and state */
    for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
        H_ref.ref[i] = H_state.joint[i].pos;
        H_ref_neck.ref[i] = H_ref.ref[i];
        H_ref.mode[i] = HUBO_REF_MODE_REF_FILTER;
        H_ref_filter.ref[i] = H_state.joint[i].pos;
        H_state.joint[i].ref = H_state.joint[i].pos;
/*
        if(true == H_state.joint[i].active) {
            hGetBoardStatus(i, &H_state, &H_param, &frame);
            readCan(getSocket(&H_param,i), &frame, HUBO_CAN_TIMEOUT_DEFAULT*100.0);
            decodeFrame(&H_state, &H_param, &frame);
            if(((int)H_state.status[i].homeFlag) == (int)HUBO_HOME_OK) H_state.joint[i].zeroed = 1;
        }
*/
    }


    




    /* put back on channels */
    ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));
    ach_put(&chan_hubo_ref_neck, &H_ref_neck, sizeof(H_ref_neck));
    ach_put(&chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd));
//    ach_put(&chan_hubo_state, &H_state, sizeof(H_state));


/* period */
//	int interval = 1000000000; // 1hz (1.0 sec)
//	int interval = 500000000; // 2hz (0.5 sec)
//	int interval = 20000000; // 50 hz (0.02 sec)
//	int interval = 10000000; // 100 hz (0.01 sec)
//	int interval = 5000000; // 200 hz (0.005 sec)
//	int interval = 4000000; // 250 hz (0.004 sec)
//	int interval = 2000000; // 500 hz (0.002 sec)

//	double T = (double)interval/(double)NSEC_PER_SEC; // 100 hz (0.01 sec)
	double T = (double)HUBO_LOOP_PERIOD;
        int interval = (int)((double)NSEC_PER_SEC*T);
	printf("T = %1.3f sec\n",T);


    // time info
    struct timespec t, time;
    double tsec;

    // get current time
    //clock_gettime( CLOCK_MONOTONIC,&t);
    clock_gettime( 0,&t);


    int startFlag = 1;

    printf("Start Hubo Loop\n");
    while(!hubo_sig_quit) {

        // wait until next shot
        clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

        fs = 0;

        /* Get latest ACH message */
        if(HUBO_VIRTUAL_MODE_OPENHUBO == vflag) {
            r = ach_get( &chan_hubo_from_sim, &H_virtual, sizeof(H_virtual), &fs, NULL, ACH_O_WAIT );
            if(ACH_OK != r) {
                    if(debug) {
                        fprintf(stderr, "Sim r = %s\n",ach_result_to_string(r));}
                }
            else{    hubo_assert( sizeof(H_virtual) == fs, __LINE__ ); }

            /* get state from sim */
            r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
            if(ACH_OK != r) {
                    if(debug) {
                        fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}
                }
            else{    hubo_assert( sizeof(H_state) == fs, __LINE__ ); }
        }


        r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
        if(ACH_OK != r) {
            if(debug) {
                    fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}
            }
        else{    hubo_assert( sizeof(H_ref) == fs, __LINE__ ); }

	/* Modified for neck - get neck first, then put it on the ref channel struct */
        r = ach_get( &chan_hubo_ref_neck, &H_ref_neck, sizeof(H_ref_neck), &fs, NULL, ACH_O_LAST );  
        if(ACH_OK != r) {
            if(debug) {
                    fprintf(stderr, "Ref Neck r = %s\n",ach_result_to_string(r));}
            }
        else{    hubo_assert( sizeof(H_ref) == fs, __LINE__ ); }

        /* set the ref to the new neck values */
	H_ref.ref[NKY] = H_ref_neck.ref[NKY];
	H_ref.ref[NK1] = H_ref_neck.ref[NK1];
	H_ref.ref[NK2] = H_ref_neck.ref[NK2];



        r = ach_get( &chan_hubo_gains, &H_gains, sizeof(H_gains), &fs, NULL, ACH_O_LAST );
        if( !( ACH_OK==r || ACH_STALE_FRAMES==r || ACH_MISSED_FRAME==r ) )
            fprintf( stderr, "Unexpected ach result in the gains channel: %s\n",
                    ach_result_to_string(r) );
        // FIXME: Add in safety checks here
        

        /* Get all Encoder data */
        getEncAllSlow(&H_state, H_param, &frame); 
        // Note: I moved the encoder reading to be ahead of the filter and send ref.
        // That way the filter is working off of the latest encoder data.
        // Hopefully there aren't unforeseen timing issues with this.
        

        /* Set all Ref */
        if(hubo_noRefTimeAll < T ) {
            refFilterMode(&H_ref, HUBO_REF_FILTER_LENGTH, H_param, &H_state, &H_ref_filter);
            setRefAll(&H_ref, H_param, &H_state, &H_gains, &frame);
            H_state.refWait = 0;
        }
        else{
            getBoardStatusAllSlow( &H_state, H_param, &frame);
            hubo_noRefTimeAll = hubo_noRefTimeAll - T;
            H_state.refWait = 1;
        }

        /* Only on startup */
        // TODO: Investigate a more meaningful way to initialize
        if( startFlag == 1) {
            getBoardStatusAllSlow( &H_state, H_param, &frame);
            for( i = 0; i < HUBO_JOINT_COUNT; i++ ){
                if(H_state.status[i].homeFlag == HUBO_HOME_OK) H_state.joint[i].zeroed = 1;
                if(H_state.status[LWR].homeFlag == HUBO_HOME_OK_WRIST) H_state.joint[LWR].zeroed = 1;
                if(H_state.status[RWR].homeFlag == HUBO_HOME_OK_WRIST) H_state.joint[RWR].zeroed = 1;
            }
            startFlag = 0;
        }

        /* read hubo console */
        huboMessage(&H_ref, &H_ref_filter, H_param, &H_state, &H_cmd, &frame);

        /* Get FT Sensor data */
        getFTAllSlow(&H_state, H_param, &frame);

        /* Get foot acceleration data */
        getAccAllSlow(&H_state, H_param, &frame);

        /* Get IMU data */
        getIMUAllSlow(&H_state, H_param, &frame);

		/* Get Power data */
		getPower(&H_state, H_param, &frame);

		/* Get Power data */
		getPower(&H_state, H_param, &frame);

        /* Update next joint status (one each loop) */
        getStatusIterate( &H_state, H_param, &frame);

        /* Read any aditional data left on the buffer */
//        clearCanBuff(&H_state, H_param, &frame);


        /* Get all Current data */
        getCurrentAllSlow(&H_state, H_param, &frame);

        // Get current timestamp to send out with the state struct


        clock_gettime( CLOCK_MONOTONIC, &time );
        tsec = (double)time.tv_sec;
        tsec += (double)(time.tv_nsec)/1.0e9;

        if(HUBO_VIRTUAL_MODE_OPENHUBO == vflag) {
            /* added time */
//            H_state.time = H_state.time + T; // add time baesd on period
            H_state.time = H_virtual.time; // add time baesd on period
        }
        else {
            H_state.time = tsec; // add time based on system time
        }

        /* put data back in ACH channel */
        ach_put( &chan_hubo_state, &H_state, sizeof(H_state));
        /*if(HUBO_VIRTUAL_MODE_OPENHUBO == vflag) {*/
        ach_put( &chan_hubo_to_sim, &H_virtual, sizeof(H_virtual));
        /*}*/

        t.tv_nsec+=interval;
        tsnorm(&t);
        fflush(stdout);
        fflush(stderr);
    }

}

void clearCanBuff(hubo_state_t *s, hubo_param_t *h, struct can_frame *f) {
    int i = 0;
    for(i = 0; i < readBuffi; i++) {
        readCan(0, f, 0);
        decodeFrame(s, h, f);
        readCan(1, f, 0);
        decodeFrame(s, h, f);
    }
}

void getStatusIterate( hubo_state_t *s, hubo_param_t *h, struct can_frame *f) {

int statusJntItt = 5;
int statusJnti = 0;
    if(statusJnti == 0) {
        if(1 == s->joint[statusJnt].active ) {
            hGetBoardStatus(statusJnt, s, h, f);
        }
        statusJnt = statusJnt + 1;
        if(statusJnt >= HUBO_JOINT_COUNT-40) statusJnt = 0;
    }
    else {
        statusJnti = statusJnti + 1;
        if( statusJnti >= statusJntItt ) statusJnti = 0;
    }
}


static inline void tsnorm(struct timespec *ts){

//	clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
	// calculates the next shot
	while (ts->tv_nsec >= NSEC_PER_SEC) {
		//usleep(100);	// sleep for 100us (1us = 1/1,000,000 sec)
		ts->tv_nsec -= NSEC_PER_SEC;
		ts->tv_sec++;
	}
}



void hSetNonComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f)
{
    fSetNonComplementaryMode(jnt, h, f);
    sendCan(getSocket(h,jnt), f);
}

void fSetNonComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f)
{
    f->can_id       = CMD_TXDF;

    f->data[0]      = getJMC(h,jnt);
    f->data[1]      = 0x13; // TODO: Change to #define in canID.h
    f->data[2]      = 0x01;

    f->can_dlc    = 2;
}

void hSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f)
{
    fSetNonComplementaryMode(jnt, h, f);
    sendCan(getSocket(h,jnt), f);
}

void fSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f)
{
    f->can_id       = CMD_TXDF;

    f->data[0]      = getJMC(h,jnt);
    f->data[1]      = 0x13; // TODO: Change to #define in canID.h
    f->data[2]      = 0x00;

    f->can_dlc    = 2;
}



void refFilterMode(hubo_ref_t *r, int L, hubo_param_t *h, hubo_state_t *s, hubo_ref_t *f) {
    int i = 0;
    double e = 0.0;
    for(i = 0; i < HUBO_JOINT_COUNT; i++) {
        int c = r->mode[i];
      switch (c) {
        case HUBO_REF_MODE_REF: // sets reference directly
          f->ref[i] = r->ref[i];
          break;
        case HUBO_REF_MODE_COMPLIANT:  // complient mode
          f->ref[i] = s->joint[i].pos; 
          break;
        case HUBO_REF_MODE_REF_FILTER: // slow ref to ref no encoder
          f->ref[i] = (f->ref[i] * ((double)L-1.0) + r->ref[i]) / ((double)L);
          break;
        case HUBO_REF_MODE_ENC_FILTER: // sets filter reference encoder feedback
          //f->ref[i] = (f->ref[i] * ((double)L-1.0) + r->ref[i]) / ((double)L);
          e = f->ref[i] - s->joint[i].pos;
          f->ref[i] = (s->joint[i].pos * ((double)L-1.0) + r->ref[i]) / ((double)L);
          
          break;

            default:
                fprintf(stderr, "Unsupported filter mode for joint %s\n", jointNames[i]);
                break;
        }


        // Handle the compliance settings:
        if( s->joint[i].comply!=1 && r->comply[i]==1 && h->joint[i].numMot <= 2 ) 
        { 
            int m0 = h->driver[h->joint[i].jmc].joints[0]; 
            int m1 = h->driver[h->joint[i].jmc].joints[1]; 
            if( s->joint[m0].zeroed == 1 && s->joint[m1].zeroed == 1 ) 
            { 
                fprintf(stdout, "Switching joint %s to compliance mode\n", jointNames[i]); 
                struct can_frame frame; 
                memset(&frame, 0, sizeof(frame)); 
                hSetNonComplementaryMode( i, h, &frame ); 
                s->joint[i].comply = 1; 
            } 
        } 
        else if( s->joint[i].comply==1 && r->comply[i]==0 && h->joint[i].numMot <= 2 ) 
        { // TODO: Handle this transition 
            int allRigid = 0; 
            int k = 0; 
            for( k=0; k<h->joint[i].numMot; k++) 
                if(r->comply[h->driver[h->joint[i].jmc].joints[k]] == 0) 
                    allRigid++; 

            if( allRigid == h->joint[i].numMot ) 
            { 
                for( k=0; k<h->joint[i].numMot; k++) 
                { 
                    int jnt = h->driver[h->joint[i].jmc].joints[k]; 
                    s->joint[jnt].comply = 3; 
                    //s->joint[jnt].ref = s->joint[jnt].pos; 
                    f->ref[jnt] = s->joint[jnt].pos; 
                } 
            } 
        } 
        else if( s->joint[i].comply==2  && 
                 fabs(s->joint[i].ref - r->ref[i]) > HUBO_COMP_RIGID_TRANS_THRESHOLD ) 
        { 
            int m0 = h->driver[h->joint[i].jmc].joints[0]; 
            int m1 = h->driver[h->joint[i].jmc].joints[1]; 
            double F = L*HUBO_COMP_RIGID_TRANS_MULTIPLIER;  
            f->ref[i] = (s->joint[i].ref * ((double)F-1.0) + r->ref[i]) / ((double)F); 
        } 
        else if( s->joint[i].comply==2  && 
                 fabs(s->joint[i].ref - r->ref[i]) <= HUBO_COMP_RIGID_TRANS_THRESHOLD )
        { 
            fprintf(stdout, "Joint %s has returned to rigid mode\n", jointNames[i]); 
            s->joint[i].comply = 0; 
        } 
            
        s->joint[i].ref = f->ref[i]; 
    } 
 
} 




/*
uint32_t getEncRef(int jnt, struct hubo *h)
{
    //return (uint32_t)((double)h->joint[jnt].drive/(double)h->joint[jnt].driven/(double)h->joint[jnt].harmonic/(double)h->joint[jnt].enc*2.0*M_PI);
    return (uint32_t)((double)h->joint[jnt].drive/(double)h->joint[jnt].driven/(double)h->joint[jnt].harmonic/(double)h->joint[jnt].ref*2.0*M_PI);
}
*/

void setRefAll(hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s,
                hubo_pwm_gains_t *g, struct can_frame *f) {
	///> Requests all encoder and records to hubo_state
	int c[HUBO_JMC_COUNT];
	memset( &c, 0, sizeof(c));
	int jmc = 0;
	int i = 0;
	int canChan = 0;

  for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++) {
    for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
        jmc = h->joint[i].jmc;
        if((0 == c[jmc]) & (canChan == h->joint[i].can) & (s->joint[i].active == true))
    {
                    // check to see if already asked that motor controller

        if( slowLoopi < slowLoopSplit ) {
            slowLoop = 1;
            slowLoopi = 0;
        }
        else {
            slowLoop = 0;
            slowLoopi = slowLoopi+1;
        }
        /* ------------------------- */
        /* --- Choose Hubo Type ---- */
        /* ------------------------- */
        if(HUBO_ROBOT_TYPE_DRC_HUBO == hubo_type){
           // if( (i == RF1) | (i == RF2) | (i == RF3) | (i == RF4) | (i == RF5) |
           //     (i == LF1) | (i == LF2) | (i == LF3) | (i == LF4) | (i == LF5) ) { }
           // else {
                hSetEncRef(i, s, r, h, g, f);
                c[jmc] = 1;
           // }
        }
        else if(HUBO_ROBOT_TYPE_HUBO_PLUS == hubo_type) {
            if( (i == RF2) | (i == RF3) | (i == RF4) | (i == RF5) |
                (i == LF2) | (i == LF3) | (i == LF4) | (i == LF5) ) { }
            else {
                hSetEncRef(i, s, r, h, g, f);
                c[jmc] = 1;
            }
        }
      } // If we want to send a command
    } // for every joint
  } // for each CAN channel
}

void getEncAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f)
{
    char c[HUBO_JMC_COUNT];
    memset( &c, 0, sizeof(c));
    //memset( &c, 1, sizeof(c));
    int jmc = 0;
    int i = 0;
//	c[h->joint[REB].jmc] = 0;
    int canChan = 0;
    for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++) {
        for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
            jmc = h->joint[i].jmc;
            if((0 == c[jmc]) & (canChan == h->joint[i].can)){	// check to see if already asked that motor controller
                hGetEncValue(i, 0x00, h, f);
                readCan(hubo_socket[h->joint[i].can], f, HUBO_CAN_TIMEOUT_DEFAULT);
                decodeFrame(s, h, f);
                if(RF1 == i | RF2 == i | RF3 == i | RF4 == i | RF5 == i | 
                   LF1 == i | LF2 == i | LF3 == i | LF4 == i | LF5 == i) { 	
                        hGetEncValue(i, 0x01, h, f);
                        readCan(hubo_socket[h->joint[i].can], f, HUBO_CAN_TIMEOUT_DEFAULT);
                        decodeFrame(s, h, f);
                }
                c[jmc] = 1;
            }
        }
    }
}


void hgetPowerVals(hubo_param_t *h, struct can_frame *f){
    fgetPowerVals(h, f);
    sendCan(hubo_socket[UPPER_CAN], f);
}

void fgetPowerVals(hubo_param_t *h, struct can_frame *f){
	f->can_id = CMD_TXDF;
	//f->can_id = REF_BASE_TXDF + 14;
	f->data[0] = JMC14;
	f->data[1] = H_VCREAD;
	f->can_dlc = 2;
}

void getPower(hubo_state_t *s, hubo_param_t *h, struct can_frame *f){
	hgetPowerVals(h, f);
	readCan(hubo_socket[UPPER_CAN], f, HUBO_CAN_TIMEOUT_DEFAULT);
	decodeFrame(s, h, f);	
}






void getBoardStatusAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f)
{
    ///> Requests all encoder and records to hubo_state
    char c[HUBO_JMC_COUNT];
    memset( &c, 0, sizeof(c) );
    int jmc = 0;
    int i = 0;
    int canChan = 0;
    for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++)
    {
        for( i = 0; i < HUBO_JOINT_COUNT; i++ )
        {
            jmc = h->joint[i].jmc;
            if((0 == c[jmc]) && (canChan == h->joint[i].can)) 
            {
                hGetBoardStatus(i, s, h, f);
                readCan(getSocket(h,i), f, HUBO_CAN_TIMEOUT_DEFAULT);
                decodeFrame(s, h, f);
                c[jmc] = 1;
            }
        }
    }
}

void hGetFT(int board, struct can_frame *f, int can)
{
    fGetFT(board,f);
    sendCan(hubo_socket[can],f);
    readCan(hubo_socket[can], f, HUBO_CAN_TIMEOUT_DEFAULT);
}

void fGetFT(int board, struct can_frame *f)
{
    f->can_id    = REQ_SENSOR_TXDF;
    
    f->data[0]    = (uint8_t)board;
    f->data[1]    = H_GET_FT_SCALED;

    f->can_dlc     = 2;
}


void getFTAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f)
{
    hGetFT(h->sensor[HUBO_FT_R_FOOT].boardNo, f, h->sensor[HUBO_FT_R_FOOT].can);
    decodeFrame(s, h, f);

    hGetFT(h->sensor[HUBO_FT_L_FOOT].boardNo, f, h->sensor[HUBO_FT_L_FOOT].can);
    decodeFrame(s, h, f);

    hGetFT(h->sensor[HUBO_FT_R_HAND].boardNo, f, h->sensor[HUBO_FT_R_HAND].can);
    decodeFrame(s, h, f);

    hGetFT(h->sensor[HUBO_FT_L_HAND].boardNo, f, h->sensor[HUBO_FT_L_HAND].can);
    decodeFrame(s, h, f);
}

void fGetAcc(int board, struct can_frame *f)
{
    f->can_id    = REQ_SENSOR_TXDF;

    f->data[0]    = (uint8_t)board;
    f->data[1]    = H_GET_ACC_SCALED;

    f->can_dlc    = 2;
}

void hGetAcc(int board, struct can_frame *f)
{
    fGetAcc(board,f);
    sendCan(hubo_socket[LOWER_CAN],f);
    readCan(hubo_socket[LOWER_CAN], f, HUBO_CAN_TIMEOUT_DEFAULT);
}

void getAccAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f)
{
    hGetAcc(h->sensor[HUBO_FT_R_FOOT].boardNo, f);
    decodeFrame(s, h, f);

    hGetAcc(h->sensor[HUBO_FT_L_FOOT].boardNo, f);
    decodeFrame(s, h, f);
}

void fGetIMU(int board, struct can_frame *f)
{
    f->can_id    = REQ_SENSOR_TXDF;

    f->data[0]    = (uint8_t)board;
    f->data[1]    = 0x00;
    f->data[2]    = 0x01;

    f->can_dlc    = 3;
}

void hGetIMU(int board, struct can_frame *f)
{
    fGetIMU(board,f);
    sendCan(hubo_socket[LOWER_CAN],f);
    readCan(hubo_socket[LOWER_CAN], f, HUBO_CAN_TIMEOUT_DEFAULT);
}

void getIMUAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f)
{
    hGetIMU(h->sensor[HUBO_IMU0].boardNo, f);
    decodeFrame(s, h, f);

    // I have been told that there is only one IMU,
    // so the rest of these are probably worthless.
/*
    hGetIMU(h->sensor[HUBO_IMU1].boardNo, f);
    decodeFrame(s, h, f);

    hGetIMU(h->sensor[HUBO_IMU2].boardNo, f);
    decodeFrame(s, h, f);
*/
}

void getCurrentAllSlow(hubo_state_t *s, hubo_param_t *h, struct can_frame *f) {
    ///> Requests all motor currents and records to hubo_state
    char c[HUBO_JMC_COUNT];
    memset( &c, 0, sizeof(c));
    //memset( &c, 1, sizeof(c));
    int jmc = 0;
    int i = 0;
//    c[h->joint[REB].jmc] = 0;
    int canChan = 0;
    for( canChan = 0; canChan < HUBO_CAN_CHAN_NUM; canChan++)
    {
        for( i = 0; i < HUBO_JOINT_COUNT; i++ )
        {
            jmc = h->joint[i].jmc;
            if(0 == c[jmc])    // check to see if already asked that motor controller
            {
                hGetCurrentValue(i, h, f);
                readCan(getSocket(h,i), f, HUBO_CAN_TIMEOUT_DEFAULT);
                decodeFrame(s, h, f);
                c[jmc] = 1;
            }
        }
    }    

}

int getEncRef(int jnt, hubo_state_t *s , hubo_param_t *h) {
    // set encoder from reference
    return ref2enc( jnt, s->joint[jnt].ref, h );
}

int ref2enc( int jnt, double ref, hubo_param_t *h )
{
    hubo_joint_param_t *p = &h->joint[jnt];
    return (int32_t)((double)p->driven/(double)p->drive*(double)p->harmonic*
                (double)p->enc*(double)ref/2.0/M_PI);
}

unsigned long signConvention(long _input) {
    if (_input < 0) return (unsigned long)( ((-_input)&0x007FFFFF) | (1<<23) );
    else return (unsigned long)_input;
}

void fSetEncRef(int jnt, hubo_state_t *s, hubo_ref_t *r, hubo_param_t *h,
                    hubo_pwm_gains_t *g, struct can_frame *f)
{
    memset(f, 0, sizeof(*f));
    // set ref
    f->can_id     = REF_BASE_TXDF + h->joint[jnt].jmc;  //CMD_TXD;F// Set ID
    uint16_t jmc = h->joint[jnt].jmc;
    if(HUBO_ROBOT_TYPE_HUBO_PLUS == hubo_type){
      if(h->joint[jnt].numMot <= 2) {
        
        int m0 = h->driver[jmc].joints[0];
        int m1;
        
        unsigned long pos0 = signConvention((int)getEncRef(m0, s, h));
        f->data[0] =     int_to_bytes(pos0,1);
        f->data[1] =     int_to_bytes(pos0,2);
        f->data[2] =     int_to_bytes(pos0,3);

        if(h->joint[jnt].numMot == 1)
            m1 = m0; // If there is not a second joint, use the ref of the first
        else
            m1 = h->driver[jmc].joints[1];

        unsigned long pos1 = signConvention((int)getEncRef(m1, s, h));

        f->data[3] =     int_to_bytes(pos1,1);
        f->data[4] =     int_to_bytes(pos1,2);
        f->data[5] =     int_to_bytes(pos1,3);

        f->can_dlc = 6; //= strlen( data );    // Set DLC


      }
      else if(h->joint[jnt].numMot == 5) { // Fingers
        int fing[5];
	if(jnt == RF1) {
            fing[0] = RF1;
            fing[1] = RF2;
            fing[2] = RF3;
            fing[3] = RF4;
            fing[4] = RF5;
        }
        else if(jnt == LF1) {
            fing[0] = LF1;
            fing[1] = LF2;
            fing[2] = LF3;
            fing[3] = LF4;
            fing[4] = LF5;
        }

        f->can_id = 0x01;
        f->data[0] = (uint8_t)h->joint[jnt].jmc;
        f->data[1] = (uint8_t)0x0D;
        f->data[2] = (uint8_t)0x01;
        f->data[3] = getFingerInt(s->joint[fing[0]].ref);
        f->data[4] = getFingerInt(s->joint[fing[1]].ref);
        f->data[5] = getFingerInt(s->joint[fing[2]].ref);
        f->data[6] = getFingerInt(s->joint[fing[3]].ref);
        f->data[7] = getFingerInt(s->joint[fing[4]].ref);

        f->can_dlc = 8;
      }
    }
    else if(HUBO_ROBOT_TYPE_DRC_HUBO == hubo_type){
      if ( (NKY == jnt) | (NK1 == jnt) | (NK2 == jnt) ){
         int16_t jntTmp = (int16_t)( (s->joint[NKY].ref+M_PI)/(2*M_PI)*4095 );
         f->data[0] =  jntTmp      & 0x00FF;
         f->data[1] = (jntTmp>>8)  & 0x00FF;
         jntTmp = (int16_t)( (s->joint[NK1].ref+M_PI)/(2*M_PI)*4095 );
         f->data[2] =  jntTmp      & 0x00FF;
         f->data[3] = (jntTmp>>8)  & 0x00FF;
         jntTmp = (int16_t)( (s->joint[NK2].ref+M_PI)/(2*M_PI)*4095 );
         f->data[4] =  jntTmp      & 0x00FF;
         f->data[5] = (jntTmp>>8)  & 0x00FF;
         f->can_dlc = 6;
      }
      else if((RWR == jnt) | (LWR == jnt) | (RF1 == jnt) | (RF2 == jnt) | (LF1 == jnt) | (LF2 == jnt)) { // Fingers and wrist 2 (W2)


         int jntF1 = RF1;
         int jntF2 = RF2;
         int jntW = RWR;
         if(jnt == RWR){
            jntF1 = RF1;
            jntF2 = RF2;
            jntW  = RWR;
         }
         else if (jnt == LWR){
            jntF1 = LF1;
            jntF2 = LF2;
            jntW  = LWR;
         }

         double jntF1Val = s->joint[jntF1].ref * (double)h->joint[jntF1].dir;
         double jntF2Val = s->joint[jntF2].ref * (double)h->joint[jntF2].dir;

         unsigned long pos0 = signConvention((int)getEncRef(jntW, s, h));
         f->data[0] =     int_to_bytes(pos0,1);
         f->data[1] =     int_to_bytes(pos0,2);
         f->data[2] =     int_to_bytes(pos0,3);
         
         short jntFinVal = (short)(jntF1Val/0.01);  // convert from amps to units
         if(jntFinVal > HUBO_FINGER_SAT_VALUE)
            jntFinVal = HUBO_FINGER_SAT_VALUE;
         else if(jntFinVal < -HUBO_FINGER_SAT_VALUE)
            jntFinVal = -HUBO_FINGER_SAT_VALUE;

         f->data[3] = DrcFingerSignConvention((short)(jntFinVal), HUBO_FINGER_CURRENT_CTRL_MODE);
         
         jntFinVal = (short)(jntF2Val/0.01);  // convert from amps to units
         if(jntFinVal > HUBO_FINGER_SAT_VALUE)
            jntFinVal = HUBO_FINGER_SAT_VALUE;
         else if(jntFinVal < -HUBO_FINGER_SAT_VALUE)
            jntFinVal = -HUBO_FINGER_SAT_VALUE;

         f->data[4] = DrcFingerSignConvention((short)(jntFinVal), HUBO_FINGER_CURRENT_CTRL_MODE);
      
         f->can_dlc = 5;
         
      }
        else if((h->joint[jnt].numMot <= 2) )
        {
            int m0 = h->driver[jmc].joints[0];
            int m1;

            if(h->joint[jnt].numMot == 1)
                m1 = m0; // If there is not a second joint, use the ref of the first
            else
                m1 = h->driver[jmc].joints[1];

            if( s->joint[m0].comply != 1 && s->joint[m1].comply != 1 ) 
            { 

                unsigned long pos0 = signConvention((int)getEncRef(m0, s, h)); 
                f->data[0] =     int_to_bytes(pos0,1); 
                f->data[1] =     int_to_bytes(pos0,2); 
                f->data[2] =     int_to_bytes(pos0,3); 


                unsigned long pos1 = signConvention((int)getEncRef(m1, s, h)); 

                f->data[3] =     int_to_bytes(pos1,1); 
                f->data[4] =     int_to_bytes(pos1,2); 
                f->data[5] =     int_to_bytes(pos1,3); 

                f->can_dlc = 6; //= strlen( data );    // Set DLC 
            } 
            else if( s->joint[m0].comply == 1 || s->joint[m1].comply == 1 ) 
            {   // TODO: Make this section much less sloppy 

                if(abs(g->joint[m0].maxPWM) > 100) 
                    g->joint[m0].maxPWM = 100; 
                int pwmLimit = 10*abs(g->joint[m0].maxPWM); 

                // Multiplying by 10 makes the gains equal:
                // Kp -- duty% per radian
                // Kd -- duty% reduction per radian/sec
                // pwmCommand -- duty%
                int kP_err = 10*g->joint[m0].Kp*(s->joint[m0].ref - s->joint[m0].pos);
                int kD_err = 10*g->joint[m0].Kd*s->joint[m0].vel;

                int duty0 = -(kP_err - kD_err + g->joint[m0].pwmCommand*10);

                int dir0;
                if(duty0 >= 0)
                    dir0 = 0x00; // Counter-Clockwise
                else
                    dir0 = 0x01; // Clockwise
                duty0 = abs(duty0);

                if( duty0 > abs(pwmLimit) )
                    duty0 = abs(pwmLimit);
/*                
                fprintf(stderr, "%s Duty:%f Dir:%d ref:%f pos:%f Kp:%f grav:%f limit:%d | ",
                                jointNames[m0], (double)(duty0)/10.0, dir0, s->joint[m0].ref,
                                    s->joint[m0].pos, g->joint[m0].Kp, g->joint[m0].pwmCommand, g->joint[m0].maxPWM);
*/
                // Multiplying by 10 makes the gains equal:
                // Kp -- duty% per radian
                // Kd -- duty% reduction per radian/sec
                // pwmCommand -- duty%


                if(abs(g->joint[m1].maxPWM) > 100)
                    g->joint[m1].maxPWM = 100;
                pwmLimit = 10*abs(g->joint[m1].maxPWM);

                kP_err = 10*g->joint[m1].Kp*(s->joint[m1].ref - s->joint[m1].pos);
                kD_err = 10*g->joint[m1].Kd*s->joint[m1].vel;

                int duty1 = -(kP_err - kD_err + g->joint[m1].pwmCommand*10);
                // Multiplying by 10 makes the gains equal:
                // Kp -- duty% per radian
                // Kd -- duty% reduction per radian/sec

                int dir1;
                if(duty1 >= 0)
                    dir1 = 0x00; // Counter-Clockwise
                else
                    dir1 = 0x01; // Clockwise
                duty1 = abs(duty1);

                if( duty1 > abs(pwmLimit) )
                    duty1 = abs(pwmLimit);
                
                f->can_id = 0x01;
                f->data[0] = getJMC(h,m0);
                f->data[1] = 0x0D; // TODO: Put this in the header
                f->data[2] = 0x03;

                f->data[3] = (( duty0 & 0x0F00 )>>4) | dir0;
                f->data[4] =  duty0 & 0x00FF;

                f->data[5] = (( duty1 & 0x0F00 )>>4) | dir1;
                f->data[6] =  duty1 & 0x00FF;

                f->can_dlc = 7;
/*
                fprintf(stderr, "%s Duty:%f Dir:%d ref:%f pos:%f Kp:%f grav:%f limit:%d\n",
                                jointNames[m1], (double)(duty1)/10.0, dir1, s->joint[m1].ref,
                                    s->joint[m1].pos, g->joint[m1].Kp, g->joint[m1].pwmCommand, g->joint[m1].maxPWM);
*/
            }
            
            if( s->joint[m0].comply == 3 && s->joint[m1].comply == 3 ) 
            { 
                struct can_frame frame; 
                memset(&frame, 0, sizeof(frame)); 
                fEnableFeedbackController(m0, h, &frame); 
                sendCan(getSocket(h,m0), &frame); 

                s->joint[m0].comply = 2; 
                s->joint[m1].comply = 2; 
            } 
        } 
      
    } 
} 

unsigned long DrcSignConvention(long h_input)
{
	if (h_input < 0) return (unsigned long)( ((-h_input)&0x007FFFFF) | (1<<23) );
	 else return (unsigned long)h_input;
			
}

unsigned short DrcFingerSignConvention(short h_input,unsigned char h_type)
{
	if(h_type == 0x00) // Position
	{
		if (h_input < 0) return ((h_input)&0x000000FF);
		else return (unsigned short)h_input;
	}
	else if(h_type == 0x01) // Current
	{
		if (h_input < 0) return (unsigned short)( ((-h_input)&0x0000007F) | (1<<7) );
		else return (unsigned short)h_input;
	}
	else return 0x00;
}

uint8_t getFingerInt(double n){
///> takes a values between -1 and 1 and returns the proper can unsigned value to go into the can packet

	uint8_t t = 0;
	if( n < -1) { n = -1.0; }
	if( n >  1) { n =  1.0; }
	int N = (int)(n*100.0);		// scale
	N = abs(N);			// absolute value
	//if( N > 100 ){ N = 100; }	// saturation
	if( N > 100 ){ N = 100; }	// saturation

	t = ((uint8_t)N) & 0x7F;			// convert to uint8
	if(n < 0){ t = t | 0x80; }

	return t;
}

void hSetPosGain(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    int jnt = c->joint;
    if(h->joint[jnt].motNo == 0 || h->joint[jnt].numMot > 2)
        fSetPosGain0(jnt, h, f, c->iValues[0], c->iValues[1], c->iValues[2]);
    else if(h->joint[jnt].motNo == 1)
        fSetPosGain1(jnt, h, f, c->iValues[0], c->iValues[1], c->iValues[2]);
    
    sendCan(getSocket(h,jnt), f);
    fprintf(stdout, "Joint number %d has changed position gain values to:\n\tKp:%d\tKi:%d\tKd:%d\n",
            jnt, c->iValues[0], c->iValues[1], c->iValues[2] );
}

void fSetPosGain0(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_SET_POS_GAIN_0;
    f->data[2]    = int_to_bytes(Kp,1);
    f->data[3]    = int_to_bytes(Kp,2);
    f->data[4]    = int_to_bytes(Ki,1);
    f->data[5]    = int_to_bytes(Ki,2);
    f->data[6]    = int_to_bytes(Kd,1);
    f->data[7]    = int_to_bytes(Kd,2);

    f->can_dlc     = 8;
}
void fSetPosGain1(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd){

    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_SET_POS_GAIN_1;
    f->data[2]    = int_to_bytes(Kp,1);
    f->data[3]    = int_to_bytes(Kp,2);
    f->data[4]    = int_to_bytes(Ki,1);
    f->data[5]    = int_to_bytes(Ki,2);
    f->data[6]    = int_to_bytes(Kd,1);
    f->data[7]    = int_to_bytes(Kd,2);

    f->can_dlc    = 8;
}

void hSetCurGain(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    int jnt = c->joint;
    if(h->joint[jnt].motNo == 0 || h->joint[jnt].numMot > 2)
        fSetCurGain0(jnt, h, f, c->iValues[0], c->iValues[1], c->iValues[2]);
    else if(h->joint[jnt].motNo == 1)
        fSetCurGain1(jnt, h, f, c->iValues[0], c->iValues[1], c->iValues[2]);
    
    sendCan(getSocket(h,jnt), f);
    fprintf(stdout, "Joint number %d has changed current gain values to:\n\tKp:%d\tKi:%d\tKd:%d\n",
            jnt, c->iValues[0], c->iValues[1], c->iValues[2] );
}

void fSetCurGain0(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd){

    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_SET_CUR_GAIN_0;
    f->data[2]    = int_to_bytes(Kp,1);
    f->data[3]    = int_to_bytes(Kp,2);
    f->data[4]    = int_to_bytes(Ki,1);
    f->data[5]    = int_to_bytes(Ki,2);
    f->data[6]    = int_to_bytes(Kd,1);
    f->data[7]    = int_to_bytes(Kd,2);

    f->can_dlc    = 8;
}

void fSetCurGain1(int jnt, hubo_param_t *h, struct can_frame *f, int Kp, int Ki, int Kd){

    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_SET_CUR_GAIN_1;
    f->data[2]    = int_to_bytes(Kp,1);
    f->data[3]    = int_to_bytes(Kp,2);
    f->data[4]    = int_to_bytes(Ki,1);
    f->data[5]    = int_to_bytes(Ki,2);
    f->data[6]    = int_to_bytes(Kd,1);
    f->data[7]    = int_to_bytes(Kd,2);

    f->can_dlc    = 8;
}
// 30

/*
void RBpwmCommandHR2ch(unsigned char _canch, unsigned char _bno, int _duty1, int _duty2, unsigned char _zeroduty) // High resolution, 0.1%
{
	// MsgID		Byte0	Byte1	Byte2		Byte3	Byte4	Byte5	Byte6
	// CMD_TXDF		BNO		0x0D	0x03		DIR0	DUTY0	DIR1	DUTY1
	
	int result;
	
	// CAN message data
	unsigned char tempData[8];
	unsigned short stemp;
	
	// CAN message data assign
	tempData[0] = _bno;							// board no.
	tempData[1] = 0x0D;							// command
	if(_zeroduty != 0)
		tempData[2] = 0x03;						// High Resolution PWM mode
	else
		tempData[2] = 0x00;
	
	stemp = (unsigned short)(abs(_duty1));	// 1st motor duty
	if(stemp > 1000)
		stemp = 1000;
	tempData[3] = (unsigned char)((stemp & 0x0F00) >> 4);  // 1st motor direction : CCW

	if(_duty1 < 0)  		 
		tempData[3] |= 0x01;	// 1st motor direction : CW
	tempData[4] = (unsigned char)(stemp & 0x00FF);
	
	stemp = (unsigned short)(abs(_duty2));	// 1st motor duty
	if(stemp > 1000)
		stemp = 1000;
	tempData[5] = (unsigned char)((stemp & 0x0F00) >> 4);  // 2nd motor direction : CCW
	if(_duty2 < 0)  
		tempData[5] |= 0x01;					// 2nd motor direction : CW
	tempData[6] = (unsigned char)(stemp & 0x00FF);	// 2nd motor duty
	
	// Send CAN message
	result = PushCANMsg(_canch, CMD_TXDF, tempData, 7, 0);
	
	if(result == 0x00) 
		return true;
	else 
		return false;
}

*/

void fSetOpenLoopPwmDutyCycle(int jnt, hubo_state_t *s, hubo_param_t *h, struct can_frame *f, int the_mode){
// For DRC-Hubo

    double duty = s->joint[jnt].duty;

    /* Saturation */
    if ( duty > 1.0 ) duty = 1.0;
    if ( duty < -1.0) duty = -1.0;

    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_OPENLOOP_PWM_DUTY_CYCLE_MODE;
    f->data[2]    = 0x03; // PUL_ON

    f->can_dlc    = 3;


}


/*
void fSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f, int the_mode){

    if (the_mode == D_ENABLE)
      f->data[2] = 0x01;    // turn on complementry switching mode
    else
      f->data[2] = 0x00;    // turn off complementry switching mode  

    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_COMPLEMENTARY_SWITCHING_MODE;

    f->can_dlc    = 3;
}

void hSetComplementaryMode(int jnt, hubo_param_t *h, struct can_frame *f, int the_mode) {

    fSetComplementaryMode(jnt, h, f, the_mode);
    sendCan(getSocket(h,jnt), f);
}
*/
// 4
void fGetCurrentValue(int jnt, hubo_param_t *h, struct can_frame *f) {
    // get the value of the current in 10mA units A = V/100
    f->can_id     = CMD_TXDF;    // Set ID

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_GET_CURRENT;

    f->can_dlc     = 2;
}

// 4

void hSetAlarm(int jnt, hubo_param_t *h, struct can_frame *f, hubo_d_param_t sound)
{
    uint8_t h_sound;
    switch (sound) {
        case D_ALARM_SOUND1:
            h_sound = H_ALARM_S1; break;
        case D_ALARM_SOUND2:
            h_sound = H_ALARM_S2; break;
        case D_ALARM_SOUND3:
            h_sound = H_ALARM_S3; break;
        case D_ALARM_SOUND4:
            h_sound = H_ALARM_S4; break;
        case D_ALARM_OFF:
            h_sound = H_ALARM_OFF; break;
        default:
            h_sound = H_ALARM_OFF;
            fprintf(stderr, "Invalid parameter given for Alarm Sound: %d\n", (int)sound);
            break;
    }
    fSetAlarm(jnt, h, f, h_sound);
    sendCan(getSocket(h,jnt), f);
}

void fSetAlarm(int jnt, hubo_param_t *h, struct can_frame *f, int sound)
{
    f->can_id     = CMD_TXDF;    // Set ID

    f->data[0]     = getJMC(h,jnt);    
    f->data[1]    = H_ALARM;
    f->data[2]     = (uint8_t)sound;     // Use H_ALARM_S1, S2, S3, S4, or OFF

    f->can_dlc = 3;
}


void hOpenLoopPWM(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    int jnt = c->joint;
    
    if(h->joint[jnt].numMot == 2)
    {
        if(    (c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
            (c->param[1]!=D_CLOCKWISE&&c->param[1]!=D_COUNTERCLOCKWISE) ||
            (c->iValues[0]>100&&c->iValues[0]<-100) ||
            (c->iValues[1]>100&&c->iValues[1]<-100) )

        {
            fprintf(stderr, "Invalid PWM Values:\n\tDir:%d\tDuty:%d\tDir:%d\tDuty:%d\n\t"
                    "Dir must be D_CLOCKWISE (%d) or D_COUNTERCLOCKWISE (%d)\n\t"
                    "Duty range: [-100,100]\n",
                    c->param[0], c->iValues[0], c->param[1], c->iValues[1],
                    D_CLOCKWISE, D_COUNTERCLOCKWISE );
        }
        else
        {
            int dir1, dir2;
            if(c->param[0]==D_CLOCKWISE) dir1=0; else dir1=1;
            if(c->param[1]==D_CLOCKWISE) dir2=0; else dir2=1;

            fOpenLoopPWM_2CH(jnt, h, f, dir1, c->iValues[0], dir2, c->iValues[1]);
            
            sendCan(getSocket(h,jnt), f);
        }
    }
    else if(h->joint[jnt].numMot == 3)
    {
        if(    (c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
            (c->param[1]!=D_CLOCKWISE&&c->param[1]!=D_COUNTERCLOCKWISE) ||
            (c->param[2]!=D_CLOCKWISE&&c->param[2]!=D_COUNTERCLOCKWISE) ||
            (c->iValues[0]>100&&c->iValues[0]<-100) ||
            (c->iValues[1]>100&&c->iValues[1]<-100) ||
            (c->iValues[2]>100&&c->iValues[2]<-100) )
        {
            fprintf(stderr, "Invalid PWM Values:\n\tDir:%d\tDuty:%d\tDir:%d\tDuty:%d\t"
                    "Dir:%d\tDuty:%d\n\t"
                    "Dir must be D_CLOCKWISE (%d) or D_COUNTERCLOCKWISE (%d)\n\t"
                    "Duty range: [-100,100]\n",
                    (int)c->param[0], c->iValues[0], (int)c->param[1], c->iValues[1],
                    (int)c->param[2], c->iValues[2],
                    (int)D_CLOCKWISE, (int)D_COUNTERCLOCKWISE );
        }
        else
        {
            int dir1, dir2, dir3;
            if(c->param[0]==D_CLOCKWISE) dir1=0; else dir1=1;
            if(c->param[1]==D_CLOCKWISE) dir2=0; else dir2=1;
            if(c->param[2]==D_CLOCKWISE) dir3=0; else dir3=1;
            

            fOpenLoopPWM_3CH(jnt, h, f, dir1, c->iValues[0], dir2, c->iValues[1],
                         dir3, c->iValues[2] );
            
            sendCan(getSocket(h,jnt), f);
        }
    }
    else if(h->joint[jnt].numMot == 5)
    {
        if(    (c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
            (c->param[1]!=D_CLOCKWISE&&c->param[1]!=D_COUNTERCLOCKWISE) ||
            (c->param[2]!=D_CLOCKWISE&&c->param[2]!=D_COUNTERCLOCKWISE) ||
            (c->param[3]!=D_CLOCKWISE&&c->param[3]!=D_COUNTERCLOCKWISE) ||
            (c->param[4]!=D_CLOCKWISE&&c->param[4]!=D_COUNTERCLOCKWISE) ||
            (c->iValues[0]>100&&c->iValues[0]<-100) ||
            (c->iValues[1]>100&&c->iValues[1]<-100) ||
            (c->iValues[2]>100&&c->iValues[2]<-100) ||
            (c->iValues[3]>100&&c->iValues[3]<-100) ||
            (c->iValues[4]>100&&c->iValues[4]<-100) )
        {
            fprintf(stderr, "Invalid PWM Values:\n\tDir:%d\tDuty:%d\tDir:%d\tDuty:%d\t"
                    "Dir:%d\tDuty:%d\tDir:%d\tDuty:%d\tDir:%d\tDuty:%d\n\t"
                    "Dir must be D_CLOCKWISE (%d) or D_COUNTERCLOCKWISE (%d)\n\t"
                    "Duty range: [-100,100]\n",
                    (int)c->param[0], c->iValues[0], (int)c->param[1], c->iValues[1],
                    (int)c->param[2], c->iValues[2], (int)c->param[3], c->iValues[3],
                    (int)c->param[4], c->iValues[4],
                    (int)D_CLOCKWISE, (int)D_COUNTERCLOCKWISE );    
        }
        else
        {
            int dir1, dir2, dir3, dir4, dir5;
            if(c->param[0]==D_CLOCKWISE) dir1=0; else dir1=1;
            if(c->param[1]==D_CLOCKWISE) dir2=0; else dir2=1;
            if(c->param[2]==D_CLOCKWISE) dir3=0; else dir3=1;
            if(c->param[3]==D_CLOCKWISE) dir4=0; else dir4=1;
            if(c->param[4]==D_CLOCKWISE) dir5=0; else dir5=1;

            fOpenLoopPWM_5CH(jnt, h, f, dir1, c->iValues[0], dir2, c->iValues[1],
                        dir3, c->iValues[2], dir4, c->iValues[3],
                        dir5, c->iValues[4] );

            sendCan(getSocket(h,jnt), f);
        }
    }
} 


void fOpenLoopPWM_2CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int duty0, int dir1, int duty1)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_OPENLOOP_PWM;
    f->data[2]    = 0x01; // Pulse according to duty cycle
                //    Alternative value is 0x00 which enforces
                //    zero duty in order to stop the motor
    f->data[3]    = (uint8_t)dir0;
    f->data[4]    = (uint8_t)duty0;
    f->data[5]    = (uint8_t)dir1;
    f->data[6]    = (uint8_t)duty1;
    f->data[7]    = H_BLANK;

    f->can_dlc    = 8;
}

void fOpenLoopPWM_3CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int dt0, int dir1, int dt1, int dir2, int dt2)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_OPENLOOP_PWM;
    f->data[2]    = 0x01; // Pulse according to duty cycle
                //    Alternative value is 0x00 which enforces
                //    zero duty in order to stop the motor
    f->data[3]    = duty_to_byte(dir0, dt0);
    f->data[4]    = duty_to_byte(dir1, dt1);
    f->data[5]    = duty_to_byte(dir2, dt2); 

    f->can_dlc    = 6;
}

void fOpenLoopPWM_5CH(int jnt, hubo_param_t *h, struct can_frame *f,
            int dir0, int dt0, int dir1, int dt1, int dir2, int dt2,
            int dir3, int dt3, int dir4, int dt4)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_OPENLOOP_PWM;
    f->data[2]    = 0x01; // Pulse according to duty cycle
                //    Alternative value is 0x00 which enforces
                //    zero duty in order to stop the motor
    f->data[3]    = duty_to_byte(dir0, dt0);
    f->data[4]    = duty_to_byte(dir1, dt1);
    f->data[5]    = duty_to_byte(dir2, dt2); 
    f->data[6]    = duty_to_byte(dir3, dt3);
    f->data[7]    = duty_to_byte(dir4, dt4);

    f->can_dlc    = 8;
}

void hSetControlMode(int jnt, hubo_param_t *h, struct can_frame *f, hubo_d_param_t mode)
{
    switch (mode)
    {
        case D_POSITION:
            fSetControlMode(jnt, h, f, 0); // 0 > Position control
            sendCan(getSocket(h,jnt),f);
//            s->driver[h->joint[jnt].jmc].ctrlMode = D_POSITION;
            break;
        case D_CURRENT:
            fSetControlMode(jnt, h, f, 1); // 1 > Current control
            sendCan(getSocket(h,jnt),f);
//            s->driver[h->joint[jnt].jmc].ctrlMode = D_CURRENT;
            break;
        case D_HYBRID:
            fSetControlMode(jnt, h, f, 2);
            sendCan(getSocket(h,jnt),f);
            break;
        default:
            fprintf(stderr,"Invalid Control Mode: %d\n\t"
                    "Must use: D_POSITION (%d) or D_CURRENT (%d)\n",
                    (int)mode, (int)D_POSITION, (int)D_CURRENT);
            break;
    }
}


void fSetControlMode(int jnt, hubo_param_t *h, struct can_frame *f, int mode)
{
    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_SET_CTRL_MODE;
    f->data[2]    = (uint8_t)mode;

    f->can_dlc    = 3;
}


void hGetEncValue(int jnt, uint8_t encChoice, hubo_param_t *h, struct can_frame *f) { ///> make can frame for getting the value of the Encoder
    fGetEncValue( jnt, encChoice, h, f);
    sendCan(getSocket(h,jnt), f);
}
void fGetEncValue(int jnt, uint8_t encChoice, hubo_param_t *h, struct can_frame *f) { ///> make can frame for getting the value of the Encoder
    f->can_id       = CMD_TXDF;     // Set ID
    f->data[0]      = h->joint[jnt].jmc;
    f->data[1]    = H_GET_ENCODER;
    f->data[2]    = encChoice;
    f->can_dlc = 3; //= strlen( data );     // Set DLC
}

void fGetBoardStatusAndErrorFlags(int jnt, hubo_param_t *h, struct can_frame *f)
{
    f->can_id    = CMD_TXDF;
    f->data[0]    = h->joint[jnt].jmc;
    f->data[1]    = H_GET_STATUS;
    f->can_dlc    = 3;
}


void hGetBoardStatus(int jnt, hubo_state_t *s, hubo_param_t *h, struct can_frame *f)
{
    fGetBoardStatusAndErrorFlags( jnt, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, HUBO_CAN_TIMEOUT_DEFAULT);
    decodeFrame(s, h, f);
}

void hGetCurrentValue(int jnt, hubo_param_t *h, struct can_frame *f) { ///> make can frame for getting the motor current in amps (10mA resolution)
    fGetCurrentValue( jnt, h, f);
    sendCan(getSocket(h,jnt), f);
}


void hSetBeep(int jnt, hubo_param_t *h, struct can_frame *f, double beepTime)
{
    fSetBeep(jnt, h, f, beepTime);
    sendCan(getSocket(h,jnt), f);
}

void fSetBeep(int jnt, hubo_param_t *h, struct can_frame *f, double beepTime)
{
    f->can_id     = CMD_TXDF;    // Set ID

    f->data[0]     = getJMC(h,jnt);    // BNO // Try 0x82
    f->data[1]    = H_BEEP;        // beep
    f->data[2]    = (uint8_t)floor(beepTime/0.1);
    
    f->can_dlc = 3;
}


// 16  home
void fGotoLimitAndGoOffset(int jnt, hubo_param_t *h, struct can_frame *f)
{
    f->can_id     = CMD_TXDF;
    
    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_HOME;
    f->data[2]     = (((uint8_t)h->joint[jnt].motNo+1) << 4)|2; // set /DT high
    f->data[3]      = H_BLANK;
    f->data[4]      = H_BLANK;
    f->data[5]      = H_BLANK;
    f->data[6]      = H_BLANK;
    f->data[7]      = H_BLANK;
    
    f->can_dlc    = 8;
}


void hSetDeadZone(int jnt, hubo_param_t *h, struct can_frame *f, int deadzone)
{
    if(deadzone>=0&&deadzone<=255)
    {
        fSetDeadZone(jnt, h, f, deadzone);
        sendCan(getSocket(h,jnt),f);
    }
    else
        fprintf(stderr,"Invalid value for deadzone: %d\n\t"
                    "Range: [0,255]\n", deadzone);
}

void fSetDeadZone(int jnt, hubo_param_t *h, struct can_frame *f, int deadzone)
{
    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = (uint8_t)(H_SET_DEADZONE + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
    f->data[2]    = (uint8_t)deadzone;

    f->can_dlc    = 3;
}

void hSetHomeSearchParams( hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f )
{
    unsigned int dir;
    int32_t offset;
    
    switch (c->param[0])
    {
        default:
            fprintf(stderr, "Invalid parameter for Limit switch search direction: %d\n\t"
                        "Defaulting to D_CLOCKWISE (%d)\n",
                        (int)c->param[0], (int)D_CLOCKWISE);
        case D_CLOCKWISE:
            dir = 0; break;
        case D_COUNTERCLOCKWISE:
            dir = 1; break;
    }

    offset = (int32_t)ref2enc(c->joint, c->dValues[0], h);
    fSetHomeSearchParams(c->joint, h, f, c->iValues[0], dir, offset);
    
    sendCan(getSocket(h,c->joint),f);
}

void hSetHomeSearchParamsRaw( hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f )
{
    int32_t offset = (int32_t)(c->iValues[2]);

    if( c->iValues[1] < 0 )
    {
        fprintf(stderr, "Raw search direction for %s should be 0 or 1 -- not negative!\n"
                        " -- Assuming you want 0 (Clockwise)", jointNames[c->joint]);
        c->iValues[1] = 0;
    }
    if( c->iValues[1] > 1 )
    {
        fprintf(stderr, "Raw search direction for %s should be 0 or 1 -- not %d!\n"
                        " -- Assuming you want 1 (Counter-Clockwise)", jointNames[c->joint], c->iValues[1] );
        c->iValues[1] = 1;
    }
    uint8_t dir = (uint8_t)c->iValues[1];

    fprintf(stdout, "Setting home parameters:\n"
                    " -- Joint:%s\tOffset:%d\tDir:%d\tLim:%d\n", jointNames[c->joint], offset, dir, c->iValues[0]);
    fSetHomeSearchParams(c->joint, h, f, c->iValues[0], dir, offset);

    sendCan(getSocket(h,c->joint),f);
}

void fSetHomeSearchParams(int jnt, hubo_param_t *h, struct can_frame *f, int limit,
                unsigned int dir, int32_t offset)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = (uint8_t)(H_SET_HOME_PARAM + h->joint[jnt].motNo);
    f->data[2]    = (uint8_t)limit;
    f->data[3]    = (uint8_t)dir;
    f->data[4]    = int_to_bytes(offset,1);
    f->data[5]    = int_to_bytes(offset,2);
    f->data[6]    = int_to_bytes(offset,3);
    f->data[7]    = int_to_bytes(offset,4);

    f->can_dlc    = 8;
}

void hSetEncoderResolution(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    uint16_t res;
    
    switch(c->param[0])
    {
        case D_CLOCKWISE:
            res = 1 << 15; break;
        default:
            fprintf(stderr, "Invalid parameter for Motor Direction: %d\n\t"
                        "Defaulting to D_CLOCKWISE (%d)\n",
                        (int)c->param[0], (int)D_CLOCKWISE);
        case D_COUNTERCLOCKWISE:
            res = 0 << 15; break;
    }
    
    switch(c->param[1])
    {
        default:
            fprintf(stderr, "Invalid Parameter for Auto-Scale: %d\n\t"
                        "Defauling to D_ENABLE (%d)\n", (int)c->param[1], D_ENABLE);
        case D_ENABLE:
            res = res | (1<<14); break;
        case D_DISABLE:
            res = res | (0<<14); break;
    }

    if(c->iValues[0] >= 16384 || c->iValues[0]<0) // Cannot exceed 14 bits. 2^14 = 16384
    {
        fprintf(stderr, "Encoder resolution value out of range: %d\n\t"
                    "Defaulting to max resolution (16383)\n", (int)c->iValues[0] );
        res = res | 16383;
    }
    else
        res = res | c->iValues[0];

    fSetEncoderResolution(c->joint, h, f, res);
    sendCan(getSocket(h,c->joint),f);
}

void fSetEncoderResolution(int jnt, hubo_param_t *h, struct can_frame *f, int res)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = (uint8_t)(H_SET_ENC_RES + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
    f->data[2]    = int_to_bytes(res,1); // TODO: Have handler construct res properly
    f->data[3]    = int_to_bytes(res,2);

    f->can_dlc    = 4;
}

void hSetMaxAccVel(int jnt, hubo_param_t *h, struct can_frame *f, int maxAcc, int maxVel)
{
    if( maxAcc >= 65536 || maxAcc <=0 )
        fprintf(stderr, "Max Acceleration value out of bounds: %d\n\t"
                "Maximum value is 65535\n", maxAcc);
    else if( maxVel < 65536 && maxVel > 0 )
    {
        fSetMaxAccVel(jnt, h, f, maxAcc, maxVel);
        sendCan(getSocket(h,jnt),f);
    }
    else
        fprintf(stderr, "Max Velocity value is out of bounds: %d\n\t"
                "Maximum value is 65535\n", maxVel);
}

void fSetMaxAccVel(int jnt, hubo_param_t *h, struct can_frame *f, int maxAcc, int maxVel)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = (uint8_t)(H_SET_MAX_ACC_VEL + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
    f->data[2]    = int_to_bytes(maxAcc,1);
    f->data[3]    = int_to_bytes(maxAcc,2);
    f->data[4]    = int_to_bytes(maxVel,1);
    f->data[5]    = int_to_bytes(maxVel,2);

    f->can_dlc    = 6;
}

void hSetLowerPosLimit(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    uint8_t enable, update;
    
    switch(c->param[0])
    {
        case D_UPDATE:
            update = 1; break;
        default:
            fprintf(stderr, "Lower position limit update parameter invalid: %d\n\t"
                        "Defaulting to D_IGNORE (%d)\n", (int)c->param[0], (int)D_IGNORE);
        case D_IGNORE:
            update = 0; break;
    }
    
    switch(c->param[1])
    {
        default:
            fprintf(stderr, "Lower position limit enabling parameter invalid: %d\n\t"
                        "Defaulting to D_ENABLE (%d)\n", (int)c->param[1], (int)D_ENABLE);
        case D_ENABLE:
            enable = 1; break;
        case D_DISABLE:
            enable = 0; break;
    }

    fSetLowerPosLimit(c->joint, h, f, enable, update, c->iValues[0]);
    sendCan(getSocket(h,c->joint),f);
}

void hSetLowerPosLimitRaw(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    fprintf(stdout, "Setting lower position limit:\n"
                    " -- Joint:%s\tLimit:%d\tEnabled:%d\tUpdate:%d\n", jointNames[c->joint], c->iValues[0], c->iValues[2], c->iValues[1]);
    fSetLowerPosLimit(c->joint, h, f, c->iValues[2], c->iValues[1], c->iValues[0]);
    sendCan(getSocket(h,c->joint),f);
}

void fSetLowerPosLimit(int jnt, hubo_param_t *h, struct can_frame *f, int enable, int update, int limit)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = (uint8_t)(H_SET_LOW_POS_LIM + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
    f->data[2]    = (uint8_t)( (update << 1) | (enable) );
    f->data[3]    = int_to_bytes(limit,1);
    f->data[4]    = int_to_bytes(limit,2);
    f->data[5]    = int_to_bytes(limit,3);
    f->data[6]    = int_to_bytes(limit,4);

    f->can_dlc    = 7;
}



void hSetUpperPosLimit(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    uint8_t enable, update;
    
    switch(c->param[0])
    {
        case D_UPDATE:
            update = 1; break;
        default:
            fprintf(stderr, "Upper position limit update parameter invalid: %d\n\t"
                        "Defaulting to D_IGNORE (%d)\n", (int)c->param[0], (int)D_IGNORE);
        case D_IGNORE:
            update = 0; break;
    }
    
    switch(c->param[1])
    {
        default:
            fprintf(stderr, "Upper position limit enabling parameter invalid: %d\n\t"
                        "Defaulting to D_ENABLE (%d)\n", (int)c->param[1], (int)D_ENABLE);
        case D_ENABLE:
            enable = 1; break;
        case D_DISABLE:
            enable = 0; break;
    }

    fSetUpperPosLimit(c->joint, h, f, enable, update, c->iValues[0]);
    sendCan(getSocket(h,c->joint),f);
}

void hSetUpperPosLimitRaw(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    fprintf(stdout, "Setting upper position limit:\n"
                    " -- Joint:%s\tLimit:%d\tEnabled:%d\tUpdate:%d\n", jointNames[c->joint], c->iValues[0], c->iValues[2], c->iValues[1]);
    fSetUpperPosLimit(c->joint, h, f, c->iValues[2], c->iValues[1], c->iValues[0]);
    sendCan(getSocket(h,c->joint),f);
}

void fSetUpperPosLimit(int jnt, hubo_param_t *h, struct can_frame *f, int enable, int update, int limit)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = (uint8_t)(H_SET_UPP_POS_LIM + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
    f->data[2]    = (uint8_t)( (update << 1) | (enable) );
    f->data[3]    = int_to_bytes(limit,1);
    f->data[4]    = int_to_bytes(limit,2);
    f->data[5]    = int_to_bytes(limit,3);
    f->data[6]    = int_to_bytes(limit,4);

    f->can_dlc    = 7;
}

void hSetHomeAccVel(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    int mode;

    switch(c->param[0])
    {
        default:
            fprintf(stderr, "Invalid homing mode parameter: %d\n\t"
                    "Must use D_SWITCH_AND_INDEX (%d), D_SWITCH (%d),\n\t"
                    "or D_JAM_LIMIT (%d)\n"
                "Defaulting to D_SWITCH_AND_INDEX\n", (int)c->param[0],
                    (int)D_SWITCH_AND_INDEX, (int)D_SWITCH, (int)D_JAM_LIMIT);
        case D_SWITCH_AND_INDEX:
            mode = 0; break;
        case D_SWITCH:
            mode = 1; break;
        case D_JAM_LIMIT:
            mode = 2; break;
    }

    fSetHomeAccVel(c->joint, h, f, (float)c->dValues[0], c->iValues[0], c->iValues[1],
            mode, c->iValues[2]);

    sendCan(getSocket(h,c->joint),f);
}

void fSetHomeAccVel(int jnt, hubo_param_t *h, struct can_frame *f, float mAcc, int mVelS,
            int mVelP, int mode, int mDuty)
{
    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = (uint8_t)(H_SET_HOME_VEL_ACC + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
    f->data[2]    = (uint8_t)(mAcc*100);
    f->data[3]    = (uint8_t)(mVelS);
    f->data[4]    = (uint8_t)(mVelP);
    if( mode==0 || mode==1 || mode==2 )
        f->data[5]    = (uint8_t)(mode);
    else
        f->data[5]    = 0x00;
    f->data[6]    = (uint8_t)(mDuty);

    f->can_dlc    = 7;
}

void hSetGainOverride(int jnt, hubo_param_t *h, struct can_frame *f, int gain0, int gain1, double dur)
{
    fSetGainOverride(jnt, h, f, gain0, gain1, (int)(dur*1000));
    sendCan(getSocket(h,jnt),f);
}


void fSetGainOverride(int jnt, hubo_param_t *h, struct can_frame *f, int gain0, int gain1, int duration)
{
    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_GAIN_OVERRIDE;
    f->data[2]    = (uint8_t)(gain0);
    f->data[3]    = (uint8_t)(gain1);
    f->data[4]    = int_to_bytes(duration,1);
    f->data[5]    = int_to_bytes(duration,2);

    f->can_dlc = 6;
}

void hSetBoardNumber(int jnt, hubo_param_t *h, struct can_frame *f, int boardNum, int rate)
{
    fprintf(stdout, "WARNING: Changing board number %d to %d with baud rate %d\n\t",
            getJMC(h,jnt), boardNum, rate);
    fSetBoardNumber(jnt, h, f, boardNum, rate);
    sendCan(getSocket(h,jnt),f);
}


void fSetBoardNumber(int jnt, hubo_param_t *h, struct can_frame *f, int boardNum, int rate)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_SET_BOARD_NUM;
    f->data[2]    = (uint8_t)(boardNum);
    f->data[3]    = (uint8_t)(rate);

    f->can_dlc = 4;
}

void hSetJamPwmLimits(hubo_board_cmd_t *c, hubo_param_t *h, struct can_frame *f)
{
    fprintf(stdout, "Changing Jam and PWM Saturation limits:\n\t"
            "Jam Duty: %d \t Sat Duty: %d\n\t"
            "Jam Time: %g \t Sat Time: %g\n",
            c->iValues[0], c->iValues[1], c->dValues[0], c->dValues[1] );
    fSetJamPwmLimits(c->joint, h, f, (int)(c->dValues[0]*1000), (int)(c->dValues[1]*1000),
                c->iValues[0], c->iValues[1] );

    sendCan(getSocket(h,c->joint),f);
}

void fSetJamPwmLimits(int jnt, hubo_param_t *h, struct can_frame *f, int jamLimit, int pwmLimit,
            int lim_detection_duty, int jam_detection_duty )
{
    f->can_id    = CMD_TXDF;

    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_SET_JAM_SAT_LIM;
    f->data[2]    = int_to_bytes(jamLimit,1);
    f->data[3]    = int_to_bytes(jamLimit,2);
    f->data[4]    = int_to_bytes(pwmLimit,1);
    f->data[5]    = int_to_bytes(pwmLimit,2);
    f->data[6]    = (uint8_t)(lim_detection_duty);
    f->data[7]    = (uint8_t)(jam_detection_duty);
    
    f->can_dlc = 8;
}

void hSetErrorBound(int jnt, hubo_param_t *h, struct can_frame *f, int inputDiffErr, int maxError,
            int tempError)
{
    fprintf(stdout, "Changing error bounds on board %d:\n\t"
            "Input Difference error: %d\n\t"
            "Maximum error: %d\n\t"
            "Max temperature: %d\n", jnt, inputDiffErr, maxError, tempError);

    fSetErrorBound(jnt, h, f, inputDiffErr, maxError, tempError);
    sendCan(getSocket(h,jnt),f);
}

void fSetErrorBound(int jnt, hubo_param_t *h, struct can_frame *f, int inputDiffErr, int maxError,
            int tempError)
{
    f->can_id    = CMD_TXDF;
    
    f->data[0]    = getJMC(h,jnt);
    f->data[1]    = H_SET_ERR_BOUND;
    f->data[2]     = int_to_bytes(inputDiffErr,1);
    f->data[3]    = int_to_bytes(inputDiffErr,2);
    f->data[4]    = int_to_bytes(maxError,1);
    f->data[5]    = int_to_bytes(maxError,2);
    f->data[6]    = int_to_bytes(tempError,1);
    f->data[7]    = int_to_bytes(tempError,2);

    f->can_dlc = 8;
}




void hGotoLimitAndGoOffset(int jnt, hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h,
        hubo_state_t *s, struct can_frame *f, int send)
{
    fGotoLimitAndGoOffset(jnt, h, f);
    sendCan( getSocket(h,jnt), f );
    fprintf(stdout," -- Homing Joint #%d\n\t",jnt);
    r->ref[jnt] = 0.0;
    r->mode[jnt] = HUBO_REF_MODE_REF_FILTER;
    r_filt->ref[jnt] = 0.0;
    s->joint[jnt].ref = 0.0;
    s->joint[jnt].zeroed = 2; ///< 2 means it needs confirmation

    hubo_noRefTimeAll = hubo_home_noRef_delay;

    if(send==1)
        ach_put( &chan_hubo_ref, r, sizeof(*r) );


    if(HUBO_ROBOT_TYPE_DRC_HUBO == hubo_type
        && ( RWR==jnt || LWR==jnt || RF1==jnt || RF2==jnt || LF1==jnt ) )
    {
        hSetControlMode(jnt, h, f, D_HYBRID);
        hSetControlMode(jnt, h, f, D_HYBRID);
    }
}

void hGotoLimitAndGoOffsetAll(hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h, hubo_state_t *s, struct can_frame *f) {
    fprintf(stdout, "Homing all joints!\n\t");
    int i = 0;
    for(i = 0; i < HUBO_JOINT_COUNT; i++) {
        if(s->joint[i].active == true) {
            hGotoLimitAndGoOffset(i, r, r_filt, h, s, f, 0);
        }
        else
            fprintf(stdout, " -- Joint #%d is inactive!\n\t",i);
    }
    
    ach_put( &chan_hubo_ref, r, sizeof(*r) );
}

void hInitializeBoard(int jnt, hubo_param_t *h, struct can_frame *f) {
    fInitializeBoard(jnt, h, f);
    sendCan(getSocket(h,jnt), f);
    //readCan(hubo_socket[h->joint[jnt].can], f, 4);    // 8 bytes to read and 4 sec timeout
    // TODO: Why is the readCan here??
    readCan(getSocket(h,jnt), f, HUBO_CAN_TIMEOUT_DEFAULT*100);    // 8 bytes to read and 4 sec timeout
}

void fInitializeBoard(int jnt, hubo_param_t *h, struct can_frame *f) {
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_INIT_BOARD;

    f->can_dlc    = 2;
}

void hInitializeBoardAll( hubo_param_t *h, hubo_state_t *s, struct can_frame *f ) {
    ///> Initilizes all boards
    int i = 0;
    for(i = 0; i < HUBO_JOINT_COUNT; i++) {
        if(s->joint[i].active == true) {
            hInitializeBoard(i, h, f);
        }
    }
}

void hSetEncRef(int jnt, hubo_state_t *s, hubo_ref_t *r, hubo_param_t *h,
                    hubo_pwm_gains_t *g, struct can_frame *f) {

    int check = h->joint[jnt].numMot;
    uint16_t jmc = h->joint[jnt].jmc;
    if(h->joint[jnt].numMot <= 2)
    {    
      int i;
      for(i=0; i<h->joint[jnt].numMot; i++)
      {
        int j = h->driver[jmc].joints[i];
        if( s->joint[j].zeroed==2)
        {
            if(HUBO_ROBOT_TYPE_DRC_HUBO == hubo_type
                && ( RWR==jnt || LWR==jnt || RF1==jnt || RF2==jnt || LF1==jnt ) )
            {
                hSetControlMode(jnt, h, f, D_HYBRID);
                hSetControlMode(jnt, h, f, D_HYBRID);
            }

//          if( s->status[j].homeFlag==H_HOME_SUCCESS) // && s->status[j].bigError==0 )
          if( s->status[j].homeFlag==H_HOME_SUCCESS &&
                fabs(s->joint[j].pos) <= HUBO_COMP_RIGID_TRANS_THRESHOLD )
          {
            s->joint[j].zeroed=1;
          }
          else
          {
            s->joint[j].zeroed=0;
            fprintf(stdout, "Joint %s was not homed correctly!\n\t", jointNames[j] );
          }
        }
        if( s->joint[j].zeroed==1 )
        check--;
      }
    }

//    if( check==0 | jnt == RF1 | jnt == LF1 )

    if(HUBO_ROBOT_TYPE_DRC_HUBO == hubo_type){
      fSetEncRef(jnt, s, r, h, g, f);
      sendCan(getSocket(h,jnt), f);
    }
    else if(HUBO_ROBOT_TYPE_HUBO_PLUS == hubo_type & (jnt != RF2) & (jnt != RF3) & (jnt != RF4) & (jnt !=RF5) &
                                                     (jnt != LF2) & (jnt != LF3) & (jnt != LF4) & (jnt !=LF5)) 
    {
      fSetEncRef(jnt, s, r, h, g, f);
      sendCan(getSocket(h,jnt), f);
    }

    memset(f, 0, sizeof(*f));

}

void hIniAll(hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s, struct can_frame *f) {
// --std=c99
        printf("2\n");
    int i = 0;
    for( i = 0; i < HUBO_JOINT_COUNT; i++ ) {
        if(s->joint[i].active) {
            hInitializeBoard(i, h, f);
            printf("%i\n",i);
        }
    }
}

void hMotorDriverOnOff(int jnt, hubo_param_t *h, struct can_frame *f, hubo_d_param_t onOff)
{
    if(onOff == D_ENABLE) { // turn on FET
        fEnableMotorDriver(jnt, h, f);
        sendCan(getSocket(h,jnt), f); 
        
    }
    else if(onOff == D_DISABLE) { // turn off FET
        fDisableMotorDriver(jnt, h, f);
        sendCan(getSocket(h,jnt), f); }
    else
        fprintf(stderr, "FET Switch Error: Invalid param[0]\n\t"
                "Must be D_ENABLE (%d) or D_DISABLE (%d)",
                D_ENABLE, D_DISABLE);
}

void fEnableMotorDriver(int jnt, hubo_param_t *h, struct can_frame *f)
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_SWITCH_DRIVER;
    f->data[2]     = 0x01; // Turn on
    
    f->can_dlc    = 3;
}

void fDisableMotorDriver(int jnt, hubo_param_t *h,  struct can_frame *f) {
    f->can_id     = CMD_TXDF;
    
    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_SWITCH_DRIVER;
    f->data[2]     = 0x00; // Turn off
    
    f->can_dlc    = 3;
}

void hFeedbackControllerOnOff(int jnt, hubo_ref_t *r, hubo_state_t *s, hubo_param_t *h, struct can_frame *f, hubo_d_param_t onOff) {

if(isHands(jnt) == 0) {
    if(onOff == D_ENABLE) { // ctrl on FET

        // FIXME:   It makes absolutely no sense for hubo-daemon
        //          to be writing to the ref channel. We need a
        //          better protocol here

        r->ref[jnt] = s->joint[jnt].pos; 
        s->joint[jnt].ref = s->joint[jnt].pos;
        r->mode[jnt] = HUBO_REF_MODE_REF_FILTER;
        ach_put( &chan_hubo_ref, r, sizeof(*r));

        /* set new position reference */
//        hSetEncRef(jnt, s, h, f);
//        fSetEncRef(jnt, s, h, f);
//        sendCan(getSocket(h,jnt), f);
        fEnableFeedbackController(jnt, h, f);
        sendCan(hubo_socket[h->joint[jnt].can], f); 
//        hubo_noRefTimeAll = 0.01;
        }
    else if(onOff == D_DISABLE) { // turn ctrol off
        r->mode[jnt] = HUBO_REF_MODE_COMPLIANT;
        ach_put( &chan_hubo_ref, r, sizeof(*r));
        fDisableFeedbackController(jnt, h, f);
        sendCan(hubo_socket[h->joint[jnt].can], f); }

    else
        fprintf(stderr, "Controller Switch Error: Invalid param[0] (%d)\n\t"
                    "Must be D_ENABLE (%d) or D_DISABLE (%d)", onOff,
                    D_ENABLE, D_DISABLE);
}
}

void fEnableFeedbackController(int jnt, hubo_param_t *h, struct can_frame *f)
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_MOTOR_CTRL_ON;
    
    f->can_dlc    = 2;
}

void fDisableFeedbackController(int jnt, hubo_param_t *h, struct can_frame *f)
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_MOTOR_CTRL_OFF;
    
    f->can_dlc    = 2;
}

void hResetEncoderToZero(int jnt, hubo_ref_t *r, hubo_param_t *h, hubo_state_t *s, struct can_frame *f) {
    /* Get Board Status */
    hGetBoardStatus(jnt, s, h, f);
    if( 1 == isError(jnt, s) | jnt == RWP | jnt == LWP){
        fResetEncoderToZero(jnt, h, f);

        /* Set New Ref to Zero */
        r->ref[jnt] = 0.0;
        r->mode[jnt] = HUBO_REF_MODE_REF_FILTER;
        s->joint[jnt].ref = 0.0;
//    s->joint[jnt].pos = 0;
        ach_put( &chan_hubo_ref, r, sizeof(*r) );
//    ach_put( &chan_hubo_ref, r, sizeof(*r) );

        sendCan(getSocket(h,jnt), f);
        s->joint[jnt].zeroed == 2;        // need to add a can read back to confirm it was zeroed
    }
}

void fResetEncoderToZero(int jnt, hubo_param_t *h, struct can_frame *f) {
    /* Reset Encoder to Zero (REZ: 0x06)
    CH: Channel No.
    CH= 0 ~ 4 according to the board.
    CH= 0xF selects ALL Channel
    Action:
    1. Set encoder(s) to Zero.
    2. Initialize internal parameters.
    3. Reset Fault and Error Flags.
    */

    f->can_id     = CMD_TXDF;    // Set ID

    f->data[0]     = getJMC(h,jnt);
    f->data[1]    = H_SET_ENC_ZERO;
    f->data[2]     = h->joint[jnt].motNo;
    
    f->can_dlc = 3; //= strlen( data );    // Set DLC
}

void fNullIMUSensor( int bno, struct can_frame *f )
{
    f->can_id    = CMD_TXDF;

    f->data[0]    = (uint8_t)bno + BNO_SENSOR_BASE;
    f->data[1]    = H_REQ_NULL;

    f->can_dlc = 2;
}

void hNullIMUSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f )
{
    switch (board)
    {
        case D_IMU_SENSOR_0:
            fNullIMUSensor( h->sensor[HUBO_IMU0].boardNo, f );
            sendCan(sensorSocket(h, HUBO_IMU0), f);
            break;
        case D_IMU_SENSOR_1:
            fNullIMUSensor( h->sensor[HUBO_IMU1].boardNo, f );
            sendCan(sensorSocket(h, HUBO_IMU1), f);
            break;
        case D_IMU_SENSOR_2:
            fNullIMUSensor( h->sensor[HUBO_IMU2].boardNo, f );
            sendCan(sensorSocket(h, HUBO_IMU2), f);
            break;
        default:
            fprintf(stderr, "Invalid parameter for nulling IMU Sensor: %d\n\t"
                    "Must be D_IMU_SENSOR_0 (%d), D_IMU_SENSOR_1 (%d),\n\t"
                    "        D_IMU_SENSOR_2 (%d)\n",
                    (int)board,
                    (int)D_IMU_SENSOR_0, (int)D_IMU_SENSOR_1,
                    (int)D_IMU_SENSOR_2 );
            break;
    }

}

void hNullAllIMUSensors( hubo_param_t *h, struct can_frame *f )
{
    hNullIMUSensor( D_IMU_SENSOR_0, h, f );
    hNullIMUSensor( D_IMU_SENSOR_1, h, f );
    hNullIMUSensor( D_IMU_SENSOR_2, h, f );
}

void fInitAccFTSensor( int bno, struct can_frame *f )
{
    f->can_id    = CMD_TXDF;

    f->data[0]    = (uint8_t)bno + BNO_SENSOR_BASE;
    f->data[1]    = H_INIT_BOARD;
    f->data[2]    = H_INIT_DEFAULT_2;

    f->can_dlc = 3;
}

void hInitAccFTSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f )
{
    switch (board)
    {
        case D_R_FOOT_FT:
        case D_R_FOOT_ACC:
            fInitAccFTSensor( h->sensor[HUBO_FT_R_FOOT].boardNo, f );
            sendCan(hubo_socket[h->sensor[HUBO_FT_R_FOOT].can], f);
            break;
        case D_L_FOOT_FT:
        case D_L_FOOT_ACC:
            fInitAccFTSensor( h->sensor[HUBO_FT_L_FOOT].boardNo, f );
            sendCan(hubo_socket[h->sensor[HUBO_FT_L_FOOT].can], f);
            break;
        case D_R_HAND_FT:
            fInitAccFTSensor( h->sensor[HUBO_FT_R_HAND].boardNo, f );
            sendCan(hubo_socket[h->sensor[HUBO_FT_R_HAND].can], f);
            break;
        case D_L_HAND_FT:
            fInitAccFTSensor( h->sensor[HUBO_FT_L_HAND].boardNo, f );
            sendCan(hubo_socket[h->sensor[HUBO_FT_L_HAND].can], f);
            break;
        default:
            fprintf(stderr, "Invalid parameter for nulling FT Sensor: %d\n\t"
                    "Must be D_R_FOOT_FT (%d), D_L_FOOT_FT(%d),\n\t"
                    "        D_R_HAND_FT (%d), D_L_HAND_FT(%d)\n\t"
                    "        D_R_FOOT_ACC (%d), D_L_FOOT_ACC (%d)\n\t",
                    (int)board,
                    (int)D_R_FOOT_FT, (int)D_L_FOOT_FT,
                    (int)D_R_HAND_FT, (int)D_L_HAND_FT,
                    (int)D_R_FOOT_ACC, (int)D_L_FOOT_ACC );
            break;
    }
}

void hInitAllAccFTSensors( hubo_param_t *h, struct can_frame *f )
{
    hInitAccFTSensor( D_R_FOOT_FT, h, f );
    hInitAccFTSensor( D_L_FOOT_FT, h, f );
    hInitAccFTSensor( D_R_HAND_FT, h, f );
    hInitAccFTSensor( D_L_HAND_FT, h, f );
}

void fNullAccFTSensor( int bno, int nullType, struct can_frame *f )
{
    f->can_id    = CMD_TXDF;

    f->data[0]    = (uint8_t)bno + BNO_SENSOR_BASE;
    f->data[1]    = H_REQ_NULL;
    f->data[2]    = (uint8_t)nullType;

    f->can_dlc = 3;
}

void hNullSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f )
{
    switch(board)
    {
        case D_R_FOOT_FT:
        case D_L_FOOT_FT:
        case D_R_HAND_FT:
        case D_L_HAND_FT:
            hNullFTSensor( board, h, f );
            break;
        case D_R_FOOT_ACC:
        case D_L_FOOT_ACC:
            hNullAccSensor( board, h, f );
            break;
        case D_IMU_SENSOR_0:
        case D_IMU_SENSOR_1:
        case D_IMU_SENSOR_2:
            hNullIMUSensor( board, h, f );
            break;
        default:
            fprintf(stderr, "Invalid parameter for nulling sensor: %d\n\t"
                    "Must be D_R_FOOT_FT (%d), D_L_FOOT_FT(%d),\n\t"
                    "        D_R_HAND_FT (%d), D_L_HAND_FT(%d),\n\t"
                    "        D_R_FOOT_ACC (%d), or D_L_FOOT_ACC(%d),\n\t"
                    "        D_IMU_SENSOR_0 (%d), D_IMU_SENSOR_1 (%d),\n\t"
                    "        or D_IMU_SENSOR_2 (%d)\n\t",
                    (int)board,
                    (int)D_R_FOOT_FT, (int)D_L_FOOT_FT,
                    (int)D_R_HAND_FT, (int)D_L_HAND_FT,
                    (int)D_R_FOOT_ACC, (int)D_L_FOOT_ACC, 
                    (int)D_IMU_SENSOR_0, (int)D_IMU_SENSOR_1,
                    (int)D_IMU_SENSOR_2 );
            break;
    }

}

void hNullFTSensor( hubo_d_param_t board, hubo_param_t *h, struct can_frame *f )
{
    switch (board)
    {
        case D_R_FOOT_FT:
            fNullAccFTSensor(h->sensor[HUBO_FT_R_FOOT].boardNo, H_NULL_FT, f);
            sendCan(sensorSocket(h, HUBO_FT_R_FOOT), f);
            break;
        case D_L_FOOT_FT:
            fNullAccFTSensor(h->sensor[HUBO_FT_L_FOOT].boardNo,  H_NULL_FT, f);
            sendCan(sensorSocket(h, HUBO_FT_L_FOOT), f);
            break;
        case D_R_HAND_FT:
            fNullAccFTSensor(h->sensor[HUBO_FT_R_HAND].boardNo, H_NULL_FT, f);
            sendCan(sensorSocket(h, HUBO_FT_R_HAND), f);
            break;
        case D_L_HAND_FT:
            fNullAccFTSensor(h->sensor[HUBO_FT_L_HAND].boardNo,  H_NULL_FT, f);
            sendCan(sensorSocket(h, HUBO_FT_L_HAND), f);
            break;
        default:
            fprintf(stderr, "Invalid parameter for nulling FT Sensor: %d\n\t"
                    "Must be D_R_FOOT_FT (%d), D_L_FOOT_FT(%d),\n\t"
                    "        D_R_HAND_FT (%d), D_L_HAND_FT(%d)\n\t",
                    (int)board,
                    (int)D_R_FOOT_FT, (int)D_L_FOOT_FT,
                   (int)D_R_HAND_FT, (int)D_L_HAND_FT );
            break;
    }
}

void hNullAccSensor(hubo_d_param_t board, hubo_param_t *h, struct can_frame *f)
{
    switch (board)
    {
        case D_R_FOOT_ACC:
            fNullAccFTSensor(h->sensor[HUBO_FT_R_FOOT].boardNo, H_NULL_ACC, f);
            sendCan(sensorSocket(h, HUBO_FT_R_FOOT), f);
            break;
        case D_L_FOOT_ACC:
            fNullAccFTSensor(h->sensor[HUBO_FT_L_FOOT].boardNo,  H_NULL_ACC, f);
            sendCan(sensorSocket(h, HUBO_FT_L_FOOT), f);
            break;
        default:
            fprintf(stderr, "Invalid parameter for nulling Tilt Sensor: %d\n\t"
                    "Must be D_R_FOOT_ACC (%d) or D_L_FOOT_ACC(%d)\n\t",
                    (int)board,
                    (int)D_R_FOOT_ACC, (int)D_L_FOOT_ACC );  
            break;
    }
}

void hNullAllFTSensors(hubo_param_t *h, struct can_frame *f)
{
    hNullFTSensor(D_R_FOOT_FT, h, f);
    hNullFTSensor(D_L_FOOT_FT, h, f);
    hNullFTSensor(D_R_HAND_FT, h, f);
    hNullFTSensor(D_L_HAND_FT, h, f);
}

void hNullAllAccSensors(hubo_param_t *h, struct can_frame *f)
{
    hNullAccSensor(D_R_FOOT_ACC, h, f);
    hNullAccSensor(D_L_FOOT_ACC, h, f);
}

void hNullAllSensors( hubo_param_t *h, struct can_frame *f )
{
    hNullAllAccSensors( h, f );
    hNullAllFTSensors( h, f );
    hNullAllIMUSensors( h, f );
}

void hInitAllSensors( hubo_param_t *h, struct can_frame *f )
{
    hInitAllAccFTSensors( h, f );
    hNullAllSensors( h, f );
}

void fGetBoardParamA( int jnt, int offset, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_A + offset);
    
    f->can_dlc    = 3;
}

void fGetBoardParamB( int jnt, int offset, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_B + offset);
    
    f->can_dlc    = 3;
}

void fGetBoardParamC( int jnt, int offset, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_C + offset);
    
    f->can_dlc    = 3;
}

void fGetBoardParamD( int jnt, int offset, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_D + offset);
    
    f->can_dlc    = 3;
}

void fGetBoardParamE( int jnt, int offset, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_E + offset);
    
    f->can_dlc    = 3;
}

void fGetBoardParamF( int jnt, int offset, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_F + offset);
    
    f->can_dlc    = 3;
}

void fGetBoardParamG( int jnt, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = H_GET_PARAM_G;
    
    f->can_dlc    = 3;
}

void fGetBoardParamH( int jnt, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = H_GET_PARAM_H;
    
    f->can_dlc    = 3;
}

void fGetBoardParamI( int jnt, hubo_param_t *h, struct can_frame *f )
{
    f->can_id     = CMD_TXDF;

    f->data[0]     = getJMC(h,jnt);
    f->data[1]     = H_REQ_PARAMS;
    f->data[2]     = H_GET_PARAM_I;
    
    f->can_dlc    = 3;
}

void hGetAllBoardParams( hubo_param_t *h, hubo_state_t *s, struct can_frame *f )
{
    hubo_board_param_t b;
    memset( &b, 0, sizeof(b));
    struct timespec t;
    clock_gettime( CLOCK_MONOTONIC, &t);
    int i=0;
    for(i=0; i<HUBO_JOINT_COUNT; i++)
    {
        if(s->joint[i].active == 1)
            hGetBoardParams( i, 0, h, &b, f, 0 );

        t.tv_nsec+=NSEC_PER_SEC*0.01;
        tsnorm(&t);

        clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);
    }


    ach_put(&chan_hubo_board_param, &b, sizeof(b));
}

void hGetBoardParams( int jnt, hubo_d_param_t param, hubo_param_t *h, // TODO: Remove hubo_d_param_t
                        hubo_board_param_t *b, struct can_frame *f, int send )
{
    double giveUp = 1;
    if( flushCan( hubo_socket[h->joint[jnt].can],
            HUBO_CAN_TIMEOUT_DEFAULT, giveUp ) > 0 )
    {
        fprintf(stderr, "Could not flush the CAN within %f sec!"
                        "\n -- We will not read the parameters for joint %d\n",
                        giveUp, jnt);
        return;
    }

    int offset = 0;
    if(h->joint[jnt].motNo >= 3)
        offset = 5;

    char bytes = 0;

// Dan put in headder and change to 0 for virtual
    double timeoutScale = HUBO_BOARD_PARAM_CHECK_TIMEOUT;

    // TODO: Make a loop here instead of this stupidity
    fGetBoardParamA( jnt, offset, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    bytes += decodeParamFrame(jnt, b, h, f, 1);

    fGetBoardParamB( jnt, offset, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    bytes += decodeParamFrame(jnt, b, h, f, 2);

    fGetBoardParamC( jnt, offset, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    bytes += decodeParamFrame(jnt, b, h, f, 3);

    fGetBoardParamD( jnt, offset, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    bytes += decodeParamFrame(jnt, b, h, f, 4);

    fGetBoardParamE( jnt, offset, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    bytes += decodeParamFrame(jnt, b, h, f, 5);

    fGetBoardParamF( jnt, offset, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    bytes += decodeParamFrame(jnt, b, h, f, 6);

    if( bytes > 0 )
        b->joint[jnt].confidence = 0;
    else if( bytes == 0 )
        b->joint[jnt].confidence = 1;

    // Note: These next three are redundant because they are params that are
    // shared among all the joints on a board
    fGetBoardParamG( jnt, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    decodeParamFrame(jnt, b, h, f, 7);

    fGetBoardParamH( jnt, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    decodeParamFrame(jnt, b, h, f, 8);

    fGetBoardParamI( jnt, h, f );
    sendCan(getSocket(h,jnt), f);
    readCan(hubo_socket[h->joint[jnt].can], f, timeoutScale*HUBO_CAN_TIMEOUT_DEFAULT);
    decodeParamFrame(jnt, b, h, f, 9);

    b->joint[jnt].homeOffset   = enc2rad(jnt, b->joint[jnt].homeOffsetRaw, h);
    b->joint[jnt].maxHomeAccel = enc2rad(jnt, b->joint[jnt].maxHomeAccelRaw/100, h);
    b->joint[jnt].maxHomeLimitVel = enc2rad(jnt, b->joint[jnt].maxHomeLimitVelRaw, h);
    b->joint[jnt].maxHomeOffsetVel = enc2rad(jnt, b->joint[jnt].maxHomeOffsetVelRaw, h);
    b->joint[jnt].lowerLimit   = enc2rad(jnt, b->joint[jnt].lowerLimitRaw, h);
    b->joint[jnt].upperLimit   = enc2rad(jnt, b->joint[jnt].upperLimitRaw, h);
    b->joint[jnt].maxAccel     = enc2rad(jnt, b->joint[jnt].maxAccelRaw, h);
    b->joint[jnt].maxVel       = enc2rad(jnt, b->joint[jnt].maxVel, h);
    b->joint[jnt].jamTime      = 0.1*b->joint[jnt].jamTimeRaw;
    b->joint[jnt].pwmSaturationTime = 0.1*b->joint[jnt].pwmSaturationTimeRaw;

    //FIXME: Figure out how the parity checking is actually supposed to work
//    if(bytes&0xFF != 0)
//        fprintf(stdout, "Failed to read frames for joint %d correctly!\n", jnt);

    if(send==1)
        ach_put(&chan_hubo_board_param, b, sizeof(*b));
}

char decodeParamFrame(int num, hubo_board_param_t *b, hubo_param_t *h, struct can_frame *f, int type)
{
    int fs = (int)f->can_id;

    if( (fs >= H_CURRENT_BASE_RXDF) && (fs < H_CURRENT_MAX_RXDF) ) 
    {
//        int num = fs - H_CURRENT_BASE_RXDF;
        switch( type )
        {
            case 1:
                b->joint[num].Kp                = f->data[0] | (f->data[1]<<8);
                b->joint[num].Ki                = f->data[2] | (f->data[3]<<8);
                b->joint[num].Kd                = f->data[4] | (f->data[5]<<8);
                b->joint[num].encoderResolution = f->data[6] | ((f->data[7]&0x3F)<<8);
                b->joint[num].autoScale         = (f->data[7]>>6) & 0x01;
                b->joint[num].motorDirection    = (f->data[7]>>7) & 0x01;
//                b->joint[num].motorDirection    = (f->data[7]>>6) & 0x01;
//                b->joint[num].autoScale         = (f->data[7]>>7) & 0x01;
                break;
            case 2:
                b->joint[num].deadZone          = f->data[0] | (f->data[1]<<8);
                b->joint[num].searchDirection   = f->data[2];
                b->joint[num].searchMode        = f->data[3];
//                b->joint[num].searchLimitRaw    = f->data[4] | (f->data[5]<<8);
                b->joint[num].searchLimit       = f->data[4] | (f->data[5]<<8);
                b->joint[num].homeOffsetRaw     = f->data[6] | (f->data[7]<<8);
                break;
            case 3:
                b->joint[num].homeOffsetRaw     = b->joint[num].homeOffsetRaw | (f->data[0]<<16) | (f->data[1]<<24);
                b->joint[num].lowerLimitRaw     = f->data[2] | (f->data[3]<<8)
                                            | (f->data[4]<<16) | (f->data[5]<<24);
                b->joint[num].upperLimitRaw     = f->data[6] | (f->data[7]<<8);
                break;
            case 4:
                b->joint[num].upperLimitRaw     = b->joint[num].upperLimitRaw | (f->data[0]<<16) | (f->data[1]<<24);
                b->joint[num].maxAccelRaw       = f->data[2] | (f->data[3]<<8);
                b->joint[num].maxVelRaw         = f->data[4] | (f->data[5]<<8);
                b->joint[num].maxPWM            = f->data[6] | (f->data[7]<<8);
                break;
            case 5:
                b->joint[num].maxCurrent= f->data[0] | (f->data[1]<<8);
                // data[2] and data[3] are RSRV (reserved)
                b->joint[num].Kpt       = f->data[4] | (f->data[5]<<8);
                b->joint[num].Kdt       = f->data[6] | (f->data[7]<<8);
                break;
            case 6:
                b->joint[num].Kft       = f->data[0] | (f->data[1]<<8);
                break;
            case 7:
                b->joint[num].canRate   = f->data[2] | (f->data[3]<<8);
                b->joint[num].boardType = f->data[4];
                b->joint[num].maxHomeAccelRaw = f->data[6] | (f->data[7]<<8);
                break;
            case 8:
                b->joint[num].maxHomeLimitVelRaw   = f->data[0] | (f->data[1]<<8);
                b->joint[num].maxHomeOffsetVelRaw  = f->data[2] | (f->data[3]<<8);
                // FIXME: Jam time seems to disagree with Hubo-i. Find out which is correct.
                b->joint[num].jamTimeRaw           = f->data[4] | (f->data[5]<<8);
                b->joint[num].pwmSaturationTimeRaw = f->data[6] | (f->data[7]<<8);
                break;
            case 9:
                b->joint[num].pwmDutyLimit  = f->data[0];
                b->joint[num].pwmDutyJam    = f->data[1];
                b->joint[num].maxInputDifference = f->data[2] | (f->data[3]<<8);
                b->joint[num].maxError      = f->data[4] | (f->data[5]<<8);
                b->joint[num].maxEncError   = f->data[6] | (f->data[7]<<8);
                break;
        }
    }
    else
    {
        fprintf(stderr, "Missed a parameter frame for %s:%d! Trust nothing in hubo_board_param!\n",
                        jointNames[num], type);
        return 1;
    }

    // FIXME: Figure out how the parity checking is actually supposed to work
    char return_val=0;
/*
    int i=0;
    for(i=0; i<8; i++)
        return_val += f->data[i];
*/

    return return_val;
}

void huboMessage(hubo_ref_t *r, hubo_ref_t *r_filt, hubo_param_t *h,
                hubo_state_t *s, hubo_board_cmd_t *c, struct can_frame *f)
{

    size_t fs;
    int status = 0;
    int i = 0;
    while ( status == 0 | status == ACH_OK | status == ACH_MISSED_FRAME ) {
    
        status = ach_get( &chan_hubo_board_cmd, c, sizeof(*c), &fs, NULL, 0 );
        if( status == ACH_STALE_FRAMES) {
            break; }
        else {
        
            switch (c->type)
            {
                case D_GET_STATUS:
                     hGetBoardStatus(c->joint, s, h, f);
                     break;
                case D_JMC_INITIALIZE_ALL:
                    hInitializeBoardAll( h, s, f );
                    break;
                case D_JMC_INITIALIZE:
                    hInitializeBoard( c->joint, h, f );
                    break;
                case D_FET_SWITCH:
                    hMotorDriverOnOff( c->joint, h, f, c->param[0] );
                    break;
                case D_CTRL_SWITCH:
                    hFeedbackControllerOnOff( c->joint, r, s, h, f, c->param[0] );
                    break;
                case D_ZERO_ENCODER:
                    hResetEncoderToZero( c->joint, r, h, s, f );
                    break;
                case D_JMC_BEEP:
                    hSetBeep( c->joint, h, f, c->dValues[0] );
                    break;
                case D_GOTO_HOME_ALL:
                    hGotoLimitAndGoOffsetAll( r, r_filt, h, s, f );
                    break;
                case D_GOTO_HOME:
                    hGotoLimitAndGoOffset( c->joint, r, r_filt, h, s, f, 1 );
                    break;
                case D_JMC_ALARM:
                    hSetAlarm( c->joint, h, f, c->param[0] );
                    break;
                case D_SET_POS_GAIN:
// Not tested DML 2013-02-07                    hSetPosGain( c, h, f );
                    break;
                case D_SET_CUR_GAIN:
// Not tested DML 2013-02-07                    hSetCurGain( c, h, f );
                    break;
                case D_OPENLOOP_PWM:
// Not tested DML 2013-02-07                    hOpenLoopPWM( c, h, f );
                    break;
                case D_CTRL_ON_OFF:
                
                    if(true == s->joint[c->joint].active & HUBO_HOME_OK == s->status[c->joint].homeFlag) 
                       {hFeedbackControllerOnOff(c->joint,r,s,h,f,c->param[0]);}
                    else if(RWR == c->joint | LWR == c->joint) {
                       if(true == s->joint[c->joint].active & HUBO_HOME_OK_WRIST == s->status[c->joint].homeFlag) 
                          {hFeedbackControllerOnOff(c->joint,r,s,h,f,c->param[0]);}
                    }
                    //if( 0 < s->joint[c->joint].zeroed) {hFeedbackControllerOnOff(c->joint,r,s,h,f,c->param[0]);}
                    break;
                case D_CTRL_ON_OFF_ALL:
                   for(i = 0; i < HUBO_JOINT_COUNT; i++) {
                       if(true == s->joint[i].active & HUBO_HOME_OK == s->status[i].homeFlag ) 
                           {hFeedbackControllerOnOff(i,r,s,h,f,c->param[0]);}
                       else if(RWR == c->joint | LWR == c->joint) {
                          if(true == s->joint[i].active & HUBO_HOME_OK_WRIST == s->status[i].homeFlag ) 
                            {hFeedbackControllerOnOff(i,r,s,h,f,c->param[0]);}
                          }
                       }
                   break;
                case D_CTRL_ON:
                    hFeedbackControllerOnOff( c->joint, r, s, h, f, D_ENABLE);
                    break;
                case D_CTRL_OFF:
                    hFeedbackControllerOnOff( c->joint, r, s, h, f, D_DISABLE);
                    break;
                case D_CTRL_MODE:
// Not tested DML 2013-02-07                    hSetControlMode( c->joint, h, s, f, c->param[0] );
                    break;
                case D_SET_DEAD_ZONE:
// Not tested DML 2013-02-07                    hSetDeadZone( c->joint, h, f, c->iValues[0] );
                    break;
                case D_SET_HOME_PARAMS:
                    hSetHomeSearchParams( c, h, f );
                    break;
                case D_SET_HOME_PARAMS_RAW:
                    hSetHomeSearchParamsRaw( c, h, f );
                    break;
                case D_SET_ENC_RESOLUTION:
// Not tested DML 2013-02-07                    hSetEncoderResolution( c, h, f );
                    break;
                case D_SET_MAX_ACC_VEL:
// Not tested DML 2013-02-07                    hSetMaxAccVel( c->joint, h, f, c->iValues[0], c->iValues[1] );
                    break;
                case D_SET_LOW_POS_LIM:
                    hSetLowerPosLimit( c, h, f );
                    break;
                case D_SET_LOW_POS_LIM_RAW:
                    hSetLowerPosLimitRaw( c, h, f );
                    break;
                case D_SET_UPP_POS_LIM:
                    hSetUpperPosLimit( c, h, f );
                    break;
                case D_SET_UPP_POS_LIM_RAW:
                    hSetUpperPosLimitRaw( c, h, f );
                    break;
                case D_SET_HOME_VEL_ACC:
// Not tested DML 2013-02-07                    hSetHomeAccVel( c, h, f);
                    break;
                case D_SET_GAIN_SCALE:
// Not tested DML 2013-02-07                    hSetGainOverride(c->joint, h, f, c->iValues[0],
// Not tested DML 2013-02-07                            c->iValues[1], c->dValues[0]); break;
                case D_SET_BOARD_NUM:
// Not tested DML 2013-02-07 - This should NOT be used when multiple controllers are connected                    hSetBoardNumber(c->joint, h, f, c->iValues[0], c->iValues[1]);
// Not tested DML 2013-02-07                    break;
                case D_SET_JAM_SAT_LIMIT:
// Not tested DML 2013-02-07                    hSetJamPwmLimits( c, h, f );
                    break;
                case D_SET_ERR_BOUND:
// Not tested DML 2013-02-07                    hSetErrorBound( c->joint, h, f, c->iValues[0], c->iValues[1],
// Not tested DML 2013-02-07                            c->iValues[2] ); break;
                case D_NULL_SENSOR:
                    hNullSensor( c->param[0], h, f );
                    break;
                case D_NULL_SENSORS_ALL:
                    hNullAllSensors( h, f );
                    break;
                case D_NULL_FT_SENSOR:
                    hNullFTSensor( c->param[0], h, f );
                    break;
                case D_NULL_ACC_SENSOR:
                    hNullAccSensor( c->param[0], h, f );
                    break;
                case D_NULL_FT_SENSOR_ALL:
                    hNullAllFTSensors( h, f );
                    break;
                case D_NULL_ACC_SENSOR_ALL:
                    hNullAllAccSensors( h, f );
                    break;
                case D_NULL_FT_ACC_SENSOR_ALL:
                    hNullAllFTSensors( h, f ); hNullAllAccSensors( h, f );
                    break;
                case D_NULL_IMU_SENSOR:
                    hNullIMUSensor( c->param[0], h, f );
                    break;
                case D_NULL_IMU_SENSOR_ALL:
                    hNullAllIMUSensors( h, f );
                    break;
                case D_INIT_FT_ACC_SENSOR:
                    hInitAccFTSensor( c->param[0], h, f );
                    break;
                case D_INIT_FT_ACC_SENSOR_ALL:
                    hInitAllAccFTSensors( h, f );
                    break;
                case D_SENSOR_STARTUP:
                    hInitAllSensors( h, f );
                    break;
/*              // This should be decided by the ref command, not by an initialization command
                case D_COMP_MODE_ON_OFF:
                    hSetComplementaryMode(c->joint,h,f,c->param[0]);
                    break;
*/
                case D_GET_BOARD_PARAMS_ALL:
                    hGetAllBoardParams( h, s, f );
                    break;
//                case D_GET_BOARD_PARAMS:
//                    hGetBoardParams( c->joint, c->param[0], h, s, f ); // TODO: Do this.
//                    break;
                case 0:
                    break;
                default:
                    fprintf(stderr,"Unrecognized command type: %d\n\t",c->type);
                    break;
            }
        }
    }
}


double enc2rad(int jnt, int enc, hubo_param_t *h) {
    hubo_joint_param_t *p = &h->joint[jnt];
        return (double)(enc*(double)p->drive/(double)p->driven/(double)p->harmonic/(double)p->enc*2.0*M_PI);
}

double enc2radNkDrc(int jnt, int enc, hubo_param_t *h) {
    hubo_joint_param_t *p = &h->joint[jnt];
        return (double)( ((double)enc/4096.0*2.0*M_PI) - M_PI);
}

double doubleFromBytePair(uint8_t data0, uint8_t data1){
	unsigned int tmp = 0;

	tmp |= (( ((uint16_t)data0) << 8 ) & 0xFFFF); 
	tmp |= (( ((uint16_t)data1)  ) & 0x00FF); 
	return (double) (int16_t)tmp;
}



void decodeFTFrame(int num, struct hubo_state *s, struct hubo_param *h, struct can_frame *f){

    double Mx = doubleFromBytePair(f->data[1],f->data[0])/100.0;		// moment in Nm
    double My = doubleFromBytePair(f->data[3],f->data[2])/100.0;		// moment in Nm
    double Fz = doubleFromBytePair(f->data[5],f->data[4])/10.0;		// moment in Nm
    s->ft[num].m_x = Mx*h->sensor[num].xsign;
    s->ft[num].m_y = My*h->sensor[num].ysign;
    s->ft[num].f_z = Fz*h->sensor[num].zsign;
}


void decodeADFrame(int num, struct hubo_state *s, struct can_frame *f){

    double Ax = doubleFromBytePair(f->data[1],f->data[0])/100.0;		
    double Ay = doubleFromBytePair(f->data[3],f->data[2])/100.0;		
    double Az = doubleFromBytePair(f->data[5],f->data[4])/100.0;		
    s->imu[num].a_x = Ax;
    s->imu[num].a_y = Ay;
    s->imu[num].a_z = Az;
}

void decodeIMUFrame(int num, struct hubo_state *s, struct can_frame *f){

    double Ra = doubleFromBytePair(f->data[1],f->data[0])/100.0;		
    double Pa = doubleFromBytePair(f->data[3],f->data[2])/100.0;		
    double Rr = doubleFromBytePair(f->data[5],f->data[4])/100.0;		
    double Pr = doubleFromBytePair(f->data[5],f->data[6])/100.0;
    //TODO: Check that "Roll" and "Pitch" names make sense
    s->imu[num].a_x = Ra;
    s->imu[num].a_y = Pa;
    s->imu[num].w_x = Rr;
    s->imu[num].w_y = Pr;
}








int decodeFrame(hubo_state_t *s, hubo_param_t *h, struct can_frame *f) {
    int fs = (int)f->can_id;

    if (fs == H_ENC_BASE_RXDF + 0xE)
    {
        // Voltage, Current Return Message
        double voltage = doubleFromBytePair(f->data[1],f->data[0])/100.0;
        double current = doubleFromBytePair(f->data[3],f->data[2])/100.0;
        double power   = doubleFromBytePair(f->data[5],f->data[4])/10.0;
        s->power.voltage = voltage;
        s->power.current = current;
        s->power.power = power;	
    }
    /* Force-Torque Readings */
    else if( (fs >= H_SENSOR_FT_BASE_RXDF) && (fs <= H_SENSOR_FT_MAX_RXDF) )
    {
        int num = fs - H_SENSOR_FT_BASE_RXDF;
        int16_t val;
        if(num==h->sensor[HUBO_FT_R_FOOT].boardNo)
        {
            int num2 = HUBO_FT_R_FOOT;
            decodeFTFrame(num2,s,h,f);
/*
            val = (f->data[1]<<8) | f->data[0];
            s->ft[HUBO_FT_R_FOOT].m_x = ((double)(val))/100.0;
        
            val = (f->data[3]<<8) | f->data[2];
            s->ft[HUBO_FT_R_FOOT].m_y = ((double)(val))/100.0;

            val = (f->data[5]<<8) | f->data[4];
            s->ft[HUBO_FT_R_FOOT].f_z = ((double)(val))/10.0;
*/
        }
        else if(num==h->sensor[HUBO_FT_L_FOOT].boardNo)
        {
            int num2 = HUBO_FT_L_FOOT;
            decodeFTFrame(num2,s,h,f);

/*
            val =  (f->data[1]<<8) | f->data[0];
            s->ft[HUBO_FT_L_FOOT].m_x = ((double)(val))/100.0;
            
            val =  (f->data[3]<<8) | f->data[2];
            s->ft[HUBO_FT_L_FOOT].m_y = ((double)(val))/100.0;

            val =  (f->data[5]<<8) | f->data[4];
            s->ft[HUBO_FT_L_FOOT].f_z = ((double)(val))/10.0;
*/

        }
        else if(num==h->sensor[HUBO_FT_R_HAND].boardNo)
        {
            int num2 = HUBO_FT_R_HAND;
            decodeFTFrame(num2,s,h,f);

/*
            val =  (f->data[1]<<8) | f->data[0];
            s->ft[HUBO_FT_R_HAND].m_x = ((double)(val))/100.0;
            
            val =  (f->data[3]<<8) | f->data[2];
            s->ft[HUBO_FT_R_HAND].m_y = ((double)(val))/100.0;

            val =  (f->data[5]<<8) | f->data[4];
            s->ft[HUBO_FT_R_HAND].f_z = ((double)(val))/10.0;
*/

/*
            val =  (f->data[1]<<8) | f->data[0];
            s->ft[HUBO_FT_R_HAND].m_x = ((double)(val))/100.0;
            
            val =  (f->data[3]<<8) | f->data[2];
            s->ft[HUBO_FT_R_HAND].m_y = ((double)(val))/100.0;

            s->ft[HUBO_FT_R_HAND].f_z = 0;
*/
            //val = (f->data[5]<<8) | f->data[4]; // This does not exist
            //s->ft[HUBO_FT_R_HAND].f_z = ((double)(val))/10.0;
        }
        else if(num==h->sensor[HUBO_FT_L_HAND].boardNo)
        {
            int num2 = HUBO_FT_L_HAND;
            decodeFTFrame(num2,s,h,f);

/*
            val =  (f->data[1]<<8) | f->data[0];
            s->ft[HUBO_FT_L_HAND].m_x = ((double)(val))/100.0;
            
            val =  (f->data[3]<<8) | f->data[2];
            s->ft[HUBO_FT_L_HAND].m_y = ((double)(val))/100.0;

            val =  (f->data[5]<<8) | f->data[4];
            s->ft[HUBO_FT_L_HAND].f_z = ((double)(val))/10.0;
*/
/*
            val =  (f->data[1]<<8) | f->data[0];
            s->ft[HUBO_FT_L_HAND].m_x = ((double)(val))/100.0;
            
            val =  (f->data[3]<<8) | f->data[2];
            s->ft[HUBO_FT_L_HAND].m_y = ((double)(val))/100.0;

            s->ft[HUBO_FT_L_HAND].f_z = 0;
*/
            //val =  (f->data[5]<<8) | f->data[4]; // This does not exist
            //s->ft[HUBO_FT_L_HAND].f_z = ((double)(val))/10.0;
        }
        else
            fprintf(stderr, "Invalid value for FT Sensor: %d\n\t"
                    "Must be 1, 2, 6, or 7\n", num);
    }
    /* IMU Readings */
    else if( (fs >= H_SENSOR_IMU_BASE_RXDF) && (fs <= H_SENSOR_IMU_MAX_RXDF) )
    {
        int num = fs - H_SENSOR_IMU_BASE_RXDF;
        int16_t val;
        
        if(num==h->sensor[HUBO_FT_R_FOOT].boardNo)
        {
            val = (f->data[1]<<8) | f->data[0];
            s->imu[TILT_R].a_x = ((double)(val))/100.0 * M_PI/180.0;
            
            val = (f->data[3]<<8) | f->data[2];
            s->imu[TILT_R].a_y = ((double)(val))/100.0 * M_PI/180.0;

            val = (f->data[5]<<8) | f->data[4];
            s->imu[TILT_R].a_z = ((double)(val))/750.0 * M_PI/180.0;
        }
        else if(num==h->sensor[HUBO_FT_L_FOOT].boardNo)
        {
            val = (f->data[1]<<8) | f->data[0];
            s->imu[TILT_L].a_x = ((double)(val))/100.0 * M_PI/180.0;
            
            val = (f->data[3]<<8) | f->data[2];
            s->imu[TILT_L].a_y = ((double)(val))/100.0 * M_PI/180.0;

            val = (f->data[5]<<8) | f->data[4];
            s->imu[TILT_L].a_z = ((double)(val))/750.0 * M_PI/180.0;
        }   
        else if(num==h->sensor[HUBO_IMU0].boardNo ||
                num==h->sensor[HUBO_IMU1].boardNo ||
                num==h->sensor[HUBO_IMU2].boardNo)
        {
            val = (f->data[1]<<8) | f->data[0];
            s->imu[IMU].a_x = ((double)(val))/100.0 * M_PI/180.0;

            val = (f->data[3]<<8) | f->data[2];
            s->imu[IMU].a_y = ((double)(val))/100.0 * M_PI/180.0;
            
            val = (f->data[5]<<8) | f->data[4];
            s->imu[IMU].w_x = ((double)(val))/100.0 * M_PI/180.0;

            val = (f->data[7]<<8) | f->data[6];
            s->imu[IMU].w_y = ((double)(val))/100.0 * M_PI/180.0;
        }
        else
            fprintf(stderr, "Invalid value for IMU sensor: %d\n\t"
                    "Must range from 1 to 5\n", num);
    }

    /* Current and temperature readings */
    else if( (fs >= H_CURRENT_BASE_RXDF) && (fs < H_CURRENT_MAX_RXDF) ) 
    {
        int jmc = fs - H_CURRENT_BASE_RXDF;    // Find the jmc value
        int jnt0 = h->driver[jmc].joints[0];    // First joint number
        int numMot = h->joint[jnt0].numMot;    // Number of motors
        double current = 0;            // Initialize motor current variable
        double temp = 0;            // Initialize temperature variable
        if( numMot == 1 || numMot == 2 )
        {
            int i;
            for(i = 0; i < numMot; i++)
            {
                int jnt = h->driver[jmc].joints[i];
                s->joint[jnt].cur = f->data[0+i*1]/100.0;
		int cur0 = (f->data[0+i]<<2) | ( (f->data[3] >> (i*2))  & 0x03 );
		float cur0f = (float)cur0/100.0f;
		s->joint[jnt].cur = (double)cur0f;

            }
            s->driver[jmc].temp = f->data[2]; // TODO: Check if this is correct. I changed "3" to "2"
            //temp = temp/100.0; // I don't see anywhere in the docs that it says to do this
        }
        else if( numMot == 3 )
        {
            int jnt;

            jnt = h->driver[jmc].joints[0];
            s->joint[jnt].cur = f->data[0]/250.0; // The 250 scale comes from the documentation

            jnt = h->driver[jmc].joints[1];
            s->joint[jnt].cur = f->data[1]/250.0;

            jnt = h->driver[jmc].joints[2];
            s->joint[jnt].cur = f->data[3]/250.0;


            s->driver[jmc].temp = f->data[2]; // TODO: Check if this is correct. I changed "3" to "2"
            //temp = temp/100.0; // I don't see anywhere in the docs that it says to do this
        }
        else if( numMot == 5 )
        {
            int jnt;

            jnt = h->driver[jmc].joints[0];
            s->joint[jnt].cur = f->data[0]/1000.0; // The 1000 scale comes from the documentation

            jnt = h->driver[jmc].joints[1];
            s->joint[jnt].cur = f->data[1]/1000.0;

            jnt = h->driver[jmc].joints[2];
            s->joint[jnt].cur = f->data[2]/1000.0;

            jnt = h->driver[jmc].joints[3];
            s->joint[jnt].cur = f->data[4]/1000.0;

            jnt = h->driver[jmc].joints[4];
            s->joint[jnt].cur = f->data[5]/1000.0;

            s->driver[jmc].temp = 0; // 0 indicates that no temperature reading is available
        }
    }



    
    /* Return Motor Position */
    else if( (fs >= H_ENC_BASE_RXDF) && (fs < H_ENC_MAX_RXDF) )
    {
        int jmc = fs - H_ENC_BASE_RXDF;
        int i = 0;
        int jnt0 = h->driver[jmc].joints[0];    // jmc number
        int numMot = h->joint[jnt0].numMot;    // motor number   
        int32_t enc = 0;
        int16_t enc16 = 0;            // encoder value for neck and fingers
        if( numMot == 1 || numMot == 2 )
        {
            for( i = 0; i < numMot; i++ )
            {
                enc = 0;
                enc = (enc << 8) + f->data[3 + i*4];
                enc = (enc << 8) + f->data[2 + i*4];
                enc = (enc << 8) + f->data[1 + i*4];
                enc = (enc << 8) + f->data[0 + i*4];
                int jnt = h->driver[jmc].joints[i];          // motor on the same drive
                double newPos = enc2rad(jnt, enc, h);
                s->joint[jnt].vel = (newPos - s->joint[jnt].pos)/HUBO_LOOP_PERIOD;
                s->joint[jnt].pos = newPos;
            }
        }        
        else if( (numMot == 3) & (hubo_type == HUBO_ROBOT_TYPE_DRC_HUBO) & (jmc == EJMC2) ) {
            for( i = 0; i < numMot; i++ )
            {
                enc16 = 0;
                enc16 = (enc << 8) + f->data[0 + i*2];
                enc16 = (enc << 8) + f->data[1 + i*2];
                int jnt = h->driver[jmc].joints[i];          // motor on the same drive
                int jntInt = (int)((short)((f->data[0+i*2])|(f->data[1+i*2]<<8)));
                double newPos = enc2radNkDrc(jnt, jntInt, h);
                s->joint[jnt].vel = (newPos - s->joint[jnt].pos)/HUBO_LOOP_PERIOD;
                s->joint[jnt].pos = newPos;
            }

	}
        /* DRC Fingers and Writst Yaw 2 */
        else if( (numMot == 3) & (hubo_type == HUBO_ROBOT_TYPE_DRC_HUBO)) {

            for( i = 0; i < numMot; i++ )
            {
                enc16 = 0;
                enc16 = (enc << 8) + f->data[0 + i*2];
                enc16 = (enc << 8) + f->data[1 + i*2];
                int jnt = h->driver[jmc].joints[i];          // motor on the same drive
                int jntInt = (int)((short)((f->data[0+i*2])|(f->data[1+i*2]<<8)));
                double newPos = enc2rad(jnt, jntInt, h);
                s->joint[jnt].vel = (newPos - s->joint[jnt].pos)/HUBO_LOOP_PERIOD;
                s->joint[jnt].pos = newPos;
            }
        }

        else if( numMot == 3 ) // neck
        {
            for( i = 0; i < numMot; i++ )
            {
                enc16 = 0;
                enc16 = (enc << 8) + f->data[1 + i*4];
                enc16 = (enc << 8) + f->data[0 + i*4];
                int jnt = h->driver[jmc].joints[i];          // motor on the same drive
                double newPos = enc2rad(jnt, enc16, h);
                s->joint[jnt].vel = (newPos - s->joint[jnt].pos)/HUBO_LOOP_PERIOD;
                s->joint[jnt].pos = newPos;
            }
        }
            
        else if( numMot == 5 & f->can_dlc == 6 ) {
            for( i = 0; i < 3 ; i++ ) {
                enc16 = 0;
                enc16 = (enc16 << 8) + f->data[1 + i*2];
                enc16 = (enc16 << 8) + f->data[0 + i*2];
                int jnt = h->driver[jmc].joints[i];          // motor on the same drive
//i				enc = 0x8000 & enc16;
                double newPos = enc2rad(jnt, enc16, h);
                s->joint[jnt].vel = (newPos - s->joint[jnt].pos)/HUBO_LOOP_PERIOD;
                s->joint[jnt].pos = newPos;
             }
        }

        else if( numMot == 5 & f->can_dlc == 4 ) {
            for( i = 0; i < 2; i++ ) {
                enc16 = 0;
                enc16 = (enc16 << 8) + f->data[1 + i*2];
                enc16 = (enc16 << 8) + f->data[0 + i*2];
                int jnt = h->driver[jmc].joints[i+3];          // motor on the same drive
                double newPos = enc2rad(jnt, enc16, h);
                s->joint[jnt].vel = (newPos - s->joint[jnt].pos)*HUBO_LOOP_PERIOD;
                s->joint[jnt].pos = newPos;
            }
        }



    
    }
    else if( (fs >= H_STAT_BASE_RXDF) && (fs < H_STAT_MAX_RXDF) )
    {
        int jmc = fs - H_STAT_BASE_RXDF;
        int i = 0;
        int jnt0 = h->driver[jmc].joints[0];    // First joint
        int numMot = h->joint[jnt0].numMot;     // Number of motors

        if( numMot <= 2 )
        {
            int j=0;
            for(j=0; j<numMot; j++)
            {
                jnt0 = h->driver[jmc].joints[j];    // First joint
                s->status[jnt0].driverOn    = (f->data[4*j+0])      & 0x01;
                s->status[jnt0].ctrlOn      = (f->data[4*j+0]>>1)   & 0x01;
                s->status[jnt0].mode        = (f->data[4*j+0]>>2)   & 0x01;
                s->status[jnt0].limitSwitch = (f->data[4*j+0]>>3)   & 0x01;
                s->status[jnt0].homeFlag    = (f->data[4*j+0]>>4)   & 0x0F;

                s->status[jnt0].jam         = (f->data[4*j+1])      & 0x01;
                s->status[jnt0].pwmSaturated= (f->data[4*j+1]>>1)   & 0x01;
                s->status[jnt0].bigError    = (f->data[4*j+1]>>2)   & 0x01;
                s->status[jnt0].encError    = (f->data[4*j+1]>>3)   & 0x01;
                s->status[jnt0].driverFault = (f->data[4*j+1]>>4)   & 0x01;
                s->status[jnt0].motorFail0  = (f->data[4*j+1]>>5)   & 0x01;
                s->status[jnt0].motorFail1  = (f->data[4*j+1]>>6)   & 0x01;

                s->status[jnt0].posMinError = (f->data[4*j+2])      & 0x01;
                s->status[jnt0].posMaxError = (f->data[4*j+2]>>1)   & 0x01;
                s->status[jnt0].velError    = (f->data[4*j+2]>>2)   & 0x01;
                s->status[jnt0].accError    = (f->data[4*j+2]>>3)   & 0x01;
                s->status[jnt0].tempError   = (f->data[4*j+2]>>4)   & 0x01;
            }

        }
        // TODO: Find out what the frames are for the new hybrid 3/2 board type
        if( numMot >=3 )
        {
            int j=0;
            for(j=0; j<numMot; j++)
            {
                jnt0 = h->driver[jmc].joints[j];
                s->status[jnt0].driverOn    = (f->data[j])      & 0x01;
                s->status[jnt0].ctrlOn      = (f->data[j]>>1)   & 0x01;
                s->status[jnt0].mode        = (f->data[j]>>2)   & 0x01;
                s->status[jnt0].limitSwitch = (f->data[j]>>3)   & 0x01;
                s->status[jnt0].jam         = (f->data[j]>>4)   & 0x01;
                s->status[jnt0].pwmSaturated= (f->data[j]>>5)   & 0x01;
                s->status[jnt0].bigError    = (f->data[j]>>6)   & 0x01;
                s->status[jnt0].encError    = (f->data[j]>>7)   & 0x01;
            }
        }
    } 
    return 0;
}

int isError( int jnt, hubo_state_t *s) {
    int v = 0;
    hubo_joint_status_t e = s->status[jnt];
    v = e.jam + 
        e.pwmSaturated + 
        e.bigError + 
        e.encError + 
        e.driverFault + 
        e.posMinError +
        e.posMaxError +
        e.velError + 
        e.accError +
        e.tempError;
    
    if( v > 0 ) return 1;
    else return 0;

}

int main(int argc, char **argv) {

    // Parse user input
    int vflag = HUBO_VIRTUAL_MODE_NONE;
    debug = 0;



    int i = 1;
    while(argc > i)
    {
        if(strcmp(argv[i], "-d") == 0) {
            debug = 1;
        }
        if(strcmp(argv[i], "-v") == 0){
            vflag = HUBO_VIRTUAL_MODE_VIRTUAL;
//            vflag = HUBO_VIRTUAL_MODE_OPENHUBO;
            HUBO_BOARD_PARAM_CHECK_TIMEOUT = 1; // change board timeout to 1ms for initial can check
            HUBO_FLAG_GET_DRC_BOARD_PARAM = OFF;
        }
        if(strcmp(argv[i], "-o") == 0){
            vflag = HUBO_VIRTUAL_MODE_OPENHUBO;
            HUBO_BOARD_PARAM_CHECK_TIMEOUT = 1; // change board timeout to 1ms for initial can check
            HUBO_FLAG_GET_DRC_BOARD_PARAM = OFF;
        }
        if(strcmp(argv[i], "-drc") == 0){
            hubo_type = HUBO_ROBOT_TYPE_DRC_HUBO;
            printf("DRC-Hubo Type \n");
        }
        i++;
    }
    // Daemonize
    hubo_daemonize();
    

    // Initialize Hubo Structs
    hubo_ref_t H_ref;
    hubo_ref_t H_ref_neck;
    hubo_board_cmd_t H_cmd;
    hubo_state_t H_state;
    hubo_param_t H_param;
    hubo_virtual_t H_virtual;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_ref_neck,   0, sizeof(H_ref_neck));
    memset( &H_cmd,  0, sizeof(H_cmd));
    memset( &H_state, 0, sizeof(H_state));
    memset( &H_param, 0, sizeof(H_param));
    memset( &H_virtual, 0, sizeof(H_virtual));


    // open hubo reference
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_ref_neck, HUBO_CHAN_REF_NECK_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_gains, HUBO_CHAN_PWM_GAINS_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // open hubo state
    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // initilize control channel
    r = ach_open(&chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // initialize parameters channel
    r = ach_open(&chan_hubo_board_param, HUBO_CHAN_BOARD_PARAM_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // open to sim chan
    r = ach_open(&chan_hubo_to_sim, HUBO_CHAN_VIRTUAL_TO_SIM_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    // open to sim chan
    r = ach_open(&chan_hubo_from_sim, HUBO_CHAN_VIRTUAL_FROM_SIM_NAME, NULL);
    hubo_assert( ACH_OK == r, __LINE__ );

    openAllCAN( vflag );
    ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));
    ach_put(&chan_hubo_ref_neck, &H_ref_neck, sizeof(H_ref_neck));
    ach_put(&chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd));
//    ach_put(&chan_hubo_state, &H_state, sizeof(H_state));
    ach_put(&chan_hubo_to_sim, &H_virtual, sizeof(H_virtual));

    // run hubo main loop
    huboLoop(&H_param, vflag);

    hubo_daemon_close();
    
    return 0;
}

uint8_t int_to_bytes(int d, int index)
{
    return (uint8_t)( ( d >> ((index-1)*8) ) & 0xFF);
}

uint8_t duty_to_byte(int dir, int duty)
{
    return (uint8_t)(duty | (dir<<8) );
}

uint8_t isHands(int jnt)
{
    if( (jnt == RF1) | (jnt == RF2) | (jnt == RF3) | (jnt == RF4) | (jnt == RF5) |
        (jnt == LF1) | (jnt == LF2) | (jnt == LF3) | (jnt == LF4) | (jnt == LF5) )     
        return 1;
    else
        return 0;
}

