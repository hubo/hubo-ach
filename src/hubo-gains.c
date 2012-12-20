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

/*   ~~~~   Added by M.X. Grey. Auxiliary CAN functions   ~~~~   */
/*
void hSetPosGain(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetPosGain0(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd);
void fSetPosGain1(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd);
void hSetCurGain(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetCurGain0(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd);
void fSetCurGain1(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd);




void hOpenLoopPWM(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fOpenLoopPWM_2CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int duty0, int dir1, int duty1);
void fOpenLoopPWM_3CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int dt0, int dir1, int dt1, int dir2, int dt2);
void fOpenLoopPWM_5CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int dt0, int dir1, int dt1, int dir2, int dt2,
			int dir3, int dt3, int dir4, int dt4);
void hSetControlMode(int jnt, struct hubo_param *h, struct hubo_state *s, struct can_frame *f, hubo_d_param_t mode);
void fSetControlMode(int jnt, struct hubo_param *h, struct can_frame *f, int mode);






void hSetAlarm(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t sound);
void fSetAlarm(int jnt, struct hubo_param *h, struct can_frame *f, int sound);







void hSetDeadZone(int jnt, struct hubo_param *h, struct can_frame *f, int deadzone);
void fSetDeadZone(int jnt, struct hubo_param *h, struct can_frame *f, int deadzone);
void hSetHomeSearchParams( struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f );
void fSetHomeSearchParams(int jnt, struct hubo_param *h, struct can_frame *f, int limit,
				unsigned int dir, unsigned int offset);
void hSetEncoderResolution(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetEncoderResolution(int jnt, struct hubo_param *h, struct can_frame *f, int res);
void hSetMaxAccVel(int jnt, struct hubo_param *h, struct can_frame *f, int maxAcc, int maxVel);
void fSetMaxAccVel(int jnt, struct hubo_param *h, struct can_frame *f, int maxAcc, int maxVel);
void hSetLowerPosLimit(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetLowerPosLimit(int jnt, struct hubo_param *h, struct can_frame *f, int enable, int update, int limit);
void hSetUpperPosLimit(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetUpperPosLimit(int jnt, struct hubo_param *h, struct can_frame *f, int enable, int update, int limit);
void hSetHomeAccVel(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f);
void fSetHomeAccVel(int jnt, struct hubo_param *h, struct can_frame *f, float mAcc, int mVelS,
			int mVelP, int mode, int mDuty);
void hSetGainOverride(int jnt, struct hubo_param *h, struct can_frame *f, int gain0, int gain1, double dur);
void fSetGainOverride(int jnt, struct hubo_param *h, struct can_frame *f, int gain0, int gain1, int duration);
void hSetBoardNumber(int jnt, struct hubo_param *h, struct can_frame *f, int boardNum, int rate);
void fSetBoardNumber(int jnt, struct hubo_param *h, struct can_frame *f, int boardNum, int rate);
void fSetJamPwmLimits(int jnt, struct hubo_param *h, struct can_frame *f, int jamLimit, int pwmLimit,
			int lim_detection_duty, int jam_detection_duty );
void hSetErrorBound(int jnt, struct hubo_param *h, struct can_frame *f, int inputDiffErr, int maxError,
			int tempError);
void fSetErrorBound(int jnt, struct hubo_param *h, struct can_frame *f, int inputDiffErr, int maxError,
			int tempError);



void fGetBoardParamA( int jnt, int offset, struct hubo_param *h, struct can_frame *f );
void fGetBoardParamB( int jnt, int offset, struct hubo_param *h, struct can_frame *f );
void fGetBoardParamC( int jnt, int offset, struct hubo_param *h, struct can_frame *f );
void fGetBoardParamD( int jnt, int offset, struct hubo_param *h, struct can_frame *f );
void fGetBoardParamE( int jnt, int offset, struct hubo_param *h, struct can_frame *f );
void fGetBoardParamF( int jnt, int offset, struct hubo_param *h, struct can_frame *f );
void fGetBoardParamG( int jnt, struct hubo_param *h, struct can_frame *f );
void fGetBoardParamH( int jnt, struct hubo_param *h, struct can_frame *f );
void fGetBoardParamI( int jnt, struct hubo_param *h, struct can_frame *f );
void hGetBoardParams( int jnt, hubo_d_param_t param, struct hubo_param *h, struct hubo_state *s, struct can_frame *f );
*/

uint8_t getJMC( struct hubo_param *h, int jnt ) { return (uint8_t)h->joint[jnt].jmc; }
uint8_t getCAN( struct hubo_param *h, int jnt ) { return h->joint[jnt].can; }
hubo_can_t getSocket( struct hubo_param *h, int jnt ) { return hubo_socket[h->joint[jnt].can]; }


uint8_t int_to_bytes(int d, int index);
uint8_t duty_to_byte(int dir, int duty);



void hSetPosGain(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
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

void fSetPosGain0(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd)
{
	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_POS_GAIN_0;
	f->data[2]	= int_to_bytes(Kp,1);
	f->data[3]	= int_to_bytes(Kp,2);
	f->data[4]	= int_to_bytes(Ki,1);
	f->data[5]	= int_to_bytes(Ki,2);
	f->data[6]	= int_to_bytes(Kd,1);
	f->data[7]	= int_to_bytes(Kd,2);

	f->can_dlc 	= 8;
}
void fSetPosGain1(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd){

	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_POS_GAIN_1;
	f->data[2]	= int_to_bytes(Kp,1);
	f->data[3]	= int_to_bytes(Kp,2);
	f->data[4]	= int_to_bytes(Ki,1);
	f->data[5]	= int_to_bytes(Ki,2);
	f->data[6]	= int_to_bytes(Kd,1);
	f->data[7]	= int_to_bytes(Kd,2);

	f->can_dlc	= 8;
}

void hSetCurGain(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
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

void fSetCurGain0(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd){

	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_CUR_GAIN_0;
	f->data[2]	= int_to_bytes(Kp,1);
	f->data[3]	= int_to_bytes(Kp,2);
	f->data[4]	= int_to_bytes(Ki,1);
	f->data[5]	= int_to_bytes(Ki,2);
	f->data[6]	= int_to_bytes(Kd,1);
	f->data[7]	= int_to_bytes(Kd,2);

	f->can_dlc	= 8;
}

void fSetCurGain1(int jnt, struct hubo_param *h, struct can_frame *f, int Kp, int Ki, int Kd){

	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_CUR_GAIN_1;
	f->data[2]	= int_to_bytes(Kp,1);
	f->data[3]	= int_to_bytes(Kp,2);
	f->data[4]	= int_to_bytes(Ki,1);
	f->data[5]	= int_to_bytes(Ki,2);
	f->data[6]	= int_to_bytes(Kd,1);
	f->data[7]	= int_to_bytes(Kd,2);

	f->can_dlc	= 8;
}




void hOpenLoopPWM(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
{
	int jnt = c->joint;

	if(h->joint[jnt].numMot == 2)
	{
		if(	(c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
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
		if(	(c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
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
		if(	(c->param[0]!=D_CLOCKWISE&&c->param[0]!=D_COUNTERCLOCKWISE) ||
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


void fOpenLoopPWM_2CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int duty0, int dir1, int duty1)
{
	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_OPENLOOP_PWM;
	f->data[2]	= 0x01; // Pulse according to duty cycle
				//	Alternative value is 0x00 which enforces
				//	zero duty in order to stop the motor
	f->data[3]	= (uint8_t)dir0;
	f->data[4]	= (uint8_t)duty0;
	f->data[5]	= (uint8_t)dir1;
	f->data[6]	= (uint8_t)duty1;
	f->data[7]	= H_BLANK;

	f->can_dlc	= 8;
}

void fOpenLoopPWM_3CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int dt0, int dir1, int dt1, int dir2, int dt2)
{
	f->can_id	= CMD_TXDF;

	__u8 data[6];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_OPENLOOP_PWM;
	f->data[2]	= 0x01; // Pulse according to duty cycle
				//	Alternative value is 0x00 which enforces
				//	zero duty in order to stop the motor
	f->data[3]	= duty_to_byte(dir0, dt0);
	f->data[4]	= duty_to_byte(dir1, dt1);
	f->data[5]	= duty_to_byte(dir2, dt2); 

	f->can_dlc	= 6;
}

void fOpenLoopPWM_5CH(int jnt, struct hubo_param *h, struct can_frame *f,
			int dir0, int dt0, int dir1, int dt1, int dir2, int dt2,
			int dir3, int dt3, int dir4, int dt4)
{
	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_OPENLOOP_PWM;
	f->data[2]	= 0x01; // Pulse according to duty cycle
				//	Alternative value is 0x00 which enforces
				//	zero duty in order to stop the motor
	f->data[3]	= duty_to_byte(dir0, dt0);
	f->data[4]	= duty_to_byte(dir1, dt1);
	f->data[5]	= duty_to_byte(dir2, dt2); 
	f->data[6]	= duty_to_byte(dir3, dt3);
	f->data[7]	= duty_to_byte(dir4, dt4);

	f->can_dlc	= 8;
}

void hSetControlMode(int jnt, struct hubo_param *h, struct hubo_state *s, struct can_frame *f, hubo_d_param_t mode)
{
	switch (mode)
	{
		case D_POSITION:
			fSetControlMode(jnt, h, f, 0); // 0 > Position control
			sendCan(getSocket(h,jnt),f);
//			s->driver[h->joint[jnt].jmc].ctrlMode = D_POSITION;
			break;
		case D_CURRENT:
			fSetControlMode(jnt, h, f, 1); // 1 > Current control
			sendCan(getSocket(h,jnt),f);
//			s->driver[h->joint[jnt].jmc].ctrlMode = D_CURRENT;
			break;
		default:
			fprintf(stderr,"Invalid Control Mode: %d\n\t"
					"Must use: D_POSITION (%d) or D_CURRENT (%d)\n",
					(int)mode, (int)D_POSITION, (int)D_CURRENT);
			break;
	}
}


void fSetControlMode(int jnt, struct hubo_param *h, struct can_frame *f, int mode)
{
	f->can_id	= CMD_TXDF;

	__u8 data[3];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_CTRL_MODE;
	f->data[2]	= (uint8_t)mode;

	f->can_dlc	= 3;
}


void hSetAlarm(int jnt, struct hubo_param *h, struct can_frame *f, hubo_d_param_t sound)
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

void fSetAlarm(int jnt, struct hubo_param *h, struct can_frame *f, int sound)
{
	f->can_id 	= CMD_TXDF;	// Set ID

	__u8 data[3];
	f->data[0] 	= getJMC(h,jnt);	
	f->data[1]	= H_ALARM;
	f->data[2] 	= (uint8_t)sound; 	// Use H_ALARM_S1, S2, S3, S4, or OFF

	f->can_dlc = 3;
}


void hSetDeadZone(int jnt, struct hubo_param *h, struct can_frame *f, int deadzone)
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

void fSetDeadZone(int jnt, struct hubo_param *h, struct can_frame *f, int deadzone)
{
	f->can_id	= CMD_TXDF;

	__u8 data[3];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_DEADZONE + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)deadzone;

	f->can_dlc	= 3;
}

void hSetHomeSearchParams( struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f )
{
	unsigned int dir, offset;

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

	offset = (unsigned int)c->iValues[1];

	fSetHomeSearchParams(c->joint, h, f, c->iValues[0], dir, offset);

	sendCan(getSocket(h,c->joint),f);
}

void fSetHomeSearchParams(int jnt, struct hubo_param *h, struct can_frame *f, int limit,
				unsigned int dir, unsigned int offset)
{
	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_HOME_PARAM + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)limit;
	f->data[3]	= (uint8_t)dir;
	f->data[4]	= int_to_bytes(offset,1);
	f->data[5]	= int_to_bytes(offset,2);
	f->data[6]	= int_to_bytes(offset,3);
	f->data[7]	= int_to_bytes(offset,4);

	f->can_dlc	= 8;
}

void hSetEncoderResolution(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
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

void fSetEncoderResolution(int jnt, struct hubo_param *h, struct can_frame *f, int res)
{
	f->can_id	= CMD_TXDF;

	__u8 data[4];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_ENC_RES + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
	f->data[2]	= int_to_bytes(res,1); // TODO: Have handler construct res properly
	f->data[3]	= int_to_bytes(res,2);

	f->can_dlc	= 4;
}

void hSetMaxAccVel(int jnt, struct hubo_param *h, struct can_frame *f, int maxAcc, int maxVel)
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

void fSetMaxAccVel(int jnt, struct hubo_param *h, struct can_frame *f, int maxAcc, int maxVel)
{
	f->can_id	= CMD_TXDF;

	__u8 data[6];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_MAX_ACC_VEL + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
	f->data[2]	= int_to_bytes(maxAcc,1);
	f->data[3]	= int_to_bytes(maxAcc,2);
	f->data[4]	= int_to_bytes(maxVel,1);
	f->data[5]	= int_to_bytes(maxVel,2);

	f->can_dlc	= 6;
}

void hSetLowerPosLimit(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
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

void fSetLowerPosLimit(int jnt, struct hubo_param *h, struct can_frame *f, int enable, int update, int limit)
{
	f->can_id	= CMD_TXDF;

	__u8 data[7];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_LOW_POS_LIM + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)( (update << 1) | (enable) );
	f->data[3]	= int_to_bytes(limit,1);
	f->data[4]	= int_to_bytes(limit,2);
	f->data[5]	= int_to_bytes(limit,3);
	f->data[6]	= int_to_bytes(limit,4);

	f->can_dlc	= 7;
}



void hSetUpperPosLimit(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
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

void fSetUpperPosLimit(int jnt, struct hubo_param *h, struct can_frame *f, int enable, int update, int limit)
{
	f->can_id	= CMD_TXDF;

	__u8 data[7];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_UPP_POS_LIM + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)( (update << 1) | (enable) );
	f->data[3]	= int_to_bytes(limit,1);
	f->data[4]	= int_to_bytes(limit,2);
	f->data[5]	= int_to_bytes(limit,3);
	f->data[6]	= int_to_bytes(limit,4);

	f->can_dlc	= 7;
}

void hSetHomeAccVel(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
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

void fSetHomeAccVel(int jnt, struct hubo_param *h, struct can_frame *f, float mAcc, int mVelS,
			int mVelP, int mode, int mDuty)
{
	f->can_id	= CMD_TXDF;

	__u8 data[7];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= (uint8_t)(H_SET_HOME_VEL_ACC + h->joint[jnt].motNo); // TODO: Find out if +1 is correct
	f->data[2]	= (uint8_t)(mAcc*100);
	f->data[3]	= (uint8_t)(mVelS);
	f->data[4]	= (uint8_t)(mVelP);
	if( mode==0 || mode==1 || mode==2 )
		f->data[5]	= (uint8_t)(mode);
	else
		f->data[5]	= 0x00;
	f->data[6]	= (uint8_t)(mDuty);

	f->can_dlc	= 7;
}

void hSetGainOverride(int jnt, struct hubo_param *h, struct can_frame *f, int gain0, int gain1, double dur)
{
	fSetGainOverride(jnt, h, f, gain0, gain1, (int)(dur*1000));
	sendCan(getSocket(h,jnt),f);
}


void fSetGainOverride(int jnt, struct hubo_param *h, struct can_frame *f, int gain0, int gain1, int duration)
{
	f->can_id	= CMD_TXDF;

	__u8 data[6];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_GAIN_OVERRIDE;
	f->data[2]	= (uint8_t)(gain0);
	f->data[3]	= (uint8_t)(gain1);
	f->data[4]	= int_to_bytes(duration,1);
	f->data[5]	= int_to_bytes(duration,2);

	f->can_dlc = 6;
}

void hSetBoardNumber(int jnt, struct hubo_param *h, struct can_frame *f, int boardNum, int rate)
{
	fprintf(stdout, "WARNING: Changing board number %d to %d with baud rate %d\n",
			getJMC(h,jnt), boardNum, rate);
	fSetBoardNumber(jnt, h, f, boardNum, rate);
	sendCan(getSocket(h,jnt),f);
}


void fSetBoardNumber(int jnt, struct hubo_param *h, struct can_frame *f, int boardNum, int rate)
{
	f->can_id	= CMD_TXDF;

	__u8 data[4];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_BOARD_NUM;
	f->data[2]	= (uint8_t)(boardNum);
	f->data[3]	= (uint8_t)(rate);

	f->can_dlc = 4;
}

void hSetJamPwmLimits(struct hubo_board_cmd *c, struct hubo_param *h, struct can_frame *f)
{
	fprintf(stdout, "Changing Jam and PWM Saturation limits:\n\t"
			"Jam Duty: %d \t Sat Duty: %d\n\t"
			"Jam Time: %g \t Sat Time: %g\n",
			c->iValues[0], c->iValues[1], c->dValues[0], c->dValues[1] );
	fSetJamPwmLimits(c->joint, h, f, (int)(c->dValues[0]*1000), (int)(c->dValues[1]*1000),
				c->iValues[0], c->iValues[1] );

	sendCan(getSocket(h,c->joint),f);
}

void fSetJamPwmLimits(int jnt, struct hubo_param *h, struct can_frame *f, int jamLimit, int pwmLimit,
			int lim_detection_duty, int jam_detection_duty )
{
	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_JAM_SAT_LIM;
	f->data[2]	= int_to_bytes(jamLimit,1);
	f->data[3]	= int_to_bytes(jamLimit,2);
	f->data[4]	= int_to_bytes(pwmLimit,1);
	f->data[5]	= int_to_bytes(pwmLimit,2);
	f->data[6]	= (uint8_t)(lim_detection_duty);
	f->data[7]	= (uint8_t)(jam_detection_duty);

	f->can_dlc = 8;
}

void hSetErrorBound(int jnt, struct hubo_param *h, struct can_frame *f, int inputDiffErr, int maxError,
			int tempError)
{
	fprintf(stdout, "Changing error bounds on board %d:\n\t"
			"Input Difference error: %d\n\t"
			"Maximum error: %d\n\t"
			"Max temperature: %d\n", jnt, inputDiffErr, maxError, tempError);

	fSetErrorBound(jnt, h, f, inputDiffErr, maxError, tempError);
	sendCan(getSocket(h,jnt),f);
}

void fSetErrorBound(int jnt, struct hubo_param *h, struct can_frame *f, int inputDiffErr, int maxError,
			int tempError)
{
	f->can_id	= CMD_TXDF;

	__u8 data[8];
	f->data[0]	= getJMC(h,jnt);
	f->data[1]	= H_SET_ERR_BOUND;
	f->data[2] 	= int_to_bytes(inputDiffErr,1);
	f->data[3]	= int_to_bytes(inputDiffErr,2);
	f->data[4]	= int_to_bytes(maxError,1);
	f->data[5]	= int_to_bytes(maxError,2);
	f->data[6]	= int_to_bytes(tempError,1);
	f->data[7]	= int_to_bytes(tempError,2);

	f->can_dlc = 8;
}




void fGetBoardParamA( int jnt, int offset, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_A + offset);

	f->can_dlc	= 3;
}

void fGetBoardParamB( int jnt, int offset, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_B + offset);

	f->can_dlc	= 3;
}

void fGetBoardParamC( int jnt, int offset, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_C + offset);

	f->can_dlc	= 3;
}

void fGetBoardParamD( int jnt, int offset, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_D + offset);

	f->can_dlc	= 3;
}

void fGetBoardParamE( int jnt, int offset, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_E + offset);

	f->can_dlc	= 3;
}

void fGetBoardParamF( int jnt, int offset, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= (uint8_t)(h->joint[jnt].motNo*6 + H_GET_PARAM_F + offset);

	f->can_dlc	= 3;
}

void fGetBoardParamG( int jnt, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= H_GET_PARAM_G;

	f->can_dlc	= 3;
}

void fGetBoardParamH( int jnt, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= H_GET_PARAM_H;

	f->can_dlc	= 3;
}

void fGetBoardParamI( int jnt, struct hubo_param *h, struct can_frame *f )
{
	f->can_id 	= CMD_TXDF;

	f->data[0] 	= getJMC(h,jnt);
	f->data[1] 	= H_REQ_PARAMS;
	f->data[2] 	= H_GET_PARAM_I;

	f->can_dlc	= 3;
}

void hGetBoardParams( int jnt, hubo_d_param_t param, struct hubo_param *h, struct hubo_state *s, struct can_frame *f )
{
	int offset = 0;
	if(h->joint[jnt].motNo >= 3)
		offset = 5;

	switch (param)
	{
		case D_PARAM_MOTOR:

			fGetBoardParamA( jnt, offset, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode
			break;

		case D_PARAM_HOME:

			fGetBoardParamB( jnt, offset, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode

			fGetBoardParamC( jnt, offset, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode
			break;

		case D_PARAM_LIMITS:

			fGetBoardParamD( jnt, offset, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode

			fGetBoardParamH( jnt, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode
			break;

		case D_PARAM_CURRENT:

			fGetBoardParamE( jnt, offset, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode
			break;

		case D_PARAM_F:

			fGetBoardParamF( jnt, offset, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode
			break;

		case D_PARAM_CAN:

			fGetBoardParamG( jnt, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode
			break;

		case D_PARAM_ERROR:

			fGetBoardParamI( jnt, h, f );
			sendCan(getSocket(h,jnt), f);
			//TODO: Read and decode
	}
}

uint8_t int_to_bytes(int d, int index)
{
	return (uint8_t)( ( d >> ((index-1)*8) ) & 0xFF);
	//return (uint8_t)(d%((int)(pow(256,index)))/((int)(pow(256,index-1))));
}

uint8_t duty_to_byte(int dir, int duty)
{
	return (uint8_t)(duty | (dir<<8) );
}
