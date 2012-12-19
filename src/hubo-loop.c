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
#include <time.h>

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
#include "hubo-jointparams.h"

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


//For fastrak:
#include <somatic.h>
#include <somatic.pb-c.h>
#include <complex.h>
#include <amino.h>


/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */

// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */


// Timing info
#define NSEC_PER_SEC    1000000000


struct timeb {
        time_t   time;
        unsigned short millitm;
        short    timezone;
        short    dstflag;
};



/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
void huboLoop(struct hubo_param *h);
int ftime(struct timeb *tp);
void matrixmult( double *A, double *B, double *C, int N ); // C = A*B; where A, B, and C are NxN
void matrixfill( double *A, double *B, int N );
void forwardKineTransform( double *T, double t, double f, double r, double d );
void forwardKine( double *B, double *q );
void inverseKine( double *B, double *q );
void transfill( double *A, double *B ); // Fills the truncated Homo matrix of A into a full Homo matrix of B




// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_board_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_fastrak; // temporary fastrak channel

void huboLoop(struct hubo_param *H_param) {
        // get initial values for hubo
        struct hubo_ref H_ref;
	struct hubo_state H_state;
	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_state, 0, sizeof(H_state));

	int debug = 1;

        size_t fs;
        //int r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
        //assert( sizeof(H) == fs );
	int r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(debug) {
                       	printf("Ref ini r = %s\n",ach_result_to_string(r));}
		}
	else{   assert( sizeof(H_ref) == fs ); }

	r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(debug) {
                       	printf("State ini r = %s\n",ach_result_to_string(r));}
		}
	else{   
		assert( sizeof(H_state) == fs );
	 }

	double coord[16] = {	-1, 0, 0, 0,
				 0, 0, 1, 0,
				 0, 1, 0, 0,
				 0, 0, 0, 1	}; // Convert from real-world to IK frame


	double B[16];
	double B0[16];
	double q[6];
	
	int joints[6] = { RSP, RSR, RSY, REB, RWY, RWP };
	for(int i=0; i<6; i++)
		q[i] = H_state.joint[joints[i]].pos;
	
	forwardKine( B0, q );
	for(int i=0; i<16; i++)
		B[i] = B0[i];
	

	double tf0[12];
	double tfull0[16];
	Somatic__MultiTransform *mtf;
	{
		uint8_t buf[4096];	
		mtf = SOMATIC_GET_LAST_UNPACK_BUF( r, somatic__multi_transform, &chan_fastrak, buf, sizeof(buf), NULL );
	}
	if( mtf ) {
		assert( mtf->n_tf && mtf->tf );

		somatic_transform_get_tf12( mtf->tf[0], tf0 );
		somatic__multi_transform__free_unpacked( mtf, NULL );
	}

	double tftemp[16];
	transfill( tf0, tftemp );
//	for(int i=0; i<16; i++)
//		tftemp[i] = tfull0[i];

	matrixmult( coord, tftemp, tfull0, 4 );

      	/* Send a message to the CAN bus */
        struct can_frame frame;

        // time info
        struct timespec t;
        //int interval = 500000000; // 2hz (0.5 sec)
        int interval = 10000000; // 100 hz (0.01 sec)
        //int interval = 5000000; // 200 hz (0.005 sec)
        //int interval = 2000000; // 500 hz (0.002 sec)

        // get current time
        //clock_gettime( CLOCK_MONOTONIC,&t);
        clock_gettime( 0,&t);
        struct timeb tp;
        struct timeb tp_0;
        struct timeb tp_f;
        int a = 0;

        /* get initial tme*/
        int jnt = RSP;

        ftime(&tp_0);
        double tt = 0.0;
        double f = 0.2;		// frequency
//        double T = (double)interval/1000000000.0;
	double T = .5;
/*        double x_bar[4][6] = {  { -1.044, -0.0066, 0.055, -0.655, 1.034, 0.1478 },
				{ -0.926, -0.784, -0.183, -0.583, 1.0045, 1.235 },
				{ -1.386, -1.182, -0.1146, -1.534, -1.076, 1.233 },
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } }; */
/*        double x_bar[4][6] = {  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
				{ -1.044, -0.0066, 0.055, -0.655, 1.034, 0.1478 },
				{ -0.926, -0.784, -0.183, -0.583, 1.0045, 1.235 },
				{ -1.386, -1.182, -0.1146, -1.534, -1.076, 1.233 } }; */
	double x_bar[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
/*      double x_bar[4][6] = {  { -1.044, -0.0066, 0.055, -0.655, 1.034, 0.1478 },
				{ -0.926, -0.784, -0.183, -0.583, 1.0045, 1.235 },
				{ -1.386, -1.182, -0.1146, -1.534, -1.076, 1.233 },
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
*/ /*     double x_bar[4][6] = { 
				{ -1.044, 0.0, 0.0, 0.0, 0.0, 0.0 },
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
*/ /*	double V_bar[4][6] = {  { -0.75, 0.75, 0.75, 0.75, 0.75, 0.75 }
				{ 0.75, 0.75, 0.75, 0.75, 0.75, 0.75 }
				{ 0.75, 0.75, 0.75, 0.75, 0.75, 0.75 }
				{ 0.75, 0.75, 0.75, 0.75, 0.75, 0.75 } }; */
/*	double V_bar[4][6];
	for(int i=0; i<6; i++)
	{
		for(int j=1; j<4; j++)
			V_bar[j][i] = (x_bar[j][i] - x_bar[j-1][i])/T;

		V_bar[0][i] = (x_bar[0][i]-x_bar[3][i])/T;
	} */
//	double x = H_state.joint[jnt].pos;

	double V_bar[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	double ep = 0.01;
	double V[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double V0[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double fail[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double V_max = 10; //TODO: Find a meaningful value for this
	double a_max = 0.4; //TODO: Find a meaningful value for this
        double dir = 1.0;
	double t0 = 0.0;
        double t1 = 0.0;
	double dr, dt, dV, norm, err;
	double tf[12];
	double qinv[6*8];
	double qgood[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double qgood0[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double tfull[16];
	double fascale = 0.50;
	double best = 0;
	int choice = 0;
	double filfactor = 1;
	double dtfx=0;
	double dtfy=0;
	double dtfz=0;

//	const int npts = 10000;
//	double ref_pos[npts];
//	double ref_time[npts];
//	double ts = T/npts;

	int traj = 0;

        while(1) {
                // wait until next shot
                clock_nanosleep(0,TIMER_ABSTIME,&t, NULL);

                /* Get latest ACH message */
		r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			if(debug) {
                        	printf("Ref r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(H_ref) == fs ); }
		r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			if(debug) {
                        	printf("State r = %s\n",ach_result_to_string(r));}
			}
		else{   assert( sizeof(H_state) == fs ); }

		double jntDiff = H_state.joint[jnt].pos - H_ref.ref[jnt];
//		printf("\033[2J");
//		printf("%s: Cur = %f \t  Diff = %f \t State = %f \t Ref = %f\n", H_param->joint[jnt].name, H_state.joint[jnt].cur, jntDiff, H_state.joint[jnt].pos, H_ref.ref[jnt]);	


                ftime(&tp);
                tp_f.time = tp.time-tp_0.time;
                tp_f.millitm = tp.millitm-tp_0.millitm;

                tt = (double)tp_f.time+((int16_t)tp_f.millitm)*0.001;

                t0 = t1;
                t1 = tt;
		dt = t1 - t0;

//		dt = 1;

		// TODO: be less gross
		Somatic__MultiTransform *mtf;
		{
			uint8_t buf[4096];	
			mtf = SOMATIC_GET_LAST_UNPACK_BUF( r, somatic__multi_transform, &chan_fastrak, buf, sizeof(buf), NULL );
		}
		if( mtf ) {
			assert( mtf->n_tf && mtf->tf );

			somatic_transform_get_tf12( mtf->tf[0], tf );
			somatic__multi_transform__free_unpacked( mtf, NULL );

			//aa_dump_vec(stderr, tf+9, 3 );
//			printf("%f\t%f\t%f\n", 
//				tf[9], tf[10], tf[11]);
		}

		double tftemp[16];
		transfill( tf, tftemp );
		matrixmult( coord, tftemp, tfull, 4 );

		dtfx = dtfx*(1-filfactor) + filfactor*(tfull[12] - tfull0[12]);
		dtfy = dtfy*(1-filfactor) + filfactor*(tfull[13] - tfull0[13]);
		dtfz = dtfz*(1-filfactor) + filfactor*(tfull[14] - tfull0[14]);
	
		B[12] = B0[12] + fascale*dtfx;
		B[13] = B0[13] + fascale*dtfy;
		B[14] = B0[14] + fascale*dtfz;
		
/*		B[0] = tfull[0];	B[4] = tfull[4];	B[8] = tfull[8];
		B[1] = tfull[1];	B[5] = tfull[5];	B[9] = tfull[9];
		B[2] = tfull[2];	B[6] = tfull[6];	B[10] = tfull[10];
		B[3] = 0;		B[7] = 0;		B[11] = 0;

		printf("__________________________________\n");
		for(int i=0; i<4; i++)
		{
			for(int j=0; j<4; j++)
			{
				printf("%f\t",B[i+4*j]);
			}
			printf("\n");
		}
		printf("__________________________________\n");*/
		
		inverseKine( B, qinv );

		choice = -1;
		best = 0;
		for(int i=0; i<8; i++)
		{
			if(	(qinv[0+6*i]>-2.0&&qinv[0+6*i]<2.0) && !isnan(qinv[0+6*i]) &&
				(qinv[1+6*i]>-2.0&&qinv[1+6*i]<0.2) && !isnan(qinv[0+6*i]) &&
				(qinv[2+6*i]>-2.0&&qinv[2+6*i]<2.0) && !isnan(qinv[0+6*i]) &&
				(qinv[3+6*i]>-2.0&&qinv[3+6*i]<0.0) && !isnan(qinv[0+6*i]) &&
				(qinv[4+6*i]>-2.0&&qinv[4+6*i]<2.0) && !isnan(qinv[0+6*i]) &&
				(qinv[5+6*i]>-1.4&&qinv[5+6*i]<1.2) && !isnan(qinv[0+6*i]) 	)
			{
				if(choice==-1)
				{
					for(int j=0; j<6; j++)
						best += (qinv[j+6*i]-H_state.joint[joints[i]].pos)*(qinv[j+6*i]-H_state.joint[joints[i]].pos);
					best = sqrt(best);
					choice = i;
				}
				else
				{
					double temp = 0;
					for(int j=0; j<6; j++)
						temp += (qinv[j+6*i]-H_state.joint[joints[i]].pos)*(qinv[j+6*i]-H_state.joint[joints[i]].pos);
					temp = sqrt(temp);
					
					if( temp < best )
					{	
						best = temp;
						choice = i;
					}
				}	
			}
		}

		if( choice >= 0 )
		{
			for(int i=0; i<6; i++)
			{
				qgood[i] = qinv[i+6*choice];
				V_bar[i] = (qgood[i]-qgood0[i])/dt;
			}
		}

		for(int i=0; i<6; i++)
		{
			err = H_ref.ref[joints[i]] - H_state.joint[joints[i]].pos;
			
			if( fabs(err) <= fabs(a_max/2.0*dt*dt + V_max*dt) && fail[i]==0 )
			{
				dV = V_bar[i] - V0[i];
				if( dV > fabs(a_max*dt) )
					dV = fabs(a_max*dt);
				else if( dV < -fabs(a_max*dt) )
					dV = -fabs(a_max*dt);

				V[i] = V0[i] + dV;

				dr = qgood[i] - H_ref.ref[joints[i]];
				if( fabs(dr)>fabs(V[i]*dt) && V[i]*dr >= 0 )
					dr = V[i]*dt;
				else if( V[i]*dr<0 )
				{
					dr = -V[i]*dt;
				}

//				H_ref.ref[joints[i]] += dr;

				V0[i] = V[i];
				qgood0[i] = qgood[i];

				printf("[%d]=(dr:%f,V:%f,dV:%f) \t ", i, dr, V[i], dV);
			}
			else if( fail[i]==0 && dt>0)
			{	
				H_ref.ref[joints[i]] = H_state.joint[joints[i]].pos;
				printf( "ERROR: EXCEEDING MAX VELOCITY: %d, %f\n",i,V_max*dt);
				fail[i]++;
			}
			else if( dt > 0 )
			{
				printf( "ERROR: JOINT (%d) HAS QUIT\n", i );
			}
		}
		printf("\n");

/*		for(int j=0; j<8; j++)
		{
			for(int i=0; i<6; i++)
				printf( "%f\t", qinv[i+j*6] );
			printf("\n");
		} */


		for(int i=0; i<6; i++)
			printf("%f \t ",qgood[i]);
		printf("CHOICE: %d", choice);
		printf("\n");
		
//		printf( "%f \t %f \t %f \n", tfull[12]-tfull0[12], tfull[13]-tfull0[13], tfull[14]-tfull0[14] );
//		printf( "%f \t %f \t %f\n", B[0+4*3], B[1+4*3], B[2+4*3] );
		

/*		for(int i=0; i<6; i++)
			printf("%f\t",q[i]);
		printf("\n");*/
		/*
		norm = 0;
		for(int i=0; i<4; i++)
		{
			err = H_ref.ref[joints[i]] - H_state.joint[joints[i]].pos;

			if( fabs(err) <= fabs(a_max/2.0*dt*dt + V_max*dt) && fail[i]==0)
			{
				dV = V_bar[traj][i] - V0[i];
				if( dV > fabs(a_max*dt) )
					dV = fabs(a_max*dt);
				else if( dV < -fabs(a_max*dt) )
					dV = -fabs(a_max*dt);
				
				V[i] = V0[i] + dV;
				
				
				norm += (x_bar[traj][i] - H_ref.ref[joints[i]])*(x_bar[traj][i] - H_ref.ref[joints[i]]);
				dr = x_bar[traj][i] - H_ref.ref[joints[i]];
				norm += dr*dr;
				if( fabs(dr)>fabs(V[i]*dt) && V[i]*dr >= 0 )
					dr = V[i]*dt;
				else if( V[i]*dr<0 )
				{
					dr = -V[i]*dt;
					//printf( "ERROR: P/V MISMATCH (%d)", i);
				}

				//dr = V[i]*dt;
					
					
				H_ref.ref[joints[i]] += dr;
			}
			else if( fail[i]==0 && dt>0)
			{	
				H_ref.ref[joints[i]] = H_state.joint[joints[i]].pos;
				//printf( "ERROR: EXCEEDING MAX VELOCITY: %d, %f\n",i,V_max*dt);
				fail[i]++;
			}
			else if( dt > 0 )
			{
				//printf( "ERROR: JOINT (%d) HAS QUIT\n", i );
			}
			
			if( fabs(V[i]) > fabs(V_max) && dt>0 )
			{
				H_ref.ref[joints[i]] = H_state.joint[joints[i]].pos;
				fail[i]++;

				//fprintf(stderr, "ERROR: Requested velocity is greater than max velocity. Joint (%d)\n",i);
				
			}
	
		//	printf( "[%d]=(r:%f,rb:%f,V:%f,Vb:%f,dV:%f) \t ", i, H_ref.ref[joints[i]], x_bar[traj][i], V[i], V_bar[traj][i], dV );
		
			
			
			V0[i] = V[i];
		}

		norm = sqrt(norm);
		printf("norm:%f \t traj:%d\n",norm,traj);
		if( norm <= ep && traj < 4 )
		{
			traj++;
			if( traj >= 4 )
				traj = 0;
		}

		*/


//		if( floor(tt/ts) > npts )
//			H_ref.ref[jnt] = ref_pos[(int)floor(tt/ts)];

//		H_ref.ref[jnt] = H_ref.ref[jnt]
//		x = H_state.joint[jnt].pos;

//                double jntTmp = A*sin(f*2*M_PI*tt);

//		H_ref.ref[jnt] = jntTmp;
//		H_ref.ref[jnt] = dir*fabs(jntTmp);
//		h_ref.ref[jnt] = dir*0.05;		
/*		if(jntTmp > 0) {
	                H_ref.ref[jnt] = -dir*jntTmp; }
		else { 
	                H_ref.ref[jnt] = dir*jntTmp; } */
		

        //	printf("time = %ld.%d %f\n",tp_f.time,tp_f.millitm,tt);
//                printf("A = %f\n",H.ref[jnt]);
                //printf("Diff(t) = %f\n",(t0-t1));

                ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));
                t.tv_nsec+=interval;
                tsnorm(&t);
        }


}


void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset( dummy, 0, MAX_SAFE_STACK );
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

int main(int argc, char **argv) {

        int vflag = 0;
        int c;
	int debug = 0;

        int i = 1;
        while(argc > i) {
                if(strcmp(argv[i], "-d") == 0) {
                        debug = 1;
                }
                i++;
        }

        /* RT */
        struct sched_param param;
        /* Declare ourself as a real time task */
        param.sched_priority = MY_PRIORITY;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }

        /* Lock memory */
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

        /* Pre-fault our stack */
        stack_prefault();


        /* open ach channel */
        int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
        assert( ACH_OK == r );

	r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
        assert( ACH_OK == r );
  
	r = ach_open(&chan_fastrak, "fastrak", NULL);
	assert( ACH_OK == r );
		
	// get initial values for hubo
        struct hubo_ref H_ref;
        struct hubo_board_cmd H_cmd;
        struct hubo_state H_state;
        struct hubo_param H_param;
        memset( &H_ref,   0, sizeof(H_ref));
        memset( &H_cmd,  0, sizeof(H_cmd));
        memset( &H_state, 0, sizeof(H_state));
        memset( &H_param, 0, sizeof(H_param));

        usleep(250000);

        // set default values for H_ref in ach
//	setPosZeros();

        // set default values for Hubo
        setJointParams(&H_param, &H_state);
      
	huboLoop(&H_param);
        pause();
        return 0;

}


void inverseKine( double *B, double *q )
{
	int N=4;
	int J=6;

	double L1 = 0.2145; double L2 = 0.17914; double L3 = 0.18159; double L4 = 0.0100;
	
	double C[16];
	for(int i=0; i<16; i++)
		C[i] = B[i];

	aa_la_inv( N, C );
	double nx, ny, nz, sx, sy, sz, ax, ay, az, px, py, pz;

	nx=C[0+N*0]; ny=C[1+N*0]; nz=C[2+N*0];
	sx=C[0+N*1]; sy=C[1+N*1]; sz=C[2+N*1];
	ax=C[0+N*2]; ay=C[1+N*2]; az=C[2+N*2];
	px=C[0+N*3]; py=C[1+N*3]; pz=C[2+N*3];
	
	double m[8][3] = {	{  1,  1,  1 }, 
				{  1,  1, -1 }, 
				{  1, -1,  1 }, 
				{  1, -1, -1 }, 
				{ -1,  1,  1 }, 
				{ -1,  1, -1 }, 
				{ -1, -1,  1 }, 
				{ -1, -1, -1 }	};

	for(int i=0; i<8; i++)
	{
		double complex C4 = (-L2*L2 - L3*L3 + L4*L4 + 2*L4*px + px*px + py*py + pz*pz)/(2.0*L2*L3);
		double complex rC4 = csqrt(1-C4*C4);
		q[3+J*i] = atan2(m[i][0]*creal(rC4),C4);
		
		double S4 = sin(q[3+J*i]);
		double complex S5 = pz/(S4*L2);
		q[4+J*i] = atan2(S5, m[i][1]*creal(csqrt(1-S5*S5)));
		
		double C5 = cos(q[4+J*i]);
		double S6 = -(C5*S4*L2 - (py*(L3 + C4*L2 + (C5*S4*L2*py)/(L4+px)))/(py*py/(L4+px) + L4 + px))/(L4 + px);
		double C6 = -(L3 + C4*L2 + (C5*S4*L2*py)/(L4+px))/(py*py/(L4+px) + L4 + px);
		q[5+J*i] = atan2(S6,C6);

		double complex S2 = C4*C6*ax - C4*S6*ay - S4*S5*az + C5*C6*S4*ay + C5*S4*S6*ax;
		q[1+J*i] = atan2(S2, m[i][2]*creal(csqrt(1-S2*S2)));

		q[2+J*i] = atan2(ay*(S4*S6 + C4*C5*C6) - ax*(C6*S4 - C4*C5*S6) - C4*S5*az, -C5*az-C6*S5*ay - S5*S6*ax);

		q[0+J*i] = atan2(nx*(C4*C6 + C5*S4*S6) - ny*(C4*S6 - C5*C6*S4) - S4*S5*nz, sy*(C4*S6 - C5*C6*S4) - sx*(C4*C6 + C5*S4*S6) + S4*S5*sz);
	}
	

}


void forwardKineTransform( double *T, double t, double f, double r, double d )
{
	T[0] = cos(t);	T[4] = -sin(t)*cos(f);	T[8] = sin(t)*sin(f);	T[12] = r*cos(t);
	T[1] = sin(t);	T[5] = cos(t)*cos(f);	T[9] = -cos(t)*sin(f);	T[13] = r*sin(t);
	T[2] = 0.0;	T[6] = sin(f);		T[10] = cos(f);		T[14] = d;
	T[3] = 0.0;	T[7] = 0.0;		T[11] = 0;		T[15] = 1;

}

void forwardKine( double *B, double *q )
{
	int N=4;

	double L1 = 0.2145; double L2 = 0.17914; double L3 = 0.18159; double L4 = 0.0100;

	double t[6] = { -M_PI/2.0, M_PI/2.0, M_PI/2.0, 0, 0, M_PI/2.0 };
	double f[6] = { M_PI/2.0, -M_PI/2.0, M_PI/2.0, M_PI/2.0, M_PI/2.0, 0 };
	double r[6] = { 0, 0, 0, 0, 0, L4 };
	double d[6] = { 0, 0, -L2, 0, L3, 0 };

/*	double B[4][4] = {
				{1, 0, 0, 0},
				{0, 1, 0, 0},
				{0, 0, 1, 0},
				{0, 0, 0, 1} }; */
	for(int i=0; i<N; i++)
		for(int j=0; j<N; j++)
			if( i==j )
				B[i+N*j] = 1;
			else
				B[i+N*j] = 0;
	
	
	
	double C[N*N];
	double T[N*N];
	for(int i=0; i<6; i++)
	{	
		forwardKineTransform( T, t[i]+q[i], f[i], r[i], d[i] );
		
		matrixmult( B, T, C, N );
		matrixfill( C, B, N );
	}

}

void matrixfill( double *A, double *B, int N )
{
	for(int i=0; i<N; i++)
		for(int j=0; j<N; j++)
			B[i+N*j] = A[i+N*j];
}

void matrixmult( double *A, double *B, double *C, int N ) // C = A*B; where A, B, and C are NxN
{
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<N; j++)
		{
			C[i+N*j] = 0;
			for(int k=0; k<N; k++)
			{
				C[i+N*j] += A[i+N*k]*B[k+N*j];
			}
		}
	}
}

void transfill( double *A, double *B ) // Fills the truncated Homo matrix of A into a full Homo matrix of B
{
	B[0] = A[0];	B[4] = A[3];	B[8]  = A[6];	B[12] = A[9];
	B[1] = A[1];	B[5] = A[4];	B[9]  = A[7];	B[13] = A[10];
	B[2] = A[2];	B[6] = A[5];	B[10] = A[8];	B[14] = A[11];
	B[3] = 0;	B[7] = 0;	B[11] = 0;	B[15] = 1;
}
