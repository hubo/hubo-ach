#include "control-daemon.h"


double sign(double x)
{
    return (x < 0) ? -1 : (x > 0);
}

void controlLoop()
{
    struct hubo_ref H_ref;
    struct hubo_board_cmd H_cmd;
    struct hubo_state H_state;
    struct hubo_control ctrl;
    struct hubo_param H_param;

    memset( &H_ref,   0, sizeof(H_ref)   );
    memset( &H_cmd,   0, sizeof(H_cmd)   );
    memset( &H_state, 0, sizeof(H_state) );
    memset( &ctrl,    0, sizeof(ctrl)    );
    memset( &H_param, 0, sizeof(H_param) );

    setJointParams( &H_param, &H_state);

    setCtrlDefaults( &ctrl );

    //ach_put( &chan_hubo_ctrl, &ctrl, sizeof(ctrl) ); 

    size_t fs;
    int result = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != result )
    {
        // TODO: Debug stuff
    }
    else
    {
        daemon_assert( sizeof(H_state) == fs, __LINE__ );
    }

    result = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != result )
    {
        // TOOD: Print a debug message
    }
    else { daemon_assert( sizeof(H_state) == fs, __LINE__ ); }

    double r[HUBO_JOINT_COUNT];
    double r0[HUBO_JOINT_COUNT];
    double dr[HUBO_JOINT_COUNT];
    double V[HUBO_JOINT_COUNT];
    double V0[HUBO_JOINT_COUNT];
    double dV[HUBO_JOINT_COUNT];
    double adr;

    int fail[HUBO_JOINT_COUNT];
    int reset[HUBO_JOINT_COUNT];
    int home[HUBO_JOINT_COUNT];
    int homing=0;
    int allhoming=0;

    // Initialize arrays
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        r[i] = 0;   r0[i] = 0;  dr[i] = 0;
        V[i] = 0;   V0[i] = 0;  dV[i] = 0;

        fail[i] = 0;
    }

    double t0 = H_state.time;
    double t, dt, err;

    fprintf(stdout, "Beginning control loop\n");

    // Main control loop
    while( !daemon_sig_quit )
    {

        int cresult = ach_get( &chan_hubo_ctrl, &ctrl, sizeof(ctrl), &fs, NULL, ACH_O_LAST );
        if( ACH_OK != cresult )
        {
            // TODO: Print a debug message
        }
        else 
	{
		
		daemon_assert( sizeof(ctrl) == fs, __LINE__ );
//		printf("Got ctrl ach.");
/*		printf("%f\t%f\t%f\t%f\t%f\t%f\n",
			ctrl.joint[LSP].position, r[LSP],
			ctrl.joint[LSR].position, r[LSR],
			ctrl.joint[LSY].position, r[LSY]);
*/	} cresult = ACH_OK;

        result = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
        if( ACH_OK != result )
        {
            // TODO: Print a debug message
        }
        else { daemon_assert( sizeof(H_ref) == fs, __LINE__ ); }

        result = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
        if( ACH_OK != result )
        {
            // TODO: Print a debug message
        }
        else { daemon_assert( sizeof(H_state) == fs, __LINE__ ); }

        t = H_state.time;
        dt = t - t0;

        if( ctrl.active==2 && allhoming==0 )
        {
            H_cmd.type = D_GOTO_HOME_ALL;
            //ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
            fprintf(stdout, "Homing all joints!\n");
            allhoming = 1;
        }
        else if( ctrl.active != 2 )
            allhoming = 0;

        if( 0 < dt && dt < 0.1 )
        {
            for(int jnt=0; jnt<HUBO_JOINT_COUNT; jnt++)
            {
                if( H_ref.ref[jnt] != r0[jnt] )
                {
                    fprintf(stdout, "Warning: Ref value of joint %d changed outside of control-daemon\n", jnt);
                    V[jnt]=0; V0[jnt]=0; r[jnt]=H_ref.ref[jnt]; r0[jnt]=H_ref.ref[jnt]; reset[jnt]=1;
                }

                err = H_ref.ref[jnt] - H_state.joint[jnt].pos;

                if( fabs(err) <= 10000//fabs(ctrl.joint[jnt].accel_limit/2.0*dt*dt
                         // + ctrl.joint[jnt].speed_limit) // TODO: Validate this condition
                    && fail[jnt]==0 )
                {

                    if( ctrl.joint[jnt].mode != CTRL_OFF && reset[jnt]==0 )
                    {

                        // Deal with homing joints -- reset all internal values
                        if( (ctrl.joint[jnt].mode == CTRL_HOME && home[jnt]==0) || ctrl.active==2 )
                        {
                            H_ref.ref[jnt] = 0; 
                            V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                            r[jnt]=0; r0[jnt]=0; dr[jnt]=0;
                            homing++; home[jnt]=1;

                            if( ctrl.active==1 )
                            {
                            	fprintf(stdout, "Homing joint #%d\n", jnt);
                                H_cmd.type = D_GOTO_HOME;
                                H_cmd.joint = jnt;
                                //ach_put( &chan_hubo_board_cmd, &H_cmd, sizeof(H_cmd) );
                            }
                        }
                        
                        if( ctrl.joint[jnt].mode != CTRL_HOME )
                            home[jnt]=0;

                        if( ctrl.joint[jnt].mode == CTRL_VEL )
                        {
                            dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity
                            if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                                dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                            else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) )
                                dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);

                            V[jnt] = V0[jnt] + dV[jnt]; // Step velocity forward

                            dr[jnt] = V[jnt]*dt;

                            if( H_ref.ref[jnt]+dr[jnt] < ctrl.joint[jnt].pos_min )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_min;
                            else if( H_ref.ref[jnt]+dr[jnt] > ctrl.joint[jnt].pos_max )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_max;
                            else
                                H_ref.ref[jnt] += dr[jnt];

                        }
                        else if( ctrl.joint[jnt].mode == CTRL_POS )
                        {
                            dr[jnt] = ctrl.joint[jnt].position - H_ref.ref[jnt]; // Check how far we are from desired position
                            ctrl.joint[jnt].velocity = sign(dr[jnt])*fabs(ctrl.joint[jnt].velocity); // Set velocity into the correct direction
			if(jnt=LSP)
				printf("dt:%f\trd:%f\tdri:%f\t",dt,ctrl.joint[jnt].position,dr[jnt]);

                            dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity
			if(jnt=LSP)
				printf("dVi:%f\t",dV[jnt]);

                            adr = sqrt(fabs(2.0*ctrl.joint[jnt].acceleration*dr[jnt]));
                            if( fabs(V0[jnt]) >= adr ) // Slow down before reaching goal
                                dV[jnt] = sign(V0[jnt])*adr-V0[jnt];
                            else if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                                dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                            else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) ) // Make sure the sign is correct
                                dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);
			
			if(jnt=LSP)
				printf("dV:%f\t",dV[jnt]);

                            V[jnt] = V0[jnt] + dV[jnt]; // Step velocity forward

                            if( fabs(dr[jnt]) > fabs(V[jnt]*dt) && V[jnt]*dr[jnt] >= 0 )
                                dr[jnt] = V[jnt]*dt;
                            else if( fabs(dr[jnt]) > fabs(V[jnt]*dt) && V[jnt]*dr[jnt] < 0 )
                                dr[jnt] = -V[jnt]*dt;

                            if( H_ref.ref[jnt]+dr[jnt] < ctrl.joint[jnt].pos_min )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_min;
                            else if( H_ref.ref[jnt]+dr[jnt] > ctrl.joint[jnt].pos_max )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_max;
                            else
                                H_ref.ref[jnt] += dr[jnt];
			if(jnt==LSP)
				printf("r:%f\tdr:%f\tV:%f\n", H_ref.ref[jnt], dr[jnt], V[jnt]);
			
                        }
                        else if( ctrl.joint[jnt].mode != CTRL_HOME )
                            fprintf(stderr, "Joint %d has invalid control mode: %d\n",
                                jnt, (int)ctrl.joint[jnt].mode );

                            r0[jnt] = H_ref.ref[jnt];
                            V0[jnt] = V[jnt];
                    }
                    else if( reset[jnt]==1 )
                    {
                        fprintf( stdout, "Joint %d has been reset\n", jnt );
                        reset[jnt]=0;
                    }


                }
                else if(fail[jnt]==0)
                {
                    H_ref.ref[jnt] = H_state.joint[jnt].pos;
                    fprintf(stderr, "JOINT %d FROZEN! Exceeded error limit(%g): %g\n", jnt,
                        fabs(ctrl.joint[jnt].accel_limit/2.0*dt*dt + ctrl.joint[jnt].speed_limit),
                        err );
                    fail[jnt]++;
                }
            }

        /*    printf( "Joint values: %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f\n",
                H_ref.ref[RSP], H_ref.ref[RSR], H_ref.ref[RSY],
                H_ref.ref[REB], H_ref.ref[RWY], H_ref.ref[RWP] );
        */
        /*    printf( "Joint values: %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f\n",
                V[LSP], V[LSR], V[LSY], V[LEB], V[LWY], V[LWP] );
        */
            if(homing>0||allhoming>0)
                sleep(5);

            if(ctrl.active == 1) 
            {
                //ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );
            }
            
            homing = 0;
        }
        else if( dt >= 0.1 )
            fprintf(stderr, "Experiencing Delay of %f seconds\n", dt);
        t0 = t;

	fflush(stdout);
	fflush(stderr);
    } // End of Main Control Loop

}




int main(int argc, char **argv)
{
    // TODO: Parse runtime arguments


    //daemonize( "control-daemon" );

    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_ctrl, HUBO_CHAN_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    controlLoop();

    daemon_close();
}


void setCtrlDefaults( struct hubo_control *ctrl )
{
    // TODO: Make this into something that parses default.ctrl table files

	memset( ctrl, 0, sizeof(struct hubo_control) );

	ctrl->active = 0;

	for(int i=0; i<HUBO_JOINT_COUNT; i++)
	{
		ctrl->joint[i].mode = CTRL_POS;
		ctrl->joint[i].velocity = 0.75;
		ctrl->joint[i].acceleration = 0.4;
		ctrl->joint[i].speed_limit = 0.75*100;
	}

	ctrl->joint[LSP].pos_min = -2.0;
	ctrl->joint[LSR].pos_min = -0.3;
	ctrl->joint[LSY].pos_min = -2.25;
	ctrl->joint[LEB].pos_min = -2.0;
	ctrl->joint[LWY].pos_min = -2.0;
	ctrl->joint[LWP].pos_min = -1.25;
	

	ctrl->joint[LSP].pos_max = 2.0;
	ctrl->joint[LSR].pos_max = 2.0;
	ctrl->joint[LSY].pos_max = 2.0;
	ctrl->joint[LEB].pos_max = 0.0;
	ctrl->joint[LWY].pos_max = 2.0;
	ctrl->joint[LWP].pos_max = 1.0;

}









