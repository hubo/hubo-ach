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

    ach_put( &chan_hubo_ctrl, &ctrl, sizeof(ctrl) ); 

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

    int fail[HUBO_JOINT_COUNT];

    // Initialize arrays
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        r[i] = 0;   r0[i] = 0;  dr[i] = 0;
        V[i] = 0;   V0[i] = 0;  dV[i] = 0;

        fail[i] = 0;
    }

    double t0 = H_state.time;
    double t, dt, err;

    // Main control loop
    while( !daemon_sig_quit )
    {

        int cresult = ach_get( &chan_hubo_ctrl, &ctrl, sizeof(ctrl), &fs, NULL, ACH_O_LAST );
        if( ACH_OK != result )
        {
            // TODO: Print a debug message
        }
        else 
	{	
		daemon_assert( sizeof(ctrl) == fs, __LINE__ );
		printf("Got ctrl ach\n");
		printf("%f\t%f\t%f\t%f\t%f\t%f\n",
			ctrl.joint[LSP].position, r[LSP],
			ctrl.joint[LSR].position, r[LSR],
			ctrl.joint[LSY].position, r[LSY]);
	}

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



        for(int jnt=0; jnt<HUBO_JOINT_COUNT; jnt++)
        {
            if( H_ref.ref[jnt] != r0[jnt] )
                ;//fprintf(stdout, "Warning: Ref value of joint %d changed outside of control-daemon\n", jnt);

            err = H_ref.ref[jnt] - H_state.joint[jnt].pos;

            if( fabs(err) <= 10000//fabs(ctrl.joint[jnt].accel_limit/2.0*dt*dt
                                 // + ctrl.joint[jnt].speed_limit) // TODO: Validate this condition
                    && fail[jnt]==0 )
            {

                if( ctrl.joint[jnt].mode != CTRL_OFF )
                {
                    if( ctrl.joint[jnt].mode == CTRL_POS )
                    {// This is to make sure the velocity's sign agrees with the desired change in position
                        dr[jnt] = ctrl.joint[jnt].position - H_ref.ref[jnt];
                        ctrl.joint[jnt].velocity = sign(dr[jnt])*fabs(ctrl.joint[jnt].velocity);
                    }
			if(cresult == ACH_OK && jnt==LSP)
				printf("dri: %f\t", dr[jnt]);

                    dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity
			
			if(cresult == ACH_OK && jnt==LSP)
				printf("V_bar: %f\t", ctrl.joint[jnt].velocity);
			if(cresult == ACH_OK && jnt==LSP)
				printf("V0: %f\t", V0[jnt]);
			if(cresult == ACH_OK && jnt==LSP)
				printf("dVi: %f\t", dV[jnt]);

			if(cresult == ACH_OK && jnt==LSP)
				printf("dt: %f\t", dt );

                    if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                        dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                    else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) )
                        dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);

			if(cresult == ACH_OK && jnt==LSP)
				printf("dV: %f\t", dV[jnt]);

                    V[jnt] = V0[jnt] + dV[jnt]; // Step velocity forward

			if(cresult == ACH_OK && jnt==LSP)
				printf("Vi: %f\t", V[jnt]);

                    if( ctrl.joint[jnt].mode == CTRL_VEL )
                    {

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

                        if( fabs(V[jnt]) > sqrt(fabs(2.0*ctrl.joint[jnt].acceleration*dr[jnt])) ) // Slow down before reaching goal
                            V[jnt] += -sign(V[jnt])*fabs(ctrl.joint[jnt].acceleration*dt);


			if(cresult == ACH_OK && jnt==LSP)
				printf("V: %f\t", V[jnt]);

                        if( fabs(dr[jnt]) > fabs(V[jnt]*dt) && V[jnt]*dr[jnt] >= 0 )
                            dr[jnt] = V[jnt]*dt;
                        else if( fabs(dr[jnt]) > fabs(V[jnt]*dt) && V[jnt]*dr[jnt] < 0 )
                            dr[jnt] = -V[jnt]*dt;

			if(cresult == ACH_OK && jnt==LSP)
				printf("dr: %f\t\n", dr[jnt]);

                        if( H_ref.ref[jnt]+dr[jnt] < ctrl.joint[jnt].pos_min )
                            H_ref.ref[jnt] = ctrl.joint[jnt].pos_min;
                        else if( H_ref.ref[jnt]+dr[jnt] > ctrl.joint[jnt].pos_max )
                            H_ref.ref[jnt] = ctrl.joint[jnt].pos_max;
                        else
                            H_ref.ref[jnt] += dr[jnt];
                    }
                    else
                        fprintf(stderr, "Joint %d has invalid control mode: %d\n",
                                jnt, (int)ctrl.joint[jnt].mode );

                    r0[jnt] = H_ref.ref[jnt];
                    V0[jnt] = V[jnt];
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

/*	printf( "Joint values: %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f\n",
                H_ref.ref[RSP], H_ref.ref[RSR], H_ref.ref[RSY],
                H_ref.ref[REB], H_ref.ref[RWY], H_ref.ref[RWP] );
*/
/*	printf( "Joint values: %2.2f, %2.2f, %2.2f, %2.2f, %2.2f, %2.2f\n",
		V[LSP], V[LSR], V[LSY], V[LEB], V[LWY], V[LWP] );
*/
        if(ctrl.active == 0) // TODO: Decide if this should be 0 or 1
        {
            ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );
        }

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









