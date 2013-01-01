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

    size_t fs;
    int result = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != result )
    {
        // TODO: Debug stuff
    }
    else
    {
        daemon_assert( sizeof(H_state) == fs );
    }

    result = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != result )
    {
        // TOOD: Print a debug message
    }
    else { daemon_assert( sizeof(H_state) == fs ); }

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

        result = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
        if( ACH_OK != result )
        {
            // TODO: Print a debug message
        }
        else { daemon_assert( sizeof(H_ref) == fs ); }

        result = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_WAIT );
        if( ACH_OK != result )
        {
            // TOOD: Print a debug message
        }
        else { daemon_assert( sizeof(H_state) == fs ); }

        t = H_state.time;
        dt = t - t0;



        for(int jnt=0; jnt<HUBO_JOINT_COUNT; jnt++)
        {
            if( H_ref.ref[jnt] != r0[jnt] )
                printf(stdout, "Warning: Ref value of joint %d changed outside of control-daemon\n", jnt);

            err = H_ref.ref[jnt] - H_state.joint[jnt].pos;

            if( fabs(err) <= fabs(ctrl.joint[jnt].accel_limit/2.0*dt*dt
                                  + ctrl.joint[jnt].speed_limit) // TODO: Validate this condition
                    && fail[jnt]==0 )
            {

                if( ctrl.joint[jnt].mode != CTRL_OFF )
                {
                    if( ctrl.joint[jnt].mode == CTRL_POS )
                    {// This is to make sure the velocity's sign agrees with the desired change in position
                        dr[jnt] = ctrl.joint[jnt].position - H_ref.ref[jnt];
                        ctrl.joint[jnt].velocity = sign(dr[jnt])*fabs(ctrl.joint[jnt].velocity);
                    }

                    dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity
                    if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                        dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                    else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) )
                        dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);

                    V[jnt] = V0[jnt] + dV; // Step velocity forward

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


                        if( fabs(dr[jnt]) > fabs(V[i]*dt) && V[i]*dr[jnt] >= 0 )
                            dr[jnt] = V[jnt]*dt;
                        else if( fabs(dr) > fabs(V[i]*dt) && V[i]*dr[jnt] < 0 )
                            dr[jnt] = -V[jnt]*dt;



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


        if(ctrl.active == 0) // TODO: Decide if this should be 0 or 1
        {
            ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );
        }

        t0 = t;

    } // End of Main Control Loop

}




int main(int argc, char **argv)
{
    // TODO: Parse runtime arguments


    // daemonize
    daemonize( "control-daemon" );

    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
    daemon_assert( ACH_OK == r );

    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
    daemon_assert( ACH_OK == r );

    r = ach_open(&chan_hubo_ctrl, HUBO_CHAN_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r );

    controlLoop();

    daemon_close();
}
