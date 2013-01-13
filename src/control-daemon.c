#include "control-daemon.h"

ach_channel_t chan_hubo_ref;
ach_channel_t chan_hubo_board_cmd;
ach_channel_t chan_hubo_state;
ach_channel_t chan_hubo_ra_ctrl;
ach_channel_t chan_hubo_la_ctrl;
ach_channel_t chan_hubo_rl_ctrl;
ach_channel_t chan_hubo_ll_ctrl;
ach_channel_t chan_hubo_rf_ctrl;
ach_channel_t chan_hubo_lf_ctrl;
ach_channel_t chan_hubo_aux_ctrl;


void controlLoop();
void setCtrlDefaults( struct hubo_control *ctrl );

void sortJointControls( struct hubo_control *ctrl, struct hubo_arm_control *ractrl, struct hubo_arm_control *lactrl,
                                                   struct hubo_leg_control *rlctrl, struct hubo_leg_control *llctrl,
                                                   struct hubo_fin_control *rfctrl, struct hubo_fin_control *lfctrl,
                        struct hubo_aux_control *auxctrl )
{
    for(int i=0; i<ARM_JOINT_COUNT; i++)
    {
        memcpy( &(ctrl->joint[rightarmjoints[i]]), &(ractrl->joint[i]), sizeof(struct hubo_joint_control) );
        memcpy( &(ctrl->joint[leftarmjoints[i]]),  &(lactrl->joint[i]), sizeof(struct hubo_joint_control) );
    }

    for(int i=0; i<LEG_JOINT_COUNT; i++)
    {
        memcpy( &(ctrl->joint[rightlegjoints[i]]), &(rlctrl->joint[i]), sizeof(struct hubo_joint_control) );
        memcpy( &(ctrl->joint[leftlegjoints[i]]),  &(llctrl->joint[i]), sizeof(struct hubo_joint_control) );
    }

    for(int i=0; i<FIN_JOINT_COUNT; i++)
    {
        memcpy( &(ctrl->joint[rightfinjoints[i]]), &(rfctrl->joint[i]), sizeof(struct hubo_joint_control) );
        memcpy( &(ctrl->joint[leftfinjoints[i]]),  &(lfctrl->joint[i]), sizeof(struct hubo_joint_control) );
    }

    for(int i=0; i<AUX_JOINT_COUNT; i++)
    {
        memcpy( &(ctrl->joint[auxjoints[i]]), &(auxctrl->joint[i]), sizeof(struct hubo_joint_control) );
    }
    
    if( ractrl->active==2 && lactrl->active==2 && rlctrl->active==2 && llctrl->active==2
        || rfctrl->active==2 && lfctrl->active==2 && auxctrl->active==2 )
    {
        ctrl->active = 2;
    }
    else if( ractrl->active==1 || lactrl->active==1 || rlctrl->active==1 || llctrl->active==1
        || rfctrl->active==1 || lfctrl->active==1 || auxctrl->active==1 )
    {
        ctrl->active = 1;
    }
    else
        ctrl->active = 0;
}

double sign(double x)
{
    return (x < 0) ? -1 : (x > 0);
}

void controlLoop()
{
    struct hubo_ref H_ref;//, r_check;
    struct hubo_board_cmd H_cmd;
    struct hubo_state H_state;
    struct hubo_control ctrl;
    struct hubo_arm_control ractrl;
    struct hubo_arm_control lactrl;
    struct hubo_leg_control rlctrl;
    struct hubo_leg_control llctrl;
    struct hubo_fin_control rfctrl;
    struct hubo_fin_control lfctrl;
    struct hubo_aux_control auxctrl;
    struct hubo_param H_param;

    memset( &H_ref,   0, sizeof(H_ref)   );
    memset( &H_cmd,   0, sizeof(H_cmd)   );
    memset( &H_state, 0, sizeof(H_state) );
    memset( &ctrl,    0, sizeof(ctrl)    );
    memset( &ractrl,  0, sizeof(ractrl)  );
    memset( &lactrl,  0, sizeof(lactrl)  );
    memset( &rlctrl,  0, sizeof(rlctrl)  );
    memset( &llctrl,  0, sizeof(llctrl)  );
    memset( &rfctrl,  0, sizeof(rfctrl)  );
    memset( &lfctrl,  0, sizeof(lfctrl)  );
    memset( &auxctrl, 0, sizeof(auxctrl) );
    memset( &H_param, 0, sizeof(H_param) );

    setJointParams( &H_param, &H_state);
//printf("About to set ctrldef\n"); fflush(stdout);
    setCtrlDefaults( &ctrl );
//printf("Set ctrldef\n"); fflush(stdout);

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
        // TODO: Print a debug message
    }
    else { daemon_assert( sizeof(H_state) == fs, __LINE__ ); }

    double dr[HUBO_JOINT_COUNT];
    double V[HUBO_JOINT_COUNT];
    double V0[HUBO_JOINT_COUNT];
    double dV[HUBO_JOINT_COUNT];
    double adr;
    double dtMax = 0.1;
    double errorFactor = 10;

    int fail[HUBO_JOINT_COUNT];
    int reset[HUBO_JOINT_COUNT];
    int cresult, rresult, sresult, presult, iter=0, maxi=15;

    // Initialize arrays
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        dr[i] = 0;  dV[i] = 0;
        V[i] = 0;   V0[i] = 0;

        fail[i] = 0;
    }

    double t0 = H_state.time;
    double t, dt, err;

    fprintf(stdout, "Beginning control loop\n");

    // Main control loop
    while( !daemon_sig_quit )
    {
/*
        cresult = ach_get( &chan_hubo_ctrl, &ctrl, sizeof(ctrl), &fs, NULL, ACH_O_LAST );
        if( ACH_OK != cresult )
        {
            // TODO: Print a debug message
        }
        else 
        {
            daemon_assert( sizeof(ctrl) == fs, __LINE__ );
            //printf("Got ctrl ach.\n");
            //printf("mode:%d\tactive:%d\n", (int)ctrl.joint[LSP].mode, ctrl.active);
        }
*/
        cresult = ach_get( &chan_hubo_ra_ctrl, &ractrl, sizeof(ractrl), &fs, NULL, ACH_O_LAST );
        cresult = ach_get( &chan_hubo_la_ctrl, &lactrl, sizeof(lactrl), &fs, NULL, ACH_O_LAST );
        cresult = ach_get( &chan_hubo_rl_ctrl, &rlctrl, sizeof(rlctrl), &fs, NULL, ACH_O_LAST );
        cresult = ach_get( &chan_hubo_ll_ctrl, &llctrl, sizeof(llctrl), &fs, NULL, ACH_O_LAST );
        cresult = ach_get( &chan_hubo_rf_ctrl, &rfctrl, sizeof(rfctrl), &fs, NULL, ACH_O_LAST );
        cresult = ach_get( &chan_hubo_lf_ctrl, &lfctrl, sizeof(lfctrl), &fs, NULL, ACH_O_LAST );
        cresult = ach_get( &chan_hubo_aux_ctrl, &auxctrl, sizeof(auxctrl), &fs, NULL, ACH_O_LAST );

        sortJointControls( &ctrl, &ractrl, &lactrl,
                                  &rlctrl, &llctrl,
                                  &rfctrl, &lfctrl, &auxctrl );



        sresult = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
        if( ACH_OK != sresult )
        {
            // TODO: Print a debug message
        }
        else { daemon_assert( sizeof(H_state) == fs, __LINE__ ); }

        t = H_state.time;
        dt = t - t0;

        
        if(ctrl.active == 2 && H_ref.paused==0)
        {
            H_ref.paused=1;
            ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );
            fprintf(stdout, "Pausing control\n");
            for(int jnt=0; jnt<HUBO_JOINT_COUNT; jnt++)
            {
                if( ctrl.joint[jnt].mode == CTRL_HOME )
                {
                    H_ref.ref[jnt] = 0; 
                    V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                    dr[jnt]=0;
//    if(jnt==LSR) printf("Homed. r:%f\n", H_ref.ref[jnt]);
                }
            }
        }
        else if(ctrl.active==1)
            H_ref.paused=0;


        if( 0 < dt && dt < dtMax && H_state.refWait==0 )
        {
            iter++; if(iter>maxi) iter=0;
//    if(iter==maxi) printf("Active:%d Joint:(%d,%d) Mode:(%d,%d) Position:%f Velocity:%f Accel:%f\n", ctrl.active, 1, leftarmjoints[1], (int)lactrl.joint[1].mode, (int)ctrl.joint[leftarmjoints[1]].mode,
//                                        ctrl.joint[leftarmjoints[1]].position,ctrl.joint[leftarmjoints[1]].velocity,ctrl.joint[leftarmjoints[1]].acceleration);
            for(int jnt=0; jnt<HUBO_JOINT_COUNT; jnt++)
            {
                err = H_ref.ref[jnt] - H_state.joint[jnt].pos;

                if( fabs(err) <=  fabs(errorFactor*ctrl.joint[jnt].speed_limit*dtMax) // TODO: Validate this condition
                    && fail[jnt]==0 )
                {
                    if( ctrl.joint[jnt].mode != CTRL_OFF && ctrl.joint[jnt].mode != CTRL_RESET )
                    {

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
//			if(jnt==LSR && maxi==iter) printf("err:%f\tVdi:%f\t",err,ctrl.joint[jnt].velocity);
			
                            ctrl.joint[jnt].velocity = sign(dr[jnt])*fabs(ctrl.joint[jnt].velocity); // Set velocity into the correct direction

//			if(jnt==LSR && maxi==iter) printf("dt:%f\trd:%f\tdri:%f\t",dt,ctrl.joint[jnt].position,dr[jnt]);

                            dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity

//			if(jnt==LSR && maxi==iter) printf("Vd:%f\tV0:%f\tdVi:%f\t",ctrl.joint[jnt].velocity,V0[jnt],dV[jnt]);

                            adr = sqrt(fabs(2.0*ctrl.joint[jnt].acceleration*dr[jnt]));
                            if( fabs(V0[jnt]) >= adr ) // Slow down before reaching goal
                                dV[jnt] = sign(dr[jnt])*adr-V0[jnt];

                            if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                                dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                            else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) ) // Make sure the sign is correct
                                dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);
			
//			if(jnt==LSR && maxi==iter) printf("dV:%f\t",dV[jnt]);

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

//			if(jnt==LSR && maxi==iter) printf("r:%f\tdr:%f\tV:%f\n", H_ref.ref[jnt], dr[jnt], V[jnt]);
			
                        }
                        else if( ctrl.joint[jnt].mode == CTRL_HOME )
                        {
                            H_ref.ref[jnt] = 0; 
                            V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                            //r[jnt]=0; r0[jnt]=0;
                            dr[jnt]=0;
//            if(jnt==LSR && maxi==iter) printf("Homed. r:%f\n", H_ref.ref[jnt]);
                            
                        }
                        else
                        {
                            fprintf(stderr, "Joint %d has invalid control mode: %d\n",
                                jnt, (int)ctrl.joint[jnt].mode );
                        }

                        //r0[jnt] = H_ref.ref[jnt];
                        V0[jnt] = V[jnt];
                        reset[jnt]=0;
                    }

                }
                else if(fail[jnt]==0 && ctrl.joint[jnt].mode != CTRL_HOME)
                {
                    fprintf(stderr, "JOINT %d FROZEN! Exceeded error limit(%g):%g, Ref:%f, State:%f\n", jnt,
                        fabs(errorFactor*ctrl.joint[jnt].speed_limit*dtMax), err, H_ref.ref[jnt], H_state.joint[jnt].pos );
                    V[jnt]=0; V0[jnt]=0;
                    H_ref.ref[jnt] = H_state.joint[jnt].pos;
                    fail[jnt]=1;
                    H_ref.status[jnt] = 1;
                }
                else if( ctrl.joint[jnt].mode == CTRL_RESET && reset[jnt]==0 )
                {
                    fprintf(stdout, "Joint %d has been reset\n", jnt);
                    V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                    dr[jnt]=0;
                    fail[jnt]=0;
                    reset[jnt]=1;
                    H_ref.status[jnt] = 0;
                }
            }

            if(ctrl.active == 1 && H_ref.paused==0) 
            { //if(iter==maxi) printf("Sending ACH, r:%f\n", H_ref.ref[LSP]);
                presult = ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );
                if(presult != ACH_OK)
                    fprintf(stderr, "Error sending ref command! (%d) %s\n",
                        presult, ach_result_to_string(presult));
            }
            
        }
        else if( dt >= 0.1 )
            fprintf(stderr, "Experiencing Delay of %f seconds\n", dt);
        else if( dt < 0 )
            fprintf(stderr, "Congratulations! You have traveled backwards"
                            " through time by %f seconds!", -dt);
        t0 = t;

        fflush(stdout);
        fflush(stderr);
    } // End of Main Control Loop

}





int main(int argc, char **argv)
{
    // TODO: Parse runtime arguments


    daemonize( "control-daemon" );

    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_ra_ctrl, HUBO_CHAN_RA_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_la_ctrl, HUBO_CHAN_LA_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_rl_ctrl, HUBO_CHAN_RL_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_ll_ctrl, HUBO_CHAN_LL_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_rf_ctrl, HUBO_CHAN_RF_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_lf_ctrl, HUBO_CHAN_LF_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_aux_ctrl, HUBO_CHAN_AUX_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    controlLoop();

    daemon_close();
}


void setCtrlDefaults( struct hubo_control *ctrl )
{
    // TODO: Make this into something that parses default.ctrl table files

    struct hubo_arm_control ractrl;
    struct hubo_arm_control lactrl;
    struct hubo_leg_control rlctrl;
    struct hubo_leg_control llctrl;
    struct hubo_fin_control rfctrl;
    struct hubo_fin_control lfctrl;
    struct hubo_aux_control auxctrl;

	memset( ctrl, 0, sizeof(struct hubo_control) );
	memset( &ractrl, 0, sizeof(struct hubo_arm_control) );
	memset( &lactrl, 0, sizeof(struct hubo_arm_control) );
	memset( &rlctrl, 0, sizeof(struct hubo_leg_control) );
	memset( &llctrl, 0, sizeof(struct hubo_leg_control) );
	memset( &rfctrl, 0, sizeof(struct hubo_fin_control) );
	memset( &lfctrl, 0, sizeof(struct hubo_fin_control) );
	memset( &auxctrl, 0, sizeof(struct hubo_aux_control) );

    for(int i=0; i<ARM_JOINT_COUNT; i++)
    {
        lactrl.joint[i].mode = CTRL_HOME;
        lactrl.joint[i].velocity = 0.75;
        lactrl.joint[i].acceleration = 0.4;
        lactrl.joint[i].speed_limit = 1.0;
        lactrl.joint[i].pos_min = 0;
        lactrl.joint[i].pos_max = 0;

        ractrl.joint[i].mode = CTRL_HOME;
        ractrl.joint[i].velocity = 0.75;
        ractrl.joint[i].acceleration = 0.4;
        ractrl.joint[i].speed_limit = 1.0;
        ractrl.joint[i].pos_min = 0;
        ractrl.joint[i].pos_max = 0;


        rlctrl.joint[i].mode = CTRL_HOME;
        rlctrl.joint[i].velocity = 0.5;
        rlctrl.joint[i].acceleration = 0.3;
        rlctrl.joint[i].speed_limit = 1.0;
        rlctrl.joint[i].pos_min = 0;
        rlctrl.joint[i].pos_max = 0;

        llctrl.joint[i].mode = CTRL_HOME;
        llctrl.joint[i].velocity = 0.5;
        llctrl.joint[i].acceleration = 0.3;
        llctrl.joint[i].speed_limit = 1.0;
        llctrl.joint[i].pos_min = 0;
        llctrl.joint[i].pos_max = 0;
    }

    for(int i=0; i<AUX_JOINT_COUNT; i++)
    {
        auxctrl.joint[i].mode = CTRL_HOME;
    }

    // Left Arm Min
    lactrl.joint[0].pos_min = -2.0;
    lactrl.joint[1].pos_min = -0.3;
    lactrl.joint[2].pos_min = -2.25;
    lactrl.joint[3].pos_min = -2.5;
    lactrl.joint[4].pos_min = -2.0;
    lactrl.joint[5].pos_min = -1.25;
    // Left Arm Max
    lactrl.joint[0].pos_max = 2.0;
    lactrl.joint[1].pos_max = 2.0;
    lactrl.joint[2].pos_max = 2.0;
    lactrl.joint[3].pos_max = 0.0;
    lactrl.joint[4].pos_max = 2.0;
    lactrl.joint[5].pos_max = 1.0;

    // Right Arm Min
    ractrl.joint[0].pos_min = -2.0;
    ractrl.joint[1].pos_min = -2.0;
    ractrl.joint[2].pos_min = -2.0;
    ractrl.joint[3].pos_min = -2.5;
    ractrl.joint[4].pos_min = -2.0;
    ractrl.joint[5].pos_min = -1.4;
    // Right Arm Max
    ractrl.joint[0].pos_max = 2.0;
    ractrl.joint[1].pos_max = 0.3;
    ractrl.joint[2].pos_max = 2.0;
    ractrl.joint[3].pos_max = 0.0;
    ractrl.joint[4].pos_max = 2.0;
    ractrl.joint[5].pos_max = 1.2;

    // Left Leg Min
    llctrl.joint[0].pos_min = -1.30;
    llctrl.joint[1].pos_min =  0.00;
    llctrl.joint[2].pos_min =  0.00;
    llctrl.joint[3].pos_min =  0.00;
    llctrl.joint[4].pos_min = -1.26;
    llctrl.joint[5].pos_min = -0.31;
    // Left Leg Max
    llctrl.joint[0].pos_max = 1.30;
    llctrl.joint[1].pos_max = 0.58;
    llctrl.joint[2].pos_max = 1.80;
    llctrl.joint[3].pos_max = 2.50;
    llctrl.joint[4].pos_max = 1.80;
    llctrl.joint[5].pos_max = 0.23;

    // Right Leg Min
    rlctrl.joint[0].pos_min = -1.30;
    rlctrl.joint[1].pos_min = -0.58;
    rlctrl.joint[2].pos_min = -1.80;
    rlctrl.joint[3].pos_min =  0.00;
    rlctrl.joint[4].pos_min = -1.26;
    rlctrl.joint[5].pos_min = -0.23;
    // Right Leg Max
    rlctrl.joint[0].pos_max = 1.30;
    rlctrl.joint[1].pos_max = 0.00;
    rlctrl.joint[2].pos_max = 0.00;
    rlctrl.joint[3].pos_max = 2.50;
    rlctrl.joint[4].pos_max = 1.80;
    rlctrl.joint[5].pos_max = 0.31;


    ach_put( &chan_hubo_ra_ctrl, &ractrl, sizeof(ractrl) );
    ach_put( &chan_hubo_la_ctrl, &lactrl, sizeof(lactrl) );
    ach_put( &chan_hubo_rl_ctrl, &rlctrl, sizeof(rlctrl) ); 
    ach_put( &chan_hubo_ll_ctrl, &llctrl, sizeof(llctrl) );
    ach_put( &chan_hubo_rf_ctrl, &rfctrl, sizeof(rfctrl) );
    ach_put( &chan_hubo_lf_ctrl, &lfctrl, sizeof(lfctrl) );
    ach_put( &chan_hubo_aux_ctrl, &auxctrl, sizeof(auxctrl) );
/*
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        ctrl->joint[i].mode = CTRL_HOME;
        ctrl->joint[i].velocity = 0.75;
        ctrl->joint[i].acceleration = 0.4;
        ctrl->joint[i].speed_limit = 1.0;
        ctrl->joint[i].pos_min = 0;
        ctrl->joint[i].pos_max = 0;
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


    ctrl->joint[RSP].pos_min = -2.0;
    ctrl->joint[RSR].pos_min = -2.0;
    ctrl->joint[RSY].pos_min = -2.0;
    ctrl->joint[REB].pos_min = -2.0;
    ctrl->joint[RWY].pos_min = -2.0;
    ctrl->joint[RWP].pos_min = -1.4;


    ctrl->joint[RSP].pos_max = 2.0;
    ctrl->joint[RSR].pos_max = 0.3;
    ctrl->joint[RSY].pos_max = 2.0;
    ctrl->joint[REB].pos_max = 0.0;
    ctrl->joint[RWY].pos_max = 2.0;
    ctrl->joint[RWP].pos_max = 1.2;
*/
 
    

}







