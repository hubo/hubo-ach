#include "../include/hubo_plus.h"

hubo_plus::hubo_plus()
{
    memset( &H_Ref,   0, sizeof(H_Ref)   );
    memset( &H_Cmd,   0, sizeof(H_Cmd)   );
    memset( &H_State, 0, sizeof(H_State) );
    memset( &H_Ctrl,  0, sizeof(H_Ctrl)  );
    memset( &H_Param, 0, sizeof(H_Param) );

    setJointParams( &H_Param, &H_State );

    int r = ach_open( &chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_ctrl, HUBO_CHAN_CTRL_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL );
    assert( ACH_OK == r );

    size_t fs;

    ach_get( &chan_hubo_ref, &H_Ref, sizeof(H_Ref), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_ctrl, &H_Ctrl, sizeof(H_Ctrl), &fs, NULL, ACH_O_LAST );


    armjoints[LEFT][0] = LSP;
    armjoints[LEFT][1] = LSR;
    armjoints[LEFT][2] = LSY;
    armjoints[LEFT][3] = LEB;
    armjoints[LEFT][4] = LWY;
    armjoints[LEFT][5] = LWP;

    armjoints[RIGHT][0] = RSP;
    armjoints[RIGHT][1] = RSR;
    armjoints[RIGHT][2] = RSY;
    armjoints[RIGHT][3] = REB;
    armjoints[RIGHT][4] = RWY;
    armjoints[RIGHT][5] = RWP;

    legjoints[LEFT][0] = LHP;
    legjoints[LEFT][1] = LHR;
    legjoints[LEFT][2] = LHY;
    legjoints[LEFT][3] = LKN;
    legjoints[LEFT][4] = LAP;
    legjoints[LEFT][5] = LAR;

    legjoints[RIGHT][0] = RHP;
    legjoints[RIGHT][1] = RHR;
    legjoints[RIGHT][2] = RHY;
    legjoints[RIGHT][3] = RKN;
    legjoints[RIGHT][4] = RAP;
    legjoints[RIGHT][5] = RAR;
}

hubo_plus::hubo_plus(const char *daemon_name)
{
    hubo_plus();

    daemonize(daemon_name);
}

double hubo_plus::getTime() { return H_State.time; }

hp_flag_t hubo_plus::update(bool printError)
{
    int r1, r2;
    size_t fs;
    r1 = ach_get( &chan_hubo_ref, &H_Ref, sizeof(H_Ref), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != r1 && printError )
        fprintf( stdout, "Ach report -- Ref Channel: %s at time=%f",
			ach_result_to_string((ach_status_t)r1), getTime() );

    r2 = ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != r2 && printError )
        fprintf( stdout, "Ach report -- State Channel: %s at time=%f",
			ach_result_to_string((ach_status_t)r2), getTime() );

    if( r1==ACH_OK && r2==ACH_OK )
        return SUCCESS;
    else
    {
        if( r1==ACH_OK )
            return STATE_STALE;
        else if( r2==ACH_OK )
            return REF_STALE;
        else
            return ALL_STALE;
    }
}

void hubo_plus::sendControls()
{
    int r = ach_put( &chan_hubo_ctrl, &H_Ctrl, sizeof(H_Ctrl) );
    if( r != ACH_OK ) fprintf(stderr, "Problem sending control commands: (%d) %s\n",
                                r, ach_result_to_string((ach_status_t)r));
}
void hubo_plus::sendCommands()
{
    int r = ach_put( &chan_hubo_board_cmd, &H_Cmd, sizeof(H_Cmd) );
    if( r != ACH_OK ) fprintf(stderr, "Problem sending board commands: (%d) %s\n",
                                r, ach_result_to_string((ach_status_t)r));
}

// ~~~*** Sending Control Commands ***~~~ //
// ~~** Setting reference values

// ~* General sets
hp_flag_t hubo_plus::resetJointStatus( int joint, bool send )
{
    if( joint < HUBO_JOINT_COUNT )
        H_Ctrl.joint[joint].mode = CTRL_RESET;
    else
        return JOINT_OOB;

    if( send )
        sendControls();

    return SUCCESS;
}
// Position control
hp_flag_t hubo_plus::setPositionControl(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Ctrl.joint[joint].mode = CTRL_POS;
        H_Ctrl.joint[joint].position = H_State.joint[joint].pos;
        H_Ctrl.active=1;
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

hp_flag_t hubo_plus::setJointAngle(int joint, double radians, bool send)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Ctrl.joint[joint].position = radians;
        H_Ctrl.joint[joint].mode = CTRL_POS;
        H_Ctrl.active=1;
        if(send)
            sendControls();
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

hp_flag_t hubo_plus::setJointNominalSpeed(int joint, double speed)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        if( H_Ctrl.joint[joint].mode == CTRL_POS )
            H_Ctrl.joint[joint].velocity = speed;
        else
            return WRONG_MODE;
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

// Velocity control
hp_flag_t hubo_plus::setVelocityControl( int joint )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Ctrl.joint[joint].mode = CTRL_VEL;
        H_Ctrl.joint[joint].velocity = 0;
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

hp_flag_t hubo_plus::setJointVelocity(int joint, double vel, bool send)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Ctrl.joint[joint].velocity = vel;
        H_Ctrl.joint[joint].mode = CTRL_VEL;
        H_Ctrl.active=1;
        if(send)
            sendControls();
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

// Acceleration setting
hp_flag_t hubo_plus::setJointNominalAcceleration(int joint, double acc)
{
    if( joint < HUBO_JOINT_COUNT )
        H_Ctrl.joint[joint].acceleration = acc;
    else
        return JOINT_OOB;

    return SUCCESS;
}


// ~* Arm control sets
// Position control
hp_flag_t hubo_plus::setArmPosCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setPositionControl(armjoints[side][i]);
    else
        return BAD_SIDE;

    return SUCCESS;
}

hp_flag_t hubo_plus::setArmAngles(int side, Eigen::VectorXd angles, bool send)
{
    H_Ctrl.active=1;
    if( angles.size() < ARM_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( angles.size() > ARM_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setJointAngle(armjoints[side][i], angles(i), false);
    else
        return BAD_SIDE;


    if(send)
        sendControls();

    return SUCCESS;
}

void hubo_plus::setLeftArmPosCtrl() { setArmPosCtrl(LEFT); }

hp_flag_t hubo_plus::setLeftArmAngles(Eigen::VectorXd angles, bool send)
{ return setArmAngles( LEFT, angles, send ); }

void hubo_plus::setRightArmPosCtrl() { setArmPosCtrl(RIGHT); }

hp_flag_t hubo_plus::setRightArmAngles(Eigen::VectorXd angles, bool send)
{ return setArmAngles( RIGHT, angles, send ); }


hp_flag_t hubo_plus::setArmNomSpeeds(int side, Eigen::VectorXd speeds)
{
    if( speeds.size() < ARM_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( speeds.size() > ARM_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            if( H_Ctrl.joint[armjoints[side][i]].mode != CTRL_POS )
                return WRONG_MODE;

        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setJointNominalSpeed( armjoints[side][i], speeds(i) );
    }
    else
        return BAD_SIDE;


    return SUCCESS;
}

hp_flag_t hubo_plus::setLeftArmNomSpeeds(Eigen::VectorXd speeds)
{ return setArmNomSpeeds(LEFT, speeds); }

hp_flag_t hubo_plus::setRightArmNomSpeeds(Eigen::VectorXd speeds)
{ return setArmNomSpeeds(RIGHT, speeds); }


// Velocity Control
hp_flag_t hubo_plus::setArmVelCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setVelocityControl( armjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
}

hp_flag_t hubo_plus::setArmVels(int side, Eigen::VectorXd vels, bool send)
{
    H_Ctrl.active=1;
    if( vels.size() < ARM_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( vels.size() > ARM_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setJointVelocity(armjoints[side][i], vels(i), false);
    else
        return BAD_SIDE;

    if(send)
        sendControls();

    return SUCCESS;
}

void hubo_plus::setLeftArmVelCtrl() { setArmVelCtrl(LEFT); }

hp_flag_t hubo_plus::setLeftArmVels(Eigen::VectorXd vels, bool send)
{ return setArmVels(LEFT, vels, send); }

void hubo_plus::setRightArmVelCtrl() { setArmVelCtrl(RIGHT); }

hp_flag_t hubo_plus::setRightArmVels(Eigen::VectorXd vels, bool send)
{ return setArmVels(RIGHT, vels, send); }


// Acceleration settings
hp_flag_t hubo_plus::setArmNomAcc(int side, Eigen::VectorXd acc)
{
    if( acc.size() < ARM_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( acc.size() > ARM_JOINT_COUNT )
        return LONG_VECTOR;

    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setJointNominalAcceleration( armjoints[side][i], acc(i) );
    else
        return BAD_SIDE;

    return SUCCESS;
}

hp_flag_t hubo_plus::setLeftArmNomAcc(Eigen::VectorXd acc)
{ return setArmNomAcc( LEFT, acc ); }

hp_flag_t hubo_plus::setRightArmNomAcc(Eigen::VectorXd acc)
{ return setArmNomAcc( RIGHT, acc ); }


// ~* Leg control sets
// Position control
hp_flag_t hubo_plus::setLegPosCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setPositionControl(legjoints[side][i]);
    else
        return BAD_SIDE;

    return SUCCESS;
}

hp_flag_t hubo_plus::setLegAngles(int side, Eigen::VectorXd angles, bool send)
{
    if( angles.size() < LEG_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( angles.size() > LEG_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setJointAngle(legjoints[side][i], angles(i), false);
    else
        return BAD_SIDE;


    if(send)
        sendControls();

    return SUCCESS;
}

void hubo_plus::setLeftLegPosCtrl() { setLegPosCtrl(LEFT); }

hp_flag_t hubo_plus::setLeftLegAngles(Eigen::VectorXd angles, bool send)
{ return setLegAngles( LEFT, angles, send ); }

void hubo_plus::setRightLegPosCtrl() { setLegPosCtrl(RIGHT); }

hp_flag_t hubo_plus::setRightLegAngles(Eigen::VectorXd angles, bool send)
{ return setLegAngles( RIGHT, angles, send ); }


hp_flag_t hubo_plus::setLegNomSpeeds(int side, Eigen::VectorXd speeds)
{
    if( speeds.size() < LEG_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( speeds.size() > LEG_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            if( H_Ctrl.joint[legjoints[side][i]].mode != CTRL_POS )
                return WRONG_MODE;

        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setJointNominalSpeed( legjoints[side][i], speeds(i) );
    }
    else
        return BAD_SIDE;


    return SUCCESS;
}

hp_flag_t hubo_plus::setLeftLegNomSpeeds(Eigen::VectorXd speeds)
{ return setLegNomSpeeds(LEFT, speeds); }

hp_flag_t hubo_plus::setRightLegNomSpeeds(Eigen::VectorXd speeds)
{ return setLegNomSpeeds(RIGHT, speeds); }


// Velocity Control
hp_flag_t hubo_plus::setLegVelCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setVelocityControl( legjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
}

hp_flag_t hubo_plus::setLegVels(int side, Eigen::VectorXd vels, bool send)
{
    H_Ctrl.active=1;
    if( vels.size() < LEG_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( vels.size() > LEG_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setJointVelocity(legjoints[side][i], vels(i), false);
    else
        return BAD_SIDE;

    if(send)
        sendControls();

    return SUCCESS;
}

void hubo_plus::setLeftLegVelCtrl() { setLegVelCtrl(LEFT); }

hp_flag_t hubo_plus::setLeftLegVels(Eigen::VectorXd vels, bool send)
{ return setLegVels(LEFT, vels, send); }

void hubo_plus::setRightLegVelCtrl() { setLegVelCtrl(RIGHT); }

hp_flag_t hubo_plus::setRightLegVels(Eigen::VectorXd vels, bool send)
{ return setLegVels(RIGHT, vels, send); }


// Acceleration settings
hp_flag_t hubo_plus::setLegNomAcc(int side, Eigen::VectorXd acc)
{
    if( acc.size() < LEG_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( acc.size() > LEG_JOINT_COUNT )
        return LONG_VECTOR;

    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setJointNominalAcceleration( legjoints[side][i], acc(i) );
    else
        return BAD_SIDE;

    return SUCCESS;
}

hp_flag_t hubo_plus::setLeftLegNomAcc(Eigen::VectorXd acc)
{ return setLegNomAcc( LEFT, acc ); }

hp_flag_t hubo_plus::setRightLegNomAcc(Eigen::VectorXd acc)
{ return setLegNomAcc( RIGHT, acc ); }


// ~~** Setting limit values
// ~* General sets
hp_flag_t hubo_plus::setJointAngleMin(int joint, double radians)
{
    if( joint < HUBO_JOINT_COUNT )
        H_Ctrl.joint[joint].pos_min = radians;
    else
        return JOINT_OOB;

    return SUCCESS;
}

hp_flag_t hubo_plus::setJointAngleMax(int joint, double radians)
{
    if( joint < HUBO_JOINT_COUNT )
        H_Ctrl.joint[joint].pos_max = radians;
    else
        return JOINT_OOB;

    return SUCCESS;
}

hp_flag_t hubo_plus::setJointSpeedMax(int joint, double speed)
{
    if( joint < HUBO_JOINT_COUNT )
        H_Ctrl.joint[joint].speed_limit = speed;
    else
        return JOINT_OOB;

    return SUCCESS;
}

hp_flag_t hubo_plus::setJointAccelMax(int joint, double accel)
{
    if( joint < HUBO_JOINT_COUNT )
        H_Ctrl.joint[joint].accel_limit = accel;
    else
        return JOINT_OOB;

    return SUCCESS;
}



// ~~** Getting Reference Values
// ~* General gets
// Position control
hubo_ctrl_mode_t hubo_plus::getCtrlMode(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ctrl.joint[joint].mode;
    else
        return CTRL_OFF;
}

double hubo_plus::getJointAngle(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ctrl.joint[joint].position;
    else
        return 0;
}

double hubo_plus::getJointNominalSpeed(int joint)
{ return getJointVelocity(joint); }

// Velocity control
double hubo_plus::getJointVelocity(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ctrl.joint[joint].velocity;
    else
        return 0;
}

// Acceleration setting
double hubo_plus::getJointNominalAcceleration(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ctrl.joint[joint].acceleration;
    else
        return 0;
}

int hubo_plus::getJointStatus( int joint )
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ref.status[joint];
    else
        return 0;
}

// ~* Arm control gets
// Position control
hp_flag_t hubo_plus::getArmAngles(int side, Eigen::VectorXd &angles)
{
    if( side==LEFT || side==RIGHT )
    {
        if(angles.size() != ARM_JOINT_COUNT)
            angles.resize(ARM_JOINT_COUNT);
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            angles[i] = H_Ctrl.joint[armjoints[side][i]].position;
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void hubo_plus::getLeftArmAngles(Eigen::VectorXd &angles)
{ getArmAngles(LEFT, angles); }
void hubo_plus::getRightArmAngles(Eigen::VectorXd &angles)
{ getArmAngles(RIGHT, angles); }

hp_flag_t hubo_plus::getArmNomSpeeds(int side, Eigen::VectorXd &speeds)
{ return getArmVels(side, speeds); }
void hubo_plus::getLeftArmNomSpeeds(Eigen::VectorXd &speeds)
{ getArmVels(LEFT, speeds); }
void hubo_plus::getRightArmNomSpeeds(Eigen::VectorXd &speeds)
{ getArmVels(RIGHT, speeds); }

// Velocity control
hp_flag_t hubo_plus::getArmVels(int side, Eigen::VectorXd &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        if(vels.size() != ARM_JOINT_COUNT)
            vels.resize(ARM_JOINT_COUNT);

        for(int i=0; i<ARM_JOINT_COUNT; i++)
            vels[i] = H_Ctrl.joint[armjoints[side][i]].velocity;
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void hubo_plus::getLeftArmVels(Eigen::VectorXd &vels)
{ getArmVels(LEFT, vels); }
void hubo_plus::getRightArmVels(Eigen::VectorXd &vels)
{ getArmVels(RIGHT, vels); }

// Acceleration settings
hp_flag_t hubo_plus::getArmNomAcc(int side, Eigen::VectorXd &acc)
{
    if( side==LEFT || side==RIGHT )
    {
        if(acc.size() != ARM_JOINT_COUNT)
            acc.resize(ARM_JOINT_COUNT);

        for(int i=0; i<ARM_JOINT_COUNT; i++)
            acc[i] = H_Ctrl.joint[armjoints[side][i]].acceleration;
    }
    else
        return BAD_SIDE;
}
void hubo_plus::getLeftArmNomAcc(Eigen::VectorXd &acc)
{ getArmNomAcc(LEFT, acc); }
void hubo_plus::getRightArmNomAcc(Eigen::VectorXd &acc)
{ getArmNomAcc(RIGHT, acc); }


// ~* Leg control gets
// Position control
hp_flag_t hubo_plus::getLegAngles(int side, Eigen::VectorXd &angles)
{
    if( side==LEFT || side==RIGHT )
    {
        if(angles.size() != LEG_JOINT_COUNT)
            angles.resize(LEG_JOINT_COUNT);
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            angles[i] = H_Ctrl.joint[legjoints[side][i]].position;
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void hubo_plus::getLeftLegAngles(Eigen::VectorXd &angles)
{ getLegAngles(LEFT, angles); }
void hubo_plus::getRightLegAngles(Eigen::VectorXd &angles)
{ getLegAngles(RIGHT, angles); }

hp_flag_t hubo_plus::getLegNomSpeeds(int side, Eigen::VectorXd &speeds)
{ return getLegVels(side, speeds); }
void hubo_plus::getLeftLegNomSpeeds(Eigen::VectorXd &speeds)
{ getLegVels(LEFT, speeds); }
void hubo_plus::getRightLegNomSpeeds(Eigen::VectorXd &speeds)
{ getLegVels(RIGHT, speeds); }

// Velocity control
hp_flag_t hubo_plus::getLegVels(int side, Eigen::VectorXd &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        if(vels.size() != LEG_JOINT_COUNT)
            vels.resize(LEG_JOINT_COUNT);

        for(int i=0; i<LEG_JOINT_COUNT; i++)
            vels[i] = H_Ctrl.joint[legjoints[side][i]].velocity;
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void hubo_plus::getLeftLegVels(Eigen::VectorXd &vels)
{ getLegVels(LEFT, vels); }
void hubo_plus::getRightLegVels(Eigen::VectorXd &vels)
{ getLegVels(RIGHT, vels); }

// Acceleration settings
hp_flag_t hubo_plus::getLegNomAcc(int side, Eigen::VectorXd &acc)
{
    if( side==LEFT || side==RIGHT )
    {
        if(acc.size() != LEG_JOINT_COUNT)
            acc.resize(LEG_JOINT_COUNT);

        for(int i=0; i<LEG_JOINT_COUNT; i++)
            acc[i] = H_Ctrl.joint[legjoints[side][i]].acceleration;
    }
    else
        return BAD_SIDE;
}
void hubo_plus::getLeftLegNomAcc(Eigen::VectorXd &acc)
{ getLegNomAcc(LEFT, acc); }
void hubo_plus::getRightLegNomAcc(Eigen::VectorXd &acc)
{ getLegNomAcc(RIGHT, acc); }


// ~~** Getting limit values
// ~* General gets
double hubo_plus::getJointAngleMin( int joint )
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ctrl.joint[joint].pos_min;
    else
        return 0;
}

double hubo_plus::getJointAngleMax(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ctrl.joint[joint].pos_max;
    else
        return 0;
}

double hubo_plus::getJointSpeedMax(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ctrl.joint[joint].speed_limit;
    else
        return 0;
}

double hubo_plus::getJointAccelMax(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ctrl.joint[joint].accel_limit;
    else
        return 0;
}



// ~~~*** State Readings ***~~~ //

// ~~** State
// TODO: Stuff like state position, velocity, whatever

// ~~** Sensors
// ~* Force-torque
// Mx
double hubo_plus::getMx(hubo_ft_index_t sensor)
{
    if( sensor==HUBO_FT_R_FOOT || sensor==HUBO_FT_R_HAND ||
        sensor==HUBO_FT_L_FOOT || sensor==HUBO_FT_L_HAND )
        return H_State.ft[sensor].m_x;
    else
        return 0;
}
double hubo_plus::getRightHandMx() { getMx(HUBO_FT_R_HAND); }
double hubo_plus::getLeftHandMx()  { getMx(HUBO_FT_L_HAND); }
double hubo_plus::getRightFootMx() { getMx(HUBO_FT_R_FOOT); }
double hubo_plus::getLeftFootMx()  { getMx(HUBO_FT_L_FOOT); }

// My
double hubo_plus::getMy(hubo_ft_index_t sensor)
{
    if( sensor==HUBO_FT_R_FOOT || sensor==HUBO_FT_R_HAND ||
        sensor==HUBO_FT_L_FOOT || sensor==HUBO_FT_L_HAND )
        return H_State.ft[sensor].m_y;
    else
        return 0;
}
double hubo_plus::getRightHandMy() { getMy(HUBO_FT_R_HAND); }
double hubo_plus::getLeftHandMy()  { getMy(HUBO_FT_L_HAND); }
double hubo_plus::getRightFootMy() { getMy(HUBO_FT_R_FOOT); }
double hubo_plus::getLeftFootMy()  { getMy(HUBO_FT_L_FOOT); }

// Fz
double hubo_plus::getFz(hubo_ft_index_t sensor)
{
    if( sensor==HUBO_FT_R_FOOT || sensor==HUBO_FT_L_FOOT )
        return H_State.ft[sensor].f_z;
    else
        return 0;
}
double hubo_plus::getRightFootFz() { getFz(HUBO_FT_R_FOOT); }
double hubo_plus::getLeftFootFz()  { getFz(HUBO_FT_L_FOOT); }


// ~* Accelerometers
// AccX
double hubo_plus::getAccX(int side)
{
    if( side==LEFT || side==RIGHT )
        return H_State.imu.a_foot_x[side];
    else
        return 0;
}
double hubo_plus::getLeftAccX() { return getAccX(LEFT); }
double hubo_plus::getRightAccX() { return getAccX(RIGHT); }

// AccY
double hubo_plus::getAccY(int side)
{
    if( side==LEFT || side==RIGHT )
        return H_State.imu.a_foot_y[side];
    else
        return 0;
}
double hubo_plus::getLeftAccY() { return getAccY(LEFT); }
double hubo_plus::getRightAccY() { return getAccY(RIGHT); }

// AccZ
double hubo_plus::getAccZ(int side)
{
    if( side==LEFT || side==RIGHT )
        return H_State.imu.a_foot_z[side];
    else
        return 0;
}
double hubo_plus::getLeftAccZ() { return getAccZ(LEFT); }
double hubo_plus::getRightAccZ() { return getAccZ(RIGHT); }

// ~* IMU
double hubo_plus::getAngleX() { return H_State.imu.angle_x; }
double hubo_plus::getAngleY() { return H_State.imu.angle_y; }
double hubo_plus::getRotVelX() { return H_State.imu.w_x; }
double hubo_plus::getRotVelY() { return H_State.imu.w_y; }



// ~~~*** Board Commands ***~~~ //
hp_flag_t hubo_plus::homeJoint( int joint, bool send, double wait )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Ctrl.joint[joint].position = 0;
        H_Ctrl.joint[joint].mode = CTRL_HOME;
        H_Ctrl.active = 2;
        
        H_Cmd.type = D_GOTO_HOME;
        H_Cmd.joint = joint;
    }
    else
        return JOINT_OOB;

    if(send)
    {
        sendControls();
        while( H_Ref.paused==0 )
            update();
        sendCommands();
    }

    return SUCCESS;
}

void hubo_plus::homeAllJoints( bool send, double wait )
{
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        homeJoint( i, false );
    
    H_Ctrl.active = 2;

    H_Cmd.type = D_GOTO_HOME_ALL;

    if(send)
    {
        sendControls();
        while( H_Ref.paused==0 )
            update();
        sendCommands();
    }
}






