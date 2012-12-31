#ifndef HUBO_PLUS_H
#define HUBO_PLUS_H

// For Hubo
#include "hubo.h"
#include "hubo-jointparams.h"

// For process management
#include "daemonizer.h"
#include <ach.h>

// For data handling
#include <math.h>
#include <eigen.h>

#define ARM_JOINT_COUNT 6
#define LEG_JOINT_COUNT 6

class hubo_plus
{
public:
    hubo_plus(); // Use daemonize(const char *daemon_name) after calling this constructor
    hubo_plus(const char *daemon_name);

    double time;

    // ~~~*** Sending Control Commands ***~~~ //
    // ~~** Setting reference values

    // ~* General sets
    // Position control
    void setJointAngle( int joint, double radians, bool send=false );
    void setJointNominalSpeed( int joint, double speed );
    // Velocity control
    void setJointVelocity( int joint, double vel, bool send=false );
    // Acceleration setting
    void setJointNominalAcceleration( int joint, double acc );

    // ~* Arm control sets
    // Position control
    void setArmAngles( int side, Eigen::VectorXd angles, bool send=false );
    void setLeftArmAngles( Eigen::VectorXd angles, bool send=false );
    void setRightArmAngles( Eigen::VectorXd angles, bool send=false );
    void setArmNomSpeeds( int side, Eigen::VectorXd speeds );
    void setLeftArmNomSpeeds( Eigen::VectorXd speeds );
    void setRightArmNomSpeeds( Eigen::VectorXd speeds );
    // Velocity control
    void setArmVels( int side, Eigen::VectorXd vels, bool send=false );
    void setLeftArmVels( Eigen::VectorXd vels, bool send=false );
    void setRightArmVels( Eigen::VectorXd vels, bool send=false );
    // Acceleration settings
    void setArmNomAcc(int side, Eigen::VectorXd acc );
    void setLeftArmNomAcc( Eigen::VectorXd acc );
    void setRightArmNomAcc( Eigen::VectorXd acc );

    // ~* Leg control sets
    // Position control
    void setLegAngles( int side, Eigen::VectorXd angles, bool send=false );
    void setLeftLegAngles( Eigen::VectorXd angles, bool send=false );
    void setRightLegAngles( Eigen::VectorXd angles, bool send=false );
    void setLegNomSpeeds( int side, Eigen::VectorXd speeds );
    void setLeftLegNomSpeeds( Eigen::VectorXd speeds );
    void setRightLegNomSpeeds( Eigen::VectorXd speeds );
    // Velocity control
    void setLegVels( int side, Eigen::VectorXd vels, bool send=false );
    void setLeftLegVels( Eigen::VectorXd vels, bool send=false );
    void setRightLegVels( Eigen::VectorXd vels, bool send=false );
    // Acceleration settings
    void setLegNomAcc(int side, Eigen::VectorXd acc );
    void setLeftLegNomAcc( Eigen::VectorXd acc );
    void setRightLegNomAcc( Eigen::VectorXd acc );

    // ~~** Setting limit values
    // ~* General sets
    void setJointAngleMin( int joint, double radians );
    void setJointAngleMax( int joint, double radians );
    void setJointSpeedMax( int joint, double speed );
    void setJointAccelMax( int joint, double accel );

    // ~~** Send Off Latest Control Commands
    void sendCommands();


    // ~~** Getting Reference Values
    // ~* General gets
    // Position control
    double getJointAngle( int joint );
    double getJointNominalSpeed( int joint );
    // Velocity control
    double getJointVelocity( int joint );
    // Acceleration setting
    double getJointNominalAcceleration( int joint );

    // ~* Arm control sets
    // Position control
    void getArmAngles( int side, Eigen::VectorXd &angles );
    void getLeftArmAngles( Eigen::VectorXd &angles );
    void getRightArmAngles( Eigen::VectorXd &angles );
    void getArmNomSpeeds( int side, Eigen::VectorXd &speeds );
    void getLeftArmNomSpeeds( Eigen::VectorXd &speeds );
    void getRightArmNomSpeeds( Eigen::VectorXd &speeds );
    // Velocity control
    void getArmVels( int side, Eigen::VectorXd &vels );
    void getLeftArmVels( Eigen::VectorXd &vels );
    void getRightArmVels( Eigen::VectorXd &vels );
    // Acceleration settings
    void getArmNomAcc(int side, Eigen::VectorXd &acc );
    void getLeftArmNomAcc( Eigen::VectorXd &acc );
    void getRightArmNomAcc( Eigen::VectorXd &acc );

    // ~* Leg control sets
    // Position control
    void getLegAngles( int side, Eigen::VectorXd &angles );
    void getLeftLegAngles( Eigen::VectorXd &angles );
    void getRightLegAngles( Eigen::VectorXd &angles );
    void getLegNomSpeeds( int side, Eigen::VectorXd &speeds );
    void getLeftLegNomSpeeds( Eigen::VectorXd &speeds );
    void getRightLegNomSpeeds( Eigen::VectorXd &speeds );
    // Velocity control
    void getLegVels( int side, Eigen::VectorXd &vels );
    void getLeftLegVels( Eigen::VectorXd &vels );
    void getRightLegVels( Eigen::VectorXd &vels );
    // Acceleration settings
    void getLegNomAcc(int side, Eigen::VectorXd &acc );
    void getLeftLegNomAcc( Eigen::VectorXd &acc );
    void getRightLegNomAcc( Eigen::VectorXd &acc );

    // ~~** Setting limit values
    // ~* General sets
    double getJointAngleMin( int joint );
    double getJointAngleMax( int joint );
    double getJointSpeedMax( int joint );
    double getJointAccelMax( int joint );


    // ~~~*** Sensor Readings ***~~~ //
    // ~~** Force-torques
    double getMx( hubo_ft_index_t sensor );
    double getRightHandMx();
    double getLeftHandMx();
    double getRightFootMx();
    double getLeftFootMx();
    double getMy( hubo_ft_index_t sensor );
    double getRightHandMy();
    double getLeftHandMy();
    double getRightFootMy();
    double getLeftFootMy();
    double getFz( hubo_ft_index_t sensor );
    double getRightFootFz();
    double getLeftFootFz();

    // ~~** Accelerometers
    double getAccX( int side );
    double getLeftAccX();
    double getRightAccX();
    double getAccY( int side );
    double getLeftAccY();
    double getRightAccY();
    double getAccY( int side );
    double getLeftAccZ();
    double getRightAccZ();

    // ~~** IMU
    double getAngleX();
    double getAngleY();
    double getRotVelX();
    double getRotVelY();


    // ~~~*** Board Commands ***~~~ //
    // TODO: All of these

protected:

    ach_channel_t chan_hubo_ref;
    ach_channel_t chan_hubo_board_cmd;
    ach_channel_t chan_hubo_state;
    ach_channel_t chan_hubo_ctrl;

    hubo_ref H_Ref;
    hubo_board_cmd H_Cmd;
    hubo_state H_State;
    hubo_control H_Ctrl;
    hubo_param H_Param;


};

#endif // HUBO_PLUS_H
