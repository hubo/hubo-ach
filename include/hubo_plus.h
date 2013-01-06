#ifndef HUBO_PLUS_H
#define HUBO_PLUS_H

// C Headers
extern "C" {
// For Hubo
#include "hubo.h"
#include "hubo-jointparams.h"

// For process management
#include "daemonizer.h"
}


// For data handling
#include <math.h>
#include </usr/include/eigen3/Eigen/Core>
#include <vector>

#define ARM_JOINT_COUNT 6
#define LEG_JOINT_COUNT 6

typedef enum {

    SUCCESS = 0,
    JOINT_OOB,      // The joint you tried to specify is out of bounds
    VALUE_OOB,      // Some generic value was out of acceptable bounds
    WRONG_MODE,     // You are not in the correct control mode to do what you asked
    BAD_SIDE,       // You did not use LEFT or RIGHT correctly
    SHORT_VECTOR,   // The VectorXd you tried to use has too few entries
    LONG_VECTOR,    // The VectorXd you tried to use has too many entries
    REF_STALE,      // The reference values were not able to update for some reason
    STATE_STALE,    // The state values were not able to update for some reason
    ALL_STALE       // Nothing was able to update for some reason

} hp_flag_t;





class hubo_plus
{
public:
    hubo_plus(); // Use daemonize(const char *daemon_name) after calling this constructor
    hubo_plus(const char *daemon_name);

    double getTime();

    hp_flag_t update(bool printError=false);   // Retrieves latest data
                    // Returns true if successful
                    // Returns false if both channels are not ACH_OK
    // TODO: Consider making the update return more meaningful


    // ~~~*** Sending Control Commands ***~~~ //
    // ~~** Setting reference values

    // ~* General sets
    hp_flag_t resetJointStatus( int joint, bool send=false );
    // Position control
    hp_flag_t setPositionControl( int joint );
    hp_flag_t setJointAngle( int joint, double radians, bool send=false );
    hp_flag_t setJointNominalSpeed( int joint, double speed );
    // Velocity control
    hp_flag_t setVelocityControl( int joint );
    hp_flag_t setJointVelocity( int joint, double vel, bool send=false );
    // Acceleration setting
    hp_flag_t setJointNominalAcceleration( int joint, double acc );

    // ~* Arm control sets
    // Position control
    hp_flag_t setArmPosCtrl( int side );
    hp_flag_t setArmAngles( int side, Eigen::VectorXd angles, bool send=false );
    void setLeftArmPosCtrl();
    hp_flag_t setLeftArmAngles( Eigen::VectorXd angles, bool send=false );
    void setRightArmPosCtrl();
    hp_flag_t setRightArmAngles( Eigen::VectorXd angles, bool send=false );
    hp_flag_t setArmNomSpeeds( int side, Eigen::VectorXd speeds );
    hp_flag_t setLeftArmNomSpeeds( Eigen::VectorXd speeds );
    hp_flag_t setRightArmNomSpeeds( Eigen::VectorXd speeds );
    // Velocity control
    hp_flag_t setArmVelCtrl( int side );
    hp_flag_t setArmVels( int side, Eigen::VectorXd vels, bool send=false );
    void setLeftArmVelCtrl();
    hp_flag_t setLeftArmVels( Eigen::VectorXd vels, bool send=false );
    void setRightArmVelCtrl();
    hp_flag_t setRightArmVels( Eigen::VectorXd vels, bool send=false );
    // Acceleration settings
    hp_flag_t setArmNomAcc(int side, Eigen::VectorXd acc );
    hp_flag_t setLeftArmNomAcc( Eigen::VectorXd acc );
    hp_flag_t setRightArmNomAcc( Eigen::VectorXd acc );

    // ~* Leg control sets
    // Position control
    hp_flag_t setLegPosCtrl( int side );
    hp_flag_t setLegAngles( int side, Eigen::VectorXd angles, bool send=false );
    void setLeftLegPosCtrl();
    hp_flag_t setLeftLegAngles( Eigen::VectorXd angles, bool send=false );
    void setRightLegPosCtrl();
    hp_flag_t setRightLegAngles( Eigen::VectorXd angles, bool send=false );
    hp_flag_t setLegNomSpeeds( int side, Eigen::VectorXd speeds );
    hp_flag_t setLeftLegNomSpeeds( Eigen::VectorXd speeds );
    hp_flag_t setRightLegNomSpeeds( Eigen::VectorXd speeds );
    // Velocity control
    hp_flag_t setLegVelCtrl( int side );
    hp_flag_t setLegVels( int side, Eigen::VectorXd vels, bool send=false );
    void setLeftLegVelCtrl();
    hp_flag_t setLeftLegVels( Eigen::VectorXd vels, bool send=false );
    void setRightLegVelCtrl();
    hp_flag_t setRightLegVels( Eigen::VectorXd vels, bool send=false );
    // Acceleration settings
    hp_flag_t setLegNomAcc(int side, Eigen::VectorXd acc );
    hp_flag_t setLeftLegNomAcc( Eigen::VectorXd acc );
    hp_flag_t setRightLegNomAcc( Eigen::VectorXd acc );

    // ~~** Setting limit values
    // ~* General sets
    hp_flag_t setJointAngleMin( int joint, double radians );
    hp_flag_t setJointAngleMax( int joint, double radians );
    hp_flag_t setJointSpeedMax( int joint, double speed );
    hp_flag_t setJointAccelMax( int joint, double accel );

    // ~~** Send Off Latest Control Commands
    void sendControls();


    // ~~** Getting Reference Values
    // ~* General gets
    // Position control
    hubo_ctrl_mode_t getCtrlMode( int joint );
    double getJointAngle( int joint );
    double getJointNominalSpeed( int joint );
    // Velocity control
    double getJointVelocity( int joint );
    //double getJointVelocityState( int joint ); // TODO: add velocity to the state
    // Acceleration setting
    double getJointNominalAcceleration( int joint );

    int getJointStatus( int joint ); // 0:Good 1:Frozen

    // ~* Arm control gets
    // Position control
    hp_flag_t getArmAngles( int side, Eigen::VectorXd &angles );
    void getLeftArmAngles( Eigen::VectorXd &angles );
    void getRightArmAngles( Eigen::VectorXd &angles );
    hp_flag_t getArmNomSpeeds( int side, Eigen::VectorXd &speeds );
    void getLeftArmNomSpeeds( Eigen::VectorXd &speeds );
    void getRightArmNomSpeeds( Eigen::VectorXd &speeds );
    // Velocity control
    hp_flag_t getArmVels( int side, Eigen::VectorXd &vels );
    void getLeftArmVels( Eigen::VectorXd &vels );
    void getRightArmVels( Eigen::VectorXd &vels );
    // Acceleration settings
    hp_flag_t getArmNomAcc(int side, Eigen::VectorXd &acc );
    void getLeftArmNomAcc( Eigen::VectorXd &acc );
    void getRightArmNomAcc( Eigen::VectorXd &acc );

    // ~* Leg control gets
    // Position control
    hp_flag_t getLegAngles( int side, Eigen::VectorXd &angles );
    void getLeftLegAngles( Eigen::VectorXd &angles );
    void getRightLegAngles( Eigen::VectorXd &angles );
    hp_flag_t getLegNomSpeeds( int side, Eigen::VectorXd &speeds );
    void getLeftLegNomSpeeds( Eigen::VectorXd &speeds );
    void getRightLegNomSpeeds( Eigen::VectorXd &speeds );
    // Velocity control
    hp_flag_t getLegVels( int side, Eigen::VectorXd &vels );
    void getLeftLegVels( Eigen::VectorXd &vels );
    void getRightLegVels( Eigen::VectorXd &vels );
    // Acceleration settings
    hp_flag_t getLegNomAcc(int side, Eigen::VectorXd &acc );
    void getLeftLegNomAcc( Eigen::VectorXd &acc );
    void getRightLegNomAcc( Eigen::VectorXd &acc );

    // ~~** Getting limit values
    // ~* General gets
    double getJointAngleMin( int joint );
    double getJointAngleMax( int joint );
    double getJointSpeedMax( int joint );
    double getJointAccelMax( int joint );


    // ~~~*** State Readings ***~~~ //

    // ~~** State
    // TODO: All of these (state position, velocity, whatever)

    // ~~** Sensors
    // ~* Force-torque
    // Mx
    double getMx( hubo_ft_index_t sensor );
    double getRightHandMx();
    double getLeftHandMx();
    double getRightFootMx();
    double getLeftFootMx();
    // My
    double getMy( hubo_ft_index_t sensor );
    double getRightHandMy();
    double getLeftHandMy();
    double getRightFootMy();
    double getLeftFootMy();
    // Fz
    double getFz( hubo_ft_index_t sensor );
    double getRightFootFz();
    double getLeftFootFz();
    // ~* Accelerometers
    // AccX
    double getAccX( int side );
    double getLeftAccX();
    double getRightAccX();
    // AccY
    double getAccY( int side );
    double getLeftAccY();
    double getRightAccY();
    // AccZ
    double getAccZ( int side );
    double getLeftAccZ();
    double getRightAccZ();
    // ~* IMU
    double getAngleX();
    double getAngleY();
    double getRotVelX();
    double getRotVelY();



    // ~~~*** Board Commands ***~~~ //
    // TODO: All of these
    void sendCommands();

    hp_flag_t homeJoint( int joint, bool send=true, double wait=1.0 );
    void homeAllJoints( bool send=true, double wait=1.0 );

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

    int armjoints[2][ARM_JOINT_COUNT];
    int legjoints[2][LEG_JOINT_COUNT];

};

#endif // HUBO_PLUS_H
