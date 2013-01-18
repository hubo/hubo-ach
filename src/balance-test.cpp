#include "hubo_plus.h"
#include <iostream>

 
int main(int argc, char **argv)
{
    hubo_plus hubo;
    hubo.initFastrak();
    int i=0, imax=20;

/*    hubo.setJointNominalSpeed( LAR, 0.2 );
    hubo.setJointNominalSpeed( LAP, 0.2 );
    hubo.setJointNominalSpeed( RAR, 0.2 );
    hubo.setJointNominalSpeed( RAP, 0.2 );
*/
    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );

    hubo.setJointNominalAcceleration( RKN, 0.6 );
    hubo.setJointNominalAcceleration( RHP, 0.6 );
    hubo.setJointNominalAcceleration( LKN, 0.6 );
    hubo.setJointNominalAcceleration( LHP, 0.6 );

    hubo.setJointNominalAcceleration( RHR, 0.6 );
    hubo.setJointNominalAcceleration( LHR, 0.6 );
    

    hubo.setJointAngleMax( RHP, 0 );
    hubo.setJointAngleMax( LHP, 0 );
/*
    hubo.setPositionControl( RKN );
    hubo.setPositionControl( RHP );
    hubo.setPositionControl( LKN );
    hubo.setPositionControl( LHP );

    hubo.setJointNominalSpeed( RKN, 0.05 );
    hubo.setJointNominalSpeed( RHP, 0.025 );
    hubo.setJointNominalSpeed( LKN, 0.05 );
    hubo.setJointNominalSpeed( LHP, 0.025 );
*/
    double L1 = 2*0.3002;
    double L2 = 0.28947 + 0.0795;
    double height = 0;
    Eigen::Vector3d trans; Eigen::Quaterniond quat;

    double leftP=0, leftR=0, rightP=0, rightR=0, ptime, dt, knee, kneeVel, kneeAngleError;

    double compRollGain = 0.0015;
    double compPitchGain = 0.0015;
    double pitchAngleGain = 0.5*M_PI/180.0;
    double pitchRotVelGain = 0.0*M_PI/180;
    double rollAngleGain = 0.3*M_PI/180;
    double rollRotVelGain = 0.0*M_PI/180;
    double integralGain = 0; //0.05*M_PI/180;
	double kneeVelGain = 0.2;
    double leftPIntegral = 0.0;
    double leftRIntegral = 0.0;
    double rightPIntegral = 0.0;
    double rightRIntegral = 0.0;
    double pTiltIntegral = 0.0;
    double rTiltIntegral = 0.0;

    double springGainUp = 0.001;
    double springGainDown = 0.0075;

    double compLP;
    double compLR;
    double compRP;
    double compRR;

    ptime = hubo.getTime();
    double atime = hubo.getTime();
	Eigen::Vector3d fastrakOrigin;
	hubo.getFastrak(fastrakOrigin, quat);

    while(true)
    {
        hubo.update();

        dt = hubo.getTime() - ptime;
        atime += dt;
        
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            compLP = hubo.getLeftFootMy();
            compLR = hubo.getLeftFootMx();
            compRP = hubo.getRightFootMy();
            compRR = hubo.getRightFootMx();

/*
            if( compLP >= 0.005 )
                compLP = 0.005;
            else if( compLP <= -0.005 )
                compLP = -0.005;

            if( compLR >= 0.005 )
                compLR = 0.005;
            else if( compLR <= -0.005 )
                compLR = -0.005;

            if( compRP >= 0.005 )
                compRP = 0.005;
            else if( compRP <= -0.005 )
                compRP = -0.005;

            if( compRR >= 0.005 )
                compRR = 0.005;
            else if( compRR <= -0.005 )
                compRR = -0.005;
*/            
            hubo.getFastrak( trans, quat );
            if( i==imax )
                std::cout << trans.transpose() << "\t";
			trans -= fastrakOrigin;	
            height = trans(2) + L1 + L2;
            if( height-L2 > L1 )
                height = L1+L2;
            else if( height-L2 < 0.25 )
                height = L1 + 0.2; // TODO: Check if this is a good value

            knee = acos( (height-L2)/L1 )*2;
			kneeAngleError = knee - hubo.getJointAngle( RKN );
		    	
            kneeVel = kneeVelGain*kneeAngleError;			

            leftP = pitchAngleGain*hubo.getAngleY() - compPitchGain*compLP + (-kneeVel/2);
            leftR = rollAngleGain*hubo.getAngleX() - compRollGain*compLR;
            rightP = pitchAngleGain*hubo.getAngleY() - compPitchGain*compRP + (-kneeVel/2);
            rightR = rollAngleGain*hubo.getAngleX() - compRollGain*compRR;

            hubo.setJointVelocity( RAP, rightP );
            hubo.setJointVelocity( RAR, rightR );
            hubo.setJointVelocity( LAP, leftP );
            hubo.setJointVelocity( LAR, leftR );
            
            hubo.setJointVelocity( RKN, kneeVel );
            hubo.setJointVelocity( RHP, -kneeVel/2.0 );
            hubo.setJointVelocity( LKN, kneeVel );
            hubo.setJointVelocity( LHP, -kneeVel/2.0 );
//            hubo.setJointVelocity( RHR, -rightR ); // These cannot be permitted to comply
//            hubo.setJointVelocity( LHR, -leftR );

/* 
            knee = ( hubo.getLeftFootFz() + hubo.getRightFootFz() - 440 );

            if( knee > 0 )
                knee *= springGainDown;
            else
                knee *= springGainUp;


            if( knee >= 0.3 )
                knee = 0.3;
            else if( knee <= -0.3 )
                knee = -0.3;*/
/*
*/            

/*
            if( fabs(leftP)<=0.0002 && fabs(rightP)<=0.0002 && atime>=10 )
            {
                hubo.setJointAngle( RKN, 0.5 );
                hubo.setJointAngle( LKN, 0.5 );
                hubo.setJointAngle( RHP, -0.25 );
                hubo.setJointAngle( LHP, -0.25 );
            }
*/
            
            hubo.sendControls();
 
            // Display IMU readings
            if( i==imax )
                std::cout << "Height:" << height  <<  "\tLP:" << leftP << "\tRP:" << rightP << "\tLR:" << leftR << "\tRR:" << rightR << "\tkneeError: " << kneeAngleError << std::endl;

//                std::cout << "IMU:" << "\tAngleX:" << hubo.getAngleX() << "\tAngleY:" << hubo.getAngleY()
//                        << "\tRotVelX:" << hubo.getRotVelX() << "\tRotVelY:" << hubo.getRotVelY()
//                        << std::endl;

/*            // Display feet inclinometer readings
            if( i==imax )
                std::cout << "LAccX:" << hubo.getLeftAccX() << "\tLAccY:" << hubo.getLeftAccY()
                        << "\tLAccZ:" << hubo.getLeftAccZ()-9.8
                        << "\tRAccX:" << hubo.getRightAccX() << "\tRAccY:" << hubo.getRightAccY()
                        << "\tRAccZ:" << hubo.getRightAccZ()-9.8 << std::endl;
*///                std::cout << "RAP:" << rightP << "\t" << "RAR:" << rightR << "\t"
//                    <<   "LAP:" << leftP  << "\t" << "LAR:" << leftR << std::endl;
        }
        
        ptime = hubo.getTime();
    }

}
