#include "hubo_plus.h"
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
	
	hubo_plus hubo;

//    Vector6d acc;
//    acc << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
//    hubo.setRightArmNomAcc( acc );
/*
    Vector6d angles1, angles2, check;
    angles1 << -0.6703, -0.101632, 0.3296, -1.8217, -1.9074, -0.0;
    angles2 << -0.6055, -0.0782,  -0.2416, -1.6655, -1.9774, 0.0;

    hubo.setRightArmAngles( angles1, true ); 
    std::cout << "Arm angles set to Angles1" << std::endl;
    hubo.update();
    hubo.getRightArmAngles( check );
    while( (angles1-check).norm() > 0.075 )
    {
        hubo.update();
        hubo.getRightArmAngleStates( check );
    }

    hubo.setRightArmAngles( angles2, true );
    std::cout << "Arm angles set to Angles2" << std::endl;*/
/*
    while(true)
    {
        hubo.update();
        printf("RMx:%3.2f\tRMy:%3.2f\tRFz:%3.2f\tLMx:%3.2f\tLMy:%3.2f\tLFz:%3.2f\n",
                hubo.getRightFootMx(), hubo.getRightFootMy(), hubo.getRightFootFz(),
                hubo.getLeftFootMx(),  hubo.getLeftFootMy(),  hubo.getLeftFootFz() );
        usleep(50000);
    }
*/


//    Vector6d qr;
//    qr << 0, 0, 0, -M_PI/2, -0.3, 0;

//    std::vector<Vector6d> angles(5);
//    std::vector<Vector6d> angles(2);
//    angles[1] << -0.6703, -0.101632, 0.3296, -1.8217, -1.9074, -0.0;
//    angles[0] << -0.6055, -0.0782,  -0.2416, -1.6655, -1.9774, 0.0;
//    angles[0] << -0.6055, -0.0782,  -0.2416, -1.6655, 0.0, 0.0;
    
/*    angles[0] <<  0.0556916,   0.577126,  0.0816814,  -0.492327, 0, 0;
    angles[1] <<  -1.07878,  0.408266, -0.477742, -0.665062, 0, 0;
    angles[2] <<   -1.17367, -0.0540511,  -0.772141,  -0.503859, 0, 0;
    angles[3] <<  -0.518417,   0.172191,  -0.566084, -0.0727671, 0, 0;
    angles[4] << 0, 0, 0, 0, 0, 0;
*/
    Vector6d q;
    double y=0, V=0, tprev=hubo.getTime(), dt=0;
    int i=0, imax=100;
    hubo.HuboDrillIK( q, y );

    hubo.setRightArmAngles( q, true );

    Vector6d current;
    hubo.update();
    hubo.getRightArmAngles( current );

    while( (current-q).norm() > 0.075 )
    {
        hubo.update();
        hubo.getRightArmAngles( current );
    }

    V = -( 0.1850 + 0.2050 )/8.0;
    
    while(true)
    {
        i++;
        if(i>imax) i = 0;

        hubo.update();
        dt = hubo.getTime() - tprev;
        tprev = hubo.getTime();
        if( y > 0.170 )
            V = -fabs(V);
        else if( y < -0.190 )
            V = fabs(V);
        y += V*dt; if(i==imax) std::cout << y << std::endl;
        hubo.HuboDrillIK( q, y );

        hubo.setRightArmAngles( q );

        hubo.sendControls();
   } 
    
    
/*    
    double tol = 0.075;
    int traj = 0;
    while(true)
    {
        hubo.update();
        
        hubo.getRightArmAngles( current );
//      hubo.getLeftArmAngleStates( current );
        if( (current-angles[traj]).norm() < tol )
        {
            traj++;
            if(traj > 2)
                traj = 0;
        }

//        hubo.setRightArmAngles( angles[traj] );
//      hubo.setLeftArmAngles( angles[traj] );

        hubo.sendControls();
    }
*/
/*
    Vector6d angles;

    while(true)
    {
        hubo.getRightLegAngleStates( angles );
        std::cout << angles.transpose() << std::endl;
        hubo.update();
        usleep(10000);
    }
*/
    
}





