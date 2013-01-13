#include "hubo_plus.h"
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
	
	hubo_plus hubo;

    
    Vector6d angles;
    angles << 0.7, 0.2, .5, 0.7, .3, 0.1;

    hubo.setLeftLegAngles(angles, true);



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
/*
    Vector6d qr;
    qr << 0, 0, 0, -M_PI/2, -0.3, 0;

    std::vector<Vector6d> angles(5);
    angles[0] <<  0.0556916,   0.577126,  0.0816814,  -0.492327, 0, 0;
    angles[1] <<  -1.07878,  0.408266, -0.477742, -0.665062, 0, 0;
    angles[2] <<   -1.17367, -0.0540511,  -0.772141,  -0.503859, 0, 0;
    angles[3] <<  -0.518417,   0.172191,  -0.566084, -0.0727671, 0, 0;
    angles[4] << 0, 0, 0, 0, 0, 0;

    Vector6d current;
    
    double tol = 0.075;
    int traj = 0;
    while(true)
    {
        hubo.update();
        
        hubo.getLeftArmAngleStates( current );

        if( (current-angles[traj]).norm() < tol )
        {
            traj++;
            if(traj > 4)
                traj = 0;
        }

        hubo.setLeftArmAngles( angles[traj] );

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





