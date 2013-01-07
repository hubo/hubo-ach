#include "hubo_plus.h"
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
	
	hubo_plus hubo;

/*	Eigen::VectorXd angles(6);
	angles << -2, -0.3, -2.25, -2.0, -2.0, -1.25;


	hp_flag_t result = hubo.setLeftArmAngles(angles);

	Eigen::VectorXd vels(6);
	hubo.getLeftArmNomSpeeds(vels);

	printf("%f\t%f\t%f\n", vels[0], vels[1], vels[2]);*/

//    if( hubo.getJointStatus(LSP)==1 )
//       printf("Joint status:%d",hubo.getJointStatus(LSP)); 

//    hubo.resetJointStatus(LSP,true);
    Vector6d angles1, angles2, angles3, angles4, angles5;


    angles1 <<  0.0556916,   0.577126,  0.0816814,  -0.492327, 0, 0;
    angles2 <<  -1.07878,  0.408266, -0.477742, -0.665062, 0, 0;
    angles3 <<   -1.17367, -0.0540511,  -0.772141,  -0.503859, 0, 0;
    angles4 <<  -0.518417,   0.172191,  -0.566084, -0.0727671, 0, 0;
    angles5 << 0, 0, 0, 0, 0, 0;

    Vector6d current;

    double tol = 0.075;
    int traj = 0;
    while(true)
    {
        hubo.update();

        hubo.getLeftArmAngleStates( current );

	if(traj==0)
	{
		hubo.setLeftArmAngles( angles1 );
		if( (current-angles1).norm() < tol )
			traj++;
	}
	else if(traj==1)
	{
		hubo.setLeftArmAngles( angles2 );
		if( (current-angles2).norm() < tol )
			traj++;
	}
	else if(traj==2)
	{
		hubo.setLeftArmAngles( angles3 );
		if( (current-angles3).norm() < tol )
			traj++;
	}
	if( traj==3 )
	{
		hubo.setLeftArmAngles( angles4 );
		if( (current-angles4).norm() < tol )
			traj++;
        }
	if( traj==4 )
	{
		hubo.setLeftArmAngles( angles5 );
		if( (current-angles5).norm() < tol )
			traj = 0;
	}

        hubo.sendControls();

    }

}





