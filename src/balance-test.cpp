#include "hubo_plus.h"
#include <iostream>

 
int main(int argc, char **argv)
{
    hubo_plus hubo;

    int i=0, imax=20;

    double leftP=0, leftR=0, rightP=0, rightR=0, ptime, dt;

    double rollgain = 0.01;
    double pitchgain = 0.01;

    ptime = hubo.getTime(); 

    while(true)
    {
        hubo.update();

        dt = hubo.getTime() - ptime;
        
        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            leftP -= pitchgain*hubo.getLeftFootMy()*dt;
            leftR -= rollgain*hubo.getLeftFootMx()*dt;
            rightP -= pitchgain*hubo.getRightFootMy()*dt;
            rightR -= rollgain*hubo.getRightFootMx()*dt;

/*
            leftP += pitchgain*hubo.getLeftFootMy()*dt;
            leftR += rollgain*hubo.getLeftFootMx()*dt;
            rightP += pitchgain*hubo.getRightFootMy()*dt;
            rightR += rollgain*hubo.getRightFootMx()*dt;
*/


            hubo.setJointAngle( RAP, rightP );
            hubo.setJointAngle( RAR, rightR );
            hubo.setJointAngle( LAP, leftP );
            hubo.setJointAngle( LAR, leftR );
            
            hubo.sendControls();
     
            if( i==imax )
                std::cout << "LAccX:" << hubo.getLeftAccX() << "\tLAccY:" << hubo.getLeftAccY()
                        << "\tLAccZ:" << hubo.getLeftAccZ()
                        << "\tRAccX:" << hubo.getRightAccX() << "\tRAccY:" << hubo.getRightAccY()
                        << "\tRAccZ:" << hubo.getRightAccZ() << std::endl;
//                std::cout << "RAP:" << rightP << "\t" << "RAR:" << rightR << "\t"
//                    <<   "LAP:" << leftP  << "\t" << "LAR:" << leftR << std::endl;
        }
        
        ptime = hubo.getTime();
    }

}
