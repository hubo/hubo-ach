#include "hubo_plus.h"



int main(int argc, char **argv)
{
    hubo_plus hubo;

//    hubo.homeJoint( LSR, true );
    hubo.homeAllJoints(true);

    return 0;
}
