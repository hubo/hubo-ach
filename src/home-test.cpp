#include "hubo_plus.h"



int main(int argc, char **argv)
{
    printf("About to construct hubo\n");
    hubo_plus hubo;
    printf("About to home everything\n");
//    hubo.homeJoint( LSR, true );
    hubo.homeAllJoints(true);

    return 0;
}
