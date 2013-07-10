/*
* Copyright (c) 2013, Georgia Tech Research Corporation
* All rights reserved.
*
* Author: Michael X. Grey <mxgrey@gatech.edu>
* Date: July 7, 2013
*
* Humanoid Robotics Lab Georgia Institute of Technology
* Director: Mike Stilman http://www.golems.org
*
*
* This file is provided under the following "BSD-style" License:
* Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following
* conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
* USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

extern "C" {
#include "hubo.h"
#include "hubo-jointparams.h"
}

#include <stdio.h>

void printHelp()
{
    fprintf(stdout, "\nusage: hubo-homeparams [argument] [target file] [optional]\n"
                    "\n"
                    "       arguments:\n"
                    "           -s or --save  : Saves the current home settings into [target file]\n"
                    "           -l or --load  : Loads the home settings from [target file] onto the boards\n"
                    "           -p pr --print : Prints a table of joint settings out to the terminal\n"
                    "\n"
                    "       target file: You are recommend to provide the full path of a file\n"
                    "\n"
                    "       optional: Tells the -s/--save argument what format to store position data in\n"
                    "           raw : Position data will be stored in encoder units\n"
                    "           rad : Position data will be stored in radians\n\n");
}


int main(int argc, char **argv)
{
    if(argc <= 1)
    {
        printHelp();
        return 0;
    }

    if( 0 == strcmp(argv[1], "-s") || 0 == strcmp(argv[1], "--save") )
    {
        if(argc <= 2)
        {
            fprintf(stdout, "-s/--save needs a target file name!\n\n");
            printHelp();
            return 1;
        }
        
        int type = 0;
        if(argc > 3)
        {
            if( 0 == strcmp(argv[3], "rad") )
            {
                type = 1;
            }
            else if( 0 == strcmp(argv[3], "raw") )
            {
                type = 0;
            }
            else
            {
                fprintf(stdout, "Unrecognized type: '%s'. Should be raw or rad.\n"
                                " -- Defaulting to raw\n", argv[3]);
                type = 0;
            }
        }

        return saveHomingParams( argv[2], type );
    }
    else if( 0 == strcmp(argv[1], "-l") || 0 == strcmp(argv[1], "--load") )
    {
        if(argc <= 2)
        {
            fprintf(stdout, "-l/--load needs a target file name!\n\n");
            printHelp();
            return 1;
        }
        
        return loadHomingParams( argv[2] );
    }
    else if( 0 == strcmp(argv[1], "-p") || 0 == strcmp(argv[1], "--print") )
    {
        fprintf(stdout, "\n");
        int r = printHomingParams(stdout, 0);
        fprintf(stdout, "\n\n"); fflush(stdout);
        return r;
    }
    else
    {
        fprintf(stdout, "Invalid argument: %s\n\n", argv[1]);
        printHelp();
        return 0;
    }
    return 0;
}




