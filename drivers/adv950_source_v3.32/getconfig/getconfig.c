//******************************************************************************
//
// Copyright (c) 2008 Advantech Industrial Automation Group.
//
// Oxford PCI-954/952/16C950 with Advantech RS232/422/485 capacities
// 
// This program is free software; you can redistribute it and/or modify it 
// under the terms of the GNU General Public License as published by the Free 
// Software Foundation; either version 2 of the License, or (at your option) 
// any later version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT 
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for 
// more details.
// 
// You should have received a copy of the GNU General Public License along with
// this program; if not, write to the Free Software Foundation, Inc., 59 
// Temple Place - Suite 330, Boston, MA  02111-1307, USA.
// 
//
//
//******************************************************************************

//***********************************************************************
// File:        getconfig.c
// Version:     1.01.1
// Author:      Po-Cheng Chen
// Purpose:	Get serial port configuration, such as RS232 or RS422/485
//***********************************************************************
//***********************************************************************
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <asm/ioctls.h>
#include <linux/serial.h>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    int i, ttynum, fd;
    char ttyname[80];
    struct serial_struct serinfo;

    if(argc < 3)
    {
	printf("Usage:\n");
	printf("%s TTYNAME TTYNUM", argv[0]);
	return 1;
    }

    ttynum = atoi(argv[2]);
    for(i=0;i<ttynum;i++)
    {
	sprintf(ttyname, "/dev/%s%d", argv[1], i);
	if((fd = open(ttyname, O_RDWR | O_NDELAY)) == -1)
	    break;

	// test serial port exist or not
	if(write(fd, ttyname, strlen(ttyname)) == -1)
	    break;

	// get serial port information
	serinfo.reserved_char[0] = 0;
	if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
	{
	    printf("Cannot get serial info\n");
	    close(fd);
	    return 2;
	}

	// get serial port type configuration
	if (serinfo.reserved_char[0] == 0)
	{
	    // RS232
	    printf("%s is RS232\n", ttyname);
	}
	else
	{
	    // RS422/485
	    printf("%s is RS422/485\n", ttyname);
	}

	close(fd);
    }
    return 0;
}
