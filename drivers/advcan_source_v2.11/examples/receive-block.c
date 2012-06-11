/* This example puts all received datas in the file 'logfile.txt',
 * you can compare them with the transmited datas to test whether they are 
 * right
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "can4linux.h"

#define STDDEV "can0"
#define RXBUFFERSIZE 100

/***********************************************************************
*
* main 
*
*/

int main(int argc,char **argv)
{
   int fd;
   int got;
   int c,count = 0;
   char *pname;
   extern char *optarg;
   extern int optind;

   canmsg_t rx[RXBUFFERSIZE];
   char device[50];

   printf("usage: %s [dev] \n", argv[0]);
   if(argc > 1) 
   {
      sprintf(device, "/dev/%s", argv[1]);
   }
   else 
   {
		sprintf(device, "/dev/%s", STDDEV);
   }
   
   fd = open(device, O_RDONLY);
   if( fd < 0 ) 
   {
      fprintf(stderr,"Error opening CAN device %s\n", device);
      perror("open");
      exit(1);
   }
   printf("using CAN device %s\n", device);
   FILE * logFile = fopen("logfile.txt","w");
   while(1) //count<500)
   { 
      got=read(fd, &rx, 1);
      if( got > 0) 
      {  
         int i;
         int j;
	 
         count+=got;
         for(i = 0; i < got ; i++) 
         {
            fprintf(logFile,"%d\t",*(int*)rx[i].data ); 
            if(count%10==0)
              fprintf(logFile,"\n" ); 
            fflush(stdout);
           fflush(logFile);
          }
      } 
      else 
      {
         printf("Received with ret=%d\n", got);
         fflush(stdout);
      }
   }
   printf("count: %d\n",count);
   fclose(logFile);
   close(fd);
   return 0;
}
