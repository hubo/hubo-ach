
/*   Transmit sends a bunch of telegrams by filling them with       */
/*   different CANids and data bytes. You may send more than one    */
/*   telegram with write just by sending out an array of telegrams. */
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "can4linux.h"

#define STDDEV "can1"

int main(int argc,char **argv)
{
   int fd;
   int i, sent = 0;
   canmsg_t tx;
   char device[40];
   if(argc == 2) 
   {
      sprintf(device, "/dev/%s", argv[1]);
   }
   else 
   {
      sprintf(device, "/dev/%s", STDDEV);
   }
   printf("using CAN device %s\n", device);
   if(( fd = open(device, O_RDWR|O_NONBLOCK )) < 0 ) 
   {
      fprintf(stderr,"Error opening CAN device %s\n", device);
      exit(1);
   }
   for(i=0;i<500;i++)
   {
      memcpy(&tx.data[0],&i,4);
      if(i%2)
         tx.flags = 0; 
      else
         tx.flags = MSG_EXT;  
      tx.length = 4 ;
      tx.id=500 + i;
      while (sent == 0)
      {
         sent = write(fd,&tx,1);
      }
      printf("write down %d\n",i);
      sent = 0;
   }
   close(fd);
   return 0;
}

