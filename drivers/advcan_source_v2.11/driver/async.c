/*
 * can_async - advcan CAN driver module
 *
 * advcan -- LINUX CAN device driver source
 */
#include "defs.h"


int can_fasync( __LDDK_FASYNC_PARAM ) /* inode, file, count */
{
   int retval = 0;
#if 0
   can_data_t *dev = (can_data_t *)file->private_data;

   DBGin("can_fasync");
   {
      retval = fasync_helper(fd, file, count, &dev->fasyncptr);
      DBGprint(DBG_DATA,(" -- async  count %d  retval %d\n", 
			count, retval));
      if ( retval > 0 )
	      retval = 0;
   }
   DBGout();
#endif
   return retval;
}

