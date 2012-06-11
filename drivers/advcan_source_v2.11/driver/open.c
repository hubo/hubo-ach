

#include "defs.h"
int Can_isopen[MAX_CHANNELS] = { 0 };   /* device minor already opened */

int can_open( __LDDK_OPEN_PARAM )
{
   int retval = 0;
   int lasterr = 0;
   
   DBGin("can_open");
   {	
		unsigned int minor = iminor(inode);
		if( minor > MAX_CHANNELS )
		{
	    	printk(KERN_ERR "CAN: Illegal minor number %d\n", minor);
	    	DBGout();
	    	return -EINVAL;
		}

    	/* check if device is already open, should be used only by one process */
		if(Can_isopen[minor] == 1) 
      {
         printk("can have opened");
	    	return -ENXIO;
		} 

		if( Base[minor] == 0x00) 
      {
	    	/* No device available */
	    	printk(KERN_ERR "CAN[%d]: no device available\n", minor);
	    	DBGout();
	    	return -ENXIO;
		}

		if (IOModel[minor] == 'm'){
			if( (lasterr = CAN_VendorInit(minor)) < 0 )
      	{
	    		DBGout();
	    		return lasterr;
			}
		}

  		else if (IOModel[minor] == 'p')
		{
			if( IRQ[minor] > 0 || IRQ[minor] > MAX_IRQNUMBER )
      	{
        		if( Can_RequestIrq( minor, IRQ[minor] , CAN_Interrupt) ) 
         	{
	     			printk("Can[%d]: Can't request IRQ %d \n", minor, IRQ[minor]);
	     			DBGout(); 
            	return -EBUSY;
        		}
    		} 
      	else 
      	{
				/* Invalid IRQ number in /proc/.../IRQ */
				DBGout(); return -EBUSY;
    		}
	   }
		//James dai add end
		/* Access macros based in can_base[] should work now */
		/* CAN_ShowStat(minor); */

		Can_WaitInit(minor);	/* initialize wait queue for select() */
		Can_FifoInit(minor);
#if CAN_USE_FILTER
		Can_FilterInit(minor);
#endif

		if( CAN_ChipReset(minor) < 0 ) 
                {
	    	  DBGout();
	    	  return -EINVAL;
		}
	  	/*int status = CANin(minor,canstat);
		if(status&0x2)	
			printk("!!!!!!!!!!!!!!!!!!! RXFIFO Over Run\n");
		else 
			printk("************** RXFIFO is ok\n");
		if(status&0x1)
			printk("*************RXFIFO is full and valid!!\n");	
		else
			printk("!!!!!!!!!!!!!!!!RXFIFO is empty!!\n");*/
	                                                              
                CAN_StartChip(minor);
	  	/*status = CANin(minor,canstat);
		printk("Open: status=0x%x",status);*/
#if DEBUG
		CAN_ShowStat(minor);
#endif
		++Can_isopen[minor]; // flag device in use 
   }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
	MOD_INC_USE_COUNT;
#endif
   DBGout();
   return retval;
}

