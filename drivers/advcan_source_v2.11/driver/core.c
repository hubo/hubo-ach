// #############################################################################
// *****************************************************************************
//                  Copyright ( c ) 2011, Advantech Automation Corp.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//               INFORMATION WHICH IS THE PROPERTY OF ADVANTECH AUTOMATION CORP.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               ADVANTECH AUTOMATION CORP., IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
/***
 * Author:      Jianfeng.dai
 * Created:     2004-7-20
 * Description:  ADVANTECH CAN communication cards device driver
 * 
 */
#include <linux/init.h>
#include <linux/fs.h>			
#include <linux/pci.h>
#include "defs.h"
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0) 
# include <linux/device.h>
#endif

MODULE_AUTHOR("James Dai");
MODULE_DESCRIPTION("Advantech CAN fieldbus driver");
MODULE_LICENSE("GPL");
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0) 
MODULE_VERSION("2.5");
#endif
static char *ADVANTECH_CAN_VER = "2.11"; 
static char *ADVANTECH_CAN_REV = "2011-03-25";

#define CANREGDEVNAME "advcan"

int IRQ_requested[MAX_CHANNELS] = { 0 };
int Can_minors[MAX_CHANNELS]    = { 0 }; /* used as IRQ dev_id */
int slot[MAX_CHANNELS]    = { 0 }; /* used as IRQ dev_id */

int Can_major = 0;

int numdevs = 0;
int ioresetbase[ MAX_CHANNELS];

/*
There's a C99 way of assigning to elements of a structure,
and this is definitely preferred over using the GNU extension.
gcc 2.95, supports the new C99 syntax.
The meaning is clear, and you should be aware
that any member of the structure which you don't explicitly assign
will be initialized to NULL by gcc.
*/

static struct file_operations can_fops = { 
    .owner	=	THIS_MODULE,
    .open	=	can_open,
    .release	=	can_close,
    .read	=	can_read,
    .write	=	can_write,
    .poll	=	can_select,
    .ioctl	=	can_ioctl,
    .fasync	=	can_fasync,
};

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0) 
static int __init can_init(void)
#else
static int init_module(void)
#endif
{
	int i;


    /* do you want do see debug message already while loading the driver ?
     	* Then enable this line and set the mask != 0
     	*/
	dbgMask = 7; 

	DBGin("init_module");
	printk("\n=====================================================\n");
	printk("CAN Communication Driver.\nVersion: %s (%s)\n", 
		ADVANTECH_CAN_VER, ADVANTECH_CAN_REV);
	printk("ADVANTECH Co,Ltd.\n");
	printk("=====================================================\n");

	  
   /*
   initialize the variables layed down in /proc/sys/Can
   ====================================================
   */
   for (i = 0; i < MAX_CHANNELS; i++) 
   {
      IOModel[i]       = IO_MODEL;
      Baud[i]          = 125;
      AccCode[i]       = AccMask[i] =  STD_MASK;
      Timeout[i]       = 100;
      Outc[i]          = CAN_OUTC_VAL;
      IRQ_requested[i] = 0;
      Can_minors[i]    = i;		/* used as IRQ dev_id */
      slot[i] = -1;
   }

   /* after initializing channel based parameters
    * finish some entries 
    * and do drivers specific initialization
    */
   IOModel[i] = '\0';
   Can_major = register_chrdev(
                  0,
                  CANREGDEVNAME,
                  &can_fops);
 	
	/* Negative values signify an error */
   if ( Can_major < 0 )
   {
#ifdef CAN_DEBUG
      printk ( "%s: register_chrdev() failed with %d\n",
      CANREGDEVNAME,
      Can_major);
#endif
      return -ENODEV;
   }
   else
   {
#ifdef CAN_DEBUG
		printk("The major number is %d\n", Can_major );
#endif
   }

    //#ifdef ADVANTECH_PCI
    /* make some syctl entries read only
     * IRQ number
     * Base address
     * and access mode
     * are fixed and provided by the PCI BIOS
     */
   //Can_sysctl_table[SYSCTL_IRQ  - 1].mode = 0444;
   //Can_sysctl_table[SYSCTL_BASE - 1].mode = 0444;
   /* printk(KERN_INFO "CAN pci test loaded\n"); */
   /* dbgMask = 0; */
   if(init_adv_pci()) 
   {
      unregister_chrdev(Can_major, CANREGDEVNAME);
      return -EIO;
   }
   //printk("\nAdvantech CAN Communication card driver loaded!\n\n");
//#endif


#if LDDK_USE_PROCINFO
   register_procinfo();
#endif
#if LDDK_USE_SYSCTL
   register_systables();
#endif
   DBGout();
   return 0;
}



#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0) 
static void can_exit(void)
#else
static void cleanup_module(void)
#endif
{

	DBGin("cleanup_module");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0) 
  	if (MOD_IN_USE) 
        {
            printk(KERN_WARNING "Can : device busy, remove delayed\n");
  	}
#endif
        
//#ifdef ADVANTECH_PCI
   exit_adv_pci();
//#endif

   unregister_chrdev(Can_major, CANREGDEVNAME);

#if LDDK_USE_PROCINFO
   unregister_procinfo();
#endif
#if LDDK_USE_SYSCTL
   unregister_systables();
#endif
   DBGout();
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0) 
   module_init(can_init);
   module_exit(can_exit);
   /* EXPORT_NO_SYMBOLS; */
#endif



