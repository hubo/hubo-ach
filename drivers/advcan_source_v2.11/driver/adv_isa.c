// #############################################################################
// *****************************************************************************
//                  Copyright ( c ) 2006, Advantech Automation Corp.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//               INFORMATION WHICH IS THE PROPERTY OF ADVANTECH AUTOMATION CORP.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               ADVANTECH AUTOMATION CORP., IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
/***
 * File:              adv_isa.c
 * Author:            James Dai
 * Created:           2006-11-7
 * 
 * Description:
 *   adv_isa.c - advcan source file 
 *   Advantech  isa PC104 board using iup to two SJA1000 
 */
#include "defs.h"
int controller_available(unsigned long address, int offset)
{
	void __iomem *ptr = ioremap(address, 32 * offset);

	DBGin("controller_available");
	/* printk("controller_available 0x%lx\n", address); */


#if 0
    printk("0x%0x, ", readb(ptr + (0 * offset)) );
    printk("0x%0x, ", readb(ptr + (2 * offset)) );
    printk("0x%0x\n", readb(ptr + (3 * offset)) );
#endif

	if ( 0x21 == readb((void __iomem *)ptr))  {
		/* compare rest values of status and interrupt register */
		if(   0x0c == readb(ptr + (2 * offset))
	   	&& 0xe0 == readb(ptr + (3 * offset)) ) {
	    	return 1;
		} 
		else {
	    	return 0;
		}
	} 
	else {
		/* may be called after a 'soft reset' in 'PeliCAN' mode */
		/*   value     address                     mask    */
		if(   0x00 ==  readb(ptr + (1 * offset))
	   	&& 0x34 == (readb(ptr + (2 * offset))    & 0x37)
	   	&& 0x00 == (readb(ptr + (3 * offset))    & 0xfb)
	  	) {
	    	return 1;
		} else {
	    	return 0;
		}
	}
}

/*
 * PCM3680  Remarks
 *
 * Adresses used related to the Basde adress (set by dip switches) 
 * Base address (hex)      CAN controller
 * base:0000h - base:00FFh Basic- Port 1
 * base:0100h - base:01FFh HW reset Basic - Port 1
 * base:0200h - base:02FFh Basic- Port 2
 * base:0300h - base:03FFh HW reset Basic - Port 2
 * base:0400h - base:0FFFh Not used
 * 
 * Each CAN channel uses 0x200 bytes
 */
int CAN_VendorInit (int minor)
{
	DBGin("CAN_VendorInit");
	can_range[minor] = 0x200;
	/* Some LINUX systems, e.g. the PIP10 I tested on,
	 * locate already the memory using the information
	 * provided in the "Option ROM"
	 * The memory is marked as "Adapter-ROM" in /proc/iomem.
    * In this case the drive should not try to allocate the IO mem */

#if !defined(PC104_OPTION_ROM)
    /* Request the controllers address space */
	if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
		DBGprint(DBG_DATA,("Request_mem-Region CAN-IO failed at 0x%x\n",
		Base[minor]));
		return -EBUSY;
	}
#endif

	controller_available(Base[minor], 1);
	can_base[minor] = ioremap(Base[minor], can_range[minor]);

   //james dai add at 2006/11/15
	Canout[ minor ] = canout_isa;
	Canin[ minor ] = canin_isa;
	Canset[ minor ] = canset_isa;
	Canreset[ minor ] = canreset_isa;
	Cantest[ minor ] = cantest_isa;
   //james dai add end

	/* now the virtual address can be used for the register access macros */
	if( Base[minor] & 0x200 ) {
		/* printk("Resetting Advantech Pcm-3680 [contr 1]\n"); */
		/* perform HW reset 2. contr*/
		writeb(0xff, can_base[minor] + 0x300);
	} else {
		/* printk("Resetting Advantech Pcm-3680 [contr 0]\n"); */
		/* perform HW reset 1. contr*/
		writeb(0xff, can_base[minor] + 0x100);
	}
	mdelay(100);

	if( IRQ[minor] > 0 || IRQ[minor] > MAX_IRQNUMBER ){
		int err;
		err = request_irq( IRQ[minor], 
			CAN_Interrupt, 
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
			SA_SHIRQ, 
#else
			IRQF_SHARED,
#endif
			"advcan", 
			&Can_minors[minor]);
		if( !err ){
			DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
				IRQ[minor], (unsigned long)CAN_Interrupt));
	    	IRQ_requested[minor] = 1;
		} else {
	    	release_mem_region(Base[minor], can_range[minor]);
	    	DBGout(); return -EBUSY;
		}
    } else {
		/* Invalid IRQ number in /proc/.../IRQ */
		release_mem_region(Base[minor], can_range[minor]);
		DBGout(); return -EBUSY;
	}
	DBGout(); return 0;
}

/* Release IRQ and IO ressources */
int CAN_Release(int minor)
{
    DBGin("CAN_Release()");

    /* call this before freeing any memory or io area.
     * this can contain registers needed by Can_FreeIrq()
     */
    Can_FreeIrq(minor, IRQ[minor]);


    printk("iounmap %p \n", can_base[minor]);
    iounmap(can_base[minor]);

    /* release_mem_region(Base[minor], can_range[minor]); */
    /* Release the memory region */
    printk("release mem %lx \n", Base[minor]);
    release_mem_region(Base[minor], can_range[minor]);

    DBGout(); return 0;
}
