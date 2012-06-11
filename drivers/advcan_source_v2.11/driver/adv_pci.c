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
 * File:              adv_pci.c
 * Author:            James Dai
 * Created:           2004-7-20
 * Revision:          1.00 
 *
 * Description:
 *
 * Driver init section for PCI-1680,MIC-3680,UNO-2052
 *
 */

#include <linux/pci.h>
#include "defs.h"


//reset the CAN device
void reset_ADV_PCI(int dev)
{
	unsigned char temp;

#ifdef CAN_DEBUG
	printk("Enter can_reset, device = %d\n", dev);
#endif
	if (dev < 0 || dev>= MAX_CHANNELS )
	{
#ifdef CAN_DEBUG
		printk( "in can_get_reg function the device=%d is error!\n", dev);
#endif
		return;
	}

	temp = CANin(dev, canmode );
	CANout(dev, canmode, temp|0x01 );
	udelay(10000);
}

static struct pci_device_id can_board_table[] = {
	{ADVANTECH_VANDORID, 0x1680, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0x3680, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0x2052, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0x1681, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0xc001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0xc002, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0xc004, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0xc101, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0xc102, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{ADVANTECH_VANDORID, 0xc104, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
	{0,}
			
};

static int __init can_init_one(struct pci_dev *pdev, const struct pci_device_id *id);
static void __exit can_remove_one(struct pci_dev *dev);

static struct pci_driver can_driver = {
	name:		   "advcan_pci",
	id_table:	can_board_table,
	probe:		can_init_one,
	remove:		can_remove_one,
};



static int __init can_init_one(struct pci_dev *pdev, const struct pci_device_id *id ) 
{
   unsigned int address,portNum,i;
   unsigned int idxport,bar,barFlag,offset;
   portNum = 0;
   idxport = 0;
   bar = 0;
   barFlag = 0;
   offset = 0x100;
   if (pci_enable_device (pdev))
   {
      return -EIO;
   }

   if ( pdev->device == 0xc001
      || pdev->device == 0xc002
      || pdev->device == 0xc004
      || pdev->device == 0xc101
      || pdev->device == 0xc102
      || pdev->device == 0xc104 )
   {
      portNum = pdev->device & 0xf;
   }
   else
   {
      if (pdev->device == 0x1680
         || pdev->device == 0x2052)
      {
         portNum = 2;
         bar = 2;
         barFlag = 1;
         offset = 0x0;
      }
      else if (pdev->device == 0x1681)
      {
         portNum = 1;
         bar = 2;
         barFlag = 1;
         offset = 0x0;
      }
   }
   
   for ( i = 0; i < portNum; i++)
   {
      address = pci_resource_start(pdev, bar)+ offset * i;
      //addlen = pci_resource_len(pdev,bar);
      //printk("addlen=%d", addlen);
      Base[numdevs] = address;
      if ( request_region(Base[numdevs], 128, "advcan") == NULL ) 
      {
         printk ("Device %d I/O %lX is not free.\n", numdevs, 
            Base[numdevs]);
         goto error_out;
      }
      IOModel[numdevs] = 'p';
      IRQ[numdevs] = pdev->irq;
      slot[numdevs] = pdev->devfn;
      Canout[ numdevs ] = canout_pci;
      Canin[ numdevs ] = canin_pci;
      Canset[ numdevs ] = canset_pci;
      Canreset[ numdevs ] = canreset_pci;
      Cantest[ numdevs ] = cantest_pci;
      reset_ADV_PCI(numdevs);
      numdevs++;
      if (barFlag)
      {
         bar++ ;
      }
   }
   printk("Advanteh PCI CAN devie %x found( %x CAN port)\n", 
         pdev->device,portNum );
   if (barFlag == 0)
   {
	printk("FPGA Version:%04x",(inb(pci_resource_start(pdev, 2)+1) << 8) + inb(pci_resource_start(pdev, 2)) );
   }
   return 0;
error_out:
   release_region((int)Base[numdevs], 0x100);
   return -EIO;
}

static void __exit can_remove_one( struct pci_dev *dev) //zdd modified
{
   int  i;
   for ( i = 0 ; i < MAX_CHANNELS; i++ ) 
   {
      if (slot[i]  == dev->devfn )
      {
         if (IRQ_requested[i] == 1)
         {
            //printk("Free IRQ %x\n",IRQ[i]);
            free_irq( IRQ[i], &Can_minors[i]);
         }
         //printk("Free baseaddress\n");
         release_region((int)Base[i], 128);
      }
   }
}

int init_adv_pci(void)
{
   int ret;
	ret = 0;
	ret = pci_register_driver(&can_driver);
	if(ret < 0) 
   {
		printk("No PCI Board Found!\n");
		return -ENODEV;
   }
	return 0;
}

void exit_adv_pci(void)
{
   pci_unregister_driver(&can_driver);
}
