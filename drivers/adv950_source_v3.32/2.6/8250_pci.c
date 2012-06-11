//******************************************************************************
//
// Copyright (c) 2010 Advantech Industrial Automation Group.
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
// File:        8250_pci.c
// 
// PLEASE SEE 8250.c FOR DETAIL
//***********************************************************************
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/8250_pci.h>

#include <asm/bitops.h>
#include <asm/byteorder.h>
#include <asm/serial.h>
#include <asm/io.h>

#include "8250.h"





/*
 * Advantech IAG PCI-954/16C950 cards
 *
 */
#define ADVANTECH_16C950_VER                    "3.32"
#define ADVANTECH_16C950_DATE                   "11/12/2010"
#define PCI_VENDOR_ID_ADVANTECH                 0x13fe
#define PCI_DEVICE_ID_ADVANTECH_PCI1600         0x1600 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI1601         0x1601 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI1602         0x1602 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI1603         0x1603 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI1604         0x1604 /* Internal */
#define PCI_DEVICE_ID_ADVANTECH_PCI16ff         0x16ff /* External */
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1601    0x1601
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1602    0x1602
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1610    0x1610
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1612    0x1612 /* Also for UNO-2059 */
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1620    0x1620
#define PCI_DEVICE_ID_ADVANTECH_PCI1600_1622    0x1622
#define PCI_DEVICE_ID_ADVANTECH_UNO2050         0x2050
#define PCI_DEVICE_ID_ADVANTECH_UNOB2201        0x2201 //2668
#define PCI_DEVICE_ID_ADVANTECH_UNOBF201        0xf201 //2668
#define PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201   0x2201 //2668
#define PCI_DEVICE_ID_ADVANTECH_MIC3620         0x3620
#define PCI_DEVICE_ID_ADVANTECH_MIC3612         0X3612
#define PCI_DEVICE_ID_ADVANTECH_MIC3611         0X3611
//yongjun add 2006/11/08
#define PCI_DEVICE_ID_ADVANTECH_UNO2176         0x2176
#define PCI_DEVICE_ID_ADVANTECH_UNO2X76_2176    0x2176
//yongjun add end 2006/11/08

//jinxin added to support PCIe952/PCIe954/PCIe958
#define PCI_DEVICE_ID_ADVANTECH_PCIE952		0xA202
#define PCI_DEVICE_ID_ADVANTECH_PCIE954		0xA304
#define PCI_DEVICE_ID_ADVANTECH_PCIE958		0xA408

//james dai add 2007/5/28
//DEVICE ID
#define PCI_DEVICE_ID_ADVANTECH_PCM3614P        0x3614 //PCM-3614P
#define PCI_DEVICE_ID_ADVANTECH_PCM3641P        0x3641 //PCM-3641P

#define PCI_DEVICE_ID_ADVANTECH_PCM3618P        0x3618 //PCM-3618P
#define PCI_DEVICE_ID_ADVANTECH_PCMF618P        0xF618 //PCM-3618P
#define PCI_DEVICE_ID_ADVANTECH_PCM3681P        0x3681 //PCM-3681P
#define PCI_DEVICE_ID_ADVANTECH_PCMF681P        0xF681 //PCM-3681P

#define PCI_SUB_VENDOR_ID_ADVANTECH_PCM3614P    0x3614 //PCM-3614P
#define PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P    0x3618 //PCM-3618P
#define PCI_SUB_VENDOR_ID_ADVANTECH_PCM3641P    0x3641 //PCM-3641P
#define PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P    0x3681 //PCM-3681P
//james dai add end
//james dai add to support UNO-1150
#define PCI_DEVICE_ID_ADVANTECH_UNO1150         0x3610 //UNO-1150
//james dai add end
//james dai add 2008/07/20 to support MIC-3621
#define PCI_DEVICE_ID_ADVANTECH_MIC3621         0x3621 //PCM-3614P
#define PCI_SUB_VENDOR_ID_ADVANTECH_MIC3621     0x3621 //PCM-3614P
//james dai add end
#define PCI_DEVICE_ID_ADVANTECH_A001         0xA001
#define PCI_DEVICE_ID_ADVANTECH_A002         0xA002
#define PCI_DEVICE_ID_ADVANTECH_A004         0xA004
#define PCI_DEVICE_ID_ADVANTECH_A101         0xA101
#define PCI_DEVICE_ID_ADVANTECH_A102         0xA102
#define PCI_DEVICE_ID_ADVANTECH_A104         0xA104

#define PCI_DEVICE_ID_ADVANTECH_F001         0xF001
#define PCI_DEVICE_ID_ADVANTECH_F002         0xF002
#define PCI_DEVICE_ID_ADVANTECH_F004         0xF004
#define PCI_DEVICE_ID_ADVANTECH_F101         0xF101
#define PCI_DEVICE_ID_ADVANTECH_F102         0xF102
#define PCI_DEVICE_ID_ADVANTECH_F104         0xF104

int adv_register_serial(struct serial_struct *req);
void adv_unregister_serial(int line);

#define ACR_DTR_RS232 				0x00
#define ACR_DTR_ACTIVE_LOW_RS485        	0x10
#define ACR_DTR_ACTIVE_HIGH_RS485       	0x18

#define UART_TYPE_AUTO				0
#define UART_TYPE_RS232				1
#define UART_TYPE_RS485				2

/*
 * Definitions for PCI support.
 */
#define FL_BASE_MASK		0x0007
#define FL_BASE0		0x0000
#define FL_BASE1		0x0001
#define FL_BASE2		0x0002
#define FL_BASE3		0x0003
#define FL_BASE4		0x0004
#define FL_GET_BASE(x)		(x & FL_BASE_MASK)

/* Use successive BARs (PCI base address registers),
   else use offset into some specified BAR */
#define FL_BASE_BARS		0x0008

/* do not assign an irq */
#define FL_NOIRQ		0x0080

/* Use the Base address register size to cap number of ports */
#define FL_REGION_SZ_CAP	0x0100

static char * product_line[] = {"GENERAL","PCI","PCM","ADAM","APAX","BAS","UNO","TPC","EAMB"};
struct pci_board {
	unsigned int flags;
	unsigned int num_ports;
	unsigned int base_baud;
	unsigned int uart_offset;
	unsigned int reg_shift;
	unsigned int first_offset;
};

/*
 * init function returns:
 *  > 0 - number of ports
 *  = 0 - use board->num_ports
 *  < 0 - error
 */
struct pci_serial_quirk {
	u32	vendor;
	u32	device;
	u32	subvendor;
	u32	subdevice;
	int	(*init)(struct pci_dev *dev);
	int	(*setup)(struct pci_dev *dev, struct pci_board *board,
			 struct serial_struct *req, int idx);
	void	(*exit)(struct pci_dev *dev);
};

#define PCI_NUM_BAR_RESOURCES	6

struct serial_private {
	unsigned int		nr;
	void			*remapped_bar[PCI_NUM_BAR_RESOURCES];
	struct pci_serial_quirk	*quirk;
	int			line[0];
};

static void moan_device(const char *str, struct pci_dev *dev)
{
	printk(KERN_WARNING "%s: %s\n"
	       KERN_WARNING "Please send the output of lspci -vv, this\n"
	       KERN_WARNING "message (0x%04x,0x%04x,0x%04x,0x%04x), the\n"
	       KERN_WARNING "manufacturer and name of serial board or\n"
	       KERN_WARNING "modem board to rmk+serial@arm.linux.org.uk.\n",
	       pci_name(dev), str, dev->vendor, dev->device,
	       dev->subsystem_vendor, dev->subsystem_device);
}

static int
setup_port(struct pci_dev *dev, struct serial_struct *req,
	   int bar, int offset, int regshift)
{
	struct serial_private *priv = pci_get_drvdata(dev);
	unsigned long port, len;

	if (bar >= PCI_NUM_BAR_RESOURCES)
		return -EINVAL;

	if (pci_resource_flags(dev, bar) & IORESOURCE_MEM) {
		port = pci_resource_start(dev, bar);
		len =  pci_resource_len(dev, bar);

		if (!priv->remapped_bar[bar])
			priv->remapped_bar[bar] = ioremap(port, len);
		if (!priv->remapped_bar[bar])
			return -ENOMEM;

		req->io_type = UPIO_MEM;
		req->iomap_base = port + offset;
		req->iomem_base = priv->remapped_bar[bar] + offset;
		//printk(KERN_INFO "bar=%d, offset=%x, req->iomap_base=%x, req->iomem_base=%x, port =%x, len=%x\n", bar, offset, req->iomap_base, req->iomem_base, port, len);
		req->iomem_reg_shift = regshift;
	} else {
		port = pci_resource_start(dev, bar) + offset;
		req->io_type = UPIO_PORT;
		req->port = port;
		if (HIGH_BITS_OFFSET)
			req->port_high = port >> HIGH_BITS_OFFSET;
	}
	return 0;
}

static int
pci_default_setup(struct pci_dev *dev, struct pci_board *board,
		  struct serial_struct *req, int idx)
{
	unsigned int bar, offset = board->first_offset, maxnr;

	bar = FL_GET_BASE(board->flags);
	if (board->flags & FL_BASE_BARS)
		bar += idx;
	else{
		if(req->type & 0x10)
			offset += idx * board->uart_offset + 0x1000;
		else
			offset += idx * board->uart_offset;
	}
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18)
	maxnr = (pci_resource_len(dev, bar) - board->uart_offset) >>
		(board->reg_shift + 3);
	#else
	maxnr = (pci_resource_len(dev, bar) - board->uart_offset) /
		(8 << board->reg_shift);
	#endif
	if (board->flags & FL_REGION_SZ_CAP && idx >= maxnr)
		return 1;
			
	return setup_port(dev, req, bar, offset, board->reg_shift);
}

/*
 * Advantech IAG PCI-954/16C950 cards
 */
static int
pci_advantech_setup(struct pci_dev *dev, struct pci_board *board,
		  struct serial_struct *req, int idx)
{
	u32 bar, port485, offset485;
	u32 remap, len;
	u8 config485, activeType, configFunc, configType;
	u16  config485_958;
	struct pci_dev *cfgdev = NULL;
	int base_idx=0;
	int rc;
	
	configFunc = 1; // Default configuration BAR is function 1
	offset485 = 0x60; // Default offset to get RS232/422/485 configuration
	bar = PCI_BASE_ADDRESS_0; // Default BAR is PCI_BASE_ADDRESS_0
	activeType = ACR_DTR_ACTIVE_HIGH_RS485; // Default RS485 is active high
	configType = UART_TYPE_AUTO; // Default UART type is auto detection
	switch(dev->subsystem_vendor)
	{
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1601:
		printk("PCI-1601");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1602:
		printk("PCI-1602");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1610:
		printk("PCI-1610");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1612:	/* Also for UNO-2059 */
		printk("PCI-1612 / UNO-2059");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1620:
		printk("PCI-1620");
		break;
	case PCI_DEVICE_ID_ADVANTECH_PCI1600_1622:
		printk("PCI-1622CU");
		break;
	case PCI_DEVICE_ID_ADVANTECH_UNO2050:
		printk("UNO-2050");
		offset485 = 0x18;
		break;
	case PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201:
		printk("UNOB-2201CB");
		activeType = ACR_DTR_ACTIVE_LOW_RS485;
		break;
	//yongjun add 2006/11/10
	case PCI_DEVICE_ID_ADVANTECH_UNO2X76_2176:
		printk( "UNO-2176" );	
		activeType = ACR_DTR_ACTIVE_LOW_RS485;
		break;
	//yongjun add end 2006/11/10
	case PCI_DEVICE_ID_ADVANTECH_MIC3612:
		printk("MIC-3612");
		configFunc = 0;
		bar = PCI_BASE_ADDRESS_2;
		break;
	//James Dai add to support MIC3621
	case PCI_DEVICE_ID_ADVANTECH_MIC3621:
		printk("MIC-3621");
		configFunc = 0;
		bar = PCI_BASE_ADDRESS_2;
		break;
	//James Dai add end
	case PCI_DEVICE_ID_ADVANTECH_MIC3620:
		printk("MIC-3620");
		configType = UART_TYPE_RS232;
		break;
	/* Joshua Lan add @05/11/07 */
	case PCI_DEVICE_ID_ADVANTECH_MIC3611:
		printk("MIC-3611");
		configFunc = 0;
		bar = PCI_BASE_ADDRESS_2;
		configType = UART_TYPE_RS485;
		break;
	//james dai add @2007/5/27
	case PCI_SUB_VENDOR_ID_ADVANTECH_PCM3614P:
		printk( "PCM-3614P" );	
		activeType = ACR_DTR_ACTIVE_LOW_RS485;
		break;
	case PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P:
		printk( "PCM-3618P" );	
		activeType = ACR_DTR_ACTIVE_LOW_RS485;
		break;
	case PCI_SUB_VENDOR_ID_ADVANTECH_PCM3641P:
		printk( "PCM-3641P" );
		configType = UART_TYPE_RS232;	
		break;
	case PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P:
		printk( "PCM-3681P" );	
		configType = UART_TYPE_RS232;
		break;
	//james dai add end
	//james dai add add @2008/6/3
	case PCI_DEVICE_ID_ADVANTECH_UNO1150:
		printk( "UNO-1150" );
		configFunc = 0;
		base_idx = 2;
		bar = PCI_BASE_ADDRESS_2;
		offset485 = 0x10;	
		activeType = ACR_DTR_ACTIVE_LOW_RS485;
		break;
	//james dai add end
	case PCI_VENDOR_ID_ADVANTECH:
		switch(dev->device)
		{
		case PCI_DEVICE_ID_ADVANTECH_PCIE952:
			printk( "PCIE952");
			req->type |=  0x01;//this bit means to use new way to calculate baudrate
			req->type |= 0x02;//have DMA
			req->type |= 0x10;//is PCIe952/4/8
			configFunc = 0;

			base_idx = 13;//
			bar = PCI_BASE_ADDRESS_0;//ok
			offset485 = 0x100;//
			activeType = ACR_DTR_ACTIVE_LOW_RS485;//

			//configType = UART_TYPE_RS232;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCIE954:
			printk( "PCIE954");
			req->type |=  0x01;
			req->type |= 0x02;
			req->type |= 0x10;
			configFunc = 0;

			base_idx = 13;//
			bar = PCI_BASE_ADDRESS_0;//ok
			offset485 = 0x100;//
			activeType = ACR_DTR_ACTIVE_LOW_RS485;//

			//configType = UART_TYPE_RS232;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCIE958:
			printk( "PCIE958");
			req->type |=  0x01;
			req->type |= 0x02;
			req->type |= 0x10;
			configFunc = 0;

			base_idx = 13;//
			bar = PCI_BASE_ADDRESS_0;//ok
			offset485 = 0x100;//
			activeType = ACR_DTR_ACTIVE_LOW_RS485;//

			//configType = UART_TYPE_RS232;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCI1601:
			printk("PCI-1601A/B/AU/BU");
			configType = UART_TYPE_RS485;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCI1602:
			printk("PCI-1602A/B/AU/BU/UP");
			configType = UART_TYPE_RS485;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCI1603:
			printk("PCI-1603");
			configType = UART_TYPE_RS232;
			break;
		case PCI_DEVICE_ID_ADVANTECH_PCI1604:
			printk("PCI-1604UP");
			configType = UART_TYPE_RS232;
			break;

		}
		break;
	}
	if (dev->device == PCI_DEVICE_ID_ADVANTECH_A001
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A002
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A004
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A101
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A102
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_A104
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F001
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F002
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F004
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F101
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F102
	|| dev->device == PCI_DEVICE_ID_ADVANTECH_F104)
	{
		activeType = ACR_DTR_ACTIVE_LOW_RS485;
		if (dev->subsystem_vendor != PCI_VENDOR_ID_ADVANTECH)
		{
			printk("%s-%04x",product_line[dev->subsystem_vendor],dev->subsystem_device);
		}
		else
		{
			printk("Advantech General COM Port Device");
		}
			
	}
	if(configType == UART_TYPE_RS232)
	{
		req->reserved_char[0] = ACR_DTR_RS232;
	}
	else if(configType == UART_TYPE_RS485)
	{
		req->reserved_char[0] = activeType;
	}
	else // UART_TYPE_AUTO
	{
		// find RS232/422/485 configuration BAR
		do {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 14) 
			cfgdev = pci_find_device(PCI_VENDOR_ID_ADVANTECH,
				         PCI_ANY_ID, cfgdev);
#else
			cfgdev = pci_get_device(PCI_VENDOR_ID_ADVANTECH,
				         PCI_ANY_ID, cfgdev);
#endif
 
			if ((dev->bus->number == cfgdev->bus->number) &&
		    	    (PCI_SLOT(dev->devfn) == PCI_SLOT(cfgdev->devfn)) &&
			    (PCI_FUNC(cfgdev->devfn) == configFunc))
			{
				pci_read_config_dword(cfgdev, bar, &port485);
				if((port485 & PCI_BASE_ADDRESS_SPACE) ==
					PCI_BASE_ADDRESS_SPACE_IO)
				{
					rc = pci_enable_device(cfgdev);
					if (rc)
						return rc;
					port485 &= PCI_BASE_ADDRESS_IO_MASK;
					break;
				}
				break;
			}
		} while(cfgdev != NULL);

		// if cannot get RS232/422/485 configuration port
		if(!cfgdev)
		{
			printk("%x: cannot get RS232/422/485 configuration!\n",
				dev->subsystem_vendor);
			return -ENODEV;
		}
		if(req->type & 0x10){
			len =  pci_resource_len(cfgdev, ((bar-0x10)/0x04));
			if (pci_resource_flags(cfgdev, ((bar-0x10)/0x04)) & IORESOURCE_MEM) {
				remap = ioremap(port485, len);
				config485_958 = readw(remap + offset485 + idx*0x10);
				//printk(KERN_INFO "configure register = %x\n", config485_958);
				req->reserved_char[0] = (config485_958 & (0x01 << base_idx)) ?
					 activeType : ACR_DTR_RS232;
				goto done;
			}
			else{
				config485_958 = inw(port485 + offset485 + idx*0x10);
				req->reserved_char[0] = (config485_958 & (0x01 << base_idx)) ?
					 activeType : ACR_DTR_RS232;
				goto done;
			}
		}
		// read RS232/422/485 configuration value
		config485 = inb(port485 + offset485);
		if(PCI_FUNC(dev->devfn) == 1) base_idx=4;
		req->reserved_char[0] = (config485 & (0x01 << (base_idx+idx))) ?
					 activeType : ACR_DTR_RS232;
	}
done:
	printk(", function %d, port %d, %s",
		PCI_FUNC(dev->devfn), idx,
		(req->reserved_char[0] != ACR_DTR_RS232) ?
		"RS422/485" : "RS232");
	if(req->reserved_char[0] != ACR_DTR_RS232)
		printk(", %s\n", (activeType == ACR_DTR_ACTIVE_HIGH_RS485) ?
			"Active High" : "Active Low");
	else
		printk("\n");
	
	return pci_default_setup(dev, board, req, idx);
}


/*
 * Master list of serial port init/setup/exit quirks.
 * This does not describe the general nature of the port.
 * (ie, baud base, number and location of ports, etc)
 *
 * This list is ordered alphabetically by vendor then device.
 * Specific entries must come before more generic entries.
 */
static struct pci_serial_quirk pci_serial_quirks[] = {
	/*
	 * Advantech IAG PCI-954/16C950 cards
	 */
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1601,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1602,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1610,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1612,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1620,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI16ff,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1620,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1600,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1622,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI16ff,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_PCI1600_1622,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNO2050,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNO2050,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNOB2201,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNOBF201,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_MIC3612,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_MIC3612,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	// Joshua Lan add @2005/11/07
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_MIC3611,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_MIC3611,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_MIC3620,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_MIC3620,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1601,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1602,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1603,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCI1604,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},

	//yongjun add 2006/11/10
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNO2176,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNO2X76_2176,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	//yongjun add end
	//jinxin added to support PCIe952/PCIe954/PCIe958
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCIE952,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_DEVICE_ID_ADVANTECH_PCIE952,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCIE954,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_DEVICE_ID_ADVANTECH_PCIE954,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCIE958,
		.subvendor	= PCI_VENDOR_ID_ADVANTECH,
		.subdevice	= PCI_DEVICE_ID_ADVANTECH_PCIE958,
		.setup		= pci_advantech_setup,
	},
	//james dai add 2007/5/27
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCM3614P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3614P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCM3618P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCMF618P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCM3641P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3641P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCM3681P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_PCMF681P,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	//james dai add end
	//james dai add to support UNO-1150 @2008/6/3
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_UNO1150,
		.subvendor	= PCI_DEVICE_ID_ADVANTECH_UNO1150,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	//james dai add end
	//james dai add to support UNO-1150 @2008/6/20
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_MIC3621,
		.subvendor	= PCI_SUB_VENDOR_ID_ADVANTECH_MIC3621,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	//james dai add end
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A001,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A002,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A004,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A101,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A102,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_A104,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F001,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F002,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F004,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F101,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F102,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
	{
		.vendor		= PCI_VENDOR_ID_ADVANTECH,
		.device		= PCI_DEVICE_ID_ADVANTECH_F104,
		.subvendor	= PCI_ANY_ID,
		.subdevice	= PCI_ANY_ID,
		.setup		= pci_advantech_setup,
	},
};

static inline int quirk_id_matches(u32 quirk_id, u32 dev_id)
{
	return quirk_id == PCI_ANY_ID || quirk_id == dev_id;
}

static struct pci_serial_quirk *find_quirk(struct pci_dev *dev)
{
	struct pci_serial_quirk *quirk;

	for (quirk = pci_serial_quirks; ; quirk++)
		if (quirk_id_matches(quirk->vendor, dev->vendor) &&
		    quirk_id_matches(quirk->device, dev->device) &&
		    quirk_id_matches(quirk->subvendor, dev->subsystem_vendor) &&
		    quirk_id_matches(quirk->subdevice, dev->subsystem_device))
		 	break;
	return quirk;
}

static _INLINE_ int
get_pci_irq(struct pci_dev *dev, struct pci_board *board, int idx)
{
	if (board->flags & FL_NOIRQ)
		return 0;
	else
		return dev->irq;
}

/*
 * This is the configuration table for all of the PCI serial boards
 * which we support.  It is directly indexed by the pci_board_num_t enum
 * value, which is encoded in the pci_device_id PCI probe table's
 * driver_data member.
 *
 * The makeup of these names are:
 *  pbn_bn{_bt}_n_baud
 *
 *  bn   = PCI BAR number
 *  bt   = Index using PCI BARs
 *  n    = number of serial ports
 *  d	 = support dma or not
 *  baud = baud rate
 *
 * Please note: in theory if n = 1, _bt infix should make no difference.
 * ie, pbn_b0_1_115200 is the same as pbn_b0_bt_1_115200
 */
enum pci_board_num_t {
	pbn_default = 0,

	pbn_b0_1_115200,
	pbn_b0_2_115200,
	pbn_b0_4_115200,
	pbn_b0_5_115200,

	pbn_b0_1_921600,
	pbn_b0_2_921600,
	pbn_b0_4_921600,

	pbn_b0_2_d_921600,
	pbn_b0_4_d_921600,
	pbn_b0_8_d_921600,

	pbn_b0_bt_1_115200,
	pbn_b0_bt_2_115200,
	pbn_b0_bt_8_115200,

	pbn_b0_bt_1_460800,
	pbn_b0_bt_2_460800,
	pbn_b0_bt_4_460800,

	pbn_b0_bt_1_921600,
	pbn_b0_bt_2_921600,
	pbn_b0_bt_4_921600,
	pbn_b0_bt_8_921600,

	pbn_b1_1_115200,
	pbn_b1_2_115200,
	pbn_b1_4_115200,
	pbn_b1_8_115200,

	pbn_b1_1_921600,
	pbn_b1_2_921600,
	pbn_b1_4_921600,
	pbn_b1_8_921600,

	pbn_b1_bt_2_921600,

	pbn_b1_2_1382400,
	pbn_b1_4_1382400,
	pbn_b1_8_1382400,

	pbn_b2_1_115200,
	pbn_b2_8_115200,

	pbn_b2_1_460800,
	pbn_b2_4_460800,
	pbn_b2_8_460800,
	pbn_b2_16_460800,

	pbn_b2_1_921600,
	pbn_b2_4_921600,
	pbn_b2_8_921600,

	pbn_b2_bt_1_115200,
	pbn_b2_bt_2_115200,
	pbn_b2_bt_4_115200,

	pbn_b2_bt_2_921600,
	pbn_b2_bt_4_921600,

	pbn_b3_4_115200,
	pbn_b3_8_115200,
};

static struct pci_board pci_boards[] __devinitdata = {
	[pbn_default] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_1_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_2_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_4_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_5_115200] = {
		.flags		= FL_BASE0,
		.num_ports	= 5,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b0_1_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_2_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_4_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b0_2_d_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 2,
		.base_baud	= 62500000/16,
		.uart_offset	= 0x200,
	},
	[pbn_b0_4_d_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 4,
		.base_baud	= 62500000/16,
		.uart_offset	= 0x200,
	},

	[pbn_b0_8_d_921600] = {
		.flags		= FL_BASE0,
		.num_ports	= 8,
		.base_baud	= 62500000/16,
		.uart_offset	= 0x200,
	},
	[pbn_b0_bt_1_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_8_115200] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b0_bt_1_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_4_460800] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},

	[pbn_b0_bt_1_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_2_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_4_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b0_bt_8_921600] = {
		.flags		= FL_BASE0|FL_BASE_BARS,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b1_1_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_2_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_4_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b1_8_115200] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b1_1_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_2_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_4_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b1_8_921600] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b1_bt_2_921600] = {
		.flags		= FL_BASE1|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b1_2_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 2,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},
	[pbn_b1_4_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 4,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},
	[pbn_b1_8_1382400] = {
		.flags		= FL_BASE1,
		.num_ports	= 8,
		.base_baud	= 1382400,
		.uart_offset	= 8,
	},

	[pbn_b2_1_115200] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_8_115200] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b2_1_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_4_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 4,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_8_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 460800,
		.uart_offset	= 8,
	},
	[pbn_b2_16_460800] = {
		.flags		= FL_BASE2,
		.num_ports	= 16,
		.base_baud	= 460800,
		.uart_offset	= 8,
	 },

	[pbn_b2_1_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 1,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_4_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_8_921600] = {
		.flags		= FL_BASE2,
		.num_ports	= 8,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b2_bt_1_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 1,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_2_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_4_115200] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},

	[pbn_b2_bt_2_921600] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 2,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},
	[pbn_b2_bt_4_921600] = {
		.flags		= FL_BASE2|FL_BASE_BARS,
		.num_ports	= 4,
		.base_baud	= 921600,
		.uart_offset	= 8,
	},

	[pbn_b3_4_115200] = {
		.flags		= FL_BASE3,
		.num_ports	= 4,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
	[pbn_b3_8_115200] = {
		.flags		= FL_BASE3,
		.num_ports	= 8,
		.base_baud	= 115200,
		.uart_offset	= 8,
	},
};

/*
 * Given a complete unknown PCI device, try to use some heuristics to
 * guess what the configuration might be, based on the pitiful PCI
 * serial specs.  Returns 0 on success, 1 on failure.
 */
static int __devinit
serial_pci_guess_board(struct pci_dev *dev, struct pci_board *board)
{
	int num_iomem, num_port, first_port = -1, i;
	
	/*
	 * If it is not a communications device or the programming
	 * interface is greater than 6, give up.
	 *
	 * (Should we try to make guesses for multiport serial devices
	 * later?) 
	 */
	if ((((dev->class >> 8) != PCI_CLASS_COMMUNICATION_SERIAL) &&
	     ((dev->class >> 8) != PCI_CLASS_COMMUNICATION_MODEM)) ||
	    (dev->class & 0xff) > 6)
		return -ENODEV;

	num_iomem = num_port = 0;
	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (pci_resource_flags(dev, i) & IORESOURCE_IO) {
			num_port++;
			if (first_port == -1)
				first_port = i;
		}
		if (pci_resource_flags(dev, i) & IORESOURCE_MEM)
			num_iomem++;
	}

	/*
	 * If there is 1 or 0 iomem regions, and exactly one port,
	 * use it.  We guess the number of ports based on the IO
	 * region size.
	 */
	if (num_iomem <= 1 && num_port == 1) {
		board->flags = first_port;
		board->num_ports = pci_resource_len(dev, first_port) / 8;
		return 0;
	}

	/*
	 * Now guess if we've got a board which indexes by BARs.
	 * Each IO BAR should be 8 bytes, and they should follow
	 * consecutively.
	 */
	first_port = -1;
	num_port = 0;
	for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
		if (pci_resource_flags(dev, i) & IORESOURCE_IO &&
		    pci_resource_len(dev, i) == 8 &&
		    (first_port == -1 || (first_port + num_port) == i)) {
			num_port++;
			if (first_port == -1)
				first_port = i;
		}
	}

	if (num_port > 1) {
		board->flags = first_port | FL_BASE_BARS;
		board->num_ports = num_port;
		return 0;
	}

	return -ENODEV;
}

static inline int
serial_pci_matches(struct pci_board *board, struct pci_board *guessed)
{
	return
	    board->num_ports == guessed->num_ports &&
	    board->base_baud == guessed->base_baud &&
	    board->uart_offset == guessed->uart_offset &&
	    board->reg_shift == guessed->reg_shift &&
	    board->first_offset == guessed->first_offset;
}

/*
 * Probe one serial board.  Unfortunately, there is no rhyme nor reason
 * to the arrangement of serial ports on a PCI card.
 */
static int __devinit
pciserial_init_one(struct pci_dev *dev, const struct pci_device_id *ent)
{
	struct serial_private *priv;
	struct pci_board *board, tmp;
	struct pci_serial_quirk *quirk;
	struct serial_struct serial_req;
	int base_baud, rc, nr_ports, i;


	if (ent->driver_data >= ARRAY_SIZE(pci_boards)) {
		printk(KERN_ERR "pci_init_one: invalid driver_data: %ld\n",
			ent->driver_data);
		return -EINVAL;
	}

	board = &pci_boards[ent->driver_data];

	rc = pci_enable_device(dev);
	if (rc)
		return rc;

	if (ent->driver_data == pbn_default) {
		/*
		 * Use a copy of the pci_board entry for this;
		 * avoid changing entries in the table.
		 */
		memcpy(&tmp, board, sizeof(struct pci_board));
		board = &tmp;

		/*
		 * We matched one of our class entries.  Try to
		 * determine the parameters of this board.
		 */
		rc = serial_pci_guess_board(dev, board);
		if (rc)
			goto disable;
	} else {
		/*
		 * We matched an explicit entry.  If we are able to
		 * detect this boards settings with our heuristic,
		 * then we no longer need this entry.
		 */
		memcpy(&tmp, &pci_boards[pbn_default], sizeof(struct pci_board));
		rc = serial_pci_guess_board(dev, &tmp);
		if (rc == 0 && serial_pci_matches(board, &tmp))
			moan_device("Redundant entry in serial pci_table.",
				    dev);
	}

	nr_ports = board->num_ports;

	/*
	 * Find an init and setup quirks.
	 */
	quirk = find_quirk(dev);

	/*
	 * Run the new-style initialization function.
	 * The initialization function returns:
	 *  <0  - error
	 *   0  - use board->num_ports
	 *  >0  - number of ports
	 */
	if (quirk->init) {
		rc = quirk->init(dev);
		if (rc < 0)
			goto disable;
		if (rc)
			nr_ports = rc;
	}

	priv = kmalloc(sizeof(struct serial_private) +
		       sizeof(unsigned int) * nr_ports,
		       GFP_KERNEL);
	if (!priv) {
		rc = -ENOMEM;
		goto deinit;
	}

	memset(priv, 0, sizeof(struct serial_private) +
			sizeof(unsigned int) * nr_ports);

	priv->quirk = quirk;
	pci_set_drvdata(dev, priv);

	base_baud = board->base_baud;
	if (!base_baud) {
		moan_device("Board entry does not specify baud rate.", dev);
		base_baud = BASE_BAUD;
	}
	for (i = 0; i < nr_ports; i++) {
		memset(&serial_req, 0, sizeof(serial_req));
//lipeng modify at 06/08/2006
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
		serial_req.flags = UPF_SKIP_TEST | UPF_AUTOPROBE |
				   //UPF_RESOURCES | UPF_SHARE_IRQ;
				   UPF_SHARE_IRQ;
#else
		serial_req.flags = UPF_SKIP_TEST | UPF_BOOT_AUTOCONF | UPF_SHARE_IRQ;
#endif
//lipeng modify end
		serial_req.baud_base = base_baud;
		serial_req.irq = get_pci_irq(dev, board, i);
		if (quirk->setup(dev, board, &serial_req, i))
			break;
#ifdef SERIAL_DEBUG_PCI
		printk("Setup PCI port: port %x, irq %d, type %d\n",
		       serial_req.port, serial_req.irq, serial_req.io_type);
#endif
		
		priv->line[i] = adv_register_serial(&serial_req);
		if (priv->line[i] < 0) {
			printk(KERN_WARNING "Couldn't register serial port %s: %d\n", pci_name(dev), priv->line[i]);
			break;
		}
	}

	priv->nr = i;

	return 0;

 deinit:
	if (quirk->exit)
		quirk->exit(dev);
 disable:
	pci_disable_device(dev);
	return rc;
}

static void __devexit pciserial_remove_one(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	pci_set_drvdata(dev, NULL);

	if (priv) {
		struct pci_serial_quirk *quirk;
		int i;

		for (i = 0; i < priv->nr; i++)
			adv_unregister_serial(priv->line[i]);

		for (i = 0; i < PCI_NUM_BAR_RESOURCES; i++) {
			if (priv->remapped_bar[i])
				iounmap(priv->remapped_bar[i]);
			priv->remapped_bar[i] = NULL;
		}

		/*
		 * Find the exit quirks.
		 */
		quirk = find_quirk(dev);
		if (quirk->exit)
			quirk->exit(dev);

		pci_disable_device(dev);

		kfree(priv);
	}
}
//lipeng modify at 06/08/2006
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
static int pciserial_suspend_one(struct pci_dev *dev, u32 state)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	if (priv) {
		int i;

		for (i = 0; i < priv->nr; i++)
			adv_serial8250_suspend_port(priv->line[i]);
	}
	return 0;
}
#else
static int pciserial_suspend_one(struct pci_dev *dev, pm_message_t state)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	if (priv) {
		int i;

		for (i = 0; i < priv->nr; i++)
			adv_serial8250_suspend_port(priv->line[i]);
	}
	return 0;
}
#endif
//lipeng modify end
static int pciserial_resume_one(struct pci_dev *dev)
{
	struct serial_private *priv = pci_get_drvdata(dev);

	if (priv) {
		int i;

		/*
		 * Ensure that the board is correctly configured.
		 */
		if (priv->quirk->init)
			priv->quirk->init(dev);

		for (i = 0; i < priv->nr; i++)
			adv_serial8250_resume_port(priv->line[i]);
	}
	return 0;
}

static struct pci_device_id adv_serial_pci_tbl[] = {
	/*
	 * Advantech IAG PCI-954/16C950 cards
	 */
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600, 
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1601, PCI_ANY_ID, 0, 0, 
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600, 
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1602, PCI_ANY_ID, 0, 0, 
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600, 
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1610, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600, 
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1612, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600, 
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1620, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI16ff, 
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1620, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1600, 
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1622, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI16ff, 
		PCI_DEVICE_ID_ADVANTECH_PCI1600_1622, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNO2050, 
		PCI_DEVICE_ID_ADVANTECH_UNO2050, PCI_ANY_ID, 0, 0, 
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNOB2201, 
		PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNOBF201, 
		PCI_DEVICE_ID_ADVANTECH_UNOBX201_2201, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_MIC3612, 
		PCI_DEVICE_ID_ADVANTECH_MIC3612, PCI_ANY_ID, 0, 0, 
		pbn_b2_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_MIC3620, 
		PCI_DEVICE_ID_ADVANTECH_MIC3620, PCI_ANY_ID, 0, 0, 
		pbn_b2_8_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_MIC3611, 
		PCI_DEVICE_ID_ADVANTECH_MIC3611, PCI_ANY_ID, 0, 0, 
		pbn_b2_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1601, 
		PCI_VENDOR_ID_ADVANTECH, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1602, 
		PCI_VENDOR_ID_ADVANTECH, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1603, 
		PCI_VENDOR_ID_ADVANTECH, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCI1604, 
		PCI_VENDOR_ID_ADVANTECH, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_2_921600 },
	//yongjun add
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNO2176, 
		PCI_DEVICE_ID_ADVANTECH_UNO2X76_2176, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	//yongjun add end
	//jinxin added to support PCIe952/PCIe954/PCIe958
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCIE952, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_2_d_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCIE954, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_d_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCIE958, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_8_d_921600 },
	//james dai add
	//PCM-3614P 4 PORTS RS232/422/485
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCM3614P, 
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3614P, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	//PCM-3618P 8 PORTS RS232/422/485
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCM3618P, 
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCMF618P, 
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3618P, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	//PCM-3641P 4 PORTS RS232
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCM3641P, 
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3641P, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	//PCM-3681P 8 PORTS RS232
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCM3681P, 
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600},
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_PCMF681P, 
		PCI_SUB_VENDOR_ID_ADVANTECH_PCM3681P, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	//james dai add end
	//james dai add to support UNO-1150
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_UNO1150, 
		PCI_DEVICE_ID_ADVANTECH_UNO1150, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_2_921600 },
	//james dai add end
	//james dai add to support MIC-3621
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_MIC3621, 
		PCI_SUB_VENDOR_ID_ADVANTECH_MIC3621, PCI_ANY_ID, 0, 0, 
		pbn_b2_8_921600 },
	//james dai add end
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A001, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_1_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A002, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A004, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A101, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_1_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A102, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_A104, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F001, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_1_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F002, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F004, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_4_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F101, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_1_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F102, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_2_921600 },
	{	PCI_VENDOR_ID_ADVANTECH, PCI_DEVICE_ID_ADVANTECH_F104, 
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 
		pbn_b0_bt_4_921600 },
	{ 0, }
};

static struct pci_driver serial_pci_driver = {
	.name		= "advserial",
	.probe		= pciserial_init_one,
	.remove		= __devexit_p(pciserial_remove_one),
	.suspend	= pciserial_suspend_one,
	.resume		= pciserial_resume_one,
	.id_table	= adv_serial_pci_tbl,
};

int adv_serial8250_init(void);
void adv_serial8250_exit(void);
static int __init adv_serial8250_pci_init(void)
{
	printk("\n");
	printk("==========================================================="
	       "====\n");
	printk("Advantech PCI-954/952/16C950 Device Drivers. V%s [%s]\n",
		ADVANTECH_16C950_VER, ADVANTECH_16C950_DATE);
 	printk("Supports: RS232/422/485 auto detection and setting\n");
 	printk("Devices:  UNO:  UNO2050 [COM3/COM4]\n");
 	printk("                UNO2059 [COM1~COM4]\n");
 	printk("                UNOB-2201CB [COM1~COM8]\n");
 	printk("                UNOB-2176 [COM1~COM4]\n");
	printk("               	UNO-1150 [COM2/COM3]\n");
	printk("               	UNO-2679 [COM3~COM6]\n");
	printk("               	UNO-4672 [COM3~COM10]\n");
 	printk("          ICOM: PCI-1601, PCI-1602\n"
	       "                PCI-1603, PCI-1604\n"
	       "                PCI-1610, PCI-1612\n"
	       "                PCI-1620, PCI-1622\n");
 	printk("          MIC:  MIC-3611, MIC-3612\n");
 	printk("                MIC-3620, MIC-3621\n");
 	printk("          PCM:  PCM-3614P/I, PCM-3641P/I\n");
 	printk("                PCM-3618P/I, PCM-3681P/I\n");
	printk("Advantech Industrial Automation Group.\n"); 
	printk("==========================================================="
	       "====\n");
	if(adv_serial8250_init() >= 0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 21) 
		return pci_module_init(&serial_pci_driver);
#else
		return pci_register_driver(&serial_pci_driver);
#endif
	else
	{
		printk("Failed to do adv_serial8250_init()\n");
		return -EINVAL;
	}
}

static void __exit adv_serial8250_pci_exit(void)
{
	pci_unregister_driver(&serial_pci_driver);
	adv_serial8250_exit();
}

module_init(adv_serial8250_pci_init);
module_exit(adv_serial8250_pci_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Advantech IAG PCI-954/16C950 serial probe module");
MODULE_DEVICE_TABLE(pci, adv_serial_pci_tbl);
