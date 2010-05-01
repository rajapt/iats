/* AtherosL2Ethernet.h -- atl2 driver definitions
 *
 * Copyright (c) 2008 maolj <maolj@hotmail.com>.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Driver for Atheros L2 Fast Ethernet adapter.  This 10/100 Mbit NIC
 * is included in many ASUS products, including the ASUS EeePC.
 *
 * This driver is heavily based on atl2 Linux driver by xiong huang <xiong.huang@atheros.com>.
 */

#ifndef _ATHEROSL1ETHERNET_H_
#define _ATHEROSL1ETHERNET_H_

#include <IOKit/IOLib.h>
#include <IOKit/IOTypes.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODMACommand.h>
#include <IOKit/IOFilterInterruptEventSource.h>

#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>

#include <IOKit/pci/IOPCIDevice.h>

#include <libkern/OSAtomic.h>

extern "C"
{
	#include <sys/kpi_mbuf.h>
}

#include "mii.h"
#include "at_hw.h"
#include "at.h"
#include "at_param.h"
#include "at_main.h"

enum
{
	MEDIUM_INDEX_10HD	= 0,
	MEDIUM_INDEX_10FD	= 1,
	MEDIUM_INDEX_100HD	= 2,
	MEDIUM_INDEX_100FD	= 3,
	MEDIUM_INDEX_AUTO	= 4,
	MEDIUM_INDEX_COUNT	= 5
};

//#define MAX_TX_BUF_LEN		0x3000	/* 12KB */
//#define MAX_RX_BUF_LEN		1608
#define MAX_TX_BUF_LEN		1608
#define MBit				1000000
#define kTransmitQueueCapacity			384

class AtherosL2Ethernet : public IOEthernetController
{
	OSDeclareDefaultStructors(AtherosL2Ethernet)
public:
	virtual bool				init(OSDictionary *properties);
	virtual void				free();
	virtual bool				start(IOService *provider);
	virtual void				stop(IOService *provider);


	virtual IOReturn			enable(IONetworkInterface *netif);
    virtual IOReturn			disable(IONetworkInterface *netif);

	virtual bool				configureInterface(IONetworkInterface *netif);
    virtual const OSString		*newVendorString() const;
    virtual const OSString		*newModelString() const;
	virtual IOReturn			selectMedium(const IONetworkMedium *medium);

	virtual IOReturn			getHardwareAddress(IOEthernetAddress *addr);
	virtual IOReturn			setHardwareAddress(const IOEthernetAddress *addr);

    virtual void				getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const;
	virtual UInt32				outputPacket(mbuf_t m, void *param);

    virtual IOReturn			registerWithPolicyMaker(IOService *policyMaker);
    virtual IOReturn			setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker);

    virtual IOReturn			setPromiscuousMode(bool enabled);
    virtual IOReturn			setMulticastMode(bool enabled);
    virtual IOReturn			setMulticastList(IOEthernetAddress *addrs, UInt32 count);
private:
	virtual bool				OSAddNetworkMedium(UInt32 type, UInt32 bps, UInt32 index);
	virtual IOOutputQueue		*createOutputQueue();


	void						atIntr(OSObject *client, IOInterruptEventSource *src, int count);
	virtual void				at_intr_tx(at_adapter *adapter);
	virtual void				at_intr_rx(at_adapter *adapter);
	
	virtual int					mdio_read(int phy_id, int reg_num);
	virtual void				mdio_write(int phy_id, int reg_num, int val);

	virtual bool				atProbe();
	virtual void				atSwFree();

	//Hardware functions
    virtual SInt32				atSetupLink();
	virtual void				atGetAndUpdateLinkStatus();
private:
	//Private fields, w/o methods
	at_adapter					adapter_;

	IOMemoryMap					*hw_addr_;

	IOEthernetInterface			*netIface_;
	IONetworkStats				*netStats_;
	IOEthernetStats				*etherStats_;
	IOWorkLoop					*workLoop_;
	IOOutputQueue				*transmitQueue_;
	OSDictionary				*mediumDict;
	const IONetworkMedium		*mediumTable[MEDIUM_INDEX_COUNT];
	IOInterruptEventSource		*intSource_;
};

#endif
