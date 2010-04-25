/* AtherosL1cEthernet.h -- ATL1c driver implements
 *
 * Copyright (c) 2010 maolj <maolj@hotmail.com>.
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
 * Driver for Atheros(R) AR8131/AR8132/AR8152 PCI-E Ethernet Network Driver.
 *
 * This driver is heavily based on ATL1c Linux driver by Jie Yang <jie.yang@atheros.com>.
 */

#include "AtherosL1cEthernet.h"

#pragma mark -
#pragma mark - Various definitions & helper functions -
#pragma mark -


#define BASE				IOEthernetController
#define RELEASE(x)			do { if(x) { (x)->release(); (x) = 0; } } while(0)


OSDefineMetaClassAndStructors(AtherosL1cEthernet, IOEthernetController)

#pragma mark -
#pragma mark - IOService overrides -
#pragma mark -

bool AtherosL1cEthernet::init(OSDictionary *properties)
{
	DbgPrint("init()\n");
	if (!BASE::init(properties))
	{
		ErrPrint("Couldn't init BASE\n");
		return false;
	}

	memset(&adapter_, 0, sizeof(atl1c_adapter));
	adapter_.pdev = NULL;
	netIface_ = NULL;
	hw_addr_ = NULL;
	adapter_.hw.mmr_base = NULL;
	adapter_.hw.adapter = &adapter_;
	return true;
}

void AtherosL1cEthernet::free()
{
	DbgPrint("free()\n");

	if (intSource_ && workLoop_)
	{
		workLoop_->removeEventSource(intSource_);
	}


	RELEASE(intSource_);
	RELEASE(netIface_);
	RELEASE(adapter_.pdev);
	RELEASE(hw_addr_);
	BASE::free();
}

bool AtherosL1cEthernet::start(IOService *provider)
{
	DbgPrint("start()\n");

	atl1c_adapter *adapter=&adapter_;
	
	if (!BASE::start(provider))
	{
		ErrPrint("Couldn't start BASE\n");
		return false;
	}

	adapter->pdev = OSDynamicCast(IOPCIDevice, provider);

	if (!adapter->pdev)
	{
		ErrPrint("Unable to cast provider\n");
		return false;
	}

	adapter->pdev->retain();
	adapter->pdev->open(this);

	if (!atProbe())	//Fix false reporting int probe function
	{
		ErrPrint("Couldn't probe adapter\n");
		stop(provider);
		return false;
	}
	
	//Adding Mac OS X PHY's
	if (atGetNicType() == TYPE_GIGABIT){
		mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT_GIGABIT + 1);
		OSAddNetworkMedium(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO_GIGABIT);
	}else{ //TYPE_FAST
		mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT_FAST + 1);
		OSAddNetworkMedium(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO_FAST);
	}
	OSAddNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex, 10 * MBit, MEDIUM_INDEX_10HD);
	OSAddNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex, 10 * MBit, MEDIUM_INDEX_10FD);
	OSAddNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex, 100 * MBit, MEDIUM_INDEX_100HD);
	OSAddNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex, 100 * MBit, MEDIUM_INDEX_100FD);
	if (atGetNicType() == TYPE_GIGABIT){
		OSAddNetworkMedium(kIOMediumEthernet1000BaseTX | kIOMediumOptionHalfDuplex, 1000 * MBit, MEDIUM_INDEX_1000HD);
		OSAddNetworkMedium(kIOMediumEthernet1000BaseTX | kIOMediumOptionFullDuplex, 1000 * MBit, MEDIUM_INDEX_1000FD);
	}
	if (!publishMediumDictionary(mediumDict)) return false;



	workLoop_ = getWorkLoop();
	if (!workLoop_)
	{
		ErrPrint("workLoop is not exists\n");
		stop(provider);
		return false;
	}

	transmitQueue_ = getOutputQueue();
	if (!transmitQueue_)
	{
		ErrPrint("transmitQueue is not exists\n");
		stop(provider);
		return false;
	}

	//Looking for MSI interrupt index
	int msi_index = -1;
	int intr_index = 0, intr_type = 0;
	IOReturn intr_ret;
	while (true)
	{
		intr_ret = provider->getInterruptType(intr_index, &intr_type);
		if (intr_ret != kIOReturnSuccess) break;
		if (intr_type & kIOInterruptTypePCIMessaged) msi_index = intr_index;
		intr_index++;
	}

	if (msi_index != -1)
	{
		DbgPrint("MSI interrupt index %d\n", msi_index);
		intSource_ = IOInterruptEventSource::interruptEventSource(this,
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosL1cEthernet::atIntr),
					adapter->pdev, msi_index);
	}

	if (msi_index == -1 || intSource_ == NULL)
	{
		DbgPrint("MSI index was not found or MSI interrupt couldn't be enabled\n");
		intSource_ = IOInterruptEventSource::interruptEventSource(this,
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosL1cEthernet::atIntr),
					adapter->pdev);
	}

	//Adding interrupt to our workloop event sources
	if (!intSource_ || workLoop_->addEventSource(intSource_) != kIOReturnSuccess)
	{
		if (!intSource_) ErrPrint("Couldn't create interrupt source\n");
		else ErrPrint("Couldn't attach interrupt source\n");
		stop(provider);
		return false;
	}

	//Attaching dynamic link layer
	if (!this->attachInterface(reinterpret_cast<IONetworkInterface **>(&netIface_)), false)
	{
		DbgPrint("Failed to attach data link layer\n");
		return false;
	}

	intSource_->enable();
	
    /* allocate rx/tx dma buffer & descriptors */
	if (atl1c_setup_ring_resources(adapter))
	{
		ErrPrint("Couldn't allocate ring descriptors\n");
		adapter->pdev->close(this);
		return kIOReturnError;
	}
	
	netIface_->registerService();
	adapter->pdev->close(this);
	return true;
}

void AtherosL1cEthernet::stop(IOService *provider)
{
	DbgPrint("stop()\n");
	atl1c_adapter *adapter=&adapter_;
	
	if (intSource_) intSource_->disable();
	this->detachInterface(netIface_);
	
	atl1c_free_ring_resources(adapter);
	atSwFree();
	BASE::stop(provider);
}

bool AtherosL1cEthernet::OSAddNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
{
	DbgPrint("OSAddNetworkMedium()\n");
	
	IONetworkMedium *medium;

	medium = IONetworkMedium::medium( type, bps, 0, index );
	if (!medium)
	{
		DbgPrint("Couldn't allocate medium\n");
		return false;
	}
	if (!IONetworkMedium::addMedium(mediumDict, medium))
	{
		DbgPrint("Couldn't add medium\n");
		return false;
	}
	mediumTable[index] = medium;
	return true;
}

IOOutputQueue *AtherosL1cEthernet::createOutputQueue()
{
	DbgPrint("createOutputQueue()\n");
	//Sharing one event source with transmith/receive handles
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}


/**
 * at_intr - Interrupt Handler
 **/
void AtherosL1cEthernet::atIntr(OSObject *client, IOInterruptEventSource *src, int count)
{
    DbgPrint("atIntr()\n");
	
	atl1c_adapter *adapter = &adapter_;
    atl1c_hw *hw = &adapter->hw;
	s32 max_ints = AT_MAX_INT_WORK;

	u32 status;
	u32 reg_data;

	
	do {
		AT_READ_REG(hw, REG_ISR, &reg_data);
		status = reg_data & hw->intr_mask;
		
		if (status == 0) {
			break;
		}
		DEBUGOUT("atIntr() status = 0x%x!\n", status);
		
		/* link event */
		if (status & ISR_GPHY)
			atl1c_clear_phy_int(adapter);
		
		/* Ack ISR */
		AT_WRITE_REG(hw, REG_ISR, status | ISR_DIS_INT);
		
		/* check if PCIE PHY Link down */
		if (status & ISR_ERROR) {
			AT_ERR(
					"atl1c hardware error (status = 0x%x)\n",
					(status & ISR_ERROR));
			/* reset MAC  to-do*/
			//IOSleep(2);
			//atl1c_reset_mac(&adapter->hw);
			//return ;
			break;
		}
		if (status & ISR_RX_PKT) {

			for (int i = 0; i < adapter->num_rx_queues; i++)
				atl1c_clean_rx_irq(adapter, i );
		}
		if (status & ISR_TX_PKT)
			atl1c_clean_tx_irq(adapter, atl1c_trans_normal);
		
		if (status & ISR_OVER){
			AT_ERR(
					"TX/RX over flow (status = 0x%x)\n",
					(status & ISR_OVER));
			// for sleep issue
			// Let controller know availability of new Rx buffers.
			if (status & ISR_HW_RXF_OV){
				for (u32 i = 0; i < adapter->num_rx_queues; i++) {
					atl1c_alloc_rx_buffer(adapter, i);
				}
			}
		}
		
		/* link event */
		if (status & (ISR_GPHY | ISR_MANUAL)) {
			IOSleep(2);
			atGetAndUpdateLinkStatus();
			break;
		}
		
	} while (--max_ints > 0);
	/* re-enable Interrupt*/
	AT_WRITE_REG(&adapter->hw, REG_ISR, 0);

}


bool AtherosL1cEthernet::atl1c_clean_tx_irq(atl1c_adapter *adapter,atl1c_trans_queue type)
{
	DbgPrint("atl1c_clean_tx_irq()\n");
	
	atl1c_tpd_ring *tpd_ring = (atl1c_tpd_ring *)
	&adapter->tpd_ring[type];
	atl1c_buffer *buffer_info;
	u16 next_to_clean = atomic_read(&tpd_ring->next_to_clean);
	u16 hw_next_to_clean;
	u16 shift;
	u32 data;

	if (type == atl1c_trans_high)
		shift = MB_HTPD_CONS_IDX_SHIFT;
	else
		shift = MB_NTPD_CONS_IDX_SHIFT;
	
	AT_READ_REG(&adapter->hw, REG_MB_PRIO_CONS_IDX, &data);
	hw_next_to_clean = (data >> shift) & MB_PRIO_PROD_IDX_MASK; 
	
	DEBUGOUT("atl1c_clean_tx_irq() hw_next_to_clean=%d, next_to_clean=%d\n",
			  hw_next_to_clean, next_to_clean);
	
	while (next_to_clean != hw_next_to_clean) {
		buffer_info = &tpd_ring->buffer_info[next_to_clean];
		/*if (buffer_info->state == ATL1_BUFFER_BUSY) {
			// TO-DO clean
			buffer_info->dma = 0;
			buffer_info->state = ATL1_BUFFER_FREE;
		}*/
		if (++next_to_clean == tpd_ring->count)
			next_to_clean = 0;
		atomic_set(&tpd_ring->next_to_clean, next_to_clean);
		// tx statistics:
		netStats_->outputPackets++;
	}
	return true;
}


void AtherosL1cEthernet::atl1c_clean_rx_irq(struct atl1c_adapter *adapter, u8 que)
{
	DbgPrint("atl1c_clean_rx_irq()\n");
	
	mbuf_t skb = NULL;
	u16 rfd_num, rfd_index;
	u16 count = 0;
	u16 length;
	u32 packet_size;
	
	atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring[que];
	atl1c_rrd_ring *rrd_ring = &adapter->rrd_ring[que];

	atl1c_recv_ret_status *rrs;
	atl1c_buffer *buffer_info;
	
	DbgPrint("atl1c_clean_rx_irq()  que=%d,  rfd_ring->next_to_clean=%d,rrd_ring->next_to_clean=%d\n",
              que,  rfd_ring->next_to_clean ,rrd_ring->next_to_clean);
	
	while (1) {

		rrs = ATL1C_RRD_DESC(rrd_ring, rrd_ring->next_to_clean);
		if (RRS_RXD_IS_VALID(rrs->word3) ) {
			rfd_num = (rrs->word0 >> RRS_RX_RFD_CNT_SHIFT) &
			RRS_RX_RFD_CNT_MASK;
			if ((rfd_num ) != 1)
			/* TODO support mul rfd*/
				DEBUGOUT( "Multi rfd not support yet!\n");
			goto rrs_checked;
		} else {
			break;
		}
	rrs_checked:
		atl1c_clean_rrd(rrd_ring, rrs, rfd_num);
		if (rrs->word3 & (RRS_RX_ERR_SUM | RRS_802_3_LEN_ERR)) {
			atl1c_clean_rfd(rfd_ring, rrs, rfd_num);
			DEBUGOUT("wrong packet! rrs word3 is %x\n", rrs->word3);
			continue;
		}
		
		length = OSSwapLittleToHostInt16 ((rrs->word3 >> RRS_PKT_SIZE_SHIFT) &
							 RRS_PKT_SIZE_MASK);
		/* Good Receive */
		if ((rfd_num) == 1) {
			rfd_index = (rrs->word0 >> RRS_RX_RFD_INDEX_SHIFT) &
			RRS_RX_RFD_INDEX_MASK;
			buffer_info = &rfd_ring->buffer_info[rfd_index];
			packet_size = length - 4; // CRC
			// alloc new buffer
			skb = allocatePacket(packet_size + 2);
			if (NULL == skb) {
				DbgPrint("Memory squeeze, deferring packet.\n");
				break;
			}
			DEBUGOUT("pktsize=%d\n", packet_size);
			
			// copy packet to user buffer
			if (buffer_info->memDesc)
			{
				memcpy( mbuf_data(skb),buffer_info->memDesc->getBytesNoCopy(), packet_size);
			}
			
			//			printk(KERN_EMERG "skb addr is %x, len is %x\n", skb->data,
			//				length);
		} else {
			/* TODO */
			DEBUGOUT( "Multil rfd not support yet!\n");
			break;
		}
		atl1c_clean_rfd(rfd_ring, rrs, rfd_num);
		
		//TO-DO: Add network stack notification
		netStats_->inputPackets++;
		
		netIface_->inputPacket(skb, packet_size, IONetworkInterface::kInputOptionQueuePacket);
		netIface_->flushInputQueue();   
	
		count++;
	}
	if (count)
		atl1c_alloc_rx_buffer(adapter, que);	
}


#pragma mark -
#pragma mark - IOEthernetController overrides -
#pragma mark -

IOReturn AtherosL1cEthernet::enable(IONetworkInterface *netif)
{
	DbgPrint("enable()\n");
	
	u32 err;
	u32 i;

	atl1c_adapter *adapter=&adapter_;
	
	if (!adapter->pdev || (!adapter->pdev->isOpen () && (adapter->pdev->open(this)) == 0)) {
	    DbgPrint( "failed to open PCI device.\n");
        return kIOReturnError;
    }


	atl1c_init_ring_ptrs(adapter);
	err = atl1c_alloc_tx_buffers(adapter, atl1c_trans_normal);
	if (err) {
		DbgPrint( "failed to atl1c_alloc_tx_buffers(atl1c_trans_normal).\n");
		return kIOReturnError;
	}
	err = atl1c_alloc_tx_buffers(adapter, atl1c_trans_high);
	if (err) {
		DbgPrint( "failed to atl1c_alloc_tx_buffers(atl1c_trans_high).\n");
		return kIOReturnError;
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		err = atl1c_alloc_rx_buffer(adapter, i);
		if (err) {
			DbgPrint( "failed to atl1c_alloc_rx_buffer.\n");
			return kIOReturnError;
		}
	}
	if (atl1c_configure(adapter)) {
		DbgPrint( "failed to atl1c_configure.\n");
		return kIOReturnError;
	}

	//PHY medium selection
	const IONetworkMedium *medium = getSelectedMedium();
	if (!medium)
	{
		DbgPrint("Selected medium is NULL, forcing to autonegotiation\n");
		
		if (atGetNicType() == TYPE_GIGABIT){
			medium = mediumTable[MEDIUM_INDEX_AUTO_GIGABIT];
		}else{ //TYPE_FAST
			medium = mediumTable[MEDIUM_INDEX_AUTO_FAST];
		}
	}
	else
	{
		DbgPrint("Selected medium index %d\n", (int)medium->getIndex());
	}


	selectMedium(medium);

	transmitQueue_->setCapacity(kTransmitQueueCapacity);
	transmitQueue_->start();
	
	// Enable interrupts. 
	atl1c_irq_enable(adapter);
	//atGetAndUpdateLinkStatus();
	
	// set FPGA vesion
	if (adapter->hw.ctrl_flags & ATL1C_FPGA_VERSION) {
		u32 phy_data;
		
		AT_READ_REG(&adapter->hw, 0x1414, &phy_data);
		phy_data |= 0x10000000;
		AT_WRITE_REG(&adapter->hw, 0x1414, phy_data);
	}
	return kIOReturnSuccess;
}

IOReturn AtherosL1cEthernet::disable(IONetworkInterface *netif)
{
	DbgPrint("disable()\n");
	
	atl1c_adapter *adapter=&adapter_;
	
	setLinkStatus(kIONetworkLinkValid, 0);
	
	transmitQueue_->flush();
	transmitQueue_->stop();
	transmitQueue_->setCapacity(0);

	atl1c_irq_disable(adapter);

	/* reset MAC to disable all RX/TX */
	atl1c_reset_mac(&adapter->hw);
	IOSleep(1);
	
	adapter->link_speed = SPEED_0;
	adapter->link_duplex = -1 ;
	atl1c_clean_tx_ring(adapter, atl1c_trans_normal);
	atl1c_clean_tx_ring(adapter, atl1c_trans_high);
	atl1c_clean_rx_ring(adapter);
	
	if (adapter->pdev  && adapter->pdev->isOpen () ) adapter->pdev->close(this);
	return kIOReturnSuccess;
}

bool AtherosL1cEthernet::configureInterface(IONetworkInterface *netif)
{
	DbgPrint("configureInterface()\n");
	
	IONetworkData *data;

    if (!BASE::configureInterface(netif)) return false;

    //Get the generic network statistics structure.
    data = netif->getParameter(kIONetworkStatsKey);
    if (!data || !(netStats_ = (IONetworkStats *)data->getBuffer()))
    {
        return false;
    }

    //Get the Ethernet statistics structure.
    data = netif->getParameter(kIOEthernetStatsKey);
    if (!data || !(etherStats_ = (IOEthernetStats *)data->getBuffer()))
    {
        return false;
    }

    return true;
}

const OSString *AtherosL1cEthernet::newVendorString() const
{
	return OSString::withCString("Atheros");
}

const OSString *AtherosL1cEthernet::newModelString() const
{
	return OSString::withCString("L1c LAN");
}

IOReturn AtherosL1cEthernet::selectMedium(const IONetworkMedium *medium)
{
	DbgPrint("selectMedium()\n");

	if (medium)
	{
		if (atGetNicType() == TYPE_GIGABIT){
			switch(medium->getIndex())
			{
				case MEDIUM_INDEX_AUTO_GIGABIT:
					DbgPrint("Selected medium is autoselect\n");
					adapter_.link_speed = SPEED_1000;
					adapter_.link_duplex = FULL_DUPLEX;
					adapter_.hw.media_type = MEDIA_TYPE_AUTO_SENSOR;
					break;
				case MEDIUM_INDEX_10HD:
					DbgPrint("Selected medium is 10HD\n");
					adapter_.link_speed = SPEED_10;
					adapter_.link_duplex = HALF_DUPLEX;	
					adapter_.hw.media_type = MEDIA_TYPE_10M_HALF;
					break;
				case MEDIUM_INDEX_10FD:
					DbgPrint("Selected medium is 10FD\n");
					adapter_.link_speed = SPEED_10;
					adapter_.link_duplex = FULL_DUPLEX;	
					adapter_.hw.media_type = MEDIA_TYPE_10M_FULL;
					break;
				case MEDIUM_INDEX_100HD:
					DbgPrint("Selected medium is 100HD\n");
					adapter_.link_speed = SPEED_100;
					adapter_.link_duplex = HALF_DUPLEX;		
					adapter_.hw.media_type = MEDIA_TYPE_100M_HALF;
					break;
				case MEDIUM_INDEX_100FD:
					DbgPrint("Selected medium is 100FD\n");
					adapter_.link_speed = SPEED_100;
					adapter_.link_duplex = FULL_DUPLEX;		
					adapter_.hw.media_type = MEDIA_TYPE_100M_FULL;
					break;
				case MEDIUM_INDEX_1000HD:
					DbgPrint("Selected medium is 1000HD\n");
					adapter_.link_speed = SPEED_1000;
					adapter_.link_duplex = HALF_DUPLEX;	
					adapter_.hw.media_type = MEDIA_TYPE_1000M_FULL;
					break;
				case MEDIUM_INDEX_1000FD:
					DbgPrint("Selected medium is 1000FD\n");
					adapter_.link_speed = SPEED_1000;
					adapter_.link_duplex = FULL_DUPLEX;		
					adapter_.hw.media_type = MEDIA_TYPE_1000M_FULL;
					break;
			}	
		}else{ //TYPE_FAST
			switch(medium->getIndex())
			{
				case MEDIUM_INDEX_AUTO_FAST:
					DbgPrint("Selected medium is autoselect\n");
					adapter_.link_speed = SPEED_100;
					adapter_.link_duplex = FULL_DUPLEX;
					adapter_.hw.media_type = MEDIA_TYPE_AUTO_SENSOR;
					break;
				case MEDIUM_INDEX_10HD:
					DbgPrint("Selected medium is 10HD\n");
					adapter_.link_speed = SPEED_10;
					adapter_.link_duplex = HALF_DUPLEX;	
					adapter_.hw.media_type = MEDIA_TYPE_10M_HALF;
					break;
				case MEDIUM_INDEX_10FD:
					DbgPrint("Selected medium is 10FD\n");
					adapter_.link_speed = SPEED_10;
					adapter_.link_duplex = FULL_DUPLEX;	
					adapter_.hw.media_type = MEDIA_TYPE_10M_FULL;
					break;
				case MEDIUM_INDEX_100HD:
					DbgPrint("Selected medium is 100HD\n");
					adapter_.link_speed = SPEED_100;
					adapter_.link_duplex = HALF_DUPLEX;		
					adapter_.hw.media_type = MEDIA_TYPE_100M_HALF;
					break;
				case MEDIUM_INDEX_100FD:
					DbgPrint("Selected medium is 100FD\n");
					adapter_.link_speed = SPEED_100;
					adapter_.link_duplex = FULL_DUPLEX;		
					adapter_.hw.media_type = MEDIA_TYPE_100M_FULL;
					break;
			}	
		}
		
		atSetupLink();

		setCurrentMedium(medium);
	}
	else
	{
		DbgPrint("Selected medium is NULL\n");
		return kIOReturnError;
	}

	//Refresh link status
	if (atGetNicType() == TYPE_GIGABIT){
		IOSleep(100*((medium->getIndex() == MEDIUM_INDEX_AUTO_GIGABIT)? PHY_AUTO_NEG_TIME:PHY_FORCE_TIME));
	}else{ //TYPE_FAST
		IOSleep(100*((medium->getIndex() == MEDIUM_INDEX_AUTO_FAST)? PHY_AUTO_NEG_TIME:PHY_FORCE_TIME));
	}
	
	//atGetAndUpdateLinkStatus();

	return kIOReturnSuccess;
}

IOReturn AtherosL1cEthernet::getHardwareAddress(IOEthernetAddress *addr)
{
	DbgPrint("getHardwareAddress()\n");
	
	IOSleep(1000);

	if (is_valid_ether_addr(adapter_.hw.mac_addr))
	{
		memcpy(addr->bytes, adapter_.hw.mac_addr, ETH_ALEN);
		return kIOReturnSuccess;
	}

	if (atl1c_get_permanent_address(&adapter_.hw) == 0)
	{
		if (is_valid_ether_addr(adapter_.hw.perm_mac_addr))
		{
			memcpy(adapter_.hw.mac_addr, adapter_.hw.perm_mac_addr, ETH_ALEN);
			memcpy(addr->bytes, adapter_.hw.mac_addr, ETH_ALEN);
		}
		else
		{
			ErrPrint("Invalid mac address\n");
			return kIOReturnUnsupported;
		}
		return kIOReturnSuccess;
	}
	else
	{
		DbgPrint("Couldn't get device mac address\n");
		memcpy(adapter_.hw.mac_addr, addr->bytes, ETH_ALEN);
		//return kIOReturnUnsupported;
		return kIOReturnSuccess;
	}
}

IOReturn AtherosL1cEthernet::setHardwareAddress(const IOEthernetAddress *addr)
{
    DbgPrint("setHardwareAddress()\n");
	memcpy(adapter_.hw.mac_addr, addr->bytes, ETH_ALEN);
	atl1c_hw_set_mac_addr(&adapter_.hw);
	return kIOReturnSuccess;
}

void AtherosL1cEthernet::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
	DbgPrint("getPacketBufferConstraints()\n");
	constraints->alignStart = kIOPacketBufferAlign4;
	constraints->alignLength = kIOPacketBufferAlign4;
}


UInt32 AtherosL1cEthernet::outputPacket(mbuf_t m, void *prm)
{
	DbgPrint("outputPacket()\n");
	
	u32 buf_len;
	atl1c_adapter *adapter=&adapter_;
	
	atl1c_tpd_desc *use_tpd;
	atl1c_buffer *buffer_info = NULL;
	u16 tpd_req = 1;
	atl1c_trans_queue type = atl1c_trans_normal;
	struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
	
	use_tpd = atl1c_get_tpd(adapter, type);
	buffer_info = atl1c_get_tx_buffer(adapter, use_tpd);
	
	if (atl1c_tpd_avail(adapter, type) < tpd_req)
	{
		/* no enough descriptor, just stop queue */
		DbgPrint("no enough resource!!\n");
		freePacket(m);
		return kIOReturnOutputDropped;
	}
	 
	if (!buffer_info->memDesc)
	{
		DbgPrint("Tx buffer is null!!\n");
		freePacket(m);
		return kIOReturnOutputDropped;
	}
	 
	if (mbuf_pkthdr_len(m) <= MAX_TX_BUF_LEN) buf_len = mbuf_pkthdr_len(m);
	else
	{
		DbgPrint("Tx Packet size is too big, droping\n");
		freePacket(m);
		return kIOReturnOutputDropped;
	}
	 
	DbgPrint("outputPacket() length %d next_to_use %d\n", buf_len, tpd_ring->next_to_use);
	
	u8 *data_ptr = (u8 *)buffer_info->memDesc->getBytesNoCopy();
	u32 pkt_snd_len = 0;
	mbuf_t cur_buf = m;
	do
	{
	if (mbuf_data(cur_buf))	bcopy(mbuf_data(cur_buf), data_ptr, mbuf_len(cur_buf));
		data_ptr += mbuf_len(cur_buf);
		pkt_snd_len += mbuf_len(cur_buf);
	}
	while(((cur_buf = mbuf_next(cur_buf)) != NULL) && ((pkt_snd_len + mbuf_len(cur_buf)) <= buf_len));
	 
	buffer_info->length = pkt_snd_len;
	use_tpd->buffer_addr = OSSwapHostToLittleInt64(buffer_info->dma);
	use_tpd->buffer_len = OSSwapHostToLittleInt64(buffer_info->length);
	/* The last tpd */
	use_tpd->word1 |= 1 << TPD_EOP_SHIFT;
	
	u32 prod_data;

	AT_READ_REG(&adapter->hw, REG_MB_PRIO_PROD_IDX, &prod_data);
	switch (type) {
		case atl1c_trans_high:
			prod_data &= 0xFFFF0000;
			prod_data |= tpd_ring->next_to_use & 0xFFFF;
			//AT_WRITE_REGW(&adapter->hw, REG_MB_PRIO_PROD_IDX,
			//	tpd_ring->next_to_use & MB_PRIO_PROD_IDX_MASK);
			//	DbgPrint("high produce index is %x \n",
			//		(unsigned int)(tpd_ring->next_to_use & MB_PRIO_PROD_IDX_MASK));
			break;
		case atl1c_trans_normal:
			prod_data &= 0x0000FFFF;
			prod_data |= (tpd_ring->next_to_use & 0xFFFF) << 16;
			
			//AT_WRITE_REGW(&adapter->hw, (REG_MB_PRIO_PROD_IDX + 2),
			//tpd_ring->next_to_use & MB_PRIO_PROD_IDX_MASK);
			//	DbgPrint("normal produce index is %x \n",
			//		(unsigned int)(tpd_ring->next_to_use & 0xffff));
			break;
		default:
			break;
	}
	
	AT_WRITE_REG(&adapter->hw, REG_MB_PRIO_PROD_IDX, prod_data);

	OSSynchronizeIO();
	
	freePacket(m);
	
	return kIOReturnOutputSuccess;

}

IOReturn AtherosL1cEthernet::registerWithPolicyMaker(IOService *policyMaker)
{
	DbgPrint("registerWithPolicyMaker()\n");

	//Grabed from ViaRhine
    enum
	{
        kPowerStateOff = 0,
        kPowerStateOn,
        kPowerStateCount
    };

    static IOPMPowerState powerStateArray[ kPowerStateCount ] =
    {
        { 1,0,0,0,0,0,0,0,0,0,0,0 },
        { 1,IOPMDeviceUsable,IOPMPowerOn,IOPMPowerOn,0,0,0,0,0,0,0,0 }
    };

    IOReturn ret;

    ret = policyMaker->registerPowerDriver( this, powerStateArray,
                                            kPowerStateCount );

    return ret;
}

IOReturn AtherosL1cEthernet::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
	DbgPrint("setPowerState()\n");
	//TO-DO: Add power state support
	return IOPMAckImplied;
}

/**
 *Promiscuous mode set
 *
 * The set_multi entry point is called whenever the multicast address
 * list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper multicast,
 * promiscuous mode, and all-multi behavior.
 **/
IOReturn AtherosL1cEthernet::setPromiscuousMode(bool enabled)
{
	DbgPrint("setPromiscuousMode(), %d\n", enabled);
	
    atl1c_hw *hw = &adapter_.hw;
	u32 mac_ctrl_data = 0;
	
	/* Check for Promiscuous and All Multicast modes */
	AT_READ_REG(hw, REG_MAC_CTRL, &mac_ctrl_data);

	if (enabled)
	{
		mac_ctrl_data |= MAC_CTRL_PROMIS_EN;
	}
	else
	{
		mac_ctrl_data &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
	}

	AT_WRITE_REG(hw, REG_MAC_CTRL, mac_ctrl_data);
	
	/* clear the old settings from the multicast hash table */
	AT_WRITE_REG(hw, REG_RX_HASH_TABLE, 0);
	AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);
	
	return kIOReturnSuccess;
}
/**
 *Multicast  mode set
 *
 * The set_multi entry point is called whenever the multicast address
 * list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper multicast,
 * promiscuous mode, and all-multi behavior.
 **/
IOReturn AtherosL1cEthernet::setMulticastMode(bool enabled)
{
	DbgPrint("setMulticastMode(), %d\n", enabled);
	
    atl1c_hw *hw = &adapter_.hw;
	u32 mac_ctrl_data = 0;
	
	/* Check for Promiscuous and All Multicast modes */
	AT_READ_REG(hw, REG_MAC_CTRL, &mac_ctrl_data);
	
	if (enabled)
	{
		mac_ctrl_data |= MAC_CTRL_MC_ALL_EN;
		mac_ctrl_data &= ~MAC_CTRL_PROMIS_EN;
	}
	else
	{
		mac_ctrl_data &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
	}
	
	AT_WRITE_REG(hw, REG_MAC_CTRL, mac_ctrl_data);
	
	/* clear the old settings from the multicast hash table */
	AT_WRITE_REG(hw, REG_RX_HASH_TABLE, 0);
	AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);

	return kIOReturnSuccess;
}

IOReturn AtherosL1cEthernet::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
	DbgPrint("setMulticastList()\n");

    atl1c_hw *hw = &adapter_.hw;
	u32 hash_value;
	for(int i = 0; i < count; i++, addrs++)
	{
		hash_value = atl1c_hash_mc_addr(hw,addrs->bytes);
		atl1c_hash_set(hw,hash_value);
	}

	return kIOReturnSuccess;
}

#pragma mark -
#pragma mark - Atheros L1c Adapter
#pragma mark -


int AtherosL1cEthernet::mdio_read(int phy_id, int reg_num)
{
	u16 result;
	atl1c_read_phy_reg(&adapter_.hw, reg_num & MDIO_REG_ADDR_MASK, &result);

	return result;
}

void AtherosL1cEthernet::mdio_write(int phy_id, int reg_num, int val)
{
	atl1c_write_phy_reg(&adapter_.hw, reg_num & MDIO_REG_ADDR_MASK, val);
}

/**
 * at_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in at_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * at_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
bool AtherosL1cEthernet::atProbe()
{	
	DbgPrint("atProbe()\n");
	
    atl1c_adapter *adapter=&adapter_;
	s32 err = 0;
	
	IOPCIDevice	*pdev = adapter_.pdev;
	pdev->setBusMasterEnable(true);
	pdev->setMemoryEnable(true);
	pdev->setIOEnable(false);
	vendorId_ = pdev->configRead16(kIOPCIConfigVendorID);
	deviceId_ = pdev->configRead16(kIOPCIConfigDeviceID);

	DbgPrint("Vendor ID %x, device ID %x\n", vendorId_, deviceId_);
	DbgPrint("MMR0 address %x\n", (u32)pdev->configRead32(kIOPCIConfigBaseAddress0));

	pdev->enablePCIPowerManagement();

	hw_addr_ = pdev->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
	if (hw_addr_ == NULL)
	{
		ErrPrint("Couldn't map io regs\n");
		return false;
	}
	else
	{
		DbgPrint("Memory mapped at bus address %x, virtual address %x, length %d\n", (u32)hw_addr_->getPhysicalAddress(),
					(u32)hw_addr_->getVirtualAddress(), (s32)hw_addr_->getLength());
	}
	
	hw_addr_->retain();

	adapter->hw.mmr_base = reinterpret_cast<char *>(hw_addr_->getVirtualAddress());
 
	
	DbgPrint("REG_VPD_CAP = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_VPD_CAP));
	DbgPrint("REG_PCIE_CAP_LIST = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_PCIE_CAP_LIST));
	DbgPrint("REG_MASTER_CTRL = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_MASTER_CTRL));
	
	
	/* get user settings */
	atl1c_check_options(adapter);
	
	/* setup the private structure */
	err = atl1c_sw_init(adapter);
	if (err) {
		ErrPrint("net device private data init failed\n");
		return false;	
	}	
	/* FIXME: not sure if should exist,
	 * pci_enable_device have already do this
	 */
	atl1c_reset_pcie(pdev,&adapter->hw, ATL1C_PCIE_L0S_L1_DISABLE |
					 ATL1C_PCIE_PHY_RESET);
	
	
	/* Init GPHY as early as possible due to power saving issue  */
	atl1c_phy_reset(&adapter->hw);
	
	err = atl1c_reset_mac(&adapter->hw);
	if (err) {
		return false;	
	}
	
	/* TODO: add power saving coding ?? */
	/* reset the controller to
	 * put the device in a known good starting state */
	err = atl1c_phy_init(&adapter->hw);
	if (err) {
		return false;	
	}

    /* copy the MAC address out of the EEPROM */
    atl1c_read_mac_addr(&adapter->hw);

	return true;
}


void AtherosL1cEthernet::atSwFree()
{
	DbgPrint("atSwFree()\n");
	
/*	if (adapter_.mdio_lock)
	{
		IOSimpleLockFree(adapter_.mdio_lock);
		adapter_.mdio_lock = NULL;
	}

	if (adapter_.tx_lock)
	{
		IOSimpleLockFree(adapter_.tx_lock);
		adapter_.tx_lock = NULL;
	}*/
	
}

#pragma mark -
#pragma mark - Atheros L1c Hw methods
#pragma mark -

void AtherosL1cEthernet::atGetAndUpdateLinkStatus()
{
    DbgPrint("atGetAndUpdateLinkStatus()\n");
	
    atl1c_adapter *adapter = &adapter_;
	atl1c_hw *hw = &adapter->hw;
	
	u16 speed, duplex;
	u32 currentMediumIndex;
	if (atGetNicType() == TYPE_GIGABIT){
		currentMediumIndex = MEDIUM_INDEX_AUTO_GIGABIT;
	}else{ //TYPE_FAST
		currentMediumIndex = MEDIUM_INDEX_AUTO_FAST;
	}
	
	if(atl1c_get_speed_and_duplex(&adapter_.hw, &speed, &duplex) == 0)
	{
		DbgPrint("Link is active, speed %d, duplex %d\n", speed, duplex);


		if(speed == SPEED_10 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10FD;
		if(speed == SPEED_100 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100FD;
		if(speed == SPEED_1000 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_1000FD;
		if(speed == SPEED_10 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10HD;
		if(speed == SPEED_100 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100HD;
		if(speed == SPEED_1000 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_1000HD;	

		setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, mediumTable[currentMediumIndex], speed * MBit, NULL);

		adapter->link_speed  = speed;
		adapter->link_duplex = duplex;

		atl1c_set_aspm(hw, true);
		atl1c_enable_tx_ctrl(hw);
		atl1c_enable_rx_ctrl(hw);
		atl1c_setup_mac_ctrl(adapter);
	} else
	{
		DbgPrint("Link is down\n");
		if (atl1c_stop_mac(hw) != 0)
			DbgPrint("stop mac failed\n");
		atl1c_set_aspm(hw, false);
		atl1c_phy_reset(hw);
		atl1c_phy_init(hw);
		setLinkStatus(kIONetworkLinkValid, NULL, 0, NULL);
	}
}


/*
 * Configures link settings.
 * Assumes the hardware has previously been reset and the
 * transmitter and receiver are not enabled.
 */
s32 AtherosL1cEthernet::atSetupLink()
{
	DbgPrint("atSetupLink()\n");
	
	s32 ret_val;

	/*
	 * Options:
	 *  PHY will advertise value(s) parsed from
	 *  autoneg_advertised and fc
	 *  no matter what autoneg is , We will not wait link result.
	 */
	atl1c_hw *hw = &adapter_.hw;
	
	ret_val = atl1c_restart_autoneg(hw);
	if (ret_val) 
	{
		ErrPrint("phy autoneg failed\n");
		return ret_val;
	}

	//hw->phy_configured = true;
	return ret_val;
}
/*
 *  Driver	Model-name	vendor:device	Type
 *  atl1c 	AR8131		1969:1063	Gigabit Ethernet
 *  atl1c	AR8132		1969:1062	Fast Ethernet
 *  atl1c	AR8151(v1.0)	1969:1073	Gigabit Ethernet
 *  atl1c	AR8152(v1.1)	1969:2060	Fast Ethernet
 *  This device has no hardware available yet so it goes untested,
 *  but it should work:
 *  atl1c	AR8152(v2.0)	1969:2062	Fast Ethernet
*/
u32 AtherosL1cEthernet::atGetNicType()
{
	
	u32 ret_val = TYPE_GIGABIT;
	
	switch (deviceId_) {
		case DEV_ID_ATL1C_2_0:
			ret_val = TYPE_GIGABIT;
			break;
		case DEV_ID_ATL2C_2_0:
			ret_val = TYPE_FAST;
			break;
		case DEV_ID_ATL2C_B:
			 ret_val = TYPE_FAST;
             break;
		case DEV_ID_ATL2C_B_2:
			ret_val = TYPE_FAST;
			break;	
		case DEV_ID_ATL1D:
			ret_val = TYPE_GIGABIT;
			break;
		default:
			break;
	}
	
	return ret_val;
}

