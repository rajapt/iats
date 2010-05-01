/* AtherosL2Ethernet.cpp -- atl2 driver functions
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

#include "AtherosL2Ethernet.h"

#pragma mark -
#pragma mark - Various definitions & helper functions -
#pragma mark -


#define BASE				IOEthernetController
#define RELEASE(x)			do { if(x) { (x)->release(); (x) = 0; } } while(0)

OSDefineMetaClassAndStructors(AtherosL2Ethernet, IOEthernetController)

#pragma mark -
#pragma mark - IOService overrides -
#pragma mark -

bool AtherosL2Ethernet::init(OSDictionary *properties)
{
	DbgPrint("init()\n");
	if (!BASE::init(properties))
	{
		ErrPrint("Couldn't init BASE\n");
		return false;
	}

	memset(&adapter_, 0, sizeof(at_adapter));
	adapter_.pdev = NULL;
	netIface_ = NULL;
	hw_addr_ = NULL;
	adapter_.pci_using_64 = false;
	adapter_.hw.mmr_base = NULL;

	return true;
}

void AtherosL2Ethernet::free()
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

bool AtherosL2Ethernet::start(IOService *provider)
{
	DbgPrint("start()\n");
	
	at_adapter *adapter=&adapter_;
		
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

	//Adding Mac OS X PHY's
	mediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT + 1);

	OSAddNetworkMedium(kIOMediumEthernetAuto, 0, MEDIUM_INDEX_AUTO);
	OSAddNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex, 10 * MBit, MEDIUM_INDEX_10HD);
	OSAddNetworkMedium(kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex, 10 * MBit, MEDIUM_INDEX_10FD);
	OSAddNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex, 100 * MBit, MEDIUM_INDEX_100HD);
	OSAddNetworkMedium(kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex, 100 * MBit, MEDIUM_INDEX_100FD);

	if (!publishMediumDictionary(mediumDict)) return false;

	if (!atProbe())	//Fix false reporting int probe function
	{
		ErrPrint("Couldn't probe adapter\n");
		stop(provider);
		return false;
	}
	
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
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosL2Ethernet::atIntr),
					adapter->pdev, msi_index);
	}

	if (msi_index == -1 || intSource_ == NULL)
	{
		DbgPrint("MSI index was not found or MSI interrupt couldn't be enabled\n");
		intSource_ = IOInterruptEventSource::interruptEventSource(this,
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosL2Ethernet::atIntr),
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
	if (at_setup_ring_resources(adapter))
	{
		ErrPrint("Couldn't allocate ring descriptors\n");
		adapter->pdev->close(this);
		return kIOReturnError;
	}

	netIface_->registerService();
	adapter->pdev->close(this);
	return true;
}

void AtherosL2Ethernet::stop(IOService *provider)
{
	DbgPrint("stop()\n");
	if (intSource_) intSource_->disable();
	this->detachInterface(netIface_);
	
	at_free_ring_resources(&adapter_);
	atSwFree();
	BASE::stop(provider);
}

bool AtherosL2Ethernet::OSAddNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
{
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

IOOutputQueue *AtherosL2Ethernet::createOutputQueue()
{
	DbgPrint("createOutputQueue()\n");
	//Sharing one event source with transmith/receive handles
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

/**
 * at_intr - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 * @pt_regs: CPU registers structure
 **/
void AtherosL2Ethernet::atIntr(OSObject *client, IOInterruptEventSource *src, int count)
{
    at_adapter *adapter = &adapter_;
	at_hw *hw = &adapter->hw;
    u32 status;
	
    
    status = AT_READ_REG(hw, REG_ISR);
    if (0 == status)    
        return ;
       
    // link event
    if (status&ISR_PHY) {
        at_clear_phy_int(adapter);
    }
    
    // clear ISR status, and Enable CMB DMA/Disable Interrupt
    AT_WRITE_REG(hw, REG_ISR, status|ISR_DIS_INT);

    
    // check if PCIE PHY Link down
   if (status&ISR_PHY_LINKDOWN) {
        DEBUGOUT1("pcie phy linkdown %x\n", status);
        // reset MAC
        AT_WRITE_REG(hw, REG_ISR, 0);
        AT_WRITE_REG(hw, REG_IMR, 0);
        AT_WRITE_FLUSH(hw);
        at_reset_hw(adapter);
        return ; 

    }

    // check if DMA read/write error ?
    if (status&(ISR_DMAR_TO_RST|ISR_DMAW_TO_RST)) 
    {
        DEBUGOUT1("PCIE DMA RW error (status = 0x%x) !\n", status);
        //AT_WRITE_REG(&adapter->hw, REG_MASTER_CTRL, MASTER_CTRL_SOFT_RST);
        AT_WRITE_REG(hw, REG_ISR, 0);
        AT_WRITE_REG(hw, REG_IMR, 0);
        AT_WRITE_FLUSH(hw);
        at_reset_hw(adapter);
        return ; 
    }
    
    // link event
    if (status&(ISR_PHY|ISR_MANUAL))
    {
        IOSleep(2);
        atGetAndUpdateLinkStatus();
    }
    
    
    // transmit event
    if ( status&ISR_TX_EVENT ) {
        at_intr_tx(adapter);
    }
    
    // rx exception
    if ( status& ISR_RX_EVENT ) {
        at_intr_rx(adapter);
    }
    
    // re-enable Interrupt
    AT_WRITE_REG(&adapter->hw, REG_ISR, 0);
}


/**
 * Tx Interrupt Handler
 **/
void AtherosL2Ethernet::at_intr_tx(at_adapter *adapter)
{
    u32 txd_read_ptr;
    u32 txs_write_ptr;
    tx_pkt_status_t* txs;
    tx_pkt_header_t* txph;
    int free_hole = 0;  
    
//    DEBUGFUNC("at_intr_tx()\n"); 

    do {
        txs_write_ptr = (u32) atomic_read(&adapter->txs_write_ptr);
        txs = adapter->txs_ring + txs_write_ptr;
        if (!txs->update) 
            break; // tx stop here
            
        free_hole = 1;
        txs->update = 0;
       	
        if (++txs_write_ptr == adapter->txs_ring_size)
            txs_write_ptr = 0;
        atomic_set(&adapter->txs_write_ptr, (int)txs_write_ptr);
                
        txd_read_ptr = (u32) atomic_read(&adapter->txd_read_ptr);
        txph = (tx_pkt_header_t*) 
                    (((u8*)adapter->txd_ring) + txd_read_ptr);
        
        if ( txph->pkt_size != txs->pkt_size) {
		
            tx_pkt_status_t* old_txs = txs;
            DbgPrint("txs packet size do not coinsist with txd"
		    " txd_:0x%08x, txs_:0x%08x!\n",
		    *(u32*)txph, *(u32*)txs);
	    	DbgPrint(  "txd read ptr: 0x%x\n",
			    txd_read_ptr);
	    	txs = adapter->txs_ring + txs_write_ptr;
            DbgPrint( "txs-behind:0x%08x\n",
			    *(u32*)txs);
            if (txs_write_ptr < 2) {
		    txs = adapter->txs_ring + (adapter->txs_ring_size+
				               txs_write_ptr - 2);
	    } else {
		    txs = adapter->txs_ring + (txs_write_ptr - 2);
	    }
	    DbgPrint("txs-before:0x%08x\n",
			    *(u32*)txs);
	    txs = old_txs;
        }
        
        txd_read_ptr += (((u32)(txph->pkt_size)+7)& ~3);//4for TPH
        if (txd_read_ptr >= adapter->txd_ring_size)
            txd_read_ptr -= adapter->txd_ring_size;
        
        atomic_set(&adapter->txd_read_ptr, (int)txd_read_ptr);
        
        // tx statistics:
        if (txs->ok)            netStats_->outputPackets++;
        else                    netStats_->outputErrors++;
        if (txs->defer)         netStats_->collisions++;
	} while (1);


}
/**
 * Rx Interrupt Handler
 **/
void AtherosL2Ethernet::at_intr_rx(at_adapter *adapter)
{
	DbgPrint("at_intr_rx()\n");
	
	rx_desc_t* rxd;
	mbuf_t skb = NULL;
//DbgPrint("at_intr_rx() begin rxd_write_ptr=%d\n",adapter->rxd_write_ptr);


	do {
		rxd = adapter->rxd_ring+adapter->rxd_write_ptr;
		if (!rxd->status.update)
			break; // end of tx
		// clear this flag at once
		rxd->status.update = 0;

		if (rxd->status.ok && rxd->status.pkt_size >= 60) {
			int rx_size = (int)(rxd->status.pkt_size - 4); 
//DbgPrint("at_intr_rx()  begin receive data  , pkt_size %d\n",rxd->status.pkt_size);

			// alloc new buffer
			skb = allocatePacket(rx_size + 2);
			if (NULL == skb) {
				DbgPrint("at_intr_rx() Allocate n_skb failed!\n");
				break;
			}
			
			memcpy(mbuf_data(skb), rxd->packet, rx_size); 

			//TO-DO: Add network stack notification
			
			netIface_->inputPacket(skb, rx_size, IONetworkInterface::kInputOptionQueuePacket);
			netIface_->flushInputQueue();

			// rx statistics:
			netStats_->inputPackets++;

		} else { 
            
            netStats_->inputErrors++; 
            
        }
		
		// advance write ptr
		if (++adapter->rxd_write_ptr == adapter->rxd_ring_size)
			adapter->rxd_write_ptr = 0;
	} while (1);


	// update mailbox ?
	adapter->rxd_read_ptr = adapter->rxd_write_ptr; 
	AT_WRITE_REGW(&adapter->hw, REG_MB_RXD_RD_IDX, adapter->rxd_read_ptr);
}

#pragma mark -
#pragma mark - IOEthernetController overrides -
#pragma mark -

IOReturn AtherosL2Ethernet::enable(IONetworkInterface *netif)
{
	DbgPrint("enable()\n");
	
	u32 val;
	at_adapter *adapter=&adapter_;
	
	if (!adapter->pdev || (!adapter->pdev->isOpen () && (adapter->pdev->open(this)) == 0)) {
	    DbgPrint( "failed to open PCI device.\n");
        return kIOReturnError;
    }
  
	// hardware init
	if (at_init_hw(&adapter->hw))
	{
		ErrPrint("Couldn't init hw\n");
		//stop(provider);
		return kIOReturnError;
	}
	// hardware has been reset, we need to reload some things 
	init_ring_ptrs(adapter);

	at_configure(adapter);
	

	//PHY medium selection
	const IONetworkMedium *medium = getSelectedMedium();
	if (!medium)
	{
		DbgPrint("Selected medium is NULL, forcing to autonegotiation\n");
		medium = mediumTable[MEDIUM_INDEX_AUTO];
	}
	else
	{
		DbgPrint("Selected medium index %d\n", medium->getIndex());
	}


	selectMedium(medium);

	transmitQueue_->setCapacity(kTransmitQueueCapacity);
	transmitQueue_->start();
	
	// Enable interrupts. 
    val = AT_READ_REG(&adapter->hw, REG_MASTER_CTRL);
    AT_WRITE_REG(&adapter->hw, REG_MASTER_CTRL, val|MASTER_CTRL_MANUAL_INT);

	at_irq_enable(adapter);

	return kIOReturnSuccess;
}

IOReturn AtherosL2Ethernet::disable(IONetworkInterface *netif)
{
	DbgPrint("disable()\n");
	
	at_adapter *adapter=&adapter_;
	
	setLinkStatus(kIONetworkLinkValid, 0);
	  
	transmitQueue_->flush();
	transmitQueue_->stop();
	transmitQueue_->setCapacity(0);

	at_reset_hw(adapter);
	IOSleep(1);
	   
	at_irq_disable(adapter);
	
	adapter->link_speed = SPEED_0;
    adapter->link_duplex = -1;

// pretty wrong place for DMA stuff cleaning, eh?

	if (adapter->pdev  && adapter->pdev->isOpen () ) adapter->pdev->close(this);
	return kIOReturnSuccess;
}

bool AtherosL2Ethernet::configureInterface(IONetworkInterface *netif)
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

const OSString *AtherosL2Ethernet::newVendorString() const
{
	return OSString::withCString("Atheros");
}

const OSString *AtherosL2Ethernet::newModelString() const
{
	return OSString::withCString("L2 LAN");
}

IOReturn AtherosL2Ethernet::selectMedium(const IONetworkMedium *medium)
{
	DbgPrint("selectMedium()\n");

	if (medium)
	{
		switch(medium->getIndex())
		{
		case MEDIUM_INDEX_AUTO:
			DbgPrint("Selected medium is autoselect\n");
			adapter_.link_speed = SPEED_100;
			adapter_.link_duplex = FULL_DUPLEX;
			adapter_.hw.MediaType = MEDIA_TYPE_AUTO_SENSOR;
			break;
		case MEDIUM_INDEX_10HD:
			DbgPrint("Selected medium is 10HD\n");
			adapter_.link_speed = SPEED_10;
			adapter_.link_duplex = HALF_DUPLEX;
			adapter_.hw.MediaType = MEDIA_TYPE_10M_HALF;
			break;
		case MEDIUM_INDEX_10FD:
			DbgPrint("Selected medium is 10FD\n");
			adapter_.link_speed = SPEED_10;
			adapter_.link_duplex = FULL_DUPLEX;
			adapter_.hw.MediaType = MEDIA_TYPE_10M_FULL;
			break;
		case MEDIUM_INDEX_100HD:
			DbgPrint("Selected medium is 100HD\n");
			adapter_.link_speed = SPEED_100;
			adapter_.link_duplex = HALF_DUPLEX;
			adapter_.hw.MediaType = MEDIA_TYPE_100M_HALF;
			break;
		case MEDIUM_INDEX_100FD:
			DbgPrint("Selected medium is 100FD\n");
			adapter_.link_speed = SPEED_100;
			adapter_.link_duplex = FULL_DUPLEX;
			adapter_.hw.MediaType = MEDIA_TYPE_100M_FULL;
			break;
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
	IOSleep(100*((medium->getIndex() == MEDIUM_INDEX_AUTO)? PHY_AUTO_NEG_TIME:PHY_FORCE_TIME));
	atGetAndUpdateLinkStatus();

	return kIOReturnSuccess;
}

IOReturn AtherosL2Ethernet::getHardwareAddress(IOEthernetAddress *addr)
{
	DbgPrint("getHardwareAddress()\n");
	IOSleep(1000);

	if (is_valid_ether_addr(adapter_.hw.mac_addr))
	{
		memcpy(addr->bytes, adapter_.hw.mac_addr, NODE_ADDRESS_SIZE);
		return kIOReturnSuccess;
	}

	if (get_permanent_address(&adapter_.hw) == 0)
	{
		if (is_valid_ether_addr(adapter_.hw.perm_mac_addr))
		{
			memcpy(adapter_.hw.mac_addr, adapter_.hw.perm_mac_addr, NODE_ADDRESS_SIZE);
			memcpy(addr->bytes, adapter_.hw.mac_addr, NODE_ADDRESS_SIZE);
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
		memcpy(adapter_.hw.mac_addr, addr->bytes, NODE_ADDRESS_SIZE);
		//return kIOReturnUnsupported;
		return kIOReturnSuccess;
	}
}

IOReturn AtherosL2Ethernet::setHardwareAddress(const IOEthernetAddress *addr)
{
    DbgPrint("setHardwareAddress()\n");
	memcpy(adapter_.hw.mac_addr, addr->bytes, NODE_ADDRESS_SIZE);
	set_mac_addr(&adapter_.hw);
	return kIOReturnSuccess;
}

void AtherosL2Ethernet::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
	DbgPrint("getPacketBufferConstraints()\n");
	constraints->alignStart = kIOPacketBufferAlign4;
	constraints->alignLength = kIOPacketBufferAlign4;
}



UInt32 AtherosL2Ethernet::outputPacket(mbuf_t m, void *prm)
{
    at_adapter *adapter=&adapter_;
    tx_pkt_header_t* txph;
    u32 offset, copy_len;
    int txs_unused;
    int txbuf_unused;

    u32 buf_len;

	if (mbuf_pkthdr_len(m) <= MAX_TX_BUF_LEN) buf_len = mbuf_pkthdr_len(m);
	else
	{
		DbgPrint("Tx Packet size is too big, droping\n");
		freePacket(m);
		return kIOReturnOutputDropped;
	}

	txs_unused = TxsFreeUnit(adapter);
	txbuf_unused = TxdFreeBytes(adapter);

	if (txs_unused < 1  || buf_len  > txbuf_unused) {
		// no enough resource
		DbgPrint("no enough resource!!\n");
		freePacket(m);
		return kIOReturnOutputDropped;
	}

	offset = adapter->txd_write_ptr;
DbgPrint("outputPacket() begin, txd_write_ptr %d txs_next_clear %d length %d \n" , adapter->txd_write_ptr,adapter->txs_next_clear,buf_len);

    txph = (tx_pkt_header_t*) 
            (((u8*)adapter->txd_ring)+offset);

	offset += 4;
	if (offset >= adapter->txd_ring_size)
		offset -= adapter->txd_ring_size;

	u32 pkt_snd_len = 0;
	mbuf_t cur_buf = m;

	do
	{
		if (mbuf_data(cur_buf)){
			copy_len = adapter->txd_ring_size - offset;
			if (copy_len >=mbuf_len(cur_buf)) {
				memcpy((u8*)adapter->txd_ring+offset, mbuf_data(cur_buf), mbuf_len(cur_buf));

			} else {
				memcpy((u8*)adapter->txd_ring+offset, mbuf_data(cur_buf), copy_len);
				memcpy((u8*)adapter->txd_ring,  ((u8*)mbuf_data(cur_buf))+copy_len, mbuf_len(cur_buf)-copy_len);
			}
			offset += mbuf_len(cur_buf);
			if (offset >= adapter->txd_ring_size)
				offset -= adapter->txd_ring_size;
			pkt_snd_len += mbuf_len(cur_buf);
		}
	}
	while(((cur_buf = mbuf_next(cur_buf)) != NULL) && ((pkt_snd_len + mbuf_len(cur_buf)) <= buf_len));
	buf_len = pkt_snd_len;
	
	*(u32*)txph = 0;
	txph->pkt_size = buf_len;
	
    offset =  ((offset+3)&~3);
	
	if (offset >= adapter->txd_ring_size)
		offset -= adapter->txd_ring_size;
	adapter->txd_write_ptr = offset;

	// clear txs before send
	adapter->txs_ring[adapter->txs_next_clear].update = 0;
	if (++adapter->txs_next_clear == adapter->txs_ring_size)
		adapter->txs_next_clear = 0;

	AT_WRITE_REGW(  &adapter->hw, 
             REG_MB_TXD_WR_IDX, 
             (adapter->txd_write_ptr>>2));

	OSSynchronizeIO();
	freePacket(m);
	return kIOReturnOutputSuccess;
}

IOReturn AtherosL2Ethernet::registerWithPolicyMaker(IOService *policyMaker)
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

IOReturn AtherosL2Ethernet::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
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
IOReturn AtherosL2Ethernet::setPromiscuousMode(bool enabled)
{
	DbgPrint("setPromiscuousMode(), %d\n", enabled);
	
    at_hw *hw = &adapter_.hw;
    /* Check for Promiscuous and All Multicast modes */
    u32 ctrl = AT_READ_REG(hw, REG_MAC_CTRL);
	if (enabled)
	{
		ctrl |= MAC_CTRL_PROMIS_EN;
	}
	else
	{
		ctrl &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
	}

    AT_WRITE_REG(hw, REG_MAC_CTRL, ctrl);

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
IOReturn AtherosL2Ethernet::setMulticastMode(bool enabled)
{
	DbgPrint("setMulticastMode(), %d\n", enabled);
    at_hw *hw = &adapter_.hw;

	/* Check for Promiscuous and All Multicast modes */
    u32 ctrl = AT_READ_REG(hw, REG_MAC_CTRL);
	if (enabled)
	{
    	/* clear the old settings from the multicast hash table */
    	AT_WRITE_REG(hw, REG_RX_HASH_TABLE, 0);
    	AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);
		ctrl |= MAC_CTRL_MC_ALL_EN;
		ctrl &= ~MAC_CTRL_PROMIS_EN;
	}
	else
	{
		ctrl &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
	}

    AT_WRITE_REG(hw, REG_MAC_CTRL, ctrl);

	return kIOReturnSuccess;
}

IOReturn AtherosL2Ethernet::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
	DbgPrint("setMulticastList()\n");

    at_hw *hw = &adapter_.hw;
	u32 hash_value;
	for(int i = 0; i < count; i++, addrs++)
	{
		hash_value = at_hash_mc_addr(hw,addrs->bytes);
		at_hash_set(hw,hash_value);
	}

	return kIOReturnSuccess;
}

#pragma mark -
#pragma mark - Atheros L2 Adapter
#pragma mark -



int AtherosL2Ethernet::mdio_read(int phy_id, int reg_num)
{
	u16 result;

	at_read_phy_reg(&adapter_.hw, reg_num & 0x1f, &result);

	return result;
}

void AtherosL2Ethernet::mdio_write(int phy_id, int reg_num, int val)
{
	at_write_phy_reg(&adapter_.hw, reg_num, val);
}

/**
 * atl2_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in atl2_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * atl2_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
bool AtherosL2Ethernet::atProbe()
{
	u16	vendorId, deviceId;
    at_adapter *adapter=&adapter_;
	IOPCIDevice	*pdev = adapter_.pdev;
	pdev->setBusMasterEnable(true);
	pdev->setMemoryEnable(true);
	pdev->setIOEnable(false);
	vendorId = pdev->configRead16(kIOPCIConfigVendorID);
	deviceId = pdev->configRead16(kIOPCIConfigDeviceID);

	DbgPrint("Vendor ID %x, device ID %x\n", vendorId, deviceId);
	DbgPrint("MMR0 address %x\n", pdev->configRead32(kIOPCIConfigBaseAddress0));

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
					(u32)hw_addr_->getVirtualAddress(), (u32)hw_addr_->getLength());
	}
	
	hw_addr_->retain();

	adapter->hw.mmr_base = reinterpret_cast<char *>(hw_addr_->getVirtualAddress());

	DbgPrint("REG_VPD_CAP = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_VPD_CAP));
	DbgPrint("REG_PCIE_CAP_LIST = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_PCIE_CAP_LIST));
	DbgPrint("REG_MASTER_CTRL = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_MASTER_CTRL));
	
	at_setup_pcicmd(pdev);
	
	/* setup the private structure */
	if(at_sw_init(adapter))
	{
		ErrPrint("Couldn't init software\n");
		return false;
	}
	/* Init PHY as early as possible due to power saving issue  */
    at_phy_init(&adapter->hw);

	/* reset the controller to
	* put the device in a known good starting state */
	if (at_reset_hw(adapter))
	{
		ErrPrint("Couldn't reset hardware\n");
		//return false;			//TO-DO: Uncomment
	}


    /* copy the MAC address out of the EEPROM */
    at_read_mac_addr(&adapter->hw);
	
	/* get user settings */
    at_check_options(adapter);



	return true;
}



void AtherosL2Ethernet::atSwFree()
{
	if (adapter_.stats_lock)
	{
		IOSimpleLockFree(adapter_.stats_lock);
		adapter_.stats_lock = NULL;
	}

	if (adapter_.tx_lock)
	{
		IOSimpleLockFree(adapter_.tx_lock);
		adapter_.tx_lock = NULL;
	}
}



#pragma mark -
#pragma mark - Atheros L2 Hw methods
#pragma mark -

void AtherosL2Ethernet::atGetAndUpdateLinkStatus()
{
        u16 speed, duplex;
		u32 currentMediumIndex = MEDIUM_INDEX_AUTO;

		if(at_get_speed_and_duplex(&adapter_.hw, &speed, &duplex) == AT_SUCCESS)
		{
			DbgPrint("Link is active, speed %d, duplex %d\n", speed, duplex);


			if(speed == SPEED_10 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10FD;
			if(speed == SPEED_100 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100FD;
			if(speed == SPEED_10 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10HD;
			if(speed == SPEED_100 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100HD;

			setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, mediumTable[currentMediumIndex], speed * MBit, NULL);

			adapter_.link_speed = speed;
			adapter_.link_duplex = duplex;

			at_setup_mac_ctrl(&adapter_);
		} else
		{
			DbgPrint("Link is down\n");
			setLinkStatus(kIONetworkLinkValid, NULL, 0, NULL);
		}
}

/*
 * Configures link settings.
 * Assumes the hardware has previously been reset and the
 * transmitter and receiver are not enabled.
 */
SInt32 AtherosL2Ethernet::atSetupLink()
{
	SInt32 ret_val;

	/*
	 * Options:
	 *  PHY will advertise value(s) parsed from
	 *  autoneg_advertised and fc
	 *  no matter what autoneg is , We will not wait link result.
	 */
	at_hw *hw = &adapter_.hw;
	
	ret_val = at_phy_setup_autoneg_adv(hw);
	if (ret_val) 
	{
		ErrPrint("setting up autonegotiation\n");
		return ret_val;
	}
	/* SW.Reset , En-Auto-Neg if needed */
	ret_val = at_phy_commit(hw);
	if (ret_val) 
	{
		ErrPrint("resetting the phy\n");
		return ret_val;
	}

	return ret_val;
}
