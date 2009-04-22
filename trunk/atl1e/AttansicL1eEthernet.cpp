/* AttansicL1eEthernet.h -- ATL1e driver implements
 *
 * Copyright (c) 2009 maolj <maolj@hotmail.com>.
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
 * Driver for Atheros(R) AR8121/AR8113/AR8114 PCI-E Ethernet Network Driver.
 *
 * This driver is heavily based on ATL1e Linux driver by xiong huang <xiong.huang@atheros.com>.
 */

#include "AttansicL1eEthernet.h"

#pragma mark -
#pragma mark - Various definitions & helper functions -
#pragma mark -


#define BASE				IOEthernetController
#define RELEASE(x)			do { if(x) { (x)->release(); (x) = 0; } } while(0)


OSDefineMetaClassAndStructors(AttansicL1eEthernet, IOEthernetController)

#pragma mark -
#pragma mark - IOService overrides -
#pragma mark -

bool AttansicL1eEthernet::init(OSDictionary *properties)
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

void AttansicL1eEthernet::free()
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

bool AttansicL1eEthernet::start(IOService *provider)
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
	OSAddNetworkMedium(kIOMediumEthernet1000BaseTX | kIOMediumOptionHalfDuplex, 1000 * MBit, MEDIUM_INDEX_1000HD);
	OSAddNetworkMedium(kIOMediumEthernet1000BaseTX | kIOMediumOptionFullDuplex, 1000 * MBit, MEDIUM_INDEX_1000FD);
	
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
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AttansicL1eEthernet::atIntr),
					adapter->pdev, msi_index);
	}

	if (msi_index == -1 || intSource_ == NULL)
	{
		DbgPrint("MSI index was not found or MSI interrupt couldn't be enabled\n");
		intSource_ = IOInterruptEventSource::interruptEventSource(this,
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AttansicL1eEthernet::atIntr),
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
	
	if(at_alloc_tx_buffers(adapter))
	{
		ErrPrint("Couldn't allocate tx descriptors\n");
		adapter->pdev->close(this);
		return kIOReturnError;
	}
	

	netIface_->registerService();
	adapter->pdev->close(this);
	return true;
}

void AttansicL1eEthernet::stop(IOService *provider)
{
	DbgPrint("stop()\n");
	if (intSource_) intSource_->disable();
	this->detachInterface(netIface_);
	
	at_free_ring_resources(&adapter_);
	atSwFree();
	BASE::stop(provider);
}

bool AttansicL1eEthernet::OSAddNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
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

IOOutputQueue *AttansicL1eEthernet::createOutputQueue()
{
	DbgPrint("createOutputQueue()\n");
	//Sharing one event source with transmith/receive handles
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}


/**
 * at_intr - Interrupt Handler
 **/
void AttansicL1eEthernet::atIntr(OSObject *client, IOInterruptEventSource *src, int count)
{
    at_adapter *adapter = &adapter_;
    at_hw *hw = &adapter->hw;
    u32 status;
   
    int max_ints = 5;
    
    //DEBUGFUNC("at_intr()\n");    
    
    status = AT_READ_REG(hw, REG_ISR);
    if (0 == (status & IMR_NORMAL_MASK) ||
        0 != (status & ISR_DIS_INT))   
        return ;

	do{
		// link event
		if (status&ISR_GPHY) {
			at_clear_phy_int(adapter);
		}

		// Ack ISR
		AT_WRITE_REG(hw, REG_ISR, status | ISR_DIS_INT);  

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
			AT_WRITE_REG(hw, REG_ISR, 0);
			AT_WRITE_REG(hw, REG_IMR, 0);
			AT_WRITE_FLUSH(hw);
			at_reset_hw(adapter);
			return ; 
		}

		// link event
		if (status&(ISR_GPHY|ISR_MANUAL))
		{
			//AT_DBG("int-status=0x%x", status);
			IOSleep(2);
			atGetAndUpdateLinkStatus();
		}

		// transmit event
		if ( status & ISR_TX_EVENT ) {
			at_clean_tx_irq(adapter);
		}


		// recv event
		if ( status & ISR_RX_EVENT ) {
			at_clean_rx_irq(adapter, 0);
		}

	
		status = AT_READ_REG(hw, REG_ISR);
		
		if (--max_ints <= 0)	{
			DbgPrint("maximum interrupts reached\n");
			break;
		}
	}
	while (0 != (status & IMR_NORMAL_MASK));
	
    // re-enable Interrupt
    AT_WRITE_REG(&adapter->hw, REG_ISR, 0);

}


bool AttansicL1eEthernet::at_clean_tx_irq(at_adapter* adapter)
{ 
    at_buffer* buffer_info;
    TpdDescr* pTpd;    
    u16 hw_next_to_clean = AT_READ_REGW(&adapter->hw, REG_TPD_CONS_IDX);
    u16 next_to_clean = atomic_read(&adapter->tpd_next_clean);
	
	DEBUGOUT1("at_clean_tx_irq() hw_next_to_clean=%d, next_to_clean=%d\n",
		hw_next_to_clean, next_to_clean);
    
    while (next_to_clean != hw_next_to_clean) {
        pTpd = adapter->tpd_ring + next_to_clean;
        buffer_info = adapter->tx_buffer_info + next_to_clean;
        //TO-DO: Add cleaning

        if (++next_to_clean == adapter->tpd_ring_size)  
            next_to_clean = 0;
    }

    atomic_set(&adapter->tpd_next_clean, next_to_clean);
           
    return (next_to_clean == adapter->tpd_next_use) ? true : false;
}

static u16 vld_regs[] = 
{REG_HOST_RXF0_PAGE0_VLD, REG_HOST_RXF0_PAGE1_VLD,
 REG_HOST_RXF1_PAGE0_VLD, REG_HOST_RXF1_PAGE1_VLD,
 REG_HOST_RXF2_PAGE0_VLD, REG_HOST_RXF2_PAGE1_VLD,
 REG_HOST_RXF3_PAGE0_VLD, REG_HOST_RXF3_PAGE1_VLD};

void AttansicL1eEthernet::at_clean_rx_irq(at_adapter *adapter, u8 que)
{
    mbuf_t skb = NULL;
    at_page* pPage = &(adapter->rxf_page[que][adapter->rxf_using[que]]);
    u32 packet_size, WrtOffset;
    RecvRetStatus*  prrs; 
  
    DEBUGOUT1("at_clean_rx_irq()  que=%d, wcmb=%p, *wcmb=%x\n",
              que, pPage->pWptr, *pPage->pWptr);
    
    if (pPage->Rptr < (WrtOffset = *pPage->pWptr)) {
        
        do {
       
            // get new packet's  rrs
            prrs = (RecvRetStatus*) (pPage->addr + pPage->Rptr);
            // check sequence number
            if (prrs->seq_num != adapter->rxf_nxseq[que]) {
                AT_ERR("rx sequence number error (rx=%d) (expect=%d)\n",
                    prrs->seq_num, adapter->rxf_nxseq[que]);
                adapter->rxf_nxseq[que]++;
                AT_WRITE_REG(&adapter->hw, 
                    0,  (((u32)prrs->seq_num)<<16)| adapter->rxf_nxseq[que]);
                AT_WRITE_REG(&adapter->hw, 0, que);
				IOSleep(2);
				disable(NULL);
				enable(NULL);
                return;
            }

            adapter->rxf_nxseq[que]++;

            // error packet ?
            if (prrs->err) {
                if (prrs->crc || prrs->dribble || prrs->code || prrs->trunc) { 
                    // hardware error, discard this packet
                    AT_ERR("rx packet desc error %x\n", *((u32*)prrs + 1));
                    goto skip_pkt;
                }
            }
            
            packet_size = prrs->pkt_len - 4; // CRC
			// alloc new buffer
			skb = allocatePacket(packet_size + 2);
            if (NULL == skb) {
                DbgPrint("Memory squeeze, deferring packet.\n");
                goto skip_pkt;
            }
            DEBUGOUT1("pktsize=%d\n", packet_size);
                   
            // copy packet to user buffer
			memcpy(mbuf_data(skb),  (u8*)(prrs+1), packet_size); 

			//TO-DO: Add network stack notification

			netIface_->inputPacket(skb, packet_size, IONetworkInterface::kInputOptionQueuePacket);
			netIface_->flushInputQueue();       
        
skip_pkt:   // skip current packet whether it's ok or not.
    
            pPage->Rptr += 
                ((u32)(prrs->pkt_len + sizeof(RecvRetStatus) + 31)& 0xFFFFFFE0);
                
            if (pPage->Rptr >= adapter->rxf_length) { // reuse this page
                
                pPage->Rptr = *pPage->pWptr = 0;
                
                AT_WRITE_REGB(&adapter->hw, 
                    vld_regs[que*2+adapter->rxf_using[que]],
                    1);
                adapter->rxf_using[que] ^= 1;
                pPage = &(adapter->rxf_page[que][adapter->rxf_using[que]]);
                WrtOffset = *pPage->pWptr;
            }
            
        } while (pPage->Rptr < WrtOffset) ;
    }


    return;
    
}

#pragma mark -
#pragma mark - IOEthernetController overrides -
#pragma mark -

IOReturn AttansicL1eEthernet::enable(IONetworkInterface *netif)
{
	DbgPrint("enable()\n");

	u32 val;
	at_adapter *adapter=&adapter_;
	
	if (!adapter->pdev || (!adapter->pdev->isOpen () && (adapter->pdev->open(this)) == 0)) {
	    DbgPrint( "failed to open PCI device.\n");
        return kIOReturnError;
    }
	
	// hardware has been reset, we need to reload some things 
	if (at_init_hw(&adapter->hw))
	{
		ErrPrint("Couldn't init hw\n");
		return kIOReturnError;
	}
	
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

IOReturn AttansicL1eEthernet::disable(IONetworkInterface *netif)
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

	//at_clean_tx_ring(adapter);
	at_clean_rx_ring(adapter);

	if (adapter->pdev  && adapter->pdev->isOpen () ) adapter->pdev->close(this);
	return kIOReturnSuccess;
}

bool AttansicL1eEthernet::configureInterface(IONetworkInterface *netif)
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

const OSString *AttansicL1eEthernet::newVendorString() const
{
	return OSString::withCString("Attansic");
}

const OSString *AttansicL1eEthernet::newModelString() const
{
	return OSString::withCString("L1e LAN");
}

IOReturn AttansicL1eEthernet::selectMedium(const IONetworkMedium *medium)
{
	DbgPrint("selectMedium()\n");

	if (medium)
	{
		switch(medium->getIndex())
		{
		case MEDIUM_INDEX_AUTO:
			DbgPrint("Selected medium is autoselect\n");
			adapter_.link_speed = SPEED_1000;
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
		case MEDIUM_INDEX_1000HD:
			DbgPrint("Selected medium is 1000HD\n");
			adapter_.link_speed = SPEED_1000;
			adapter_.link_duplex = HALF_DUPLEX;	
			adapter_.hw.MediaType = MEDIA_TYPE_1000M_FULL;
			break;
		case MEDIUM_INDEX_1000FD:
			DbgPrint("Selected medium is 1000FD\n");
			adapter_.link_speed = SPEED_1000;
			adapter_.link_duplex = FULL_DUPLEX;		
			adapter_.hw.MediaType = MEDIA_TYPE_1000M_FULL;
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

IOReturn AttansicL1eEthernet::getHardwareAddress(IOEthernetAddress *addr)
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

IOReturn AttansicL1eEthernet::setHardwareAddress(const IOEthernetAddress *addr)
{
    DbgPrint("setHardwareAddress()\n");
	memcpy(adapter_.hw.mac_addr, addr->bytes, NODE_ADDRESS_SIZE);
	set_mac_addr(&adapter_.hw);
	return kIOReturnSuccess;
}

void AttansicL1eEthernet::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
	DbgPrint("getPacketBufferConstraints()\n");
	constraints->alignStart = kIOPacketBufferAlign4;
	constraints->alignLength = kIOPacketBufferAlign4;
}


UInt32 AttansicL1eEthernet::outputPacket(mbuf_t m, void *prm)
{
	UInt32 buf_len;
	at_adapter *adapter=&adapter_;

	u16 next_to_use;
	TpdDescr *pTpd ;
	struct at_buffer *buffer_info;

    if(tpd_avail(adapter) < 1) {
        // no enough descriptor
		DbgPrint("no enough resource!!\n");
		freePacket(m);
		return kIOReturnOutputDropped;
    }
    

    next_to_use = adapter->tpd_next_use;
    buffer_info = &adapter->tx_buffer_info[next_to_use]; 
	
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
	
	DbgPrint("outputPacket() length %d next_to_use=%d\n", buf_len, next_to_use);
	
	UInt8 *data_ptr = (UInt8 *)buffer_info->memDesc->getBytesNoCopy();
	UInt32 pkt_snd_len = 0;
	mbuf_t cur_buf = m;

	do
	{
		if (mbuf_data(cur_buf))	bcopy(mbuf_data(cur_buf), data_ptr, mbuf_len(cur_buf));
		data_ptr += mbuf_len(cur_buf);
		pkt_snd_len += mbuf_len(cur_buf);
	}
	while(((cur_buf = mbuf_next(cur_buf)) != NULL) && ((pkt_snd_len + mbuf_len(cur_buf)) <= buf_len));
	
	// init tpd flags
    pTpd = adapter->tpd_ring + adapter->tpd_next_use;
	u32 addr_size = sizeof(pTpd->addr);
	memset( ((u8*)pTpd + addr_size) , 0, (sizeof(TpdDescr) - addr_size) );  //addr don't clear
	
	buf_len = pkt_snd_len;
	buffer_info->length = (UInt16)buf_len;
	pTpd->buf_len= OSSwapHostToLittleInt16((UInt16)buf_len);
	pTpd->eop = 1;
	
	if(++next_to_use == adapter->tpd_ring_size) next_to_use = 0; 
	adapter->tpd_next_use = next_to_use;

	AT_WRITE_REG(&adapter->hw, REG_MB_TPD_PROD_IDX, adapter->tpd_next_use);
	
	OSSynchronizeIO();

	freePacket(m);
	return kIOReturnOutputSuccess;
}

IOReturn AttansicL1eEthernet::registerWithPolicyMaker(IOService *policyMaker)
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

IOReturn AttansicL1eEthernet::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
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
IOReturn AttansicL1eEthernet::setPromiscuousMode(bool enabled)
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
IOReturn AttansicL1eEthernet::setMulticastMode(bool enabled)
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

IOReturn AttansicL1eEthernet::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
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
#pragma mark - Attansic L2 Adapter
#pragma mark -


int AttansicL1eEthernet::mdio_read(int phy_id, int reg_num)
{
	UInt16 result;
	at_read_phy_reg(&adapter_.hw, reg_num & 0x1f, &result);

	return result;
}

void AttansicL1eEthernet::mdio_write(int phy_id, int reg_num, int val)
{
	at_write_phy_reg(&adapter_.hw, reg_num, val);
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
bool AttansicL1eEthernet::atProbe()
{
	UInt16	vendorId, deviceId;
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
		DbgPrint("Memory mapped at bus address %x, virtual address %x, length %d\n", hw_addr_->getPhysicalAddress(),
					hw_addr_->getVirtualAddress(), hw_addr_->getLength());
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
	
    /* get user settings */
    at_check_options(adapter);
		

    /* Init GPHY as early as possible due to power saving issue  */
    at_phy_init(&adapter->hw);

	/* reset the controller to
	* put the device in a known good starting state */
	if (at_reset_hw(adapter))
	{
		ErrPrint("Couldn't reset hardware\n");
		return false;			//TO-DO: Uncomment
	}


    /* copy the MAC address out of the EEPROM */
    at_read_mac_addr(&adapter->hw);

	return true;
}


void AttansicL1eEthernet::atSwFree()
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
#pragma mark - Attansic L1e Hw methods
#pragma mark -

void AttansicL1eEthernet::atGetAndUpdateLinkStatus()
{
        UInt16 speed, duplex;
		UInt32 currentMediumIndex = MEDIUM_INDEX_AUTO;

		if(at_get_speed_and_duplex(&adapter_.hw, &speed, &duplex) == AT_SUCCESS)
		{
			DbgPrint("Link is active, speed %d, duplex %d\n", speed, duplex);


			if(speed == SPEED_10 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10FD;
			if(speed == SPEED_100 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100FD;
			if(speed == SPEED_1000 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_1000FD;
			if(speed == SPEED_10 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10HD;
			if(speed == SPEED_100 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100HD;
			if(speed == SPEED_1000 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_1000HD;	

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
SInt32 AttansicL1eEthernet::atSetupLink()
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
	//hw->phy_configured = true;
	return ret_val;
}


