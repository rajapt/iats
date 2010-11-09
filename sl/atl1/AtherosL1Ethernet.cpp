/* AtherosL1Ethernet.cpp -- ATL1 driver implements
 *
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 * Copyright(c) 2010 maolj <maolj@hotmail.com> All rights reserved.
 *
 * Derived from Intel e1000 driver
 * Copyright(c) 1999 - 2005 Intel Corporation. All rights reserved.
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
 * There are a lot of defines in here that are unused and/or have cryptic
 * names.  Please leave them alone, as they're the closest thing we have
 * to a spec from Atheros at present. *ahem* -- CHS
 *
 * Driver for the Atheros(R) L1 Gigabit Ethernet Adapter.
 *
 * This driver is heavily based on ATL1 Linux driver by xiong huang <xiong.huang@atheros.com>.
 */

#include "AtherosL1Ethernet.h"

#pragma mark -
#pragma mark - Various definitions & helper functions -
#pragma mark -


#define BASE                IOEthernetController
#define RELEASE(x)          do { if(x) { (x)->release(); (x) = 0; } } while(0)


OSDefineMetaClassAndStructors(AtherosL1Ethernet, IOEthernetController)

#pragma mark -
#pragma mark - IOService overrides -
#pragma mark -

bool AtherosL1Ethernet::init(OSDictionary *properties)
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
    adapter_.hw.back = &adapter_;
    return true;
}

void AtherosL1Ethernet::free()
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

bool AtherosL1Ethernet::start(IOService *provider)
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

    if (!atProbe()) //Fix false reporting int probe function
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
                    OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosL1Ethernet::atIntr),
                    adapter->pdev, msi_index);
    }

    if (msi_index == -1 || intSource_ == NULL)
    {
        DbgPrint("MSI index was not found or MSI interrupt couldn't be enabled\n");
        intSource_ = IOInterruptEventSource::interruptEventSource(this,
                    OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AtherosL1Ethernet::atIntr),
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
    
    /* allocate Tx / RX descriptor resources */
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

void AtherosL1Ethernet::stop(IOService *provider)
{
    DbgPrint("stop()\n");
    if (intSource_) intSource_->disable();
    this->detachInterface(netIface_);
    
    at_free_ring_resources(&adapter_);

    BASE::stop(provider);
}

bool AtherosL1Ethernet::OSAddNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
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

IOOutputQueue *AtherosL1Ethernet::createOutputQueue()
{
    DbgPrint("createOutputQueue()\n");
    //Sharing one event source with transmith/receive handles
    return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}


/**
 * at_intr - Interrupt Handler
 **/
void AtherosL1Ethernet::atIntr(OSObject *client, IOInterruptEventSource *src, int count)
{
    at_adapter *adapter = &adapter_;
    at_hw *hw = &adapter->hw;

    u32 status;
    int max_ints = 12;
    
    DEBUGFUNC("at_intr() !\n");
    
    if (0 == (status = adapter->cmb.cmb->int_stats))
        return ;
    DEBUGOUT("atIntr() status = 0x%x!\n", status);

loopint:
    
    // clear CMB interrupt status at once
    adapter->cmb.cmb->int_stats = 0;
    
    if (status & ISR_GPHY) { // clear phy status
        at_clear_phy_int(adapter);
    }   
    
    // Ack ISR
    AT_WRITE_REG(hw, REG_ISR, (status|ISR_DIS_INT|ISR_DIS_DMA));  
    
    // check if PCIE PHY Link down
    if (status&ISR_PHY_LINKDOWN) {
        DEBUGOUT1("pcie phy linkdown %x\n", status);
        AT_WRITE_REG(hw, REG_ISR, 0);
        AT_WRITE_REG(hw, REG_IMR, 0);
        AT_WRITE_FLUSH(hw);
        at_reset_hw(hw);
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
        at_reset_hw(hw);
        return ; 
    }
    
    // check if SMB intr
    if (status & ISR_SMB) {
        at_inc_smb(adapter);
    }
    
    
    // link event
    if (status&(ISR_GPHY|ISR_MANUAL))
    {
        IOSleep(2);
        atGetAndUpdateLinkStatus();
    }
    
    // transmit event
    if ( status & ISR_TX_EVENT ) {
        at_clean_tx_irq(adapter);
    }

    
    // recv event
    if ( status & ISR_RX_EVENT ) {
        at_clean_rx_irq(adapter);
    }
    
    if (--max_ints > 0 && 
        0 != (status = adapter->cmb.cmb->int_stats)) {
        goto  loopint;
    }
    
    
    // re-enable Interrupt
    AT_WRITE_REG(&adapter->hw, REG_ISR, ISR_DIS_SMB|ISR_DIS_DMA);

}


bool AtherosL1Ethernet::at_clean_tx_irq(at_adapter* adapter)
{ 
    struct at_tpd_ring *tpd_ring = &adapter->tpd_ring; 
    struct at_buffer* buffer_info;
    u16 sw_tpd_next_to_clean;
    u16 cmb_tpd_next_to_clean;
    
    sw_tpd_next_to_clean = (u16)atomic_read(&tpd_ring->next_to_clean);
    cmb_tpd_next_to_clean = OSSwapLittleToHostInt16(adapter->cmb.cmb->tpd_cons_idx);
    
    DbgPrint("atl1c_clean_rx_irq() sw_tpd_next_to_clean=%d,cmb_tpd_next_to_clean=%d\n",
              sw_tpd_next_to_clean ,cmb_tpd_next_to_clean);
    
    while (cmb_tpd_next_to_clean != sw_tpd_next_to_clean) {
        TpdDescr* tpd;
        
        tpd = AT_TPD_DESC(tpd_ring, sw_tpd_next_to_clean);
        buffer_info = &tpd_ring->buffer_info[sw_tpd_next_to_clean];
         //TO-DO: Add cleaning
        
        if (++sw_tpd_next_to_clean == tpd_ring->count)  
            sw_tpd_next_to_clean = 0;
    }
    atomic_set(&tpd_ring->next_to_clean, sw_tpd_next_to_clean);
    
    
    return (sw_tpd_next_to_clean == (u16)atomic_read(&tpd_ring->next_to_use)) ? TRUE : FALSE;
}


void AtherosL1Ethernet::at_clean_rx_irq(at_adapter *adapter)
{
    mbuf_t skb = NULL;
    u32 packet_size;
    int i, count;
    u16 length, rrd_next_to_clean;
    struct at_rfd_ring *rfd_ring = &adapter->rfd_ring;
    struct at_rrd_ring *rrd_ring = &adapter->rrd_ring; 
    struct at_buffer * buffer_info;
    rx_return_desc_t* rrd;
    
    count = 0; 
    
    rrd_next_to_clean = (u16)atomic_read(&rrd_ring->next_to_clean);   
    DEBUGOUT1("at_clean_rx_irq()  rrd_next_to_clean=%d\n",
              rrd_next_to_clean);
     
    while (1)
    {
        rrd = AT_RRD_DESC(rrd_ring, rrd_next_to_clean);
        i = 1;
        if (rrd->xsz.valid) { // packet valid
        chk_rrd:        
            // check rrd status
            if (rrd->num_buf != 1) {
                DEBUGOUT1("RRD NumRfd %d\n", rrd->num_buf);
                DEBUGOUT1("packet length = %d\n", rrd->xsz.xsum_sz.pkt_size);
            } else {
                goto rrd_ok;
            }
            
            // rrd seems to be bad
            if (i-- > 0) { // rrd may not be DMAed completely
                DEBUGOUT("RRD may not be DMAed completely\n");
                usec_delay(1);
                goto chk_rrd;
            }
            // bad rrd
            AT_ERR("BAD RRD\n");
            
            // see if update RFD index
            if (rrd->num_buf > 1) {
                u16 num_buf;
                num_buf = 
                (rrd->xsz.xsum_sz.pkt_size+adapter->rx_buffer_len - 1)/
                adapter->rx_buffer_len;
                DEBUGOUT1("RRD.buf_index (%d)\n", rrd->buf_indx);
                if (rrd->num_buf == num_buf) {
                    // clean alloc flag for bad rrd
                    while (rfd_ring->next_to_clean != 
                           (rrd->buf_indx + num_buf) ) {
                        DEBUGOUT1("clear index (%d)\n", rfd_ring->next_to_clean);
                        rfd_ring->buffer_info[rfd_ring->next_to_clean].alloced = 0;
                        if (++rfd_ring->next_to_clean == rfd_ring->count) {
                            rfd_ring->next_to_clean = 0;
                        } 
                    } // end while      
                } // end if (rrd->num_buf == ...)
            }
            
            // update rrd
            rrd->xsz.valid = 0;
            if (++rrd_next_to_clean == rrd_ring->count)
                rrd_next_to_clean = 0;
            count++; 
            continue;
        } else { // current rrd still not be updated
            break;
        }
        
    rrd_ok:
        
        
        // clean alloc flag for bad rrd
        while (rfd_ring->next_to_clean != rrd->buf_indx) {
            rfd_ring->buffer_info[rfd_ring->next_to_clean].alloced = 0;
            if (++rfd_ring->next_to_clean == rfd_ring->count) {
                rfd_ring->next_to_clean = 0;
            }
        }       
        
        buffer_info = &rfd_ring->buffer_info[rrd->buf_indx];
        if (++rfd_ring->next_to_clean == rfd_ring->count) {
            rfd_ring->next_to_clean = 0;
        }
        
        
        // update rrd next to clean
        if (++rrd_next_to_clean == rrd_ring->count)
            rrd_next_to_clean = 0;
        
        count++; 
        
        if (rrd->pkt_flg&PACKET_FLAG_ERR) {
            if (rrd->err_flg& 
                (ERR_FLAG_CRC|ERR_FLAG_TRUNC|ERR_FLAG_CODE|ERR_FLAG_OV)) {
                /* packet error , don't need upstream */
                buffer_info->alloced = 0;  
                rrd->xsz.valid = 0;
                DEBUGOUT1("rrd error flag %x\n", rrd->err_flg);
                continue;
            }
        }
        
        /* Good Receive */
        length = OSSwapLittleToHostInt16(rrd->xsz.xsum_sz.pkt_size);
        
        packet_size = length - 4; // CRC
        // alloc new buffer
        skb = allocatePacket(packet_size + 2);
        if (NULL == skb) {
            DbgPrint("Memory squeeze, deferring packet.\n");
            break;
        }
        DEBUGOUT1("pktsize=%d\n", packet_size);
        
        // copy packet to user buffer
        if (buffer_info->memDesc)
        {
            memcpy( mbuf_data(skb),buffer_info->memDesc->getBytesNoCopy(), packet_size);
        } 
        
        //TO-DO: Add network stack notification
        
        netIface_->inputPacket(skb, packet_size, IONetworkInterface::kInputOptionQueuePacket);
        netIface_->flushInputQueue();    

        /*
         DEBUGOUT1("pkt:%02x %02x %02x %02x %02x %02x-%02x\n",
         skb->data[0], skb->data[1], skb->data[2],
         skb->data[3], skb->data[4], skb->data[5],
         skb->data[12]);
         */ 
        // let protocol layer free skb
        buffer_info->alloced = 0;  
        rrd->xsz.valid = 0;
        
    }
    
    atomic_set(&rrd_ring->next_to_clean, rrd_next_to_clean);
    
    at_alloc_rx_buffers(adapter);
    
    // update mailbox ?
    if (0 != count) {
        at_update_mailbox(adapter);
    }
}

#pragma mark -
#pragma mark - IOEthernetController overrides -
#pragma mark -

IOReturn AtherosL1Ethernet::enable(IONetworkInterface *netif)
{
    DbgPrint("enable()\n");
    
    int err = 0;
    u32 val;
    at_adapter *adapter=&adapter_;
    
    if (!adapter->pdev || (!adapter->pdev->isOpen () && (adapter->pdev->open(this)) == 0)) {
        DbgPrint( "failed to open PCI device.\n");
        return kIOReturnError;
    }
    
    /* hardware has been reset, we need to reload some things */
    
    err = at_init_hw(&adapter->hw);
    if (err) {
        ErrPrint("Couldn't init hw\n");
        return kIOReturnError;
    }
    

    init_ring_ptrs(adapter);

    err = at_alloc_tx_buffers(adapter);
    if (0 == err) { // no TX BUFFER allocated
        ErrPrint("Couldn't alloc TX buffer\n");
        return kIOReturnError;
    }  
    
    err = at_alloc_rx_buffers(adapter);
    if (0 == err) { // no RX BUFFER allocated
        ErrPrint("Couldn't alloc RX buffer\n");
        return kIOReturnError;
    }    
    
    if (at_configure(adapter)) {
        ErrPrint("Couldn't configure\n");
        return kIOReturnError;
    }    


    //PHY medium selection
    const IONetworkMedium *medium = getSelectedMedium();
    if (!medium)
    {
        DbgPrint("Selected medium is NULL, forcing to autonegotiation\n");
        medium = mediumTable[MEDIUM_INDEX_AUTO];
    }
    else
    {
        DbgPrint("Selected medium index %d\n", (s32)medium->getIndex());
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

IOReturn AtherosL1Ethernet::disable(IONetworkInterface *netif)
{
    DbgPrint("disable()\n");
    at_adapter *adapter=&adapter_;
    
    setLinkStatus(kIONetworkLinkValid, 0);
    
    transmitQueue_->flush();
    transmitQueue_->stop();
    transmitQueue_->setCapacity(0);

    /* reset MAC to disable all RX/TX */
    at_reset_hw(&adapter->hw);
    adapter->cmb.cmb->int_stats = 0;
    msec_delay(1);
   
    at_irq_disable(adapter);
    
    adapter->link_speed = SPEED_0;
    adapter->link_duplex = -1;
    
    at_clean_tx_ring(adapter);
    at_clean_rx_ring(adapter);

    if (adapter->pdev  && adapter->pdev->isOpen () ) adapter->pdev->close(this);
    return kIOReturnSuccess;
}

bool AtherosL1Ethernet::configureInterface(IONetworkInterface *netif)
{
    DbgPrint("configureInterface()\n");
    IONetworkData *data;
    at_adapter *adapter=&adapter_;
    
    if (!BASE::configureInterface(netif)) return false;

    //Get the generic network statistics structure.
    data = netif->getParameter(kIONetworkStatsKey);
    if (!data || !(adapter->net_stats = (IONetworkStats *)data->getBuffer()))
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

const OSString *AtherosL1Ethernet::newVendorString() const
{
    return OSString::withCString("Atheros");
}

const OSString *AtherosL1Ethernet::newModelString() const
{
    return OSString::withCString("L1 LAN");
}

IOReturn AtherosL1Ethernet::selectMedium(const IONetworkMedium *medium)
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

IOReturn AtherosL1Ethernet::getHardwareAddress(IOEthernetAddress *addr)
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

IOReturn AtherosL1Ethernet::setHardwareAddress(const IOEthernetAddress *addr)
{
    DbgPrint("setHardwareAddress()\n");
    memcpy(adapter_.hw.mac_addr, addr->bytes, NODE_ADDRESS_SIZE);
    set_mac_addr(&adapter_.hw);
    return kIOReturnSuccess;
}

void AtherosL1Ethernet::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
    DbgPrint("getPacketBufferConstraints()\n");
    constraints->alignStart = kIOPacketBufferAlign4;
    constraints->alignLength = kIOPacketBufferAlign4;
}


UInt32 AtherosL1Ethernet::outputPacket(mbuf_t m, void *prm)
{
    u32 buf_len;
    at_adapter *adapter=&adapter_;

    u16 next_to_use;
    u16 tpd_req = 1;
    TpdDescr *pTpd ;
    struct at_buffer *buffer_info;

     if(tpd_avail(&adapter->tpd_ring) < tpd_req) {
        // no enough descriptor
        DbgPrint("no enough resource!!\n");
        freePacket(m);
        return kIOReturnOutputDropped;
    }
    

    // init tpd flags
    struct at_tpd_ring* tpd_ring = &adapter->tpd_ring;
    pTpd = AT_TPD_DESC(tpd_ring,
                       ((u16)atomic_read(&tpd_ring->next_to_use)));
    //memset(pTpd, 0, sizeof(TpdDescr));
    memset(((u8*)pTpd + sizeof(pTpd->addr)), 0, (sizeof(TpdDescr) - sizeof(pTpd->addr))); //addr don't clear

    next_to_use = (u16)atomic_read(&tpd_ring->next_to_use);
    buffer_info = tpd_ring->buffer_info+next_to_use; 
    
    if (!buffer_info->memDesc)
    {
        DbgPrint("Tx buffer is null!!\n");
        freePacket(m);
        return kIOReturnOutputDropped;
    }
    
    if (mbuf_pkthdr_len(m) <= AT_TX_BUF_LEN) buf_len = mbuf_pkthdr_len(m);
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
        if (mbuf_data(cur_buf)) bcopy(mbuf_data(cur_buf), data_ptr, mbuf_len(cur_buf));
        data_ptr += mbuf_len(cur_buf);
        pkt_snd_len += mbuf_len(cur_buf);
    }
    while(((cur_buf = mbuf_next(cur_buf)) != NULL) && ((pkt_snd_len + mbuf_len(cur_buf)) <= buf_len));
    
    
    buf_len = pkt_snd_len;
    buffer_info->length = (UInt16)buf_len;
    pTpd->buf_len= OSSwapHostToLittleInt16((UInt16)buf_len);
    pTpd->eop = 1;
    
    if(++next_to_use == tpd_ring->count) next_to_use = 0; 
    
    atomic_set(&tpd_ring->next_to_use, next_to_use);
    
    // update mailbox
    at_update_mailbox(adapter);
    
    OSSynchronizeIO();

    freePacket(m);
    return kIOReturnOutputSuccess;
}

IOReturn AtherosL1Ethernet::registerWithPolicyMaker(IOService *policyMaker)
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

IOReturn AtherosL1Ethernet::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
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
IOReturn AtherosL1Ethernet::setPromiscuousMode(bool enabled)
{
    DbgPrint("setPromiscuousMode(), %d\n", enabled);
    
    at_hw *hw = &adapter_.hw;

    u32 rctl;
    
    /* Check for Promiscuous and All Multicast modes */
    
    rctl = AT_READ_REG(hw, REG_MAC_CTRL);
    
    if (enabled){
        rctl |= MAC_CTRL_PROMIS_EN;
    } else  {
        rctl &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
    }
    
    AT_WRITE_REG(hw, REG_MAC_CTRL, rctl);
    
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
IOReturn AtherosL1Ethernet::setMulticastMode(bool enabled)
{
    DbgPrint("setMulticastMode(), %d\n", enabled);
    at_hw *hw = &adapter_.hw;
    u32 rctl;
    
    
    /* Check for Promiscuous and All Multicast modes */
    
    rctl = AT_READ_REG(hw, REG_MAC_CTRL);
    
    if(enabled){
        rctl |= MAC_CTRL_MC_ALL_EN;
        rctl &= ~MAC_CTRL_PROMIS_EN;
    } else {
        rctl &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
    }
    
    AT_WRITE_REG(hw, REG_MAC_CTRL, rctl);
    
    /* clear the old settings from the multicast hash table */
    AT_WRITE_REG(hw, REG_RX_HASH_TABLE, 0);
    AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);

    return kIOReturnSuccess;
}

IOReturn AtherosL1Ethernet::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
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


int AtherosL1Ethernet::mdio_read(int phy_id, int reg_num)
{
    UInt16 result;
    at_read_phy_reg(&adapter_.hw, reg_num & 0x1f, &result);

    return result;
}

void AtherosL1Ethernet::mdio_write(int phy_id, int reg_num, int val)
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
bool AtherosL1Ethernet::atProbe()
{
    u16  vendorId, deviceId;
    at_adapter *adapter=&adapter_;
    
    IOPCIDevice *pdev = adapter_.pdev;
    pdev->setBusMasterEnable(true);
    pdev->setMemoryEnable(true);
    pdev->setIOEnable(true);
    vendorId = pdev->configRead16(kIOPCIConfigVendorID);
    deviceId = pdev->configRead16(kIOPCIConfigDeviceID);

    DbgPrint("Vendor ID %x, device ID %x\n", vendorId, deviceId);
    DbgPrint("MMR0 address %x\n", (u32)pdev->configRead32(kIOPCIConfigBaseAddress0));

    pdev->enablePCIPowerManagement();

    hw_addr_ = pdev->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
    if (hw_addr_ == NULL)
    {
        ErrPrint("Couldn't map io regs\n");
        return false;
    }

    DbgPrint("Memory mapped at bus address %x, virtual address %x, length %d\n", (u32)hw_addr_->getPhysicalAddress(),
                    (u32)hw_addr_->getVirtualAddress(), (u32)hw_addr_->getLength());
    
    
    hw_addr_->retain();

    adapter->hw.mmr_base = reinterpret_cast<char *>(hw_addr_->getVirtualAddress());

    DbgPrint("REG_VPD_CAP = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_VPD_CAP));
    DbgPrint("REG_PCIE_CAP_LIST = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_PCIE_CAP_LIST));
    DbgPrint("REG_MASTER_CTRL = %x\n", OSReadLittleInt32(adapter->hw.mmr_base, REG_MASTER_CTRL));
    
    at_setup_pcicmd(pdev);
    
    /* get user settings */
    at_check_options(adapter);
    
    /* setup the private structure */
    if(at_sw_init(adapter))
    {
        ErrPrint("Couldn't init software\n");
        return false;
    }

    /* Init GPHY as early as possible due to power saving issue  */
    at_phy_init(&adapter->hw);

    /* reset the controller to 
     * put the device in a known good starting state */
    if (at_reset_hw(&adapter->hw))
    {
        ErrPrint("Couldn't reset hardware\n");
        return false;           //TO-DO: Uncomment
    }

    /* copy the MAC address out of the EEPROM */
    at_read_mac_addr(&adapter->hw);

    return true;
}


#pragma mark -
#pragma mark - Atheros L1 Hw methods
#pragma mark -

void AtherosL1Ethernet::atGetAndUpdateLinkStatus()
{       
        at_adapter *adapter=&adapter_;
        struct at_hw *hw = &adapter->hw;
    
        u16 speed, duplex;
        u32 currentMediumIndex = MEDIUM_INDEX_AUTO;

        if(at_get_speed_and_duplex(hw, &speed, &duplex) == AT_SUCCESS)
        {
            DbgPrint("Link is active, speed %d, duplex %d\n", speed, duplex);


            if(speed == SPEED_10 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10FD;
            if(speed == SPEED_100 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100FD;
            if(speed == SPEED_1000 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_1000FD;
            if(speed == SPEED_10 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10HD;
            if(speed == SPEED_100 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100HD;
            if(speed == SPEED_1000 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_1000HD;  

            setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, mediumTable[currentMediumIndex], speed * MBit, NULL); 
            adapter->link_speed = speed;
            adapter->link_duplex = duplex;
            at_setup_mac_ctrl(adapter); 
        } else
        {
            DbgPrint("Link is down\n");
            u32 value;
            //disable rx
            value = AT_READ_REG(hw, REG_MAC_CTRL);
            value &= ~MAC_CTRL_RX_EN;
            AT_WRITE_REG(hw, REG_MAC_CTRL, value);            
            adapter->link_speed = SPEED_0; 
            setLinkStatus(kIONetworkLinkValid, NULL, 0, NULL);
        }
}


/*
 * Configures link settings.
 * Assumes the hardware has previously been reset and the
 * transmitter and receiver are not enabled.
 */
SInt32 AtherosL1Ethernet::atSetupLink()
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
