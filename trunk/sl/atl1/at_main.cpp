/* at_main.cpp -- ATL1 adapter implements
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

#include <IOKit/pci/IOPCIDevice.h>

#include "at_hw.h"
#include "at.h"
#include "at_main.h"

#define ENOMEM    12

void at_setup_pcicmd(IOPCIDevice *pdev)
{
    u16 cmd;

    cmd = pdev->configRead16(kIOPCIConfigCommand) ;

    
    if (cmd & 0x0400)
        cmd &= ~0x0400;
    if (cmd & kIOPCICommandIOSpace)
        cmd &= ~kIOPCICommandIOSpace;
    if (0 == (cmd & kIOPCICommandMemorySpace))
        cmd |= kIOPCICommandMemorySpace;
    if (0 == (cmd & kIOPCICommandBusMaster))
        cmd |= kIOPCICommandBusMaster;
    pdev->configWrite16(kIOPCIConfigCommand, cmd);

    
    /* 
     * some motherboards BIOS(PXE/EFI) driver may set PME
     * while they transfer control to OS (Windows/Linux)
     * so we should clear this bit before NIC work normally
     */
    pdev->configWrite32( REG_PM_CTRLSTAT, 0);
    msec_delay(1);         
}

    

/**
 * at_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/

void
at_irq_enable(struct at_adapter *adapter)
{
    AT_WRITE_REG(&adapter->hw, REG_ISR, 0);
    AT_WRITE_REG(&adapter->hw, REG_IMR, IMR_NORMAL_MASK);
    AT_WRITE_FLUSH(&adapter->hw);
}

/**
 * at_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/

void
at_irq_disable(struct at_adapter *adapter)
{
    AT_WRITE_REG(&adapter->hw, REG_IMR, 0);
    AT_WRITE_FLUSH(&adapter->hw);
}

/**
 * at_sw_init - Initialize general software structures (struct at_adapter)
 * @adapter: board private structure to initialize
 *
 * at_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/

int 
at_sw_init(struct at_adapter *adapter)
{
    struct at_hw *hw = &adapter->hw;
    IOPCIDevice *pdev = adapter->pdev;
    
    /* PCI config space info */
    hw->vendor_id = pdev->configRead16(kIOPCIConfigVendorID);
    hw->device_id = pdev->configRead16(kIOPCIConfigDeviceID);
    hw->subsystem_vendor_id = pdev->configRead16(kIOPCIConfigSubSystemVendorID);
    hw->subsystem_id = pdev->configRead16(kIOPCIConfigSubSystemID);
    
    hw->revision_id = pdev->configRead8(kIOPCIConfigRevisionID);

    /* default is wake on magic */
    adapter->wol = AT_WUFC_MAG;
    
    adapter->ict = 50000;  // 100ms
    
    adapter->link_speed = SPEED_0;   // hardware init
    adapter->link_duplex = FULL_DUPLEX; //
    
    hw->phy_configured = FALSE;
    hw->preamble_len = 7;
    hw->ipgt = 0x60;
    hw->min_ifg = 0x50;
    hw->ipgr1 = 0x40;
    hw->ipgr2 = 0x60;
    hw->max_retry = 0xf;
    hw->lcol = 0x37;
    hw->jam_ipg = 7;
    hw->max_frame_size = 1500;
    adapter->rx_buffer_len = AT_RX_BUF_LEN;    
    
    hw->rfd_burst = 8;
    hw->rrd_burst = 8;
    hw->rfd_fetch_gap = 1;
    hw->rx_jumbo_th = adapter->rx_buffer_len / 8;
    hw->rx_jumbo_lkah = 1;
    hw->rrd_ret_timer = 4; //8us
    
    hw->tpd_burst = 8;
    hw->tpd_fetch_th = 16; 
    hw->txf_burst = 0x200;
    hw->tx_jumbo_task_th = (hw->max_frame_size+
                            ENET_HEADER_SIZE + 
                            VLAN_SIZE +
                            ETHERNET_FCS_SIZE + 7)>>3;
    hw->tpd_fetch_gap = 1;
    
    hw->rcb_value = at_rcb_64;
    hw->dma_ord = at_dma_ord_out; //at_dma_ord_enh;
    hw->dmar_block = at_dma_req_1024;
    hw->dmaw_block = at_dma_req_1024;
    
    hw->cmb_rrd = 4; 
    hw->cmb_tpd = 4;
    hw->cmb_rx_timer = 2; //about 4us
    hw->cmb_tx_timer = 256; //about 512us
    hw->smb_timer = 100000 ; // about 200ms
    
    adapter->num_rx_queues = 1;
    
    atomic_set(&adapter->irq_sem, 1);

    return 0;
}


int
at_reset(struct at_adapter *adapter)
{
    int ret;
    
    if (AT_SUCCESS != (ret = at_reset_hw(&adapter->hw)))
        return ret;
    
    return at_init_hw(&adapter->hw);
}




/**
 * at_setup_mem_resources - allocate Tx / RX descriptor resources 
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 **/

s32
at_setup_ring_resources(struct at_adapter *adapter)
{
    struct at_tpd_ring *tpd_ring = &adapter->tpd_ring;
    struct at_rfd_ring *rfd_ring = &adapter->rfd_ring;
    struct at_rrd_ring *rrd_ring = &adapter->rrd_ring;
    struct at_ring_header * ring_header = &adapter->ring_header;

    int size;
    u8 offset = 0;
    
    //    DEBUGFUNC("at_setup_ring_resources !");
    
    //    DEBUGOUT1("TPD count = %x  RFD cont = %x  RRD = %x", 
    //        tpd_ring->count, rfd_ring->count, rrd_ring->count);
    
    DEBUGOUT1("sz TPD:%d, sz RFD:%d, sz RRD:%d\n", 
              (int)sizeof(TpdDescr),
              (int)sizeof(rx_free_desc_t),
              (int)sizeof(rx_return_desc_t));
    
    size = sizeof(struct at_buffer) * (tpd_ring->count+rfd_ring->count);
    tpd_ring->buffer_info = (at_buffer *)IOMalloc(size);
    if(!tpd_ring->buffer_info) {
        DbgPrint("Couldn't alloc memory for descriptor tpd ring, size = D%d\n", size);
        return -ENOMEM;
    }

    DbgPrint("Allocated memory for descriptor tpd and rfd ring, all size = D%d; tpd_ring->count = D%d; rfd_ring->count = D%d \n",
             size,tpd_ring->count ,rfd_ring->count);
    
    rfd_ring->buffer_info = 
    (struct at_buffer*)(tpd_ring->buffer_info+tpd_ring->count);
    tpd_ring->buffer_ring_size = size;
    memset(tpd_ring->buffer_info, 0, size);
    
    /* real ring DMA buffer */
    ring_header->size = size =  sizeof(TpdDescr ) * tpd_ring->count 
    + sizeof(rx_free_desc_t) *    rfd_ring->count 
    + sizeof(rx_return_desc_t) *  rrd_ring->count
    + sizeof(coals_msg_block_t)
    + sizeof(stats_msg_block_t)
    + 40; // 40: for 8 bytes align
    
    ring_header->memDesc= IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous|
                                                                kIODirectionOut|kIODirectionIn,
                                                                ring_header->size,
                                                                PAGE_SIZE);
    if (!ring_header->memDesc || (ring_header->memDesc->prepare() != kIOReturnSuccess))
    {
        IOSleep(1500);
        if (ring_header->memDesc)
        {
            ring_header->memDesc->release();
            ring_header->memDesc = NULL;
        }
        if (tpd_ring->buffer_info)
        {   
            IOFree(tpd_ring->buffer_info, tpd_ring->buffer_ring_size);
        }
        DbgPrint("Couldn't alloc memory for descriptor ring header, size = D%d\n", ring_header->size);
        return -ENOMEM;
    }
    DbgPrint("Allocated memory for descriptor ring header, size = D%d ; page_size = D%d\n", ring_header->size, PAGE_SIZE);
    
    IOByteCount dmaLength = 0;
    ring_header->dma = ring_header->memDesc->getPhysicalSegment(0, &dmaLength);
    ring_header->desc = ring_header->memDesc->getBytesNoCopy(); 
    DbgPrint("ring header physical segment address %X pointer %p length %d\n", 
             (u32)ring_header->dma, ring_header->desc, (u32)dmaLength);
    //    DEBUGOUT("memory allocated successfully !");    
    
    memset(ring_header->desc, 0, ring_header->size);
    DEBUGOUT1("whole ring dma addr=%x\n", (u32)ring_header->dma);
    
    // init TPD ring
    tpd_ring->dma = ring_header->dma;
    offset = (tpd_ring->dma & 0x7) ? (8 - (ring_header->dma & 0x7)) : 0;
    tpd_ring->dma += offset;
    tpd_ring->desc = (u8*) ring_header->desc + offset;
    tpd_ring->size = sizeof(TpdDescr) * tpd_ring->count;
    DEBUGOUT1("tpd ring dma addr=%x\n", (u32)tpd_ring->dma);
    
    // init RFD ring
    rfd_ring->dma = tpd_ring->dma + tpd_ring->size;
    offset = (rfd_ring->dma & 0x7) ? (8 - (rfd_ring->dma & 0x7)) : 0;
    rfd_ring->dma += offset;
    rfd_ring->desc = (u8*) tpd_ring->desc + (tpd_ring->size+offset);
    rfd_ring->size = sizeof(rx_free_desc_t) * rfd_ring->count;
    DEBUGOUT1("rfd ring dma addr=%x\n", (u32)rfd_ring->dma);
    
    // init RRD ring
    rrd_ring->dma = rfd_ring->dma + rfd_ring->size;
    offset = (rrd_ring->dma & 0x7) ? (8 - (rrd_ring->dma & 0x7)) : 0;
    rrd_ring->dma += offset;
    rrd_ring->desc = (u8*) rfd_ring->desc + (rfd_ring->size+offset);
    rrd_ring->size = sizeof(rx_return_desc_t) * rrd_ring->count;
    DEBUGOUT1("rrd ring dma addr=%x\n", (u32)rrd_ring->dma);
    
    // init CMB
    adapter->cmb.dma = rrd_ring->dma + rrd_ring->size;
    offset = (adapter->cmb.dma & 0x7)? (8-(adapter->cmb.dma & 0x7)) : 0;
    adapter->cmb.dma += offset;
    adapter->cmb.cmb = 
    (coals_msg_block_t*)
    ((u8*)rrd_ring->desc + (rrd_ring->size+offset));
    DEBUGOUT1("cmd dma addr=%x\n", (u32)adapter->cmb.dma);
    
    // init SMB
    adapter->smb.dma = adapter->cmb.dma + sizeof(coals_msg_block_t);
    offset = (adapter->smb.dma&0x7) ? (8-(adapter->smb.dma&0x7)): 0;
    adapter->smb.dma += offset;
    adapter->smb.smb = 
    (stats_msg_block_t*)
    ((u8*)adapter->cmb.cmb + (sizeof(coals_msg_block_t)+offset));
    DEBUGOUT1("smb dma addr=%x\n", (u32)adapter->smb.dma);
    
    return AT_SUCCESS;
}


void
init_ring_ptrs(struct at_adapter *adapter)
{
    struct at_tpd_ring *tpd_ring = &adapter->tpd_ring;
    struct at_rfd_ring *rfd_ring = &adapter->rfd_ring;
    struct at_rrd_ring *rrd_ring = &adapter->rrd_ring;
    
    // Read / Write Ptr Initialize:
    atomic_set(&tpd_ring->next_to_use, 0);
    atomic_set(&tpd_ring->next_to_clean, 0);
    
    rfd_ring->next_to_clean = 0;
    atomic_set(&rfd_ring->next_to_use, 0);
    
    rrd_ring->next_to_use = 0;
    atomic_set(&rrd_ring->next_to_clean, 0);
}


/**
 * at_free_ring_resources - Free Tx / RX descriptor Resources
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/

void
at_free_ring_resources(struct at_adapter *adapter)
{

    struct at_tpd_ring *tpd_ring = &adapter->tpd_ring; 
    struct at_rfd_ring *rfd_ring = &adapter->rfd_ring;
    struct at_rrd_ring * rrd_ring = &adapter->rrd_ring;
    struct at_ring_header * ring_header = &adapter->ring_header;
    
    //    DEBUGFUNC("at_free_ring_resources !");
    
    at_clean_tx_ring(adapter);
    at_clean_rx_ring(adapter);
    
    if (tpd_ring->buffer_info) {
        IOFree(tpd_ring->buffer_info, tpd_ring->buffer_ring_size);
    }
    
    if (ring_header->memDesc) {
        ring_header->memDesc->complete();
        ring_header->memDesc->release();
        ring_header->memDesc = NULL;
    }
    
    tpd_ring->buffer_info = NULL;
    tpd_ring->desc = NULL;
    tpd_ring->dma = 0;
    
    rfd_ring->buffer_info = NULL;
    rfd_ring->desc = NULL;
    rfd_ring->dma = 0;
    
    rrd_ring->desc = NULL;
    rrd_ring->dma = 0;
}


void
at_setup_mac_ctrl(struct at_adapter* adapter)
{
    u32 value;
    struct at_hw* hw = &adapter->hw;
    
    /* Config MAC CTRL Register */
    value = MAC_CTRL_TX_EN | 
    MAC_CTRL_RX_EN ;
    // duplex
    if (FULL_DUPLEX == adapter->link_duplex)
        value |= MAC_CTRL_DUPLX;
    // speed
    value |= ((u32)((SPEED_1000 == adapter->link_speed) ?
                    MAC_CTRL_SPEED_1000:MAC_CTRL_SPEED_10_100)<<MAC_CTRL_SPEED_SHIFT);
    // flow control
    value |= (MAC_CTRL_TX_FLOW|MAC_CTRL_RX_FLOW);
    
    // PAD & CRC
    value |= (MAC_CTRL_ADD_CRC|MAC_CTRL_PAD);
    // preamble length
    value |= (((u32)adapter->hw.preamble_len
               &MAC_CTRL_PRMLEN_MASK)<< MAC_CTRL_PRMLEN_SHIFT);

    
    // filter mode
    value |= MAC_CTRL_BC_EN;

    
    AT_WRITE_REG(hw, REG_MAC_CTRL, value);
}

void set_flow_ctrl_old(struct at_adapter* adapter)
{
    u32 Hi, Lo, value;
    
    // RFD Flow Control   
    value = adapter->rfd_ring.count;
    Hi = value / 16;
    if (Hi < 2) Hi = 2;
    Lo = value * 7 / 8;
    
    value = ((Hi&RXQ_RXF_PAUSE_TH_HI_MASK) << RXQ_RXF_PAUSE_TH_HI_SHIFT)|
    ((Lo&RXQ_RXF_PAUSE_TH_LO_MASK) << RXQ_RXF_PAUSE_TH_LO_SHIFT);
    AT_WRITE_REG(&adapter->hw, REG_RXQ_RXF_PAUSE_THRESH, value);
    
    // RRD Flow Control
    value = adapter->rrd_ring.count;
    Lo = value / 16;
    Hi = value * 7 / 8;
    if (Lo < 2) Lo = 2;
    
    value = ((Hi&RXQ_RRD_PAUSE_TH_HI_MASK) << RXQ_RRD_PAUSE_TH_HI_SHIFT)|
    ((Lo&RXQ_RRD_PAUSE_TH_LO_MASK) << RXQ_RRD_PAUSE_TH_LO_SHIFT);
    AT_WRITE_REG(&adapter->hw, REG_RXQ_RRD_PAUSE_THRESH, value);
}

void set_flow_ctrl_new(struct at_hw* hw)
{
    u32 Hi, Lo, value;
    
    // RXF Flow Control   
    value = AT_READ_REG(hw, REG_SRAM_RXF_LEN);
    Lo = value / 16;
    if (Lo < 192) Lo = 192;
    Hi = value * 7 / 8;
    if (Hi < Lo) Hi = Lo + 16;
    value = ((Hi&RXQ_RXF_PAUSE_TH_HI_MASK) << RXQ_RXF_PAUSE_TH_HI_SHIFT)|
    ((Lo&RXQ_RXF_PAUSE_TH_LO_MASK) << RXQ_RXF_PAUSE_TH_LO_SHIFT);
    AT_WRITE_REG(hw, REG_RXQ_RXF_PAUSE_THRESH, value);
    
    // RRD Flow Control
    value = AT_READ_REG(hw, REG_SRAM_RRD_LEN);
    Lo = value / 8;
    Hi = value * 7 / 8;
    if (Lo < 2) Lo = 2;
    if (Hi < Lo) Hi = Lo + 3;
    value = ((Hi&RXQ_RRD_PAUSE_TH_HI_MASK) << RXQ_RRD_PAUSE_TH_HI_SHIFT)|
    ((Lo&RXQ_RRD_PAUSE_TH_LO_MASK) << RXQ_RRD_PAUSE_TH_LO_SHIFT);
    AT_WRITE_REG(hw, REG_RXQ_RRD_PAUSE_THRESH, value);
}

/**
 * at_configure - Configure Transmit&Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx /Rx unit of the MAC after a reset.
 **/

s32
at_configure(struct at_adapter *adapter)
{
    struct at_hw * hw = &adapter->hw;
    u32 value;
    
    //  DEBUGFUNC("at_configure !");
    // clear interrupt status
    AT_WRITE_REG(&adapter->hw, REG_ISR, 0xffffffff);
    
    // set MAC Address
    value = (((u32)hw->mac_addr[2]) << 24) |
    (((u32)hw->mac_addr[3]) << 16) |
    (((u32)hw->mac_addr[4]) << 8 ) |
    (((u32)hw->mac_addr[5])      ) ;
    AT_WRITE_REG(hw, REG_MAC_STA_ADDR, value);
    value = (((u32)hw->mac_addr[0]) << 8 ) |
    (((u32)hw->mac_addr[1])      ) ;
    AT_WRITE_REG(hw, (REG_MAC_STA_ADDR+4), value);
    
    // tx / rx ring :
    
    // HI base address
    AT_WRITE_REG(
                 hw, 
                 REG_DESC_BASE_ADDR_HI, 
                 (u32)((adapter->tpd_ring.dma&0xffffffff00000000ULL) >>32));
    // LO base address
    AT_WRITE_REG(
                 hw, 
                 REG_DESC_RFD_ADDR_LO, 
                 (u32)(adapter->rfd_ring.dma& 0x00000000ffffffffULL));
    AT_WRITE_REG(
                 hw, 
                 REG_DESC_RRD_ADDR_LO, 
                 (u32)(adapter->rrd_ring.dma& 0x00000000ffffffffULL));
    AT_WRITE_REG(hw, 
                 REG_DESC_TPD_ADDR_LO, 
                 (u32)(adapter->tpd_ring.dma& 0x00000000ffffffffULL));
    AT_WRITE_REG(hw, 
                 REG_DESC_CMB_ADDR_LO, 
                 (u32)(adapter->cmb.dma& 0x00000000ffffffffULL));
    AT_WRITE_REG(hw, 
                 REG_DESC_SMB_ADDR_LO, 
                 (u32)(adapter->smb.dma& 0x00000000ffffffffULL));
    
    // element count
    value = adapter->rrd_ring.count;
    value <<= 16;
    value += adapter->rfd_ring.count;
    AT_WRITE_REG(hw, REG_DESC_RFD_RRD_RING_SIZE, value);
    AT_WRITE_REG(hw, REG_DESC_TPD_RING_SIZE, adapter->tpd_ring.count);
    
    /* 
     // config SRAM
     // add RXF 256*8 bytes   
     value = ((2795 + 256) << 16) | 432;
     AT_WRITE_REG(hw, REG_SRAM_RXF_ADDR, value);
     value = 2364 + 256;
     AT_WRITE_REG(hw, REG_SRAM_RXF_LEN, value);
     // sub TXF 256*8 bytes
     value = (4075 << 16) | (2796 + 256);
     AT_WRITE_REG(hw, REG_SRAM_TXF_ADDR, value);
     value = 1280 - 256;
     AT_WRITE_REG(hw, REG_SRAM_TXF_LEN, value);
     */
    // Load Ptr
    AT_WRITE_REG(hw, REG_LOAD_PTR, 1);
    
    
    /* config Mailbox */
    
    value = 
    (((u32)atomic_read(&adapter->tpd_ring.next_to_use)
      &MB_TPD_PROD_INDX_MASK)<<MB_TPD_PROD_INDX_SHIFT) |
    (((u32)atomic_read(&adapter->rrd_ring.next_to_clean)
      &MB_RRD_CONS_INDX_MASK)<<MB_RRD_CONS_INDX_SHIFT) |
    (((u32)atomic_read(&adapter->rfd_ring.next_to_use)
      &MB_RFD_PROD_INDX_MASK)<<MB_RFD_PROD_INDX_SHIFT);
    AT_WRITE_REG(hw, REG_MAILBOX, value);
    
    //    DEBUGOUT1("init Mailbox with 0x%x", value);
    
    /* config IPG/IFG */
    value = 
    (((u32)hw->ipgt&MAC_IPG_IFG_IPGT_MASK) 
     <<MAC_IPG_IFG_IPGT_SHIFT) |
    (((u32)hw->min_ifg &MAC_IPG_IFG_MIFG_MASK) 
     <<MAC_IPG_IFG_MIFG_SHIFT) |
    (((u32)hw->ipgr1&MAC_IPG_IFG_IPGR1_MASK)
     <<MAC_IPG_IFG_IPGR1_SHIFT)|
    (((u32)hw->ipgr2&MAC_IPG_IFG_IPGR2_MASK)
     <<MAC_IPG_IFG_IPGR2_SHIFT);
    AT_WRITE_REG(hw, REG_MAC_IPG_IFG, value);
    //    DEBUGOUT1("init ipg/ifg with 0x%x", value);
    
    /* config  Half-Duplex Control */
    value = 
    ((u32)hw->lcol&MAC_HALF_DUPLX_CTRL_LCOL_MASK) |
    (((u32)hw->max_retry&MAC_HALF_DUPLX_CTRL_RETRY_MASK)
     <<MAC_HALF_DUPLX_CTRL_RETRY_SHIFT) |
    MAC_HALF_DUPLX_CTRL_EXC_DEF_EN   |
    (0xa<<MAC_HALF_DUPLX_CTRL_ABEBT_SHIFT) |
    (((u32)hw->jam_ipg&MAC_HALF_DUPLX_CTRL_JAMIPG_MASK)
     <<MAC_HALF_DUPLX_CTRL_JAMIPG_SHIFT);
    AT_WRITE_REG(hw, REG_MAC_HALF_DUPLX_CTRL, value);
    //    DEBUGOUT1("init Half Duplex with 0x%x", value);
    
    
    /* set Interrupt Moderator Timer */
    AT_WRITE_REGW(hw, REG_IRQ_MODU_TIMER_INIT, adapter->imt);
    AT_WRITE_REG(hw, REG_MASTER_CTRL, MASTER_CTRL_ITIMER_EN);
    //    DEBUGOUT1("init Irq Modurator Timer with 0x%x", adapter->imt);
    
    /* set Interrupt Clear Timer */
    AT_WRITE_REGW(hw, REG_CMBDISDMA_TIMER, adapter->ict);
    //    DEBUGOUT1("init Irq Clear Timer with 0x%x", adapter->ict);
    
    /* set MTU */
    AT_WRITE_REG(hw, REG_MTU, hw->max_frame_size+
                 ENET_HEADER_SIZE + 
                 VLAN_SIZE +
                 ETHERNET_FCS_SIZE );
    DEBUGOUT1("init MTU with 0x%x\n", hw->max_frame_size); 
    
    // jumbo size & rrd retirement timer
    value = 
    (((u32)hw->rx_jumbo_th&RXQ_JMBOSZ_TH_MASK) 
     << RXQ_JMBOSZ_TH_SHIFT)|
    (((u32)hw->rx_jumbo_lkah&RXQ_JMBO_LKAH_MASK) 
     << RXQ_JMBO_LKAH_SHIFT)|
    (((u32)hw->rrd_ret_timer&RXQ_RRD_TIMER_MASK)
     << RXQ_RRD_TIMER_SHIFT) ;
    AT_WRITE_REG(hw, REG_RXQ_JMBOSZ_RRDTIM, value);
    //    DEBUGOUT1("init RXQ Jumbo size RRD retirement Timer with 0x=%x", value);
    // Flow Control
    switch (hw->dev_rev)        
    {
        case 0x8001: 
        case 0x9001:
        case 0x9002:
        case 0x9003:
            set_flow_ctrl_old(adapter);
            break;
        default:
            set_flow_ctrl_new(hw);
            break;
    }
    
    /* config TXQ */
    value =  
    (((u32)hw->tpd_burst&TXQ_CTRL_TPD_BURST_NUM_MASK)
     << TXQ_CTRL_TPD_BURST_NUM_SHIFT) |
    (((u32)hw->txf_burst&TXQ_CTRL_TXF_BURST_NUM_MASK) 
     << TXQ_CTRL_TXF_BURST_NUM_SHIFT) |
    (((u32)hw->tpd_fetch_th&TXQ_CTRL_TPD_FETCH_TH_MASK)
     << TXQ_CTRL_TPD_FETCH_TH_SHIFT) |
    TXQ_CTRL_ENH_MODE |
    TXQ_CTRL_EN;
    AT_WRITE_REG(hw, REG_TXQ_CTRL, value);
    //    DEBUGOUT1("init TXQ Control with 0x%x", value); 
    
    // min tpd fetch gap & tx jumbo packet size threshold for taskoffload
    value = 
    (((u32)hw->tx_jumbo_task_th&TX_JUMBO_TASK_TH_MASK)
     << TX_JUMBO_TASK_TH_SHIFT) |
    (((u32)hw->tpd_fetch_gap&TX_TPD_MIN_IPG_MASK)
     << TX_TPD_MIN_IPG_SHIFT);
    AT_WRITE_REG(hw, REG_TX_JUMBO_TASK_TH_TPD_IPG, value);
    //    DEBUGOUT1("init TPD fetch gap & TX jumbo taskoffload threshold with 0x%x", value);
    
    /* config RXQ */
    value =  
    (((u32)hw->rfd_burst&RXQ_CTRL_RFD_BURST_NUM_MASK) 
     << RXQ_CTRL_RFD_BURST_NUM_SHIFT)|
    (((u32)hw->rrd_burst&RXQ_CTRL_RRD_BURST_THRESH_MASK)
     << RXQ_CTRL_RRD_BURST_THRESH_SHIFT)|
    (((u32)hw->rfd_fetch_gap&RXQ_CTRL_RFD_PREF_MIN_IPG_MASK)
     << RXQ_CTRL_RFD_PREF_MIN_IPG_SHIFT) |
    RXQ_CTRL_CUT_THRU_EN | 
    RXQ_CTRL_EN ;
    AT_WRITE_REG(hw, REG_RXQ_CTRL, value);
    //     DEBUGOUT1("init RXQ control with 0x%x", value);
    
    /* config  DMA Engine */
    {
        u32 v1;
        u16 max_dmar, max_dmaw, l;
        max_dmar = hw->dmar_block;
        max_dmaw = hw->dmaw_block;
        v1 = AT_READ_REGW(hw, REG_DEVICE_CTRL);
        l = (v1>>DEVICE_CTRL_MAX_PAYLOAD_SHIFT)&DEVICE_CTRL_MAX_PAYLOAD_MASK;
        if (l < max_dmaw)
            max_dmaw = l;
        l = (v1>>DEVICE_CTRL_MAX_RREQ_SZ_SHIFT)&DEVICE_CTRL_MAX_RREQ_SZ_MASK;
        if (l < max_dmar)
            max_dmar = l;
        hw->dmar_block = at_dma_req_block(max_dmar);
        hw->dmaw_block = at_dma_req_block(max_dmaw);
    }      
    
    value = 
    ((((u32)hw->dmar_block)&DMA_CTRL_DMAR_BURST_LEN_MASK) 
     << DMA_CTRL_DMAR_BURST_LEN_SHIFT)|
    ((((u32)hw->dmaw_block)&DMA_CTRL_DMAW_BURST_LEN_MASK) 
     << DMA_CTRL_DMAW_BURST_LEN_SHIFT) |
    DMA_CTRL_DMAR_EN | 
    DMA_CTRL_DMAW_EN;
    value  |= (u32)hw->dma_ord;
    if (at_rcb_128 == hw->rcb_value) 
    {
        value |= DMA_CTRL_RCB_VALUE;
    }
    AT_WRITE_REG(hw, REG_DMA_CTRL, value);
    //    DEBUGOUT1("init DMA Engine with 0x%x", value);
    
    /* config CMB / SMB */
    value = (hw->cmb_tpd > adapter->tpd_ring.count) ?
    hw->cmb_tpd : adapter->tpd_ring.count;
    value <<= 16;
    value |= hw->cmb_rrd;
    AT_WRITE_REG(hw, REG_CMB_WRITE_TH, value);
    value = hw->cmb_rx_timer | ((u32)hw->cmb_tx_timer << 16);
    AT_WRITE_REG(hw, REG_CMB_WRITE_TIMER, value);
    AT_WRITE_REG(hw, REG_SMB_TIMER, hw->smb_timer);
    //    DEBUGOUT1("init CMB Write Timer with 0x%x", value);
    
    // --- enable CMB / SMB 
    value = CSMB_CTRL_CMB_EN | CSMB_CTRL_SMB_EN;
    AT_WRITE_REG(hw, REG_CSMB_CTRL, value);
    
    value = AT_READ_REG(&adapter->hw, REG_ISR);
    if ((value&ISR_PHY_LINKDOWN) != 0) {
        value = 1; // config failed 
    } else {
        value = 0;
    }
    // clear all interrupt status
    AT_WRITE_REG(&adapter->hw, REG_ISR, 0x3fffffff);
    AT_WRITE_REG(&adapter->hw, REG_ISR, 0);
    return value;
}

/**
 * at_alloc_tx_buffers - Alloc Tx-skb
 * @adapter: address of board private structure
 **/
u16 at_alloc_tx_buffers(struct at_adapter *adapter)
{
    struct at_tpd_ring *tpd_ring = &adapter->tpd_ring;
    struct at_buffer *buffer_info;
    
    u16 index, ring_count;
    TpdDescr *tpd_desc;
    u16 num_alloc = 0;
    
    ring_count = tpd_ring->count;
    for (index = 0; index < ring_count; index++) {
        buffer_info = &tpd_ring->buffer_info[index];        
        tpd_desc= AT_TPD_DESC(tpd_ring, index);
        
        buffer_info->memDesc = IOBufferMemoryDescriptor::withOptions(0,
                                                                     AT_TX_BUF_LEN);
        
        if (!buffer_info->memDesc || buffer_info->memDesc->prepare() != kIOReturnSuccess)
        {
            ErrPrint("Couldn't allocate o prepare memory for transmitting\n");
            break;
        }
        
        IOByteCount length;
        buffer_info->dma = buffer_info->memDesc->getPhysicalSegment(0, &length);
        buffer_info->length = length;
        tpd_desc->addr = OSSwapHostToLittleInt64(buffer_info->dma);
        num_alloc++;
    }
    DbgPrint("Total allocated space for tx descriptors D%d; num_alloc= D%d; buffer_len= D%d\n",
             num_alloc * AT_TX_BUF_LEN, num_alloc , AT_TX_BUF_LEN);
    
    return num_alloc;
}

/**
 * at_alloc_rx_buffers - Replace used receive buffers
 * @adapter: address of board private structure
 **/

u16
at_alloc_rx_buffers(struct at_adapter *adapter)
{
    struct at_rfd_ring *rfd_ring = &adapter->rfd_ring;
    struct at_buffer *buffer_info, * next_info;
    u16 num_alloc = 0;
    u16 rfd_next_to_use, next_next;
    rx_free_desc_t *rfd_desc;
    
    next_next = rfd_next_to_use = (u16)atomic_read(&rfd_ring->next_to_use);
    if (++next_next == rfd_ring->count)      next_next = 0;
    buffer_info = &rfd_ring->buffer_info[rfd_next_to_use];
    next_info = &rfd_ring->buffer_info[next_next];
    
    while (!buffer_info->alloced && !next_info->alloced) {
        if (buffer_info->memDesc) {
            buffer_info->alloced = 1;
            //DEBUGOUT1("skip rfd allocate (%d)", rfd_next_to_use);
            goto next;
        }
        
        rfd_desc = AT_RFD_DESC(rfd_ring, rfd_next_to_use);
        
        buffer_info->memDesc = IOBufferMemoryDescriptor::withOptions(0,adapter->rx_buffer_len);
        if (!buffer_info->memDesc || buffer_info->memDesc->prepare() != kIOReturnSuccess)
        {
            ErrPrint("alloc rx buffer failed\n");
            break;
        }
        
        IOByteCount length;
        buffer_info->dma = buffer_info->memDesc->getPhysicalSegment(0, &length);
        buffer_info->alloced = 1;
        buffer_info->length = (u16)adapter->rx_buffer_len;
        
        rfd_desc->buffer_addr = OSSwapHostToLittleInt64(buffer_info->dma);
        rfd_desc->buf_len = OSSwapHostToLittleInt16(adapter->rx_buffer_len);
        rfd_desc->coalese = 0;
        
    next:
        rfd_next_to_use = next_next;
        if (++next_next == rfd_ring->count)     next_next = 0;
        
        buffer_info = &rfd_ring->buffer_info[rfd_next_to_use];
        next_info = &rfd_ring->buffer_info[next_next];
        num_alloc++;
    }
    
    if (0 != num_alloc) {
        /* Force memory writes to complete before letting h/w
         * know there are new descriptors to fetch.  (Only
         * applicable for weak-ordered memory model archs,
         * such as IA-64). */
        //wmb();
        atomic_set(&rfd_ring->next_to_use, (int)rfd_next_to_use);
        DbgPrint("Total allocated space for rx descriptors D%d; num_alloc= D%d; adapter->rx_buffer_len= D%d\n",
            num_alloc * adapter->rx_buffer_len, num_alloc ,adapter->rx_buffer_len);
    }
    return num_alloc;
}


void
at_read_pci_cfg(struct at_hw *hw, u32 reg, u16 *value)
{
    struct at_adapter *adapter = hw->back;
    
    *value = adapter->pdev->configRead16(reg);
}

void
at_write_pci_cfg(struct at_hw *hw, u32 reg, u16 *value)
{
    struct at_adapter *adapter = hw->back;
    
    adapter->pdev->configWrite16(reg, *value);
}

/**
 * at_clean_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 **/

void
at_clean_tx_ring(struct at_adapter *adapter)
{
    struct at_tpd_ring *tpd_ring = &adapter->tpd_ring;
    struct at_buffer *buffer_info;
    unsigned long size;
    unsigned int i;
    
    //    DEBUGFUNC("at_clean_tx_ring !");
    
    if (NULL == tpd_ring->buffer_info ||
        NULL == tpd_ring->desc)
        return;
    
    /* Free all the Tx ring sk_buffs */
    
    for(i = 0; i < tpd_ring->count; i++) {
        buffer_info = &tpd_ring->buffer_info[i];
        if (buffer_info->dma) {
            buffer_info->dma = 0;
        }
        if (buffer_info->memDesc) {
            buffer_info->memDesc->complete();
            buffer_info->memDesc->release();
            buffer_info->memDesc = NULL;
        }
    }
    
    size = sizeof(struct at_buffer) * tpd_ring->count;
    memset(tpd_ring->buffer_info, 0, size);
    
    /* Zero out the descriptor ring */
    
    memset(tpd_ring->desc, 0, tpd_ring->size);
    
    atomic_set(&tpd_ring->next_to_use, 0);
    atomic_set(&tpd_ring->next_to_clean, 0);
}

/**
 * at_clean_rx_ring - Free RFD Buffers
 * @adapter: board private structure
 **/

void
at_clean_rx_ring(struct at_adapter *adapter)
{
    struct at_rfd_ring *rfd_ring = &adapter->rfd_ring;
    struct at_rrd_ring *rrd_ring = &adapter->rrd_ring;
    struct at_buffer *buffer_info;
    unsigned long size;
    unsigned int i;
    
    //    DEBUGFUNC("at_clean_rx_ring !");
    
    if (NULL == rfd_ring->buffer_info ||
        NULL == rfd_ring->desc)
        return;
    
    /* Free all the Rx ring sk_buffs */
    
    for(i = 0; i < rfd_ring->count; i++) {
        buffer_info = &rfd_ring->buffer_info[i];
        if(buffer_info->dma) {
            buffer_info->dma = 0;
        }
        if (buffer_info->memDesc) {
            buffer_info->memDesc->complete();
            buffer_info->memDesc->release();
            buffer_info->memDesc = NULL;
        }
    }
    
    size = sizeof(struct at_buffer) * rfd_ring->count;
    memset(rfd_ring->buffer_info, 0, size);
    
    /* Zero out the descriptor ring */
    
    memset(rfd_ring->desc, 0, rfd_ring->size);
    
    rfd_ring->next_to_clean = 0;
    atomic_set(&rfd_ring->next_to_use, 0);
    
    rrd_ring->next_to_use = 0;
    atomic_set(&rrd_ring->next_to_clean, 0);
}


void
at_clear_phy_int(struct at_adapter* adapter)
{
    u16 phy_data;
    
    at_read_phy_reg(&adapter->hw, 19, &phy_data);
}

u16 tpd_avail(struct at_tpd_ring *tpd_ring)
{
    u16 next_to_clean = (u16)atomic_read(&tpd_ring->next_to_clean);
    u16 next_to_use = (u16)atomic_read(&tpd_ring->next_to_use);
    return ((next_to_clean > next_to_use) ? 
            next_to_clean - next_to_use - 1 :
            tpd_ring->count + next_to_clean - next_to_use - 1);
}

void
at_update_mailbox(struct at_adapter* adapter)
{
    u32 tpd_next_to_use;
    u32 rfd_next_to_use;
    u32 rrd_next_to_clean;
    
    tpd_next_to_use = atomic_read(&adapter->tpd_ring.next_to_use);
    rfd_next_to_use = (u32)atomic_read(&adapter->rfd_ring.next_to_use);
    rrd_next_to_clean = (u32)atomic_read(&adapter->rrd_ring.next_to_clean);
    
    AT_WRITE_REG(&adapter->hw, REG_MAILBOX, 
                 ((rfd_next_to_use & MB_RFD_PROD_INDX_MASK) << MB_RFD_PROD_INDX_SHIFT) |
                 ((rrd_next_to_clean & MB_RRD_CONS_INDX_MASK) << MB_RRD_CONS_INDX_SHIFT) |
                 ((tpd_next_to_use & MB_TPD_PROD_INDX_MASK) << MB_TPD_PROD_INDX_SHIFT) );
    
}


void at_inc_smb(struct at_adapter * adapter)
{
    stats_msg_block_t* smb = adapter->smb.smb;
    
    /* Fill out the OS statistics structure */
    adapter->soft_stats.rx_packets          += smb->rx_ok;
    adapter->soft_stats.tx_packets          += smb->tx_ok;
    adapter->soft_stats.rx_bytes            += smb->rx_byte_cnt;
    adapter->soft_stats.tx_bytes            += smb->tx_byte_cnt;
    adapter->soft_stats.multicast           += smb->rx_mcast;
    adapter->soft_stats.collisions          += (smb->tx_1_col + 
                                                smb->tx_2_col*2 + 
                                                smb->tx_late_col + 
                                                smb->tx_abort_col * adapter->hw.max_retry);
    
    /* Rx Errors */
    
    adapter->soft_stats.rx_errors           += (smb->rx_frag +
                                                smb->rx_fcs_err + 
                                                smb->rx_len_err + 
                                                smb->rx_sz_ov + 
                                                smb->rx_rxf_ov + 
                                                smb->rx_rrd_ov +
                                                smb->rx_align_err );
    adapter->soft_stats.rx_fifo_errors      += smb->rx_rxf_ov;
    adapter->soft_stats.rx_length_errors    += smb->rx_len_err;
    adapter->soft_stats.rx_crc_errors       += smb->rx_fcs_err;
    adapter->soft_stats.rx_frame_errors     += smb->rx_align_err;
    adapter->soft_stats.rx_missed_errors    += (smb->rx_rrd_ov + 
                                                smb->rx_rxf_ov);
    
    adapter->soft_stats.rx_pause            += smb->rx_pause;
    adapter->soft_stats.rx_rrd_ov           += smb->rx_rrd_ov; 
    adapter->soft_stats.rx_trunc            += smb->rx_sz_ov;                                               
    
    /* Tx Errors */
    
    adapter->soft_stats.tx_errors           += (smb->tx_late_col +
                                                smb->tx_abort_col +
                                                smb->tx_underrun + 
                                                smb->tx_trunc );
    adapter->soft_stats.tx_fifo_errors      += smb->tx_underrun;
    adapter->soft_stats.tx_aborted_errors   += smb->tx_abort_col;
    adapter->soft_stats.tx_window_errors    += smb->tx_late_col;
    
    adapter->soft_stats.excecol             += smb->tx_abort_col;
    adapter->soft_stats.deffer              += smb->tx_defer;
    adapter->soft_stats.scc                 += smb->tx_1_col;
    adapter->soft_stats.mcc                 += smb->tx_2_col;
    adapter->soft_stats.latecol             += smb->tx_late_col;
    adapter->soft_stats.tx_underun          += smb->tx_underrun;
    adapter->soft_stats.tx_trunc            += smb->tx_trunc;
    adapter->soft_stats.tx_pause            += smb->tx_pause;
    
    
    // finally  

    adapter->net_stats->inputPackets        = adapter->soft_stats.rx_packets;                  
    adapter->net_stats->outputPackets       = adapter->soft_stats.tx_packets;                     
    adapter->net_stats->collisions          = adapter->soft_stats.collisions;          
    adapter->net_stats->inputErrors         = adapter->soft_stats.rx_errors;          
    adapter->net_stats->outputErrors        = adapter->soft_stats.tx_errors;           
    
}


/**
 * at_phy_config - Timer Call-back
 * @data: pointer to netdev cast into an unsigned long
 **/

static void
at_phy_config(unsigned long data)
{
    struct at_adapter *adapter = (struct at_adapter *) data;
    struct at_hw *hw = &adapter->hw; 
    
    DEBUGFUNC("at_phy_reconfig!\n");

    at_write_phy_reg(hw, MII_ADVERTISE, hw->mii_autoneg_adv_reg);
    at_write_phy_reg(hw, MII_AT001_CR, hw->mii_1000t_ctrl_reg);
    
    DEBUGOUT("4 register written\n");
    at_write_phy_reg(hw, MII_BMCR, MII_CR_RESET|MII_CR_AUTO_NEG_EN|MII_CR_RESTART_AUTO_NEG);

}
