/* at_main.cpp -- ATL1e adapter implements
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

#include <IOKit/pci/IOPCIDevice.h>

#include "at_hw.h"
#include "at.h"
#include "at_main.h"

#define AT_ERR_ENOMEM    12

void at_setup_pcicmd(IOPCIDevice	*pdev)
{
    u16 cmd;

    pdev->configRead16(kIOPCIConfigCommand) ;

    
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
at_irq_enable(at_adapter *adapter)
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
at_irq_disable(at_adapter *adapter)
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
at_sw_init(at_adapter *adapter)
{
    at_hw *hw = &adapter->hw;
    IOPCIDevice *pdev = adapter->pdev;
    u32 val;

    /* PCI config space info */

    hw->vendor_id = pdev->configRead16(kIOPCIConfigVendorID);
    hw->device_id = pdev->configRead16(kIOPCIConfigDeviceID);
    hw->subsystem_vendor_id = pdev->configRead16(kIOPCIConfigSubSystemVendorID);
    hw->subsystem_id = pdev->configRead16(kIOPCIConfigSubSystemID);

	hw->revision_id = pdev->configRead8(kIOPCIConfigRevisionID);
	hw->pci_cmd_word = pdev->configRead16(kIOPCIConfigCommand);

    val = AT_READ_REG(hw, REG_PHY_STATUS);

    /* nic type */
    if (hw->revision_id >= 0xF0) {
    	hw->nic_type = athr_l2e_revB;
    	DEBUGOUT("L2e version B\n");
    } else {
    	if (val&PHY_STATUS_100M) {
    		hw->nic_type = athr_l1e;
    		DEBUGOUT("L1e !!!!\n");
    	} else {
    		hw->nic_type = athr_l2e_revA;
    		DEBUGOUT("L2e version A\n");
		}
	}
    if (val&PHY_STATUS_EMI_CA) 
        hw->emi_ca = true;
    else
        hw->emi_ca = false;
        
 
    adapter->wol = AT_WUFC_MAG;

    adapter->ict = 50000;  // 100ms
    
    adapter->link_speed = SPEED_0;   // hardware init
    adapter->link_duplex = FULL_DUPLEX; //

    hw->rrs_type = at_rrs_disable;
    hw->phy_configured = false;
    hw->preamble_len = 7;

    hw->smb_timer = 200000;    

    hw->max_frame_size = 1500;
    
    hw->dmar_block = at_dma_req_1024;
    hw->dmaw_block = at_dma_req_1024;
    
    hw->rx_jumbo_th = (hw->max_frame_size+
                            ENET_HEADER_SIZE + 
                            VLAN_SIZE +
                            ETHERNET_FCS_SIZE + 7)>>3;                         
    hw->indirect_tab = 0;
    hw->base_cpu = 0;
    hw->rrs_type = at_rrs_disable;

    adapter->num_rx_queues = 1;


    atomic_set(&adapter->irq_sem, 1);
    adapter->stats_lock = IOSimpleLockAlloc();
    adapter->tx_lock = IOSimpleLockAlloc();
    
    return 0;
}

 
int
at_reset(at_adapter *adapter)
{
    int ret;
    
    if (AT_SUCCESS != (ret = at_reset_hw(adapter)))
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
at_setup_ring_resources(at_adapter *adapter)
{
    int size, i;
    u32 offset = 0;
    u32 page_size;

    DEBUGFUNC("at_setup_ring_resources()\n");

    /* real ring DMA buffer */
    page_size = adapter->rxf_length 
            + adapter->hw.max_frame_size 
            + ENET_HEADER_SIZE + VLAN_SIZE + ETHERNET_FCS_SIZE;
    page_size = (page_size + 31) & 0xFFFFFFE0;
            
    adapter->ring_size = size =   
          adapter->tpd_ring_size * sizeof(TpdDescr) + 7 // qword align
        + page_size * 2 * adapter->num_rx_queues + 31 // 32bytes algin
        + (1 + 2 * adapter->num_rx_queues) * 4 + 3; // cmb dma address
	
	adapter->memDesc = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous|
																 kIODirectionOut|kIODirectionIn,
																	size,
																	PAGE_SIZE);
	if (!adapter->memDesc || (adapter->memDesc->prepare() != kIOReturnSuccess))
	{
		IOSleep(1500);
		if (adapter->memDesc)
		{
			adapter->memDesc->release();
			adapter->memDesc = NULL;
		}
		DEBUGOUT1("Couldn't alloc memory for descriptor ring, size = D%d\n", size);
        return AT_ERR_ENOMEM;
	}
	DbgPrint("Allocated memory for descriptor ring, size = D%d ; page_size = D%d\n", size, page_size);

	IOByteCount dmaLength = 0;
	adapter->ring_dma = adapter->memDesc->getPhysicalSegment(0, &dmaLength);
	adapter->ring_vir_addr = adapter->memDesc->getBytesNoCopy();
    DbgPrint("ring Physical segment address %X pointer %p length %d\n",
		adapter->ring_dma, adapter->ring_vir_addr, dmaLength);
		
    memset(adapter->ring_vir_addr, 0, adapter->ring_size);
    
    // tx buffer_infos
    size = sizeof(at_buffer) * (adapter->tpd_ring_size);
    adapter->tx_buffer_info =  (at_buffer *)IOMalloc(size);
    if(!adapter->tx_buffer_info) {

		IOSleep(1500);
		if (adapter->memDesc)
		{
			adapter->memDesc->complete();
			adapter->memDesc->release();
			adapter->memDesc = NULL;
		}
		adapter->ring_vir_addr = NULL;
		adapter->ring_dma= 0;
        DEBUGOUT1("kmalloc failed , size = D%d\n", size);
        return AT_ERR_ENOMEM;
    }
    memset(adapter->tx_buffer_info, 0, size);
    


    // Init TPD Ring
    adapter->tpd_ring_dma = adapter->ring_dma;
    offset = (adapter->tpd_ring_dma & 0x7) ? 
                (8 - (adapter->tpd_ring_dma & 0x7)) : 0;
    adapter->tpd_ring_dma += offset;
    adapter->tpd_ring = (TpdDescr*) ((u8*)adapter->ring_vir_addr + offset);

    
    // Init RXF-Pages
    offset += (sizeof(TpdDescr) * adapter->tpd_ring_size);
    offset = (offset + 31) & 0xFFFFFFE0;
    
    for (i=0; i < adapter->num_rx_queues; i++) {
        adapter->rxf_page[i][0].dma = 
            adapter->ring_dma + (offset + i * 2 * page_size);
        adapter->rxf_page[i][0].addr = 
            (u8*)adapter->ring_vir_addr + (offset + i * 2 * page_size);
        
        adapter->rxf_page[i][1].dma = 
            adapter->rxf_page[i][0].dma + page_size;
        adapter->rxf_page[i][1].addr = 
            adapter->rxf_page[i][0].addr + page_size;
    }
    
    // Init CMB dma address
    offset += page_size * 2 * adapter->num_rx_queues;
    adapter->tpd_cmb_dma = adapter->ring_dma + offset;
    adapter->tpd_cmb = (u32*)( (u8*)adapter->ring_vir_addr + offset);
    offset += 4;
    for (i=0; i < adapter->num_rx_queues; i++) {
        adapter->rxf_page[i][0].WptrPhyAddr = adapter->ring_dma + offset;
        adapter->rxf_page[i][0].pWptr = (u32*)((u8*)adapter->ring_vir_addr + offset);
        offset += 4;
        adapter->rxf_page[i][1].WptrPhyAddr = adapter->ring_dma + offset;
        adapter->rxf_page[i][1].pWptr = (u32*)((u8*)adapter->ring_vir_addr + offset);
        offset += 4;
    }
            
    if (offset > adapter->ring_size) {
        DEBUGOUT1("offset(%d) > ring size(%d) !!\n", 
            offset, adapter->ring_size);
    }
    
	DbgPrint("dma registers ring=%X tpd_ring=%X rxf_page[0][0]=%X rxf_page[0][1]=%X tpd_cmb=%X tpd=%X \n",
			adapter->ring_dma,  adapter->tpd_ring_dma,
			adapter->rxf_page[0][0].dma, adapter->rxf_page[0][1].dma,
			adapter->tpd_cmb_dma,adapter->tpd_cmb);      
    // Read / Write Ptr Initialize:
    //   init_ring_ptrs(adapter);

    return AT_SUCCESS;
}


void
init_ring_ptrs(at_adapter *adapter)
{
    int i;
    // Read / Write Ptr Initialize:
    
    adapter->tpd_next_use = 0;
    atomic_set(&adapter->tpd_next_clean, 0);
    for (i=0; i < adapter->num_rx_queues; i++) {
        adapter->rxf_using[i] = 0;
        *adapter->rxf_page[i][0].pWptr = 0;
        *adapter->rxf_page[i][1].pWptr = 0;
        adapter->rxf_page[i][0].Rptr = 0;
        adapter->rxf_page[i][1].Rptr = 0;
        adapter->rxf_nxseq[i] = 0;
    }
}

/**
 * at_free_ring_resources - Free Tx / RX descriptor Resources
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/

void
at_free_ring_resources(at_adapter *adapter)
{
    int size; 
    
	DEBUGFUNC("at_free_ring_resources()\n");

    at_clean_tx_ring(adapter);
    at_clean_rx_ring(adapter);
    
    if (adapter->ring_vir_addr) {
		if (adapter->memDesc)
		{
			adapter->memDesc->complete();
			adapter->memDesc->release();
			adapter->memDesc = NULL;
		}
		adapter->ring_vir_addr = NULL;
		adapter->ring_dma= 0;          
    }

	size = sizeof(at_buffer) * (adapter->tpd_ring_size);   
    if (adapter->tx_buffer_info) {
        IOFree(adapter->tx_buffer_info, size);
        adapter->tx_buffer_info = NULL;
    }
         
}

void
at_setup_mac_ctrl(at_adapter* adapter)
{
    u32 value;
    at_hw* hw = &adapter->hw;
   
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
    // vlan 
    
    // filter mode
    value |= MAC_CTRL_BC_EN;
    AT_WRITE_REG(hw, REG_MAC_CTRL, value);
}



static u16 gPayLoadSize[] = {
    128, 256, 512, 1024, 2048, 4096,
};
/**
 * at_configure - Configure Transmit&Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx /Rx unit of the MAC after a reset.
 **/

s32
at_configure(at_adapter *adapter)
{
    at_hw * hw = &adapter->hw;
    u32 value, hi;
    
    DEBUGFUNC("at_configure()!\n");

    // clear interrupt status
    AT_WRITE_REG(&adapter->hw, REG_ISR, 0xffffffff);

    // 1. set MAC Address
    value = (((u32)hw->mac_addr[2]) << 24) |
            (((u32)hw->mac_addr[3]) << 16) |
            (((u32)hw->mac_addr[4]) << 8 ) |
            (((u32)hw->mac_addr[5])      ) ;
    AT_WRITE_REG(hw, REG_MAC_STA_ADDR, value);
    value = (((u32)hw->mac_addr[0]) << 8 ) |
            (((u32)hw->mac_addr[1])      ) ;
    AT_WRITE_REG(hw, (REG_MAC_STA_ADDR+4), value);

    // 2. Init the Multicast HASH table
    // done by set_muti

    // 3. Clear any WOL status
    value = AT_READ_REG(hw, REG_WOL_CTRL);
    AT_WRITE_REG(hw, REG_WOL_CTRL, 0);
    
    // 4. Descripter Ring BaseMem/Length/Read ptr/Write ptr */
    // Descripter Ring BaseMem/Length/Read ptr/Write ptr 
    // TPD Ring/SMB/RXF0 Page CMBs, they use the same High 32bits memory
    AT_WRITE_REG(hw, REG_DESC_BASE_ADDR_HI, (u32)((adapter->ring_dma&0xffffffff00000000ULL) >>32));
    AT_WRITE_REG(hw, REG_TPD_BASE_ADDR_LO, (u32)(adapter->tpd_ring_dma));
    AT_WRITE_REG(hw, REG_TPD_RING_SIZE, (u16)(adapter->tpd_ring_size));
    AT_WRITE_REG(hw, REG_HOST_TX_CMB_LO, (u32)adapter->tpd_cmb_dma);
    // RXF Page Physical address / Page Length
    // RXF0
    AT_WRITE_REG(hw, REG_HOST_RXF0_PAGE0_LO, (u32)(adapter->rxf_page[0][0].dma));
    AT_WRITE_REG(hw, REG_HOST_RXF0_PAGE1_LO, (u32)(adapter->rxf_page[0][1].dma));
    AT_WRITE_REG(hw, REG_HOST_RXF0_MB0_LO, (u32)(adapter->rxf_page[0][0].WptrPhyAddr));
    AT_WRITE_REG(hw, REG_HOST_RXF0_MB1_LO, (u32)(adapter->rxf_page[0][1].WptrPhyAddr));
    AT_WRITE_REGB(hw, REG_HOST_RXF0_PAGE0_VLD, 1);
    AT_WRITE_REGB(hw, REG_HOST_RXF0_PAGE1_VLD, 1);
    // RXF1
    AT_WRITE_REG(hw, REG_RXF1_BASE_ADDR_HI, (u32)((adapter->ring_dma&0xffffffff00000000ULL) >>32));
    AT_WRITE_REG(hw, REG_HOST_RXF1_PAGE0_LO, (u32)(adapter->rxf_page[1][0].dma));
    AT_WRITE_REG(hw, REG_HOST_RXF1_PAGE1_LO, (u32)(adapter->rxf_page[1][1].dma));
    AT_WRITE_REG(hw, REG_HOST_RXF1_MB0_LO, (u32)(adapter->rxf_page[1][0].WptrPhyAddr));
    AT_WRITE_REG(hw, REG_HOST_RXF1_MB1_LO, (u32)(adapter->rxf_page[1][1].WptrPhyAddr));  
    AT_WRITE_REGB(hw, REG_HOST_RXF1_PAGE0_VLD, 1);
    AT_WRITE_REGB(hw, REG_HOST_RXF1_PAGE1_VLD, 1);        
    // RXF2
    AT_WRITE_REG(hw, REG_RXF1_BASE_ADDR_HI, (u32)((adapter->ring_dma&0xffffffff00000000ULL) >>32));
    AT_WRITE_REG(hw, REG_HOST_RXF2_PAGE0_LO, (u32)(adapter->rxf_page[2][0].dma));
    AT_WRITE_REG(hw, REG_HOST_RXF2_PAGE1_LO, (u32)(adapter->rxf_page[2][1].dma));
    AT_WRITE_REG(hw, REG_HOST_RXF2_MB0_LO, (u32)(adapter->rxf_page[2][0].WptrPhyAddr));
    AT_WRITE_REG(hw, REG_HOST_RXF2_MB1_LO, (u32)(adapter->rxf_page[2][1].WptrPhyAddr)); 
    AT_WRITE_REGB(hw, REG_HOST_RXF2_PAGE0_VLD, 1);
    AT_WRITE_REGB(hw, REG_HOST_RXF2_PAGE1_VLD, 1);    
    // RXF3
    AT_WRITE_REG(hw, REG_RXF1_BASE_ADDR_HI, (u32)((adapter->ring_dma&0xffffffff00000000ULL) >>32));
    AT_WRITE_REG(hw, REG_HOST_RXF3_PAGE0_LO, (u32)(adapter->rxf_page[3][0].dma));
    AT_WRITE_REG(hw, REG_HOST_RXF3_PAGE1_LO, (u32)(adapter->rxf_page[3][1].dma));
    AT_WRITE_REG(hw, REG_HOST_RXF3_MB0_LO, (u32)(adapter->rxf_page[3][0].WptrPhyAddr));
    AT_WRITE_REG(hw, REG_HOST_RXF3_MB1_LO, (u32)(adapter->rxf_page[3][1].WptrPhyAddr)); 
    AT_WRITE_REGB(hw, REG_HOST_RXF3_PAGE0_VLD, 1);
    AT_WRITE_REGB(hw, REG_HOST_RXF3_PAGE1_VLD, 1);  
    // Page Length
    AT_WRITE_REG(hw, REG_HOST_RXFPAGE_SIZE, adapter->rxf_length);
    // Load all of base address above
    AT_WRITE_REG(hw, REG_LOAD_PTR, 1);


    // 5. set Interrupt Moderator Timer
    AT_WRITE_REGW(hw, REG_IRQ_MODU_TIMER_INIT, adapter->imt);
    AT_WRITE_REGW(hw, REG_IRQ_MODU_TIMER2_INIT, adapter->imt);
    AT_WRITE_REG(hw, REG_MASTER_CTRL, 
                       MASTER_CTRL_LED_MODE|MASTER_CTRL_ITIMER_EN|MASTER_CTRL_ITIMER2_EN);
    
    // 13. rx/tx threshold to trig interrupt
    AT_WRITE_REGW(hw, REG_TRIG_RRD_THRESH, 1); // 1 packets
    AT_WRITE_REGW(hw, REG_TRIG_TPD_THRESH, (u16)(adapter->tpd_ring_size/2));
    AT_WRITE_REGW(hw, REG_TRIG_RXTIMER, 4);   // 10us
    AT_WRITE_REGW(hw, REG_TRIG_TXTIMER, (u16)(adapter->imt*4/3)); // 


    // 6. set Interrupt Clear Timer
    AT_WRITE_REGW(hw, REG_CMBDISDMA_TIMER, 10000);
    

    // 7. Enable Read-Clear Interrupt Mechanism
/*
    O_32(pL1e->MemBase+REG_MASTER_CTRL, 
            MASTER_CTRL_INT_RDCLR|MASTER_CTRL_ITIMER_EN|MASTER_CTRL_ITIMER2_EN);
*/        
    // 8. set MTU
    AT_WRITE_REG(hw, REG_MTU, 
            hw->max_frame_size +
            ENET_HEADER_SIZE + 
            VLAN_SIZE +
            ETHERNET_FCS_SIZE);

    // 9. config TXQ
    // early tx threshold
    if (hw->nic_type != athr_l2e_revB) {
	    if (hw->max_frame_size <= 1500)
	        value = hw->max_frame_size + ENET_HEADER_SIZE + VLAN_SIZE + ETHERNET_FCS_SIZE ;
	    else if (hw->max_frame_size < 6*1024)
	        value = (hw->max_frame_size + ENET_HEADER_SIZE + VLAN_SIZE + ETHERNET_FCS_SIZE) * 2 / 3;
	    else
	        value = (hw->max_frame_size + ENET_HEADER_SIZE + VLAN_SIZE + ETHERNET_FCS_SIZE) / 2;
	    AT_WRITE_REG(hw, REG_TX_EARLY_TH, (value + 7) >> 3);
	}
	
    // tx control
    value = AT_READ_REG(hw, REG_DEVICE_CTRL);
    hi = (value>>DEVICE_CTRL_MAX_PAYLOAD_SHIFT)&DEVICE_CTRL_MAX_PAYLOAD_MASK;
    if (hi < (u32)hw->dmaw_block)   
        hw->dmaw_block = (at_dma_req_block) hi;
    hi = (value>>DEVICE_CTRL_MAX_RREQ_SZ_SHIFT)&DEVICE_CTRL_MAX_RREQ_SZ_MASK;
    if (hi < (u32)hw->dmar_block)
        hw->dmar_block = (at_dma_req_block) hi;

	if (hw->nic_type != athr_l2e_revB) {
		AT_WRITE_REGW(hw, REG_TXQ_CTRL+2, gPayLoadSize[hw->dmar_block]);
	}

    AT_WRITE_REGW(hw, REG_TXQ_CTRL,  
        (((u16)5&TXQ_CTRL_NUM_TPD_BURST_MASK) << TXQ_CTRL_NUM_TPD_BURST_SHIFT) |
        TXQ_CTRL_ENH_MODE | TXQ_CTRL_EN);
        
    // 10. config RXQ
 
    if (hw->nic_type != athr_l2e_revB) {    
    	// jumbo size cut-through  
    	AT_WRITE_REGW(hw, REG_RXQ_JMBOSZ_RRDTIM, 
        	(u16) ((hw->rx_jumbo_th&RXQ_JMBOSZ_TH_MASK) << RXQ_JMBOSZ_TH_SHIFT |
            	   (1&RXQ_JMBO_LKAH_MASK) << RXQ_JMBO_LKAH_SHIFT)); 
            	
	    // flow control
	    value = AT_READ_REG(hw, REG_SRAM_RXF_LEN);
	    hi = value * 4 / 5;
	    value /= 5;
	    value = 
	          ((hi&RXQ_RXF_PAUSE_TH_HI_MASK) << RXQ_RXF_PAUSE_TH_HI_SHIFT)|
	          ((value&RXQ_RXF_PAUSE_TH_LO_MASK) << RXQ_RXF_PAUSE_TH_LO_SHIFT);
	    AT_WRITE_REG(hw, REG_RXQ_RXF_PAUSE_THRESH, value);
    }
    
    // RRS  
    AT_WRITE_REG(hw, REG_IDT_TABLE, hw->indirect_tab);
    AT_WRITE_REG(hw, REG_BASE_CPU_NUMBER, hw->base_cpu);
    value = 0;
    if (hw->rrs_type&at_rrs_ipv4)       value |= RXQ_CTRL_HASH_TYPE_IPV4;
    if (hw->rrs_type&at_rrs_ipv4_tcp)   value |= RXQ_CTRL_HASH_TYPE_IPV4_TCP;
    if (hw->rrs_type&at_rrs_ipv6)       value |= RXQ_CTRL_HASH_TYPE_IPV6;
    if (hw->rrs_type&at_rss_ipv6_tcp)   value |= RXQ_CTRL_HASH_TYPE_IPV6_TCP;
    if (hw->rrs_type&(at_rrs_ipv4|at_rrs_ipv4_tcp|at_rrs_ipv6|at_rss_ipv6_tcp)) {
        value |= RXQ_CTRL_HASH_ENABLE|RXQ_CTRL_RSS_MODE_MQUESINT;
    }
    value |= RXQ_CTRL_IPV6_XSUM_VERIFY_EN|RXQ_CTRL_PBA_ALIGN_32|
            RXQ_CTRL_CUT_THRU_EN|RXQ_CTRL_EN;
    AT_WRITE_REG(hw, REG_RXQ_CTRL, value);

    // 11. config  DMA Engine
    value =  
        ((((u32)hw->dmar_block)&DMA_CTRL_DMAR_BURST_LEN_MASK)
           << DMA_CTRL_DMAR_BURST_LEN_SHIFT)|
        ((((u32)hw->dmaw_block)&DMA_CTRL_DMAW_BURST_LEN_MASK) 
           << DMA_CTRL_DMAW_BURST_LEN_SHIFT) |
        DMA_CTRL_DMAR_REQ_PRI | 
        DMA_CTRL_DMAR_OUT_ORDER |
        ((15&DMA_CTRL_DMAR_DLY_CNT_MASK)<<DMA_CTRL_DMAR_DLY_CNT_SHIFT) |
        ((4&DMA_CTRL_DMAW_DLY_CNT_MASK)<<DMA_CTRL_DMAW_DLY_CNT_SHIFT) |
        DMA_CTRL_RXCMB_EN;
	/* disable TXCMB */
	/* DMA_CTRL_TXCMB_EN */
    AT_WRITE_REG(hw, REG_DMA_CTRL, value);

    // 12. smb timer to trig interrupt
    AT_WRITE_REG(hw, REG_SMB_STAT_TIMER, 50000);
    
    value = AT_READ_REG(hw, REG_ISR);
    if ((value & ISR_PHY_LINKDOWN) != 0)
        value = 1;  /* config failed */
    else
        value = 0;

    // clear all interrupt status
    AT_WRITE_REG(hw, REG_ISR, 0x7fffffff);
    AT_WRITE_REG(hw, REG_ISR, 0);
    
    return value;
}

s32 
at_alloc_tx_buffers(at_adapter *adapter)
{

    at_buffer *buffer_info;
	
	u16 index;
	UInt16 num_alloc = 0;
	TpdDescr *tpd_desc;

	for (index=0; index < adapter->tpd_ring_size; index++) { 
		buffer_info = &adapter->tx_buffer_info[index];
		tpd_desc= &adapter->tpd_ring[index];

		buffer_info->memDesc = IOBufferMemoryDescriptor::withOptions(0,
																	  MAX_TX_BUF_LEN);

		if (!buffer_info->memDesc || buffer_info->memDesc->prepare() != kIOReturnSuccess)
		{
			ErrPrint("Couldn't allocate o prepare memory for transmitting\n");
			 return AT_ERR_ENOMEM;
		}
		
		IOByteCount length;
		buffer_info->dma = buffer_info->memDesc->getPhysicalSegment(0, &length);
		tpd_desc->addr = OSSwapHostToLittleInt64(buffer_info->dma);
		
		num_alloc++;
	}
	DbgPrint("Total allocated space for tx descriptors %d\n", num_alloc * MAX_TX_BUF_LEN);
	
	return AT_SUCCESS;
}

/**
 * at_clean_tx_ring - Free Tx-skb
 * @adapter: board private structure
 **/

void
at_clean_tx_ring(at_adapter *adapter)
{
    at_buffer *buffer_info;
    u16 index;
	
//    DEBUGFUNC("at_clean_tx_ring ()!\n");

    if (NULL == adapter->tx_buffer_info ||
        NULL == adapter->tpd_ring)
        return;
		
	/* Free all the Tx ring sk_buffs */
	for (index=0; index < adapter->tpd_ring_size; index++) {
		buffer_info = adapter->tx_buffer_info + index;
		if (buffer_info->dma) {
			buffer_info->dma = 0;
		}
		if (buffer_info->memDesc) {
			buffer_info->memDesc->complete();
			buffer_info->memDesc->release();
			buffer_info->memDesc = NULL;
		}
	}

    /* Zero out Tx-buffers */
    memset(adapter->tx_buffer_info, 0, 
            sizeof(at_buffer)*adapter->tpd_ring_size);
    
    memset(adapter->tpd_ring, 0, 
            sizeof(TpdDescr)*adapter->tpd_ring_size);
}

/**
 * at_clean_rx_ring - Free rx-reservation skbs
 * @adapter: board private structure
 **/

void
at_clean_rx_ring(at_adapter *adapter)
{
    u16 i;
//    DEBUGFUNC("at_clean_rx_ring()!\n");

    if (NULL == adapter->ring_vir_addr)
        return;
        
    /* Free all the Rx ring sk_buffs */


    /* Zero out the descriptor ring */
    
    for (i=0; i < adapter->num_rx_queues; i++) {
        if (adapter->rxf_page[i][0].addr) {
            memset(adapter->rxf_page[i][0].addr, 0, adapter->rxf_length);
            memset(adapter->rxf_page[i][1].addr, 0, adapter->rxf_length);
        }
    }
}


void
at_clear_phy_int(at_adapter* adapter)
{
    u16 phy_data;
    
    IOSimpleLockLock(adapter->stats_lock);
    at_read_phy_reg(&adapter->hw, 19, &phy_data);
    IOSimpleLockUnlock(adapter->stats_lock);
}


u16 
tpd_avail(at_adapter* adapter)
{
    u16 tpd_next_clean = atomic_read(&adapter->tpd_next_clean);
    
    return (u16)
        ((tpd_next_clean > adapter->tpd_next_use) ? \
         tpd_next_clean - adapter->tpd_next_use - 1 :   \
         adapter->tpd_ring_size + tpd_next_clean - adapter->tpd_next_use - 1);
}







