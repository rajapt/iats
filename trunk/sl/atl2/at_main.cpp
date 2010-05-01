/*
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 * Copyright(c) 2006 xiong huang <xiong.huang@atheros.com>
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
 */


#include <IOKit/pci/IOPCIDevice.h>

#include "at_hw.h"
#include "at.h"
#include "at_main.h"

#define AT_ERR_ENOMEM    12


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
}


/**
 * at_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/

void
at_irq_enable(at_adapter *adapter)
{

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

    /* PCI config space info */

    hw->vendor_id = pdev->configRead16(kIOPCIConfigVendorID);
    hw->device_id = pdev->configRead16(kIOPCIConfigDeviceID);
    hw->subsystem_vendor_id = pdev->configRead16(kIOPCIConfigSubSystemVendorID);
    hw->subsystem_id = pdev->configRead16(kIOPCIConfigSubSystemID);

	hw->revision_id = pdev->configRead8(kIOPCIConfigRevisionID);
	hw->pci_cmd_word = pdev->configRead16(kIOPCIConfigCommand);

    adapter->wol = 0;

    adapter->ict = 50000;  // 100ms
    
    adapter->link_speed = SPEED_0;   // hardware init
    adapter->link_duplex = FULL_DUPLEX; //

  
    hw->phy_configured = false;
    hw->preamble_len = 7;
    hw->ipgt = 0x60;
    hw->min_ifg = 0x50;
    hw->ipgr1 = 0x40;
    hw->ipgr2 = 0x60;
    hw->retry_buf = 2;
    
    hw->max_retry = 0xf;
    hw->lcol = 0x37;
    hw->jam_ipg = 7;
    
    hw->fc_rxd_hi = 0;
    hw->fc_rxd_lo = 0; 
    
    hw->max_frame_size = 1500;
    
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
    int size;
    u8 offset = 0;

    DEBUGFUNC("at_setup_ring_resources()\n");

    /* real ring DMA buffer */
    adapter->ring_size = size =   
	      adapter->txd_ring_size * 1   + 7         // dword align
	    + adapter->txs_ring_size * 4   + 7         // dword align
            + adapter->rxd_ring_size * 1536+ 127;    // 128bytes align
    
	adapter->memDesc = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous|
																 kIODirectionOut|kIODirectionIn,
																	size,
																	PAGE_SIZE);
	DbgPrint("Allocated memory for ring header %d\n",size);

	if (!adapter->memDesc || (adapter->memDesc->prepare() != kIOReturnSuccess))
	{
		IOSleep(1500);
		ErrPrint("Couldn't alloc memory for descriptor ring\n");
		if (adapter->memDesc)
		{
			adapter->memDesc->release();
			adapter->memDesc = NULL;
		}
		DEBUGOUT1("Couldn't alloc memory for descriptor ring, size = D%d\n", size);
        return AT_ERR_ENOMEM;
	}

	IOByteCount dmaLength = 0;
	adapter->ring_dma = adapter->memDesc->getPhysicalSegment(0, &dmaLength);
	adapter->ring_vir_addr = adapter->memDesc->getBytesNoCopy();
	DbgPrint("ring Physical segment address %X pointer %p length %d\n",
			(u32)adapter->ring_dma, adapter->ring_vir_addr, (u32)dmaLength);

   
    
    memset(adapter->ring_vir_addr, 0, adapter->ring_size);

      // Init TXD Ring
      
      adapter->txd_dma = adapter->ring_dma ;
      offset = (adapter->txd_dma & 0x7) ? (8 - (adapter->txd_dma & 0x7)) : 0;
      adapter->txd_dma += offset;
      adapter->txd_ring = (tx_pkt_header_t*) ((u8*)adapter->ring_vir_addr + offset);
      
      // Init TXS Ring
      
      adapter->txs_dma = adapter->txd_dma + adapter->txd_ring_size;
      offset = (adapter->txs_dma & 0x7) ? (8- (adapter->txs_dma & 0x7)) : 0;
      adapter->txs_dma += offset;
      adapter->txs_ring = (tx_pkt_status_t*) 
                (((u8*)adapter->txd_ring) + (adapter->txd_ring_size+offset));
                
      // Init RXD Ring
      adapter->rxd_dma = adapter->txs_dma + adapter->txs_ring_size*4;
      offset = (adapter->rxd_dma & 127) ? (128 - (adapter->rxd_dma & 127)) : 0;
      if (offset > 7) {
	  offset -= 8;
      } else {
	  offset += (128 - 8);
      }
      adapter->rxd_dma += offset;
      adapter->rxd_ring = (rx_desc_t*)
                (((u8*)adapter->txs_ring) + 
		    (adapter->txs_ring_size*4 + offset));


      // Read / Write Ptr Initialize:
  //      init_ring_ptrs(adapter);

    return AT_SUCCESS;
}


void
init_ring_ptrs(at_adapter *adapter)
{
    // Read / Write Ptr Initialize:
    adapter->txd_write_ptr = 0;
    atomic_set(&adapter->txd_read_ptr, 0);

    adapter->rxd_read_ptr = 0;
    adapter->rxd_write_ptr = 0;
    
    atomic_set(&adapter->txs_write_ptr, 0);
    adapter->txs_next_clear = 0;
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
    DEBUGFUNC("at_free_ring_resources()\n");

	if (adapter->memDesc)
	{
		adapter->memDesc->complete();
		adapter->memDesc->release();
		adapter->memDesc = NULL;
	}
    adapter->ring_vir_addr = NULL;
    adapter->ring_dma= 0;

    adapter->txd_dma = 0;
    adapter->txs_dma = 0;
    adapter->rxd_dma= 0;
         
}



void
at_setup_mac_ctrl(at_adapter* adapter)
{
    u32 value;
    at_hw* hw = &adapter->hw;
    
    /* Config MAC CTRL Register */
    value = MAC_CTRL_TX_EN | 
	    	MAC_CTRL_RX_EN |
            MAC_CTRL_MACLP_CLK_PHY;
    // duplex
    if (FULL_DUPLEX == adapter->link_duplex)    
        value |= MAC_CTRL_DUPLX;
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


    // half retry buffer
    value |= (((u32)(adapter->hw.retry_buf
                &MAC_CTRL_HALF_LEFT_BUF_MASK)) 
                    << MAC_CTRL_HALF_LEFT_BUF_SHIFT);

    AT_WRITE_REG(hw, REG_MAC_CTRL, value);
}


/**
 * at_configure - Configure Transmit&Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx /Rx unit of the MAC after a reset.
 **/

int
at_configure(at_adapter *adapter)
{
    at_hw * hw = &adapter->hw;
    u32 value;
    
//    DEBUGFUNC("at_configure() !\n");

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
          (u32)((adapter->ring_dma&0xffffffff00000000ULL) >>32));
    // LO base address
    AT_WRITE_REG(
          hw, 
          REG_TXD_BASE_ADDR_LO, 
          (u32)(adapter->txd_dma&0x00000000ffffffffULL));
    AT_WRITE_REG(
          hw, 
          REG_TXS_BASE_ADDR_LO, 
          (u32)(adapter->txs_dma& 0x00000000ffffffffULL));
    AT_WRITE_REG(hw, 
                 REG_RXD_BASE_ADDR_LO, 
                 (u32)(adapter->rxd_dma& 0x00000000ffffffffULL));
  
    // element count
    AT_WRITE_REGW(hw, REG_TXD_MEM_SIZE, (u16)(adapter->txd_ring_size/4));
    AT_WRITE_REGW(hw, REG_TXS_MEM_SIZE, (u16)adapter->txs_ring_size);
    AT_WRITE_REGW(hw, REG_RXD_BUF_NUM,  (u16)adapter->rxd_ring_size);
    DEBUGOUT1("txd ring size:%d, txs ring size:%d, rxd ring size:%d\n",
		    adapter->txd_ring_size/4,
		    adapter->txs_ring_size,
		    adapter->rxd_ring_size);
    
    /* config Internal SRAM */
/*
    AT_WRITE_REGW(hw, REG_SRAM_TXRAM_END, sram_tx_end);
    AT_WRITE_REGW(hw, REG_SRAM_TXRAM_END, sram_rx_end);    
*/
   
   
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
//    DEBUGOUT1("init ipg/ifg with 0x%x\n", value);
    
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
//    DEBUGOUT1("init Half Duplex with 0x%x\n", value);
    
    
    /* set Interrupt Moderator Timer */
    AT_WRITE_REGW(hw, REG_IRQ_MODU_TIMER_INIT, adapter->imt);
    AT_WRITE_REG(hw, REG_MASTER_CTRL, MASTER_CTRL_ITIMER_EN);
//    DEBUGOUT1("init Irq Modurator Timer with 0x%x\n", adapter->imt);
    
    /* set Interrupt Clear Timer */
    AT_WRITE_REGW(hw, REG_CMBDISDMA_TIMER, adapter->ict);
//    DEBUGOUT1("init Irq Clear Timer with 0x%x\n", adapter->ict);
    
    /* set MTU */
    AT_WRITE_REG(hw, REG_MTU, 
		     hw->max_frame_size +
		    ENET_HEADER_SIZE + 
		    VLAN_SIZE +
		    ETHERNET_FCS_SIZE);
//    DEBUGOUT1("init MTU with 0x%x\n", hw->max_frame_size); 
   
    /* 1590 */
    AT_WRITE_REG(hw, 
		 REG_TX_CUT_THRESH,
		 0x177);
    
     /* flow control */
    AT_WRITE_REGW(hw, REG_PAUSE_ON_TH, hw->fc_rxd_hi);
    AT_WRITE_REGW(hw, REG_PAUSE_OFF_TH, hw->fc_rxd_lo);
    
    /* Init mailbox */
    AT_WRITE_REGW(hw, REG_MB_TXD_WR_IDX, (u16)adapter->txd_write_ptr);
    AT_WRITE_REGW(hw, REG_MB_RXD_RD_IDX, (u16)adapter->rxd_read_ptr);
    
    /* enable DMA read/write */
    AT_WRITE_REGB(hw, REG_DMAR, DMAR_EN);
    AT_WRITE_REGB(hw, REG_DMAW, DMAW_EN);
    
    
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

void
at_clear_phy_int(at_adapter* adapter)
{
    u16 phy_data;
    
    IOSimpleLockLock(adapter->stats_lock);
    at_read_phy_reg(&adapter->hw, 19, &phy_data);
    IOSimpleLockUnlock(adapter->stats_lock);
}
 
int
TxsFreeUnit(at_adapter* adapter)
{
    u32 txs_write_ptr = (u32) atomic_read(&adapter->txs_write_ptr);
        
    return 
        (adapter->txs_next_clear >= txs_write_ptr) ?
        (int) (adapter->txs_ring_size - adapter->txs_next_clear 
                        + txs_write_ptr - 1) :
        (int) (txs_write_ptr - adapter->txs_next_clear - 1);    
}


int
TxdFreeBytes(at_adapter* adapter)
{
    u32 txd_read_ptr = (u32)atomic_read(&adapter->txd_read_ptr);
    
    return (adapter->txd_write_ptr >= txd_read_ptr) ?
            (int) (adapter->txd_ring_size - adapter->txd_write_ptr 
                        + txd_read_ptr - 1):
            (int) (txd_read_ptr - adapter->txd_write_ptr - 1);
}




