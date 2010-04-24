/* at_main.cpp -- ATL1c adapter implements
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

#include <IOKit/pci/IOPCIDevice.h>

#include "at_hw.h"
#include "at.h"
#include "at_main.h"

#define ENOMEM    12
#define roundup(x,n) (((x)+((n)-1))&(~((n)-1))) 
/*
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
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
 */

void atl1c_disable_l0s_l1(atl1c_hw *hw);

static const u16 atl1c_pay_load_size[] = {
	128, 256, 512, 1024, 2048, 4096,
};

static const u16 atl1c_rfd_prod_idx_regs[AT_MAX_RECEIVE_QUEUE] =
{
	REG_MB_RFD0_PROD_IDX,
	REG_MB_RFD1_PROD_IDX,
	REG_MB_RFD2_PROD_IDX,
	REG_MB_RFD3_PROD_IDX
};

static const u16 atl1c_rfd_addr_lo_regs[AT_MAX_RECEIVE_QUEUE] =
{
	REG_RFD0_HEAD_ADDR_LO,
	REG_RFD1_HEAD_ADDR_LO,
	REG_RFD2_HEAD_ADDR_LO,
	REG_RFD3_HEAD_ADDR_LO
};

static const u16 atl1c_rrd_addr_lo_regs[AT_MAX_RECEIVE_QUEUE] =
{
	REG_RRD0_HEAD_ADDR_LO,
	REG_RRD1_HEAD_ADDR_LO,
	REG_RRD2_HEAD_ADDR_LO,
	REG_RRD3_HEAD_ADDR_LO
};

void atl1c_pcie_patch(atl1c_hw *hw)
{
	u32 data;
	
	AT_READ_REG(hw, REG_PCIE_PHYMISC, &data);
	data |= PCIE_PHYMISC_FORCE_RCV_DET;
	AT_WRITE_REG(hw, REG_PCIE_PHYMISC, data);
	
	if (hw->nic_type == athr_l2c_b && hw->revision_id == L2CB_V10) {
		AT_READ_REG(hw, REG_PCIE_PHYMISC2, &data);
		
		data &= ~(PCIE_PHYMISC2_SERDES_CDR_MASK <<
				  PCIE_PHYMISC2_SERDES_CDR_SHIFT);
		data |= 3 << PCIE_PHYMISC2_SERDES_CDR_SHIFT;
		data &= ~(PCIE_PHYMISC2_SERDES_TH_MASK <<
				  PCIE_PHYMISC2_SERDES_TH_SHIFT);
		data |= 3 << PCIE_PHYMISC2_SERDES_TH_SHIFT;
		AT_WRITE_REG(hw, REG_PCIE_PHYMISC2, data);
	}
}

/* FIXME: no need any more ? */
/*
 * atl1c_init_pcie - init PCIE module
 */
void atl1c_reset_pcie(IOPCIDevice *pdev,atl1c_hw *hw, u32 flag)
{
	u32 data;
	u32 pci_cmd;
	
	
    pci_cmd = pdev->configRead32(kIOPCIConfigCommand) ;
	
    
    if (pci_cmd & 0x0400)
        pci_cmd &= ~0x0400;
    if (pci_cmd & kIOPCICommandIOSpace)
        pci_cmd &= ~kIOPCICommandIOSpace;
    if (0 == (pci_cmd & kIOPCICommandMemorySpace))
        pci_cmd |= kIOPCICommandMemorySpace;
    if (0 == (pci_cmd & kIOPCICommandBusMaster))
        pci_cmd |= kIOPCICommandBusMaster;
    pdev->configWrite32(kIOPCIConfigCommand, pci_cmd);

	/* 
	 * Clear any PowerSaveing Settings
	 */
	AT_WRITE_REG(hw, REG_PM_CTRLSTAT, 0);
	
	/*
	 * Mask some pcie error bits
	 */
	AT_READ_REG(hw, REG_PCIE_UC_SEVERITY, &data);
	data &= ~PCIE_UC_SERVRITY_DLP;
	data &= ~PCIE_UC_SERVRITY_FCP;
	AT_WRITE_REG(hw, REG_PCIE_UC_SEVERITY, data);
	
	AT_READ_REG(hw, REG_LTSSM_ID_CTRL, &data);
	data &= ~LTSSM_ID_EN_WRO;
	AT_WRITE_REG(hw, REG_LTSSM_ID_CTRL, data);
	
	atl1c_pcie_patch(hw);
	if (flag & ATL1C_PCIE_L0S_L1_DISABLE)
		atl1c_disable_l0s_l1(hw);
	
	if (flag & ATL1C_PCIE_PHY_RESET){
		AT_WRITE_REG(hw, REG_GPHY_CTRL, GPHY_CTRL_DEFAULT);
	}else{
		AT_WRITE_REG(hw, REG_GPHY_CTRL, 
			GPHY_CTRL_DEFAULT | GPHY_CTRL_EXT_RESET);
	}
	AT_WRITE_FLUSH(hw);
	msec_delay(5);
}

/*
 * atl1c_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 */
void atl1c_irq_enable(atl1c_adapter *adapter)
{
	AT_WRITE_REG(&adapter->hw, REG_ISR, 0x7FFFFFFF);
	AT_WRITE_REG(&adapter->hw, REG_IMR, adapter->hw.intr_mask);
	AT_WRITE_FLUSH(&adapter->hw);
}

/*
 * atl1c_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 */
 void atl1c_irq_disable(atl1c_adapter *adapter)
{
	AT_WRITE_REG(&adapter->hw, REG_IMR, 0);
	AT_WRITE_REG(&adapter->hw, REG_ISR, ISR_DIS_INT);
	AT_WRITE_FLUSH(&adapter->hw);
}

/*
 * atl1c_irq_reset - reset interrupt confiure on the NIC
 * @adapter: board private structure
 */
static  void atl1c_irq_reset(atl1c_adapter *adapter)
{
	atomic_set(&adapter->irq_sem, 1);
	atl1c_irq_enable(adapter);
}

/*
 * atl1c_phy_config - Timer Call-back
 * @data: pointer to netdev cast into an unsigned long
 */
static void atl1c_phy_config(unsigned long data)
{
	atl1c_adapter *adapter = (atl1c_adapter *) data;
	atl1c_hw *hw = &adapter->hw;
	
	//IOSimpleLockLock(adapter->mdio_lock);
	atl1c_restart_autoneg(hw);
	//IOSimpleLockFree(adapter->mdio_lock);
}


void atl1c_set_rxbufsize(atl1c_adapter *adapter)
{
	
	adapter->rx_buffer_len =  AT_RX_BUF_SIZE ;
}


/*
 * atl1c_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 *
 */
int atl1c_alloc_queues(atl1c_adapter *adapter)
{
	return 0;
}

void atl1c_set_mac_type(atl1c_hw *hw)
{
	switch (hw->device_id) {
		case DEV_ID_ATL1C_2_0:
			hw->nic_type = athr_l1c;
			break;
		case DEV_ID_ATL2C_2_0:
			hw->nic_type = athr_l2c;
			break;
		case DEV_ID_ATL2C_B:
			hw->nic_type = athr_l2c_b;
			break;
		case DEV_ID_ATL2C_B_2:
			hw->nic_type = athr_l2c_b2;
			break;
		case DEV_ID_ATL1D:
			hw->nic_type = athr_l1d;
			break;
		default:
			break;
	}
	
	return;
}

int atl1c_setup_mac_funcs(atl1c_hw *hw)
{
	u32 phy_status_data;
	u32 link_ctrl_data;
	
	atl1c_set_mac_type(hw);
	AT_READ_REG(hw, REG_PHY_STATUS, &phy_status_data);
	AT_READ_REG(hw, REG_LINK_CTRL, &link_ctrl_data);
	
	hw->ctrl_flags = ATL1C_INTR_MODRT_ENABLE  |
	ATL1C_TXQ_MODE_ENHANCE;
	if (link_ctrl_data & LINK_CTRL_L0S_EN)
		hw->ctrl_flags |= ATL1C_ASPM_L0S_SUPPORT;
	if (link_ctrl_data & LINK_CTRL_L1_EN)
		hw->ctrl_flags |= ATL1C_ASPM_L1_SUPPORT;
	if (link_ctrl_data & LINK_CTRL_EXT_SYNC)
		hw->ctrl_flags |= ATL1C_LINK_EXT_SYNC; 
	/* FIXME:
 	 * JUST FOR TEST VERSION
 	 */
	hw->ctrl_flags |= ATL1C_ASPM_CTRL_MON;
	
	if (hw->nic_type == athr_l1c || hw->nic_type == athr_l1d) {
		hw->link_cap_flags |= LINK_CAP_SPEED_1000;
	}
	return 0;	
}
/*
 * atl1c_sw_init - Initialize general software structures (struct atl1c_adapter)
 * @adapter: board private structure to initialize
 *
 * atl1c_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 */
int  atl1c_sw_init(atl1c_adapter *adapter)
{
	atl1c_hw *hw   = &adapter->hw;
    IOPCIDevice *pdev = adapter->pdev;

	hw->vendor_id = pdev->configRead16(kIOPCIConfigVendorID);
	hw->device_id = pdev->configRead16(kIOPCIConfigDeviceID);
	hw->subsystem_vendor_id = pdev->configRead16(kIOPCIConfigSubSystemVendorID);
	hw->subsystem_id = pdev->configRead16(kIOPCIConfigSubSystemID);

	hw->revision_id = pdev->configRead8(kIOPCIConfigRevisionID);

	/* before link up, we assume hibernate is true */
	hw->hibernate = true;
	if (atl1c_setup_mac_funcs(hw) != 0) {
		ErrPrint("set mac function pointers failed\n");
		return -1;
	}
	adapter->wol = 0;
	adapter->link_speed = SPEED_0;
	adapter->link_duplex = FULL_DUPLEX;
	adapter->num_rx_queues = AT_DEF_RECEIVE_QUEUE;
	
	hw->intr_mask = IMR_NORMAL_MASK;
	hw->phy_configured = false;
	hw->preamble_len = 7;
	hw->max_frame_size = 1500;
	if (adapter->num_rx_queues < 2) {
		hw->rss_type = atl1c_rss_disable;
		hw->rss_mode = atl1c_rss_mode_disable;
	} else {
		hw->rss_type = atl1c_rss_ipv4;
		hw->rss_mode = atl1c_rss_mul_que_mul_int;
		hw->rss_hash_bits = 16;
	}
	hw->autoneg_advertised = ADVERTISE_DEFAULT;
	hw->indirect_tab = 0xE4E4E4E4;
	hw->base_cpu = 0; 
	
	hw->ict = 50000;		/* 100ms */
	hw->smb_timer = 200000;	  	/* 400ms */
	hw->cmb_tpd = 4;
	hw->cmb_tx_timer = 1;		/* 2 us  */
	
	hw->tpd_burst = 5;
	hw->rfd_burst = 8;
	hw->dma_order = atl1c_dma_ord_out;
	hw->dmar_block = atl1c_dma_req_1024;
	hw->dmaw_block = atl1c_dma_req_1024;
	hw->dmar_dly_cnt = 15;
	hw->dmaw_dly_cnt = 4;
	
	if (atl1c_alloc_queues(adapter)) {
		ErrPrint("Unable to allocate memory for queues\n");
		return -ENOMEM;
	}
	/* TODO */
	atl1c_set_rxbufsize(adapter);
	atomic_set(&adapter->irq_sem, 1);
	//adapter->mdio_lock = IOSimpleLockAlloc();
	//adapter->tx_lock = IOSimpleLockAlloc();
	
	return 0;
}
/*
 * atl1c_clean_tx_ring - Alloc Tx-skb
 * @adapter: board private structure
 */
s32 atl1c_alloc_tx_buffers(atl1c_adapter *adapter,
						   enum atl1c_trans_queue type)
{
	struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
	struct atl1c_buffer *buffer_info;
	
	u16 index, ring_count;
    atl1c_tpd_desc *tpd_desc;
	u16 num_alloc = 0;
	
	ring_count = tpd_ring->count;
	for (index = 0; index < ring_count; index++) {
		buffer_info = &tpd_ring->buffer_info[index];		
		tpd_desc= ATL1C_TPD_DESC(tpd_ring, index);
		
		buffer_info->memDesc = IOBufferMemoryDescriptor::withOptions(0,
																	 MAX_TX_BUF_LEN);
		
		if (!buffer_info->memDesc || buffer_info->memDesc->prepare() != kIOReturnSuccess)
		{
			ErrPrint("Couldn't allocate o prepare memory for transmitting\n");
			return -ENOMEM;
		}
		
		IOByteCount length;
		buffer_info->dma = buffer_info->memDesc->getPhysicalSegment(0, &length);
		buffer_info->state = ATL1_BUFFER_BUSY;
		buffer_info->length = length;
		tpd_desc->buffer_addr = OSSwapHostToLittleInt64(buffer_info->dma);
		num_alloc++;
	}
	DbgPrint("Total allocated space for tx descriptors D%d; num_alloc= D%d; buffer_len= D%d\n",
			 num_alloc * MAX_TX_BUF_LEN,num_alloc , MAX_TX_BUF_LEN);
	
	return 0;
}

/*
 * atl1c_clean_tx_ring - Free Tx-skb
 * @adapter: board private structure
 */
void atl1c_clean_tx_ring(struct atl1c_adapter *adapter,
								enum atl1c_trans_queue type)
{
	struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
	struct atl1c_buffer *buffer_info;

	u16 index, ring_count;
	
	ring_count = tpd_ring->count;
	for (index = 0; index < ring_count; index++) {
		buffer_info = &tpd_ring->buffer_info[index];
		//if (buffer_info->state == ATL1_BUFFER_FREE)
		//	continue;
		if (buffer_info->dma) {
			buffer_info->dma = 0;
		}
		if (buffer_info->memDesc) {
			buffer_info->memDesc->complete();
			buffer_info->memDesc->release();
			buffer_info->memDesc = NULL;
		}
		buffer_info->state = ATL1_BUFFER_FREE;
	}
	
	/* Zero out Tx-buffers */
	memset(tpd_ring->desc, 0, sizeof(struct atl1c_tpd_desc) *
		   ring_count);
	atomic_set(&tpd_ring->next_to_clean, 0);
	tpd_ring->next_to_use = 0;
}

/*
 * atl1c_clean_rx_ring - Free rx-reservation skbs
 * @adapter: board private structure
 */
void atl1c_clean_rx_ring(atl1c_adapter *adapter)
{
	struct atl1c_rfd_ring *rfd_ring = adapter->rfd_ring;
	struct atl1c_rrd_ring *rrd_ring = adapter->rrd_ring;
	struct atl1c_buffer *buffer_info;

	int i, j;
	
	for (i = 0; i < adapter->num_rx_queues; i++) {
		for (j = 0; j < rfd_ring[i].count; j++) {
			buffer_info = &rfd_ring[i].buffer_info[j];
			//if (buffer_info->state == ATL1_BUFFER_FREE)
			//	continue;
			if (buffer_info->dma) {
				buffer_info->dma = 0;
			}
			if (buffer_info->memDesc) {
				buffer_info->memDesc->complete();
				buffer_info->memDesc->release();
				buffer_info->memDesc = NULL;
			}
			buffer_info->state = ATL1_BUFFER_FREE;
		}
		/* zero out the descriptor ring */
		memset(rfd_ring[i].desc, 0, rfd_ring[i].size);
		rfd_ring[i].next_to_clean = 0;
		rfd_ring[i].next_to_use = 0;
		rrd_ring[i].next_to_use = 0;
		rrd_ring[i].next_to_clean = 0;
	}
}

/*
 * Read / Write Ptr Initialize:
 */
void atl1c_init_ring_ptrs(atl1c_adapter *adapter)
{
	struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;
	struct atl1c_rfd_ring *rfd_ring = adapter->rfd_ring;
	struct atl1c_rrd_ring *rrd_ring = adapter->rrd_ring;
	struct atl1c_buffer *buffer_info;
	int i, j;
	
	for (i = 0; i < AT_MAX_TRANSMIT_QUEUE; i++) {
		tpd_ring[i].next_to_use = 0;
		atomic_set(&tpd_ring[i].next_to_clean, 0);
		buffer_info = tpd_ring[i].buffer_info;
		for (j = 0; j < tpd_ring->count; j++)
			buffer_info[j].state = ATL1_BUFFER_FREE;
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		rfd_ring[i].next_to_use = 0;
		rfd_ring[i].next_to_clean = 0;
		rrd_ring[i].next_to_use = 0;
		rrd_ring[i].next_to_clean = 0;
		for (j = 0; j < rfd_ring[i].count; j++) {
			buffer_info = &rfd_ring[i].buffer_info[j];
			buffer_info->state = ATL1_BUFFER_FREE;
		}
	}
}

/*
 * atl1c_free_ring_resources - Free Tx / RX descriptor Resources
 * @adapter: board private structure
 *
 * Free all transmit software resources
 */
void atl1c_free_ring_resources(struct atl1c_adapter *adapter)
{
	
	if (adapter->ring_header.dma)
		adapter->ring_header.dma = 0;

	if (adapter->ring_header.memDesc) {
		adapter->ring_header.memDesc->complete();
		adapter->ring_header.memDesc->release();
		adapter->ring_header.memDesc = NULL;
	}
	if (adapter->ring_header.desc) {
		adapter->ring_header.desc = NULL;
	}
	/* Note: just free tdp_ring.buffer_info,
	 *  it contain rfd_ring.buffer_info, do not double free */	
	struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;
	struct atl1c_rfd_ring *rfd_ring = adapter->rfd_ring;
	int num_rx_queues = adapter->num_rx_queues;
	int size = 	sizeof(struct atl1c_buffer) * (tpd_ring->count * 2 +
												rfd_ring->count * num_rx_queues);  
    if (tpd_ring->buffer_info) {
        IOFree(tpd_ring->buffer_info, size);
        tpd_ring->buffer_info = NULL;
    }
}

/*
 * atl1c_setup_mem_resources - allocate Tx / RX descriptor resources
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 */
 int atl1c_setup_ring_resources(struct atl1c_adapter *adapter)
{
	struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;
	struct atl1c_rfd_ring *rfd_ring = adapter->rfd_ring;
	struct atl1c_rrd_ring *rrd_ring = adapter->rrd_ring;
	struct atl1c_ring_header *ring_header = &adapter->ring_header;
	int num_rx_queues = adapter->num_rx_queues;
	int size;
	int i;
	int count = 0;
	int rx_desc_count = 0;
	u32 offset = 0;
	//int err = 0;
	
	rrd_ring[0].count = rfd_ring[0].count;
	for (i = 1; i < AT_MAX_TRANSMIT_QUEUE; i++)
		tpd_ring[i].count = tpd_ring[0].count;
	
	for (i = 1; i < adapter->num_rx_queues; i++)
		rfd_ring[i].count = rrd_ring[i].count = rfd_ring[0].count;
	
	/* 2 tpd queue, one high priority queue,
 	 * another normal priority queue */
	size = sizeof(struct atl1c_buffer) * (tpd_ring->count * 2 +
		rfd_ring->count * num_rx_queues);
	tpd_ring->buffer_info = (atl1c_buffer *)IOMalloc(size);
	if(!tpd_ring->buffer_info) {

        DbgPrint("Couldn't alloc memory for descriptor tpd ring, size = D%d\n", size);
        return -ENOMEM;
    }
    memset(tpd_ring->buffer_info, 0, size);
	DbgPrint("Allocated memory for descriptor tpd ring, all size = D%d; tpd_ring->count * 2 = D%d; rfd_ring->count * num_rx_queues= D%d \n",
			 size,tpd_ring->count * 2,rfd_ring->count * num_rx_queues);

	for (i = 0; i < AT_MAX_TRANSMIT_QUEUE; i++) {
		tpd_ring[i].buffer_info =
		(struct atl1c_buffer *) (tpd_ring->buffer_info + count);
		count += tpd_ring[i].count;
	}
	
	for (i = 0; i < num_rx_queues; i++) {
		rfd_ring[i].buffer_info =
		(struct atl1c_buffer *) (tpd_ring->buffer_info + count);
		count += rfd_ring[i].count;
		rx_desc_count += rfd_ring[i].count;
	}
	/*
	 * real ring DMA buffer
	 * each ring/block may need up to 8 bytes for alignment, hence the
	 * additional bytes tacked onto the end.
	 */
	ring_header->size = size =
	sizeof(struct atl1c_tpd_desc) * tpd_ring->count * 2 +
	sizeof(struct atl1c_rx_free_desc) * rx_desc_count +
	sizeof(struct atl1c_recv_ret_status) * rx_desc_count +
	sizeof(struct coals_msg_block) +
	sizeof(struct atl1c_hw_stats) +
	8 * 4 + 8 * 2 * num_rx_queues;
	
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
		DbgPrint("Couldn't alloc memory for descriptor ring header, size = D%d\n", ring_header->size);
        return -ENOMEM;
	}
	DbgPrint("Allocated memory for descriptor ring header, size = D%d ; page_size = D%d\n", ring_header->size, PAGE_SIZE);

	IOByteCount dmaLength = 0;
	ring_header->dma = ring_header->memDesc->getPhysicalSegment(0, &dmaLength);
	ring_header->desc = ring_header->memDesc->getBytesNoCopy();
	memset(ring_header->desc, 0, ring_header->size);
	
	/* init TPD ring */
	
	tpd_ring[0].dma = roundup(ring_header->dma, 8);
	offset = tpd_ring[0].dma - ring_header->dma;
	for (i = 0; i < AT_MAX_TRANSMIT_QUEUE; i++) {
		tpd_ring[i].dma = ring_header->dma + offset;
		tpd_ring[i].desc = (u8 *) ring_header->desc + offset;
		tpd_ring[i].size =
		sizeof(struct atl1c_tpd_desc) * tpd_ring[i].count;
		offset += roundup(tpd_ring[i].size, 8);
	}
	/* init RFD ring */
	for (i = 0; i < num_rx_queues; i++) {
		rfd_ring[i].dma = ring_header->dma + offset;
		rfd_ring[i].desc = (u8 *) ring_header->desc + offset;
		rfd_ring[i].size = sizeof(struct atl1c_rx_free_desc) *
		rfd_ring[i].count;
		offset += roundup(rfd_ring[i].size, 8);
	}
	
	/* init RRD ring */
	for (i = 0; i < num_rx_queues; i++) {
		rrd_ring[i].dma = ring_header->dma + offset;
		rrd_ring[i].desc = (u8 *) ring_header->desc + offset;
		rrd_ring[i].size = sizeof(struct atl1c_recv_ret_status) *
		rrd_ring[i].count;
		offset += roundup(rrd_ring[i].size, 8);
	}
	
	/* Init CMB dma address */
	adapter->cmb.dma = ring_header->dma + offset;
	adapter->cmb.cmb = (u8 *) ring_header->desc + offset;
	
	offset += roundup(sizeof(struct coals_msg_block), 8);
	adapter->smb.dma = ring_header->dma + offset;
	adapter->smb.smb = (u8 *)ring_header->desc + offset;
	return 0;
	
}

void atl1c_configure_des_ring(const atl1c_adapter *adapter)
{
	struct atl1c_hw *hw = (struct atl1c_hw *)&adapter->hw;
	struct atl1c_rfd_ring *rfd_ring = (struct atl1c_rfd_ring *)
	adapter->rfd_ring;
	struct atl1c_rrd_ring *rrd_ring = (struct atl1c_rrd_ring *)
	adapter->rrd_ring;
	struct atl1c_tpd_ring *tpd_ring = (struct atl1c_tpd_ring *)
	adapter->tpd_ring;
	struct atl1c_cmb *cmb = (struct atl1c_cmb *) &adapter->cmb;
	struct atl1c_smb *smb = (struct atl1c_smb *) &adapter->smb;
	int i;
	u32 data;
	
	/* TPD */
	AT_WRITE_REG(hw, REG_TX_BASE_ADDR_HI,
				 (u32)((tpd_ring[atl1c_trans_normal].dma &
						AT_DMA_HI_ADDR_MASK) >> 32));
	/* just enable normal priority TX queue */
	AT_WRITE_REG(hw, REG_NTPD_HEAD_ADDR_LO,
				 (u32)(tpd_ring[atl1c_trans_normal].dma &
					   AT_DMA_LO_ADDR_MASK));
	AT_WRITE_REG(hw, REG_HTPD_HEAD_ADDR_LO,
				 (u32)(tpd_ring[atl1c_trans_high].dma &
					   AT_DMA_LO_ADDR_MASK));
	AT_WRITE_REG(hw, REG_TPD_RING_SIZE,
				 (u32)(tpd_ring[0].count & TPD_RING_SIZE_MASK));
	
	
	/* RFD */
	AT_WRITE_REG(hw, REG_RX_BASE_ADDR_HI,
				 (u32)((rfd_ring[0].dma & AT_DMA_HI_ADDR_MASK) >> 32));
	for (i = 0; i < adapter->num_rx_queues; i++)
		AT_WRITE_REG(hw, atl1c_rfd_addr_lo_regs[i],
					 (u32)(rfd_ring[i].dma & AT_DMA_LO_ADDR_MASK));
	
	AT_WRITE_REG(hw, REG_RFD_RING_SIZE,
				 rfd_ring[0].count & RFD_RING_SIZE_MASK);
	AT_WRITE_REG(hw, REG_RX_BUF_SIZE,
				 adapter->rx_buffer_len & RX_BUF_SIZE_MASK);
	
	/* RRD */
	for (i = 0; i < adapter->num_rx_queues; i++)
		AT_WRITE_REG(hw, atl1c_rrd_addr_lo_regs[i],
					 (u32)(rrd_ring[i].dma & AT_DMA_LO_ADDR_MASK));
	AT_WRITE_REG(hw, REG_RRD_RING_SIZE,
				 (rrd_ring[0].count & RRD_RING_SIZE_MASK));
	
	/* CMB */
	AT_WRITE_REG(hw, REG_CMB_BASE_ADDR_LO, cmb->dma & AT_DMA_LO_ADDR_MASK);
	
	/* SMB */
	AT_WRITE_REG(hw, REG_SMB_BASE_ADDR_HI,
				 (u32)((smb->dma & AT_DMA_HI_ADDR_MASK) >> 32));
	AT_WRITE_REG(hw, REG_SMB_BASE_ADDR_LO,
				 (u32)(smb->dma & AT_DMA_LO_ADDR_MASK));
	if (hw->nic_type == athr_l2c_b) {
		AT_WRITE_REG(hw, REG_SRAM_RXF_LEN, 0x02a0L);
		AT_WRITE_REG(hw, REG_SRAM_TXF_LEN, 0x0100L);
		AT_WRITE_REG(hw, REG_SRAM_RXF_ADDR, 0x029f0000L);
		AT_WRITE_REG(hw, REG_SRAM_RFD0_INFO, 0x02bf02a0L);
		AT_WRITE_REG(hw, REG_SRAM_TXF_ADDR, 0x03bf02c0L);
		AT_WRITE_REG(hw, REG_SRAM_TRD_ADDR, 0x03df03c0L);
		AT_WRITE_REG(hw, REG_TXF_WATER_MARK, 0); /* TX watermark, to enter l1 state. */
		AT_WRITE_REG(hw, REG_RXD_DMA_CTRL, 0);   /* RXD threshold. */
		
	}
	if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l2c_b2) {
		/* Power Saving for L2c_B */
		AT_READ_REG(hw, REG_SERDES_LOCK, &data);
		data |= SERDES_MAC_CLK_SLOWDOWN;
		data |= SERDES_PYH_CLK_SLOWDOWN;
		AT_WRITE_REG(hw, REG_SERDES_LOCK, data);
	}
	/* Load all of base address above */
	AT_WRITE_REG(hw, REG_LOAD_PTR, 1);
	
	return;
}

void atl1c_configure_tx(struct atl1c_adapter *adapter)
{
	struct atl1c_hw *hw = (struct atl1c_hw *)&adapter->hw;
	u32 dev_ctrl_data;
	u32 max_pay_load;
	u16 tx_offload_thresh;
	u32 txq_ctrl_data;
	u32 extra_size = 0;     /* Jumbo frame threshold in QWORD unit */
	u32 max_pay_load_data;
	
	extra_size = ETH_HLEN + VLAN_HLEN + ETH_FCS_LEN;
	tx_offload_thresh = MAX_TX_OFFLOAD_THRESH;
	//	tx_offload_thresh = roundup(hw->max_frame_size + extra_size, 32);
	//	tx_offload_thresh = tx_offload_thresh > MAX_TX_OFFLOAD_THRESH ?
	//			MAX_TX_OFFLOAD_THRESH : tx_offload_thresh;
	AT_WRITE_REG(hw, REG_TX_TSO_OFFLOAD_THRESH,
				 (tx_offload_thresh >> 3) & TX_TSO_OFFLOAD_THRESH_MASK);
	AT_READ_REG(hw, REG_DEVICE_CTRL, &dev_ctrl_data);
	max_pay_load  = (dev_ctrl_data >> DEVICE_CTRL_MAX_PAYLOAD_SHIFT) &
	DEVICE_CTRL_MAX_PAYLOAD_MASK;
	hw->dmaw_block = atl1c_dma_req_block(min(max_pay_load, hw->dmaw_block));
	max_pay_load  = (dev_ctrl_data >> DEVICE_CTRL_MAX_RREQ_SZ_SHIFT) &
	DEVICE_CTRL_MAX_RREQ_SZ_MASK;
	hw->dmar_block = atl1c_dma_req_block(min(max_pay_load, hw->dmar_block));
	
	txq_ctrl_data = (hw->tpd_burst & TXQ_NUM_TPD_BURST_MASK) <<
	TXQ_NUM_TPD_BURST_SHIFT;
	if (hw->ctrl_flags & ATL1C_TXQ_MODE_ENHANCE)
		txq_ctrl_data |= TXQ_CTRL_ENH_MODE;
	max_pay_load_data = (atl1c_pay_load_size[hw->dmar_block] &
						 TXQ_TXF_BURST_NUM_MASK) << TXQ_TXF_BURST_NUM_SHIFT;
	if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l2c_b2)
		max_pay_load_data >>= 1;
	txq_ctrl_data |= max_pay_load_data; 
	
	AT_WRITE_REG(hw, REG_TXQ_CTRL, txq_ctrl_data);
	return;
}

void atl1c_configure_rx(struct atl1c_adapter *adapter)
{
	struct atl1c_hw *hw = (struct atl1c_hw *)&adapter->hw;
	u32 rxq_ctrl_data;
	
	rxq_ctrl_data = (hw->rfd_burst & RXQ_RFD_BURST_NUM_MASK) <<
	RXQ_RFD_BURST_NUM_SHIFT;
	
	if (hw->ctrl_flags & ATL1C_RX_IPV6_CHKSUM)
		rxq_ctrl_data |= IPV6_CHKSUM_CTRL_EN;
	if (hw->rss_type == atl1c_rss_ipv4)
		rxq_ctrl_data |= RSS_HASH_IPV4;
	if (hw->rss_type == atl1c_rss_ipv4_tcp)
		rxq_ctrl_data |= RSS_HASH_IPV4_TCP;
	if (hw->rss_type == atl1c_rss_ipv6)
		rxq_ctrl_data |= RSS_HASH_IPV6;
	if (hw->rss_type == atl1c_rss_ipv6_tcp)
		rxq_ctrl_data |= RSS_HASH_IPV6_TCP;
	if (hw->rss_type != atl1c_rss_disable)
		rxq_ctrl_data |= RRS_HASH_CTRL_EN;
	
	rxq_ctrl_data |= (hw->rss_mode & RSS_MODE_MASK) <<
	RSS_MODE_SHIFT;
	rxq_ctrl_data |= (hw->rss_hash_bits & RSS_HASH_BITS_MASK) <<
	RSS_HASH_BITS_SHIFT;
	if (hw->ctrl_flags & ATL1C_ASPM_CTRL_MON) 
		rxq_ctrl_data |= (ASPM_THRUPUT_LIMIT_1M & 
						  ASPM_THRUPUT_LIMIT_MASK) << ASPM_THRUPUT_LIMIT_SHIFT;
	
	AT_WRITE_REG(hw, REG_RXQ_CTRL, rxq_ctrl_data);
	
	return;
}

void atl1c_configure_rss(struct atl1c_adapter *adapter)
{
	struct atl1c_hw *hw = (struct atl1c_hw *)&adapter->hw;
	
	AT_WRITE_REG(hw, REG_IDT_TABLE, hw->indirect_tab);
	AT_WRITE_REG(hw, REG_BASE_CPU_NUMBER, hw->base_cpu);
	
	return;
}


void atl1c_configure_dma(struct atl1c_adapter *adapter)
{
	struct atl1c_hw *hw = &adapter->hw;
	u32 dma_ctrl_data;
	
	dma_ctrl_data = DMA_CTRL_DMAR_REQ_PRI;
	if (hw->ctrl_flags & ATL1C_CMB_ENABLE)
		dma_ctrl_data |= DMA_CTRL_CMB_EN;
	if (hw->ctrl_flags & ATL1C_SMB_ENABLE)
		dma_ctrl_data |= DMA_CTRL_SMB_EN;
	else
		dma_ctrl_data |= MAC_CTRL_SMB_DIS;
	
	switch (hw->dma_order) {
		case atl1c_dma_ord_in:
			dma_ctrl_data |= DMA_CTRL_DMAR_IN_ORDER;
			break;
		case atl1c_dma_ord_enh:
			dma_ctrl_data |= DMA_CTRL_DMAR_ENH_ORDER;
			break;
		case atl1c_dma_ord_out:
			dma_ctrl_data |= DMA_CTRL_DMAR_OUT_ORDER;
			break;
		default:
			break;
	}
	
	dma_ctrl_data |= (((u32)hw->dmar_block) & DMA_CTRL_DMAR_BURST_LEN_MASK)
	<< DMA_CTRL_DMAR_BURST_LEN_SHIFT;
	dma_ctrl_data |= (((u32)hw->dmaw_block) & DMA_CTRL_DMAW_BURST_LEN_MASK)
	<< DMA_CTRL_DMAW_BURST_LEN_SHIFT;
	dma_ctrl_data |= (((u32)hw->dmar_dly_cnt) & DMA_CTRL_DMAR_DLY_CNT_MASK)
	<< DMA_CTRL_DMAR_DLY_CNT_SHIFT;
	dma_ctrl_data |= (((u32)hw->dmaw_dly_cnt) & DMA_CTRL_DMAW_DLY_CNT_MASK)
	<< DMA_CTRL_DMAW_DLY_CNT_SHIFT;
	
	AT_WRITE_REG(hw, REG_DMA_CTRL, dma_ctrl_data);
	return;
}

/*
 * Stop the mac, transmit and receive units
 * hw - Struct containing variables accessed by shared code
 * return : 0  or  idle status (if error)
 */
int atl1c_stop_mac(struct atl1c_hw *hw)
{
	u32 data;
	int timeout;
	
	AT_READ_REG(hw, REG_RXQ_CTRL, &data);
	data &= ~(RXQ1_CTRL_EN | RXQ2_CTRL_EN |
			  RXQ3_CTRL_EN | RXQ_CTRL_EN);
	AT_WRITE_REG(hw, REG_RXQ_CTRL, data);
	
	AT_READ_REG(hw, REG_TXQ_CTRL, &data);
	data &= ~TXQ_CTRL_EN;
	AT_WRITE_REG(hw, REG_TXQ_CTRL, data);
	
	for (timeout = 0; timeout < AT_HW_MAX_IDLE_DELAY; timeout++) {
		AT_READ_REG(hw, REG_IDLE_STATUS, &data);
		if ((data & (IDLE_STATUS_RXQ_NO_IDLE |
					 IDLE_STATUS_TXQ_NO_IDLE)) == 0)
			break;
		msec_delay(5);
	}
	
	AT_READ_REG(hw, REG_MAC_CTRL, &data);
	data &= ~(MAC_CTRL_TX_EN | MAC_CTRL_RX_EN);
	AT_WRITE_REG(hw, REG_MAC_CTRL, data);
	
	for (timeout = 0; timeout < AT_HW_MAX_IDLE_DELAY; timeout++) {
		AT_READ_REG(hw, REG_IDLE_STATUS, &data);
		if ((data & IDLE_STATUS_MASK) == 0)
			return 0;
		msec_delay(5);
	}
	return data;
}

void atl1c_enable_rx_ctrl(atl1c_hw *hw)
{
	u32 data;
	
	AT_READ_REG(hw, REG_RXQ_CTRL, &data);
	switch (hw->adapter->num_rx_queues) {
        case 4:
			data |= (RXQ3_CTRL_EN | RXQ2_CTRL_EN | RXQ1_CTRL_EN);
			break;
        case 3:
			data |= (RXQ2_CTRL_EN | RXQ1_CTRL_EN);
			break;
        case 2:
			data |= RXQ1_CTRL_EN;
			break;
        default:
			break;
	}
	data |= RXQ_CTRL_EN;
	AT_WRITE_REG(hw, REG_RXQ_CTRL, data);
}

void atl1c_enable_tx_ctrl(struct atl1c_hw *hw)
{
	u32 data;
	
	AT_READ_REG(hw, REG_TXQ_CTRL, &data);
	data |= TXQ_CTRL_EN;
	AT_WRITE_REG(hw, REG_TXQ_CTRL, data);
}

/*
 * Reset the transmit and receive units; mask and clear all interrupts.
 * hw - Struct containing variables accessed by shared code
 * return : 0  or  idle status (if error)
 */
int atl1c_reset_mac(struct atl1c_hw *hw)
{
	u32 idle_status_data = 0;
	u32 master_ctrl_data = 0;
	int timeout = 0;
	
	AT_WRITE_REG(hw, REG_IMR, 0);
	AT_WRITE_REG(hw, REG_ISR, ISR_DIS_INT);
	
	atl1c_stop_mac(hw);
	/*
	 * Issue Soft Reset to the MAC.  This will reset the chip's
	 * transmit, receive, DMA.  It will not effect
	 * the current PCI configuration.  The global reset bit is self-
	 * clearing, and should clear within a microsecond.
	 */
	AT_READ_REG(hw, REG_MASTER_CTRL, &master_ctrl_data);
	master_ctrl_data |= MASTER_CTRL_OOB_DIS_OFF;
	AT_WRITE_REGW(hw, REG_MASTER_CTRL, ((master_ctrl_data | MASTER_CTRL_SOFT_RST)
										& 0xFFFF));
	msec_delay(10);

	/* Wait at least 10ms for All module to be Idle */
	for (timeout = 0; timeout < AT_HW_MAX_IDLE_DELAY; timeout++) {
		AT_READ_REG(hw, REG_IDLE_STATUS, &idle_status_data);
		if ((idle_status_data & IDLE_STATUS_MASK) == 0)
			break;
		msec_delay(1);
	}
	if (timeout >= AT_HW_MAX_IDLE_DELAY) {
		ErrPrint(
				"MAC state machine cann't be idle since"
				" disabled for 10ms second\n");
		return AT_ERR_TIMEOUT;
	}
	return 0;
}


void atl1c_disable_l0s_l1(struct atl1c_hw *hw)
{
	u32 pm_ctrl_data;
	
	AT_READ_REG(hw, REG_PM_CTRL, &pm_ctrl_data);
	pm_ctrl_data &= ~(PM_CTRL_L1_ENTRY_TIMER_MASK <<
					  PM_CTRL_L1_ENTRY_TIMER_SHIFT);
	pm_ctrl_data &= ~PM_CTRL_CLK_SWH_L1;
	pm_ctrl_data &= ~PM_CTRL_ASPM_L0S_EN;
	pm_ctrl_data &= ~PM_CTRL_ASPM_L1_EN;
	pm_ctrl_data &= ~PM_CTRL_MAC_ASPM_CHK;
	pm_ctrl_data &= ~PM_CTRL_SERDES_PD_EX_L1;
	
	pm_ctrl_data |= PM_CTRL_SERDES_BUDS_RX_L1_EN;
	pm_ctrl_data |= PM_CTRL_SERDES_PLL_L1_EN;
	pm_ctrl_data |=	PM_CTRL_SERDES_L1_EN;
	AT_WRITE_REG(hw, REG_PM_CTRL, pm_ctrl_data);
}


/*
 * Set ASPM state.
 * Enable/disable L0s/L1 depend on link state.
 */
 void atl1c_set_aspm(atl1c_hw *hw, bool linkup)
{
	u32 pm_ctrl_data;
	u32 link_ctrl_data;
	u32 link_l1_timer = 0xF;
	
	AT_READ_REG(hw, REG_PM_CTRL, &pm_ctrl_data);
	AT_READ_REG(hw, REG_LINK_CTRL, &link_ctrl_data);
	
	pm_ctrl_data &= ~PM_CTRL_SERDES_PD_EX_L1;
	pm_ctrl_data &=  ~(PM_CTRL_L1_ENTRY_TIMER_MASK <<
					   PM_CTRL_L1_ENTRY_TIMER_SHIFT);
	pm_ctrl_data &= ~(PM_CTRL_LCKDET_TIMER_MASK << 
					  PM_CTRL_LCKDET_TIMER_SHIFT);
	pm_ctrl_data |= AT_LCKDET_TIMER	<< PM_CTRL_LCKDET_TIMER_SHIFT;
	
	if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l1d ||
		hw->nic_type == athr_l2c_b2) {
		link_ctrl_data &= ~LINK_CTRL_EXT_SYNC;
		if (!(hw->ctrl_flags & ATL1C_APS_MODE_ENABLE)) {
			if (hw->nic_type == athr_l2c_b && hw->revision_id == L2CB_V10)
				link_ctrl_data |= LINK_CTRL_EXT_SYNC;
		} 
		
		AT_WRITE_REG(hw, REG_LINK_CTRL, link_ctrl_data);
		
		pm_ctrl_data |= PM_CTRL_RCVR_WT_TIMER;
		pm_ctrl_data &= ~(PM_CTRL_PM_REQ_TIMER_MASK << 
						  PM_CTRL_PM_REQ_TIMER_SHIFT);
		pm_ctrl_data |= AT_ASPM_L1_TIMER <<  
		PM_CTRL_PM_REQ_TIMER_SHIFT;
		pm_ctrl_data &= ~PM_CTRL_SA_DLY_EN;
		pm_ctrl_data &= ~PM_CTRL_HOTRST;
		pm_ctrl_data |= 1 << PM_CTRL_L1_ENTRY_TIMER_SHIFT;
		pm_ctrl_data |= PM_CTRL_SERDES_PD_EX_L1;  
	}
	pm_ctrl_data |= PM_CTRL_MAC_ASPM_CHK;
	if (linkup) {
		pm_ctrl_data &= ~PM_CTRL_ASPM_L1_EN;
		pm_ctrl_data &= ~PM_CTRL_ASPM_L0S_EN;
		if (hw->ctrl_flags & ATL1C_ASPM_L1_SUPPORT)
			pm_ctrl_data |= PM_CTRL_ASPM_L1_EN;
		if (hw->ctrl_flags & ATL1C_ASPM_L0S_SUPPORT)
			pm_ctrl_data |= PM_CTRL_ASPM_L0S_EN;
		
		if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l1d ||
			hw->nic_type == athr_l2c_b2) {
			if (hw->nic_type == athr_l2c_b)
				if (!(hw->ctrl_flags & ATL1C_APS_MODE_ENABLE))
					pm_ctrl_data &= ~PM_CTRL_ASPM_L0S_EN;
			pm_ctrl_data &= ~PM_CTRL_SERDES_L1_EN;
			pm_ctrl_data &= ~PM_CTRL_SERDES_PLL_L1_EN;
			pm_ctrl_data &= ~PM_CTRL_SERDES_BUDS_RX_L1_EN;
			pm_ctrl_data |= PM_CTRL_CLK_SWH_L1;
			if (hw->adapter->link_speed == SPEED_100 ||
				hw->adapter->link_speed == SPEED_1000) {
				pm_ctrl_data &=  ~(PM_CTRL_L1_ENTRY_TIMER_MASK <<
								   PM_CTRL_L1_ENTRY_TIMER_SHIFT);
				if (hw->nic_type == athr_l2c_b)
					link_l1_timer = 7;
				else if (hw->nic_type == athr_l2c_b2)
					link_l1_timer = 4;
				pm_ctrl_data |= link_l1_timer <<
				PM_CTRL_L1_ENTRY_TIMER_SHIFT;
			}
		} else {
			pm_ctrl_data |= PM_CTRL_SERDES_L1_EN;
			pm_ctrl_data |= PM_CTRL_SERDES_PLL_L1_EN;
			pm_ctrl_data |= PM_CTRL_SERDES_BUDS_RX_L1_EN;
			pm_ctrl_data &= ~PM_CTRL_CLK_SWH_L1;
			pm_ctrl_data &= ~PM_CTRL_ASPM_L0S_EN;
			pm_ctrl_data &= ~PM_CTRL_ASPM_L1_EN;
			
		}
	} else {
		pm_ctrl_data &= ~PM_CTRL_SERDES_L1_EN; 
		pm_ctrl_data &= ~PM_CTRL_ASPM_L0S_EN;
		pm_ctrl_data &= ~PM_CTRL_SERDES_PLL_L1_EN;
		pm_ctrl_data |= PM_CTRL_CLK_SWH_L1;
		
		if (hw->ctrl_flags & ATL1C_ASPM_L1_SUPPORT)
			pm_ctrl_data |= PM_CTRL_ASPM_L1_EN;
		else
			pm_ctrl_data &= ~PM_CTRL_ASPM_L1_EN;
	}
	AT_WRITE_REG(hw, REG_PM_CTRL, pm_ctrl_data);
	
	return;
}

void atl1c_setup_mac_ctrl(atl1c_adapter *adapter)
{
	struct atl1c_hw *hw = &adapter->hw;

	u32 mac_ctrl_data;
	
	mac_ctrl_data = MAC_CTRL_TX_EN | MAC_CTRL_RX_EN;
	mac_ctrl_data |= (MAC_CTRL_TX_FLOW | MAC_CTRL_RX_FLOW);
	
	if (adapter->link_duplex == FULL_DUPLEX) {
		hw->mac_duplex = true;
		mac_ctrl_data |= MAC_CTRL_DUPLX;
	}
	
	if (adapter->link_speed == SPEED_1000)
		hw->mac_speed = atl1c_mac_speed_1000;
	else
		hw->mac_speed = atl1c_mac_speed_10_100;
	
	mac_ctrl_data |= (hw->mac_speed & MAC_CTRL_SPEED_MASK) <<
	MAC_CTRL_SPEED_SHIFT;
	
	mac_ctrl_data |= (MAC_CTRL_ADD_CRC | MAC_CTRL_PAD);
	mac_ctrl_data |= ((hw->preamble_len & MAC_CTRL_PRMLEN_MASK) <<
					  MAC_CTRL_PRMLEN_SHIFT);
	
	
	mac_ctrl_data |= MAC_CTRL_BC_EN;
	
	mac_ctrl_data |= MAC_CTRL_SINGLE_PAUSE_EN;
	if (hw->nic_type == athr_l1d || hw->nic_type == athr_l2c_b2) {
		mac_ctrl_data |= MAC_CTRL_SPEED_MODE_SW;
		mac_ctrl_data |= MAC_CTRL_HASH_ALG_CRC32;
	}	
	AT_WRITE_REG(hw, REG_MAC_CTRL, mac_ctrl_data);
}

/*
 * atl1c_configure - Configure Transmit&Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx /Rx unit of the MAC after a reset.
 */
int atl1c_configure(struct atl1c_adapter *adapter)
{
	struct atl1c_hw *hw = &adapter->hw;
	u32 master_ctrl_data = 0;
	u32 intr_modrt_data;
	u32 data;	
	
	/* clear interrupt status */
	AT_WRITE_REG(hw, REG_ISR, 0xFFFFFFFF);
	/*  Clear any WOL status */
	AT_WRITE_REG(hw, REG_WOL_CTRL, 0);
	/* set Interrupt Clear Timer
	 * HW will enable self to assert interrupt event to system after
	 * waiting x-time for software to notify it accept interrupt.
	 */
	
	data = CLK_GATING_EN_ALL;
	if (hw->ctrl_flags & ATL1C_CLK_GATING_EN) {
		if (hw->nic_type == athr_l2c_b)
			data &= ~CLK_GATING_RXMAC_EN;
	} else
		data = 0;		
	AT_WRITE_REG(hw, REG_CLK_GATING_CTRL, data); 
	
	AT_WRITE_REG(hw, REG_INT_RETRIG_TIMER,
				 hw->ict & INT_RETRIG_TIMER_MASK);
	
	atl1c_configure_des_ring(adapter);
	
	if (hw->ctrl_flags & ATL1C_INTR_MODRT_ENABLE) {
		intr_modrt_data = (hw->tx_imt & IRQ_MODRT_TIMER_MASK) <<
		IRQ_MODRT_TX_TIMER_SHIFT;
		intr_modrt_data |= (hw->rx_imt & IRQ_MODRT_TIMER_MASK) <<
		IRQ_MODRT_RX_TIMER_SHIFT;
		AT_WRITE_REG(hw, REG_IRQ_MODRT_TIMER_INIT, intr_modrt_data);
		master_ctrl_data |=
		MASTER_CTRL_TX_ITIMER_EN | MASTER_CTRL_RX_ITIMER_EN;
	}
	
	if (hw->ctrl_flags & ATL1C_INTR_CLEAR_ON_READ)
		master_ctrl_data |= MASTER_CTRL_INT_RDCLR;
	
	master_ctrl_data |= MASTER_CTRL_SA_TIMER_EN;
	AT_WRITE_REG(hw, REG_MASTER_CTRL, master_ctrl_data);
	
	if (hw->ctrl_flags & ATL1C_CMB_ENABLE) {
		AT_WRITE_REG(hw, REG_CMB_TPD_THRESH,
					 hw->cmb_tpd & CMB_TPD_THRESH_MASK);
		AT_WRITE_REG(hw, REG_CMB_TX_TIMER,
					 hw->cmb_tx_timer & CMB_TX_TIMER_MASK);
	}
	
	if (hw->ctrl_flags & ATL1C_SMB_ENABLE)
		AT_WRITE_REG(hw, REG_SMB_STAT_TIMER,
					 hw->smb_timer & SMB_STAT_TIMER_MASK);
	/* set MTU */
	AT_WRITE_REG(hw, REG_MTU, hw->max_frame_size + ETH_HLEN +
				 VLAN_HLEN + ETH_FCS_LEN);
	/* HDS, disable */
	AT_WRITE_REG(hw, REG_HDS_CTRL, 0);
	
	atl1c_configure_tx(adapter);
	atl1c_configure_rx(adapter);
	atl1c_configure_rss(adapter);
	atl1c_configure_dma(adapter);
	
	return 0;
}

void atl1c_update_hw_stats(struct atl1c_adapter *adapter)
{
	u16 hw_reg_addr = 0;
	unsigned long *stats_item = NULL;
	u32 data;
	
	/* update rx status */
	hw_reg_addr = REG_MAC_RX_STATUS_BIN;
	stats_item  = &adapter->hw_stats.rx_ok;
	while (hw_reg_addr <= REG_MAC_RX_STATUS_END) {
		AT_READ_REG(&adapter->hw, hw_reg_addr, &data);
		*stats_item += data;
		stats_item++;
		hw_reg_addr += 4;
	}
	/* update tx status */
	hw_reg_addr = REG_MAC_TX_STATUS_BIN;
	stats_item  = &adapter->hw_stats.tx_ok;
	while (hw_reg_addr <= REG_MAC_TX_STATUS_END) {
		AT_READ_REG(&adapter->hw, hw_reg_addr, &data);
		*stats_item += data;
		stats_item++;
		hw_reg_addr += 4;
	}
}


void atl1c_clear_phy_int(atl1c_adapter *adapter)
{
	u16 phy_data;
	
	//IOSimpleLockLock(adapter->mdio_lock);
	atl1c_read_phy_reg(&adapter->hw, MII_ISR, &phy_data);
	//IOSimpleLockLock(adapter->mdio_lock);
}



 int atl1c_alloc_rx_buffer(struct atl1c_adapter *adapter, const int ringid)
{
	struct atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring[ringid];

	struct atl1c_buffer *buffer_info, *next_info;

	u16 num_alloc = 0;
	u16 rfd_next_to_use, next_next;
	struct atl1c_rx_free_desc *rfd_desc;
	
	next_next = rfd_next_to_use = rfd_ring->next_to_use;
	if (++next_next == rfd_ring->count)
		next_next = 0;
	buffer_info = &rfd_ring->buffer_info[rfd_next_to_use];
	next_info = &rfd_ring->buffer_info[next_next];
/*	
	for (u32 index = 0; index < rfd_ring->count; index++) {
		buffer_info = &rfd_ring->buffer_info[index];
		rfd_desc = ATL1C_RFD_DESC(rfd_ring, index); 
		buffer_info->memDesc = IOBufferMemoryDescriptor::withOptions(0,adapter->rx_buffer_len);
		if (!buffer_info->memDesc || buffer_info->memDesc->prepare() != kIOReturnSuccess)
		{
			ErrPrint("alloc rx buffer failed\n");
			return -ENOMEM;
		}

		IOByteCount length;
		buffer_info->dma = buffer_info->memDesc->getPhysicalSegment(0, &length);
		buffer_info->state = ATL1_BUFFER_BUSY;
		buffer_info->length = adapter->rx_buffer_len;
		
		rfd_desc->buffer_addr = OSSwapHostToLittleInt64(buffer_info->dma);
		num_alloc++;
	}
*/
	while (next_info->state == ATL1_BUFFER_FREE) {
		rfd_desc = ATL1C_RFD_DESC(rfd_ring, rfd_next_to_use);
		
		if (buffer_info->dma) {
			buffer_info->dma = 0;
		}
		if (buffer_info->memDesc) {
			buffer_info->memDesc->complete();
			buffer_info->memDesc->release();
			buffer_info->memDesc = NULL;
		}
		
		buffer_info->memDesc = IOBufferMemoryDescriptor::withOptions(0,adapter->rx_buffer_len);
		if (!buffer_info->memDesc || buffer_info->memDesc->prepare() != kIOReturnSuccess)
		{
			ErrPrint("alloc rx buffer failed\n");
			return -ENOMEM;
		}
		

		IOByteCount length;
		buffer_info->dma = buffer_info->memDesc->getPhysicalSegment(0, &length);
		//skb_reserve(skb, NET_IP_ALIGN);
		buffer_info->state = ATL1_BUFFER_BUSY;
		buffer_info->length = adapter->rx_buffer_len;

		rfd_desc->buffer_addr = OSSwapHostToLittleInt64(buffer_info->dma);
		rfd_next_to_use = next_next;
		if (++next_next == rfd_ring->count)
			next_next = 0;
		buffer_info = &rfd_ring->buffer_info[rfd_next_to_use];
		next_info = &rfd_ring->buffer_info[next_next];
		num_alloc++;
	}

    if (num_alloc) {
		// TODO: update mailbox here 
		rfd_ring->next_to_use = rfd_next_to_use;
	    AT_WRITE_REG(&adapter->hw, atl1c_rfd_prod_idx_regs[ringid],
					 rfd_ring->next_to_use & MB_RFDX_PROD_IDX_MASK);
		DbgPrint("Total allocated space for rx descriptors D%d; num_alloc= D%d; adapter->rx_buffer_len= D%d\n",
				 num_alloc * adapter->rx_buffer_len,num_alloc , adapter->rx_buffer_len);
	}
	
	return num_alloc? 0 : -1 ;
}

void atl1c_clean_rrd(struct atl1c_rrd_ring *rrd_ring,
							struct	atl1c_recv_ret_status *rrs, u16 num)
{
	u16 i;
	/* the relationship between rrd and rfd is one map one */
	for (i = 0; i < num; i++, rrs = ATL1C_RRD_DESC(rrd_ring,
												   rrd_ring->next_to_clean)) {
		rrs->word3 &= ~RRS_RXD_UPDATED;
		if (++rrd_ring->next_to_clean == rrd_ring->count)
			rrd_ring->next_to_clean = 0;
	}
	return;
}

void atl1c_clean_rfd(struct atl1c_rfd_ring *rfd_ring,
							struct atl1c_recv_ret_status *rrs, u16 num)
{
	u16 i;
	u16 rfd_index;
	struct atl1c_buffer *buffer_info = rfd_ring->buffer_info;
	
	rfd_index = (rrs->word0 >> RRS_RX_RFD_INDEX_SHIFT) &
	RRS_RX_RFD_INDEX_MASK;
	for (i = 0; i < num; i++) {
		if (buffer_info[rfd_index].dma) {
			buffer_info[rfd_index].dma = 0;
		}
		if (buffer_info[rfd_index].memDesc) {
			buffer_info[rfd_index].memDesc->complete();
			buffer_info[rfd_index].memDesc->release();
			buffer_info[rfd_index].memDesc = NULL;
		}
		buffer_info[rfd_index].state = ATL1_BUFFER_FREE;
		if (++rfd_index == rfd_ring->count)
			rfd_index = 0;
	}
	rfd_ring->next_to_clean = rfd_index;
	
	return;
}



u16 atl1c_tpd_avail(struct atl1c_adapter *adapter, enum atl1c_trans_queue type)
{
	struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
	u16 next_to_use = 0;
	u16 next_to_clean = 0;
	
	next_to_clean = atomic_read(&tpd_ring->next_to_clean);
	next_to_use   = tpd_ring->next_to_use;
	
	return (u16)(next_to_clean > next_to_use) ?
	(next_to_clean - next_to_use - 1) :
	(tpd_ring->count + next_to_clean - next_to_use - 1);
}

/*
 * get next usable tpd
 * Note: should call atl1c_tdp_avail to make sure
 * there is enough tpd to use
 */
struct atl1c_tpd_desc *atl1c_get_tpd(struct atl1c_adapter *adapter,
											enum atl1c_trans_queue type)
{
	struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
	struct atl1c_tpd_desc *tpd_desc;
	u16 next_to_use = 0;
	
	next_to_use = tpd_ring->next_to_use;
	if (++tpd_ring->next_to_use == tpd_ring->count)
		tpd_ring->next_to_use = 0;
	tpd_desc = ATL1C_TPD_DESC(tpd_ring, next_to_use);
	memset(tpd_desc, 0, sizeof(struct atl1c_tpd_desc));
	return	tpd_desc;
}

struct atl1c_buffer *
atl1c_get_tx_buffer(struct atl1c_adapter *adapter, struct atl1c_tpd_desc *tpd)
{
	struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;
	
	return &tpd_ring->buffer_info[tpd -
								  (struct atl1c_tpd_desc *)tpd_ring->desc];
}

























