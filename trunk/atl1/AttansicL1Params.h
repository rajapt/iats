#ifndef _ATTANSICL1PARAMS_H_
#define _ATTANSICL1PARAMS_H_

#include <IOKit/IOTypes.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODMACommand.h>

#include <libkern/OSAtomic.h>

extern "C"
{
	#include <sys/kpi_mbuf.h>
}

#include "AttansicL1Hw.h"

#define ETH_ALEN				6 


#define ATL1_MAX_INTR			3

#define ATL1_DEFAULT_TPD		256
#define ATL1_MAX_TPD			1023
#define ATL1_MIN_TPD			64
#define ATL1_DEFAULT_RFD		256
#define ATL1_MIN_RFD			128
#define ATL1_MAX_RFD			2047


#define ATL1_GET_DESC(R, i, type)	(&(((type *)((R)->desc))[i]))
#define ATL1_RFD_DESC(R, i)	ATL1_GET_DESC(R, i, struct rx_free_desc)
#define ATL1_TPD_DESC(R, i)	ATL1_GET_DESC(R, i, struct tx_packet_desc)
#define ATL1_RRD_DESC(R, i)	ATL1_GET_DESC(R, i, struct rx_return_desc)


/*
 * wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct atl1_buffer 
{
	//For rx part
	//mbuf_t packet;
	//For tx part
	IOBufferMemoryDescriptor *memDesc;
	
	UInt16 length;
	UInt16 alloced;
	IOPhysicalAddress dma;
};

//#define MAX_TX_BUF_LEN		0x3000	/* 12KB */
#define MAX_RX_BUF_LEN		1700
#define MAX_TX_BUF_LEN		1700

struct atl1_tpd_ring 
{
	void *desc;					/* pointer to the descriptor ring memory */
	IOPhysicalAddress dma;		/* physical adress of the descriptor ring */
	UInt16 size;				/* length of descriptor ring in bytes */
	UInt16 count;				/* number of descriptors in the ring */
	UInt16 hw_idx;				/* hardware index */
	UInt32 next_to_clean;
	UInt32 next_to_use;
	atl1_buffer *buffer_info;
	UInt32 buffer_size;
};

struct atl1_rfd_ring 
{
	void *desc;
	IOPhysicalAddress dma;
	UInt16 size;
	UInt16 count;
	UInt32 next_to_use;
	UInt16 next_to_clean;
	atl1_buffer *buffer_info;
};

struct atl1_rrd_ring 
{
	void *desc;
	IOPhysicalAddress dma;
	unsigned int size;
	UInt16 count;
	UInt16 next_to_use;
	UInt32 next_to_clean;
};

struct atl1_ring_header 
{
	void *desc;				/* pointer to the descriptor ring memory */
	IOPhysicalAddress dma;	/* physical adress of the descriptor ring */
	unsigned int size;		/* length of descriptor ring in bytes */
	IOBufferMemoryDescriptor *memDesc;
};

struct atl1_cmb 
{
	coals_msg_block *cmb;
	IOPhysicalAddress dma;
};

struct atl1_smb 
{
	stats_msg_block *smb;
	IOPhysicalAddress dma;
};

/* Structure containing variables used by the shared code */
struct AttansicL1Hardware
{
	struct atl1_adapter *back;
	enum atl1_dma_order dma_ord;
	enum atl1_dma_rcb rcb_value;
	enum atl1_dma_req_block dmar_block;
	enum atl1_dma_req_block dmaw_block;
	UInt8 preamble_len;
	UInt8 max_retry;		/* Retransmission maximum, after which the packet will be discarded */
	UInt8 jam_ipg;		/* IPG to start JAM for collision based flow control in half-duplex mode. In units of 8-bit time */
	UInt8 ipgt;		/* Desired back to back inter-packet gap. The default is 96-bit time */
	UInt8 min_ifg;		/* Minimum number of IFG to enforce in between RX frames. Frame gap below such IFP is dropped */
	UInt8 ipgr1;		/* 64bit Carrier-Sense window */
	UInt8 ipgr2;		/* 96-bit IPG window */
	UInt8 tpd_burst;		/* Number of TPD to prefetch in cache-aligned burst. Each TPD is 16 bytes long */
	UInt8 rfd_burst;		/* Number of RFD to prefetch in cache-aligned burst. Each RFD is 12 bytes long */
	UInt8 rfd_fetch_gap;
	UInt8 rrd_burst;		/* Threshold number of RRDs that can be retired in a burst. Each RRD is 16 bytes long */
	UInt8 tpd_fetch_th;
	UInt8 tpd_fetch_gap;
	UInt16 tx_jumbo_task_th;
	UInt16 txf_burst;		/* Number of data bytes to read in a cache-aligned burst. Each SRAM entry is
				   8 bytes long */
	UInt16 rx_jumbo_th;	/* Jumbo packet size for non-VLAN packet. VLAN packets should add 4 bytes */
	UInt16 rx_jumbo_lkah;
	UInt16 rrd_ret_timer;	/* RRD retirement timer. Decrement by 1 after every 512ns passes. */
	UInt16 lcol;		/* Collision Window */

	UInt16 cmb_tpd;
	UInt16 cmb_rrd;
	UInt16 cmb_rx_timer;
	UInt16 cmb_tx_timer;
	UInt32 smb_timer;
	UInt16 media_type;
	UInt16 autoneg_advertised;
	UInt16 pci_cmd_word;

	UInt16 mii_autoneg_adv_reg;
	UInt16 mii_1000t_ctrl_reg;

	UInt32 mem_rang;
	UInt32 txcw;
	UInt32 max_frame_size;
	UInt32 min_frame_size;
	UInt32 mc_filter_type;
	UInt32 num_mc_addrs;
	UInt32 collision_delta;
	UInt32 tx_packet_delta;
	UInt16 phy_spd_default;

	UInt16 dev_rev;
	UInt8 revision_id;

	/* spi flash */
	UInt8 flash_vendor;

	UInt8 dma_fairness;
	UInt8 mac_addr[ETH_ALEN];
	UInt8 perm_mac_addr[ETH_ALEN];

	/* bool phy_preamble_sup; */
	bool phy_configured;
};

struct AttansicL1Adapter 
{
	/* OS defined structs */
	UInt32 rx_buffer_len;
	UInt32 wol;
	UInt16 link_speed;
	UInt16 link_duplex;
	IOSimpleLock *lock;
	SInt32 irq_sem;
	bool phy_timer_pending;

	bool mac_disabled;

	/* All descriptor rings' memory */
	atl1_ring_header ring_header;

	/* TX */
	atl1_tpd_ring tpd_ring;
	IOSimpleLock *mb_lock;

	/* RX */
	atl1_rfd_ring rfd_ring;
	atl1_rrd_ring rrd_ring;
	UInt64 hw_csum_err;
	UInt64 hw_csum_good;

	UInt32 gorcl;
	UInt64 gorcl_old;

	/* Interrupt Moderator timer ( 2us resolution) */
	UInt16 imt;
	/* Interrupt Clear timer (2us resolution) */
	UInt16 ict;

	/* structs defined in atl1_hw.h */
	UInt32 bd_number;		/* board number */
	bool pci_using_64;
	atl1_smb smb;
	atl1_cmb cmb;

	UInt32 pci_state[16];
};


#endif
