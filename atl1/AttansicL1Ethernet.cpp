#include "AttansicL1Ethernet.h"
#include "AttansicL1Debug.h"

#pragma mark -
#pragma mark - Various definitions & helper functions -
#pragma mark -

#define CUSTOM_SPI_CS_SETUP	2
#define CUSTOM_SPI_CLK_HI	2
#define CUSTOM_SPI_CLK_LO	2
#define CUSTOM_SPI_CS_HOLD	2
#define CUSTOM_SPI_CS_HI	3

#define BASE				IOEthernetController
#define RELEASE(x)			do { if(x) { (x)->release(); (x) = 0; } } while(0)
#define MBit				1000000

#if defined(DEBUG)
#define PrintPacket(data)	for(UInt8 i = 0; i < 20; i++)			\
								IOLog("%02x ", data[i]);			\
							IOLog("\n");		
#else
#define PrintPacket(data)
#endif
							


static inline int is_zero_ether_addr(const UInt8 *addr)
{
	return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}

static inline int is_multicast_ether_addr(const UInt8 *addr)
{
	return (0x01 & addr[0]);
}
 
static inline int is_valid_ether_addr(const UInt8 *addr)
{
	/* FF:FF:FF:FF:FF:FF is a multicast address so we don't need to
	* explicitly check for it here. */
	return !is_multicast_ether_addr(addr) && !is_zero_ether_addr(addr);
}

#define CRCPOLY_LE 0xedb88320

UInt32 crc32_le(UInt32 crc, unsigned char const *p, size_t len)
{
	int i;
	while (len--) 
	{
		crc ^= *p++;
		for (i = 0; i < 8; i++)
			crc = (crc >> 1) ^ ((crc & 1) ? CRCPOLY_LE : 0);
	}

	return crc;
}

#define ether_crc_le(length, data) crc32_le(~0, data, length)

OSDefineMetaClassAndStructors(AttansicL1Ethernet, IOEthernetController)

#pragma mark -
#pragma mark - IOService overrides -
#pragma mark -

bool AttansicL1Ethernet::init(OSDictionary *properties)
{
	DbgPrint("AttansicL1Ethernet::init\n");
	if (!BASE::init(properties))
	{
		ErrPrint("Couldn't init BASE\n");
		return false;
	}

	memset(&atl1Adapter_, 0, sizeof(AttansicL1Adapter));
	memset(&atl1Hw_, 0, sizeof(AttansicL1Hardware));	
	pciDev_ = NULL;
	netIface_ = NULL;
	hw_addr = NULL;
	atl1Adapter_.pci_using_64 = false;
	mmr_base = NULL;
	descMemAllocated_ = false;
	
	return true;
}

void AttansicL1Ethernet::free()
{
	DbgPrint("AttansicL1Ethernet::free\n");
	
	if (intSource_ && workLoop_)
	{
		workLoop_->removeEventSource(intSource_);
	}
	
	
	RELEASE(intSource_);
	RELEASE(netIface_);
	RELEASE(pciDev_);
	RELEASE(hw_addr);
	BASE::free();
}

bool AttansicL1Ethernet::start(IOService *provider)
{
	DbgPrint("AttansicL1Ethernet::start\n");
	if (!BASE::start(provider))
	{
		ErrPrint("Couldn't start BASE\n");
		return false;
	}
	
	pciDev_ = OSDynamicCast(IOPCIDevice, provider);
	
	if (!pciDev_)
	{
		ErrPrint("Unable to cast provider\n");
		return false;
	}
	
	pciDev_->retain();
	pciDev_->open(this);
	
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
	
	if (!atl1Probe())	//Fix false reporting int probe function
	{
		ErrPrint("Couldn't probe adapter\n");
		stop(provider);
		return false;	
	}
	
	if (!atl1SoftwareInit())
	{
		ErrPrint("Couldn't init adapter\n");
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
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AttansicL1Ethernet::atl1InterruptHandler),
					pciDev_, msi_index);
	}
	
	if (msi_index == -1 || intSource_ == NULL)
	{
		DbgPrint("MSI index was not found or MSI interrupt couldn't be enabled\n");
		intSource_ = IOInterruptEventSource::interruptEventSource(this,
					OSMemberFunctionCast(IOInterruptEventSource::Action, this, &AttansicL1Ethernet::atl1InterruptHandler),
					pciDev_);
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

	if (!atl1AllocateDescriptors())
	{
		ErrPrint("Couldn't allocate descriptors\n");
		pciDev_->close(this);
		return kIOReturnError;
	}
	
	atl1AllocTxBuffers();
	atl1AllocRxBuffers();
	
	intSource_->enable();
	netIface_->registerService();
	pciDev_->close(this);
	return true;
}

void AttansicL1Ethernet::stop(IOService *provider)
{
	DbgPrint("AttansicL1Ethernet::stop\n");
	regWrite16(REG_GPHY_ENABLE, 0);
	if (intSource_) intSource_->disable();
	atl1FreeDescriptors();	
	this->detachInterface(netIface_);
	atl1SoftwareFree();
	BASE::stop(provider);
}

bool AttansicL1Ethernet::OSAddNetworkMedium(UInt32 type, UInt32 bps, UInt32 index)
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

IOOutputQueue *AttansicL1Ethernet::createOutputQueue()
{
	DbgPrint("AttansicL1Ethernet::createOutputQueue\n");
	//Sharing one event source with transmith/receive handles
	return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

bool AttansicL1Ethernet::atl1AllocateDescriptors()
{
	DbgPrint("AttansicL1Ethernet::atl1AllocateDescriptors\n");

	atl1_tpd_ring *tpd_ring = &atl1Adapter_.tpd_ring;
	atl1_rfd_ring *rfd_ring = &atl1Adapter_.rfd_ring;
	atl1_rrd_ring *rrd_ring = &atl1Adapter_.rrd_ring;
	atl1_ring_header *ring_header = &atl1Adapter_.ring_header;
	int size;
	UInt8 offset = 0;

	size = sizeof(struct atl1_buffer) * (tpd_ring->count + rfd_ring->count);
	DbgPrint("tpd_ring_count=%d rfd_ring_count=%d size=%d\n",tpd_ring->count,rfd_ring->count,size);
	tpd_ring->buffer_size = size;
	tpd_ring->buffer_info = (atl1_buffer *)IOMalloc(size);
	memset(tpd_ring->buffer_info, 0, size);
	
	if (!tpd_ring->buffer_info)
	{
		ErrPrint("No memory for tpd_ring");
		return false;
	}
	
	rfd_ring->buffer_info = (struct atl1_buffer *)&tpd_ring->buffer_info[tpd_ring->count];

	/* real ring DMA buffer */
	ring_header->size = size = sizeof(struct tx_packet_desc) * tpd_ring->count
	    + sizeof(struct rx_free_desc) * rfd_ring->count
	    + sizeof(struct rx_return_desc) * rrd_ring->count
	    + sizeof(struct coals_msg_block)
	    + sizeof(struct stats_msg_block)
	    + 40;		/* "40: for 8 bytes align" huh? -- CHS */
		
	ring_header->memDesc = IOBufferMemoryDescriptor::withOptions(kIOMemoryPhysicallyContiguous|
																 kIODirectionOut|kIODirectionIn,
																	ring_header->size,
																	PAGE_SIZE);
	DbgPrint("Allocated memory for ring header %d\n", ring_header->size);
	if (!ring_header->memDesc || (ring_header->memDesc->prepare() != kIOReturnSuccess))
	{
		IOSleep(1500);
		ErrPrint("Couldn't alloc memory for descriptor ring\n");
		if (ring_header->memDesc)
		{
			ring_header->memDesc->release();
			ring_header->memDesc = NULL;
		}
		IOFree(tpd_ring->buffer_info, tpd_ring->buffer_size);
	}
	
	IOByteCount dmaLength = 0;
	ring_header->dma = ring_header->memDesc->getPhysicalSegment(0, &dmaLength);
	ring_header->desc = ring_header->memDesc->getBytesNoCopy();
	DbgPrint("Physical segment address %X pointer %p length %d\n", 
			ring_header->dma, ring_header->desc, dmaLength);
	memset(ring_header->desc, 0, ring_header->size);
	
	offset = 0;
	/* init TPD ring */
	tpd_ring->dma = ring_header->dma;
	offset = (tpd_ring->dma & 0x7) ? (8 - (ring_header->dma & 0x7)) : 0;
	tpd_ring->dma += offset;
	tpd_ring->desc = (UInt8 *)ring_header->desc + offset;
	tpd_ring->size = sizeof(struct tx_packet_desc) * tpd_ring->count;
	tpd_ring->next_to_use = 0;
	tpd_ring->next_to_clean = 255;
	
	/* init RFD ring */
	rfd_ring->dma = tpd_ring->dma + tpd_ring->size;
	offset = (rfd_ring->dma & 0x7) ? (8 - (rfd_ring->dma & 0x7)) : 0;
	rfd_ring->dma += offset;
	rfd_ring->desc = (UInt8 *)tpd_ring->desc + (tpd_ring->size + offset);
	rfd_ring->size = sizeof(struct rx_free_desc) * rfd_ring->count;
	rfd_ring->next_to_clean = 0;
    rfd_ring->next_to_use = rfd_ring->count - 1; // ???madint
	//rfd_ring->next_to_use = 0;
	
	/* init RRD ring */
	rrd_ring->dma = rfd_ring->dma + rfd_ring->size;
	offset = (rrd_ring->dma & 0x7) ? (8 - (rrd_ring->dma & 0x7)) : 0;
	rrd_ring->dma += offset;
	rrd_ring->desc = (UInt8 *)rfd_ring->desc + (rfd_ring->size + offset);
	rrd_ring->size = sizeof(struct rx_return_desc) * rrd_ring->count;
	rrd_ring->next_to_use = 0;
	rrd_ring->next_to_clean = 0; 
	
	/* init CMB */
	atl1Adapter_.cmb.dma = rrd_ring->dma + rrd_ring->size;
	offset = (atl1Adapter_.cmb.dma & 0x7) ? (8 - (atl1Adapter_.cmb.dma & 0x7)) : 0;
	atl1Adapter_.cmb.dma += offset;
	atl1Adapter_.cmb.cmb = (struct coals_msg_block *)((UInt8 *)rrd_ring->desc + (rrd_ring->size + offset));	
	
	/* init SMB */
	atl1Adapter_.smb.dma = atl1Adapter_.cmb.dma + sizeof(struct coals_msg_block);
	offset = (atl1Adapter_.smb.dma & 0x7) ? (8 - (atl1Adapter_.smb.dma & 0x7)) : 0;
	atl1Adapter_.smb.dma += offset;
	atl1Adapter_.smb.smb = (struct stats_msg_block *)((UInt8 *) atl1Adapter_.cmb.cmb + 
					(sizeof(struct coals_msg_block) + offset));

	descMemAllocated_ = true;
	return true;
}

void AttansicL1Ethernet::atl1FreeDescriptors()
{
	DbgPrint("AttansicL1Ethernet::atl1FreeDescriptors\n");
	if (!descMemAllocated_) return ;
	
	struct atl1_tpd_ring *tpd_ring = &atl1Adapter_.tpd_ring;
	struct atl1_rfd_ring *rfd_ring = &atl1Adapter_.rfd_ring;
	struct atl1_rrd_ring *rrd_ring = &atl1Adapter_.rrd_ring;
	struct atl1_ring_header *ring_header = &atl1Adapter_.ring_header;

	atl1CleanTxRing();
	atl1CleanRxRing();

	if (tpd_ring->buffer_info) IOFree(tpd_ring->buffer_info, tpd_ring->buffer_size);
	
	if (ring_header->memDesc)
	{
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

UInt16 AttansicL1Ethernet::atl1AllocRxBuffers()
{
	DbgPrint("AttansicL1Ethernet::atl1AllocRxBuffers\n");

	atl1_rfd_ring *rfd_ring = &atl1Adapter_.rfd_ring;
	atl1_buffer *buffer_info, *next_info;
	UInt16 num_alloc = 0;
	UInt16 rfd_next_to_use, next_next;
	rx_free_desc *rfd_desc;
	
	next_next = rfd_next_to_use = rfd_ring->next_to_use;
	if (++next_next == rfd_ring->count)
		next_next = 0;
	buffer_info = &rfd_ring->buffer_info[rfd_next_to_use];
	next_info = &rfd_ring->buffer_info[next_next];	
	
	for (UInt32 i = 0; i < rfd_ring->count; i++) 
	{
		buffer_info = &rfd_ring->buffer_info[i];
		rfd_desc = &((rx_free_desc *)rfd_ring->desc)[i]; //ATL1_RFD_DESC(rfd_ring, i);
		// madint
		buffer_info->memDesc = IOBufferMemoryDescriptor::withOptions( kIOMemoryPhysicallyContiguous|
																 kIODirectionOut|kIODirectionIn,
																	  MAX_RX_BUF_LEN,PAGE_SIZE);

		if (!buffer_info->memDesc || buffer_info->memDesc->prepare() != kIOReturnSuccess)
		{
			ErrPrint("Couldn't allocate o prepare memory for transmitting");
			return false;
		}
		IOByteCount length;
		
		buffer_info->alloced = 1;
		buffer_info->length = (UInt16)MAX_RX_BUF_LEN;
		buffer_info->dma = buffer_info->memDesc->getPhysicalSegment(0, &length);
		rfd_desc->buffer_addr = OSSwapHostToLittleInt64(buffer_info->dma);
		rfd_desc->buf_len = OSSwapHostToLittleInt16((UInt16)buffer_info->length);
		rfd_desc->coalese = 0;
	    DbgPrint("Allocated rx free buffer %d/%d address %X size %d\n", 
					i, rfd_ring->count, buffer_info->dma, buffer_info->length);
		num_alloc++;
	}
	DbgPrint("Total allocated space for rx descriptors %d\n", num_alloc * MAX_RX_BUF_LEN);
	
	/*

	while (!buffer_info->alloced && !next_info->alloced) 
	{
		if (buffer_info->packet)
		{
			buffer_info->alloced = 1;
		}
		else
		{
			rfd_desc = ATL1_RFD_DESC(rfd_ring, rfd_next_to_use);
			packet = allocatePacket(atl1Adapter_.rx_buffer_len);
			
			
			if (!buffer_info->packet)
			{
				//Add increment of rx dropped counter
				break;
			}
			
			buffer_info->memoryDescriptor = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
												//Task to hold memory
												kernel_task,
												//options
												kIOMemoryPhysicallyContiguous,
												//size
												atl1Adapter_.rx_buffer_len,
												//physical mask, 32 bit addressablr and page aligned 
												0x00000000FFFFF000ULL);
			if (!buffer_info->memoryDescriptor)
			{
				//Add increment of rx dropped counter
				break;
			}
			
			
			buffer_info->dmaCommand = IODMACommand::withSpecification(
											kIODMACommandOutputHost32,
											//num address bits
											32,
											//max segment size
											0,
											//mappingOptions - kMaped for DMA addresses
											IODMACommand::kMapped,
											//max transfer size
											0,
											//alignment - no restiction
											1);
							
			if (!buffer_info->dmaCommand)
			{
				buffer_info->memoryDescriptor->release();
				buffer_info->memoryDescriptor = NULL;
				//increment rx dropped
				break;
			}
			
			IOReturn ret;
			ret = buffer_info->dmaCommand->setMemoryDescriptor(buffer_info->memoryDescriptor);
			
			if (ret != kIOReturnSuccess)
			{
				RELEASE(buffer_info->memoryDescriptor);
				RELEASE(buffer_info->dmaCommand);
			}
			
			IODMACommand::Segment32 segments[1];
			UInt32 numSeg = 1;
			
			

			buffer_info->alloced = 1;
			buffer_info->packet = packet;
			buffer_info->length = (UInt16)atl1Adapter_.rx_buffer_len;
			rfd_desc->buffer_addr = OSSwapHostToLittleInt64(mbuf_data_to_physical(mbuf_data(packet)));
			rfd_desc->buf_len = OSSwapHostToLittleInt16(atl1Adapter_.rx_buffer_len);
			rfd_desc->coalese = 0;
		}
		
		rfd_next_to_use = next_next;
		
		if (++next_next == rfd_ring->count) next_next = 0;
		buffer_info = &rfd_ring->buffer_info[rfd_next_to_use];
		next_info = &rfd_ring->buffer_info[next_next];
		num_alloc++;
	}*/
	
	if (num_alloc)
	{
		DbgPrint("Allocated memory for buffers %d\n", num_alloc*atl1Adapter_.rx_buffer_len);
		OSSynchronizeIO();
		//rfd_ring->next_to_use = rfd_next_to_use;
	}

	return num_alloc;
}

bool AttansicL1Ethernet::atl1AllocTxBuffers()
{
	atl1_tpd_ring *tpd_ring = &atl1Adapter_.tpd_ring;
	atl1_buffer *buffer_info;
	unsigned int i;
	UInt16 num_alloc = 0;
	tx_packet_desc *tpd_desc;

	for (i = 0; i < tpd_ring->count; i++) 
	{
		buffer_info = &tpd_ring->buffer_info[i];
		tpd_desc = ATL1_TPD_DESC(tpd_ring, i);
		buffer_info->memDesc = IOBufferMemoryDescriptor::withOptions(0,
																	  MAX_TX_BUF_LEN);

		if (!buffer_info->memDesc || buffer_info->memDesc->prepare() != kIOReturnSuccess)
		{
			ErrPrint("Couldn't allocate o prepare memory for transmitting");
			return false;
		}
		IOByteCount length;
		
		buffer_info->alloced = 1;
		buffer_info->dma = buffer_info->memDesc->getPhysicalSegment(0, &length);
		tpd_desc->buffer_addr = OSSwapHostToLittleInt64(buffer_info->dma);
		num_alloc++;
	}
	DbgPrint("Total allocated space for tx descriptors %d\n", num_alloc * MAX_TX_BUF_LEN);
	
	return true;
}

void AttansicL1Ethernet::atl1CleanRxRing()
{
	DbgPrint("AttansicL1Ethernet::atl1CleanRxRing\n");
	atl1_rfd_ring *rfd_ring = &atl1Adapter_.rfd_ring;
	atl1_rrd_ring *rrd_ring = &atl1Adapter_.rrd_ring;
	atl1_buffer *buffer_info;
	unsigned long size;
	unsigned int i;
	
	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rfd_ring->count; i++) 
	{
		buffer_info = &rfd_ring->buffer_info[i];
		buffer_info->dma = 0;

		if (buffer_info->dma) 
		{
			buffer_info->dma = 0;
		}
		if (buffer_info->memDesc)
		{
			buffer_info->memDesc->complete();
			buffer_info->memDesc->release();
			buffer_info->memDesc = NULL;
		}
	}
	
	size = sizeof(struct atl1_buffer) * rfd_ring->count;
	memset(rfd_ring->buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(rfd_ring->desc, 0, rfd_ring->size);

	rfd_ring->next_to_clean = 0;
	rfd_ring->next_to_use = 0;

	rrd_ring->next_to_use = 0;
	rrd_ring->next_to_clean = 0;
}

void AttansicL1Ethernet::atl1CleanTxRing()
{
	DbgPrint("AttansicL1Ethernet::atl1CleanTxRing\n");

	atl1_tpd_ring *tpd_ring = &atl1Adapter_.tpd_ring;
	atl1_buffer *buffer_info;
	unsigned long size;
	unsigned int i;
	
	/* Free all the Tx ring sk_buffs */
	for (i = 0; i < tpd_ring->count; i++) 
	{
		buffer_info = &tpd_ring->buffer_info[i];
		if (buffer_info->dma) 
		{
			buffer_info->dma = 0;
		}
		if (buffer_info->memDesc)
		{
			buffer_info->memDesc->complete();
			buffer_info->memDesc->release();
			buffer_info->memDesc = NULL;
		}
	}
	
	size = sizeof(struct atl1_buffer) * tpd_ring->count;
	memset(tpd_ring->buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(tpd_ring->desc, 0, tpd_ring->size);

	tpd_ring->next_to_use = 0;
	tpd_ring->next_to_clean = 0;
}

void AttansicL1Ethernet::atl1RxInterrupt()
{
	//DbgPrint("AttansicL1Ethernet::atl1RxInterrupt\n");

	int i, count;
	UInt16 length, rrd_prod_idx, rfd_cons_idx; 
	UInt16 rrd_next_to_clean;
	UInt32 value;
	atl1_rfd_ring *rfd_ring = &atl1Adapter_.rfd_ring;
	atl1_rrd_ring *rrd_ring = &atl1Adapter_.rrd_ring;
	atl1_buffer *buffer_info;
	rx_return_desc *rrd;
	mbuf_t packet;

	count = 0;

	rrd_next_to_clean = rrd_ring->next_to_clean;
	rrd_prod_idx = OSSwapLittleToHostInt16(atl1Adapter_.cmb.cmb->rrd_prod_idx);
	rfd_cons_idx = OSSwapLittleToHostInt16(atl1Adapter_.cmb.cmb->rfd_cons_idx);
	
	//DbgPrint("-- rrd_next_to_clean=%d rrd_prod_idx=%d rfd_cons_idx=%d rfd_next_to_use=%d\n",
	//		 rrd_next_to_clean, rrd_prod_idx, rfd_cons_idx, rfd_ring->next_to_use);
	if(rrd_next_to_clean == rrd_prod_idx)
	{
		//DbgPrint("!! Card failed to produce another return descriptor. Huh?\n");
		// so patiently wait for another interrupt
		return;
		//rrd_next_to_clean = 0;
	}
	/*
	for(i = 0; i < rrd_ring->count; i ++)
	{
		rrd = &((rx_return_desc *)rrd_ring->desc)[i];
		if(rrd->xsz.valid) 
			DbgPrint("-- RRD DUMP: rrd entry %d seems to be valid (0x%04X)...\n", i, rrd->xsz.valid);
		if(rrd->num_buf)
			DbgPrint("-- RRD DUMP: rrd entry %d strange: num_buf=%d buf_indx=%d\n", i,
					rrd->num_buf, OSSwapLittleToHostInt16(rrd->buf_indx));
	}*/
	
	while (rrd_next_to_clean != rrd_prod_idx)
	{
		//DbgPrint("count=%d rrd_next_to_clean=%d\n",count,rrd_next_to_clean);
		rrd = &((rx_return_desc *)rrd_ring->desc)[rrd_next_to_clean]; //ATL1_RRD_DESC(rrd_ring, rrd_next_to_clean);
		i = 1;
		
		if (rrd->num_buf > 0)	
		{
			if (rrd->num_buf > 1)
			{			
				//rrd seems to be bad
				while (i-- > 0)
				{
					//rrd may not be DMAed completely
					//ErrPrint("RRD may not be DMAed completely\n");
					IODelay(1);
				}
				//bad rrd
				//DbgPrint("Bad RRD\n");
				//see if update RFD index

				UInt16 num_buf;
				num_buf = (OSSwapLittleToHostInt16(rrd->xsz.xsum_sz.pkt_size) + atl1Adapter_.rx_buffer_len - 1) / atl1Adapter_.rx_buffer_len;

				if (rrd->num_buf == num_buf) 
				{
					//clean alloc flag for bad rrd
					while (rfd_ring->next_to_clean != (OSSwapLittleToHostInt16(rrd->buf_indx) + num_buf)) 
					{
						rfd_ring->buffer_info[rfd_ring->next_to_clean].alloced = 0;
						if (++rfd_ring->next_to_clean == rfd_ring->count) 
						{
							rfd_ring->next_to_clean = 0;
						}
					}
				}
				rrd->xsz.valid = 0;
				rrd->buf_indx = 0;
				rrd->resved = 0;
				rrd->num_buf = 0;
							
				if (++rrd_next_to_clean >= rrd_ring->count) rrd_next_to_clean = 0;
				
				count++;
				continue;
			}
			
			//clean alloc flag for bad rrd
		
			while (rfd_ring->next_to_clean != OSSwapLittleToHostInt16(rrd->buf_indx)) 
			{
				rfd_ring->buffer_info[rfd_ring->next_to_clean].alloced = 0;
				if (++rfd_ring->next_to_clean >= rfd_ring->count) rfd_ring->next_to_clean = 0;
			}
		
			buffer_info = &rfd_ring->buffer_info[OSSwapLittleToHostInt16(rrd->buf_indx)];
			if (++rfd_ring->next_to_clean >= rfd_ring->count) rfd_ring->next_to_clean = 0;

			//update rrd next to clean
			if (++rrd_next_to_clean >= rrd_ring->count) rrd_next_to_clean = 0;
			count++;
		
			/*if (rrd->pkt_flg & PACKET_FLAG_ERR) 
			{
				if (!(rrd->err_flg & (ERR_FLAG_IP_CHKSUM | ERR_FLAG_L4_CHKSUM | ERR_FLAG_LEN))) 
				{
					//packet error, don't need upstream
					buffer_info->alloced = 0;
					rrd->xsz.valid = 0;
					continue;
				}
			}*/
			//packet = buffer_info->packet;	//TO-DO: Fix this
			length = OSSwapLittleToHostInt16(rrd->xsz.xsum_sz.pkt_size);
			packet = allocatePacket(MAX_RX_BUF_LEN);
			if (!packet) continue;
			if (buffer_info->memDesc)
			{
				bcopy(buffer_info->memDesc->getBytesNoCopy(), mbuf_data(packet), length);
			}
			//DbgPrint("++ Received packet!!! length %d\n", length);
		
			//UInt8 *tmp = (UInt8 *)mbuf_data(packet);			//Debug only
			//if (length >= 20) PrintPacket(tmp);
		
			netIface_->inputPacket(packet, length - 4 , IONetworkInterface::kInputOptionQueuePacket);
			netIface_->flushInputQueue();
		
			if(++rfd_ring->next_to_use >= rfd_ring->count) rfd_ring->next_to_use = 0;

			rrd->buf_indx = 0;
			rrd->resved = 0;
			rrd->num_buf = 0;
			//buffer_info->packet = NULL;
			rrd->xsz.valid = 0;
		} else break;
	}
	
	//rrd_next_to_clean = 0 ;
	rrd_ring->next_to_clean = rrd_next_to_clean;
	
	//atl1AllocRxBuffers();
	
	/* update mailbox ? */
	if (count) 
	{
		UInt32 tpd_next_to_use;
//		UInt32 rfd_next_to_use;
		//UInt32 rrd_next_to_clean;

		tpd_next_to_use = atl1Adapter_.tpd_ring.next_to_use;
		//rfd_next_to_use = atl1Adapter_.rfd_ring.next_to_use;
		//rrd_next_to_clean = atl1Adapter_.rrd_ring.next_to_clean; 
		

		value = ((rfd_ring->next_to_use & MB_RFD_PROD_INDX_MASK) << MB_RFD_PROD_INDX_SHIFT) |
                        ((rrd_next_to_clean & MB_RRD_CONS_INDX_MASK) << MB_RRD_CONS_INDX_SHIFT) |
                        ((tpd_next_to_use & MB_TPD_PROD_INDX_MASK) << MB_TPD_PROD_INDX_SHIFT);
						
//		DbgPrint("--- packets handled=%d, updating mailbox rrd_next_to_clean=%d rfd_next_to_use=%d value=0x%04X\n", 
//			count, rrd_next_to_clean, rfd_next_to_use, value);
		regWrite32(REG_MAILBOX, value);
	}
}

void AttansicL1Ethernet::atl1TxInterrupt()
{

	//return; // TMP!

	//DbgPrint("AttansicL1Ethernet::atl1TxInterrupt\n");
	
	atl1_tpd_ring *tpd_ring = &atl1Adapter_.tpd_ring;
	atl1_buffer *buffer_info;
	
	UInt16 sw_tpd_next_to_clean;
	UInt16 cmb_tpd_next_to_clean;
	UInt8 update = 0;
	
	sw_tpd_next_to_clean = tpd_ring->next_to_clean;
	cmb_tpd_next_to_clean = OSSwapLittleToHostInt16(atl1Adapter_.cmb.cmb->tpd_cons_idx);
	
	//DbgPrint("sw_tpd_next_to_clean %d, cmb_tpd_next_to_clean %d, next_to_use %d\n", 
	//	sw_tpd_next_to_clean, cmb_tpd_next_to_clean, tpd_ring->next_to_use);
	while(cmb_tpd_next_to_clean != sw_tpd_next_to_clean)
	{
		tx_packet_desc *tpd;
		update = 1;
		tpd = ATL1_TPD_DESC(tpd_ring, sw_tpd_next_to_clean);
		//tpd->buffer_addr = 0;
		//tpd->desc.data = 0;
		//memset(&tpd->desc, 0, sizeof(&tpd->desc));
		buffer_info = &tpd_ring->buffer_info[sw_tpd_next_to_clean];
		//TO-DO: Add cleaning
		if (++sw_tpd_next_to_clean >= tpd_ring->count) sw_tpd_next_to_clean = 0;
	}
	
	tpd_ring->next_to_clean = sw_tpd_next_to_clean;
}

void AttansicL1Ethernet::atl1InterruptHandler(OSObject *client, IOInterruptEventSource *src, int count)
{
	UInt32 status;
	UInt8 update_rx;
	int max_ints = 10;
	UInt16 phy_data = 0;
	
	status = atl1Adapter_.cmb.cmb->int_stats;
	update_rx = 0;
	
	do
	{
			/* clear CMB interrupt status at once */
		atl1Adapter_.cmb.cmb->int_stats = 0;

		if (status & ISR_GPHY)	/* clear phy status */
		{
			atl1ReadPhyReg(19, &phy_data);
			DbgPrint("link event (phy_data=0x%02X status=0x%04X)!\n", phy_data, status);
			IOSleep(2);
			atl1GetAndUpdateLinkStatus();
		}

		/* clear ISR status, and Enable CMB DMA/Disable Interrupt */
		regWrite32(REG_ISR, status | ISR_DIS_INT);
		
		/* transmit event */
		if (status & ISR_CMB_TX) atl1TxInterrupt();
		
		/* rx exception */
		if (status & (ISR_RXF_OV | ISR_RFD_UNRUN | ISR_RRD_OV | ISR_HOST_RFD_UNRUN | 
				ISR_HOST_RRD_OV | ISR_CMB_RX)) 
		{
			if (status & (ISR_RXF_OV | ISR_RFD_UNRUN | ISR_RRD_OV |  ISR_HOST_RFD_UNRUN | ISR_HOST_RRD_OV))
				ErrPrint(">>> rx exception: status = 0x%x\n", status);
			atl1RxInterrupt();	
		}

		if(status & ISR_SMB)
		{
			DbgPrint("SMB RX: rx_ok=%d rx_bcast=%d rx_mcast=%d rx_sz_ov=%d rx_rxf_ov=%d rx_rrd_ov=%d rx_align_err=%d\n",
					atl1Adapter_.smb.smb->rx_ok,atl1Adapter_.smb.smb->rx_bcast,
					atl1Adapter_.smb.smb->rx_mcast, atl1Adapter_.smb.smb->rx_sz_ov,
					atl1Adapter_.smb.smb->rx_rxf_ov,atl1Adapter_.smb.smb->rx_rrd_ov,
					atl1Adapter_.smb.smb->rx_align_err);
			DbgPrint("SMB TX: tx_ok=%d tx_exc_defer=%d tx_defer=%d tx_bcast=%d tx_mcast=%d tx_underrun=%d tx_rd_eop=%d tx_trunc=%d\n",	
					atl1Adapter_.smb.smb->tx_ok, atl1Adapter_.smb.smb->tx_exc_defer,
					atl1Adapter_.smb.smb->tx_defer, atl1Adapter_.smb.smb->tx_bcast,
					atl1Adapter_.smb.smb->tx_mcast,atl1Adapter_.smb.smb->tx_underrun,
					atl1Adapter_.smb.smb->tx_rd_eop,atl1Adapter_.smb.smb->tx_trunc);
			atl1Adapter_.smb.smb->smb_updated = 0;
		}
		

		if (--max_ints < 0)
		{
			DbgPrint("maximum interrupts reached\n");
			break;
		}
	}
	while ((status = atl1Adapter_.cmb.cmb->int_stats));
	
	/* re-enable Interrupt */
	regWrite32(REG_ISR, ISR_DIS_SMB | ISR_DIS_DMA);
}

#pragma mark -
#pragma mark - IOEthernetController overrides -
#pragma mark -
	
IOReturn AttansicL1Ethernet::enable(IONetworkInterface *netif)
{
	DbgPrint("AttansicL1Ethernet::enable\n");
	
	pciDev_->open(this);
		
	//atl1AllocRxBuffers();
	
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
		

	atl1Configure();
	selectMedium(medium);

	transmitQueue_->setCapacity(kTransmitQueueCapacity);
	transmitQueue_->start();
	atl1InterruptsEnable();
		
	return kIOReturnSuccess;
}

IOReturn AttansicL1Ethernet::disable(IONetworkInterface *netif)
{
	DbgPrint("AttansicL1Ethernet::disable\n");
	
	transmitQueue_->stop();
	transmitQueue_->setCapacity(0);
	transmitQueue_->flush();

	atl1ResetHw();
	atl1InterruptsDisable();

// pretty wrong place for DMA stuff cleaning, eh?
	atl1CleanTxRing();
	atl1CleanRxRing();

	if (pciDev_) pciDev_->close(this);	
	return kIOReturnSuccess;
}

bool AttansicL1Ethernet::configureInterface(IONetworkInterface *netif)
{
	DbgPrint("AttansicL1Ethernet::configureInterface\n");
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

const OSString *AttansicL1Ethernet::newVendorString() const
{
	return OSString::withCString("Attansic");
}

const OSString *AttansicL1Ethernet::newModelString() const
{
	return OSString::withCString("L1 Gigabit LAN");
}

IOReturn AttansicL1Ethernet::selectMedium(const IONetworkMedium *medium)
{
	DbgPrint("AttansicL1Ethernet::selectMedium\n");

	if (medium)
	{
		switch(medium->getIndex())
		{
		case MEDIUM_INDEX_AUTO:
			DbgPrint("Selected medium is autoselect\n");
			atl1Adapter_.link_speed = SPEED_1000;
			atl1Adapter_.link_duplex = FULL_DUPLEX;
			atl1Hw_.media_type = MEDIA_TYPE_AUTO_SENSOR;
			break;
		case MEDIUM_INDEX_10HD:
			DbgPrint("Selected medium is 10HD\n");
			atl1Adapter_.link_speed = SPEED_10;
			atl1Adapter_.link_duplex = HALF_DUPLEX;	
			atl1Hw_.media_type = MEDIA_TYPE_10M_HALF;
			break;
		case MEDIUM_INDEX_10FD:
			DbgPrint("Selected medium is 10FD\n");
			atl1Adapter_.link_speed = SPEED_10;
			atl1Adapter_.link_duplex = FULL_DUPLEX;	
			atl1Hw_.media_type = MEDIA_TYPE_10M_FULL;
			break;
		case MEDIUM_INDEX_100HD:
			DbgPrint("Selected medium is 100HD\n");
			atl1Adapter_.link_speed = SPEED_100;
			atl1Adapter_.link_duplex = HALF_DUPLEX;		
			atl1Hw_.media_type = MEDIA_TYPE_100M_HALF;
			break;
		case MEDIUM_INDEX_100FD:
			DbgPrint("Selected medium is 100FD\n");
			atl1Adapter_.link_speed = SPEED_100;
			atl1Adapter_.link_duplex = FULL_DUPLEX;		
			atl1Hw_.media_type = MEDIA_TYPE_100M_FULL;
			break;
		case MEDIUM_INDEX_1000HD:
			DbgPrint("Selected medium is 1000HD\n");
			atl1Adapter_.link_speed = SPEED_1000;
			atl1Adapter_.link_duplex = HALF_DUPLEX;	
			atl1Hw_.media_type = MEDIA_TYPE_1000M_FULL;
			break;
		case MEDIUM_INDEX_1000FD:
			DbgPrint("Selected medium is 1000FD\n");
			atl1Adapter_.link_speed = SPEED_1000;
			atl1Adapter_.link_duplex = FULL_DUPLEX;		
			atl1Hw_.media_type = MEDIA_TYPE_1000M_FULL;
			break;
		}	
		atl1SetupLink();
		//atl1PhySetupAutonegAdv();
		//atl1SetupMacCtrl();
		setCurrentMedium(medium);
	}
	else 
	{
		DbgPrint("Selected medium is NULL\n");
		return kIOReturnError;
	}
	
	//Refresh link status
	IOSleep(100*((medium->getIndex() == MEDIUM_INDEX_AUTO)? PHY_AUTO_NEG_TIME:PHY_FORCE_TIME));
	atl1GetAndUpdateLinkStatus();

	return kIOReturnSuccess;
}

IOReturn AttansicL1Ethernet::getHardwareAddress(IOEthernetAddress *addr)
{
	DbgPrint("AttansicL1Ethernet::getHardwareAddress\n");
	IOSleep(1000);
	
	if (is_valid_ether_addr(atl1Hw_.mac_addr))
	{
		memcpy(addr->bytes, atl1Hw_.mac_addr, ETH_ALEN);
		return kIOReturnSuccess;
	}
	
	if (atl1GetPermanentAddress() == 0)
	{
		if (is_valid_ether_addr(atl1Hw_.perm_mac_addr))
		{
			memcpy(atl1Hw_.mac_addr, atl1Hw_.perm_mac_addr, ETH_ALEN);
			memcpy(addr->bytes, atl1Hw_.mac_addr, ETH_ALEN);
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
		memcpy(atl1Hw_.mac_addr, addr->bytes, ETH_ALEN);
		//return kIOReturnUnsupported;
		return kIOReturnSuccess;
	}
}

IOReturn AttansicL1Ethernet::setHardwareAddress(const IOEthernetAddress *addr)
{
	memcpy(atl1Hw_.mac_addr, addr->bytes, ETH_ALEN);
	atl1SetMacAddr();
	return kIOReturnSuccess;
}

void AttansicL1Ethernet::getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const
{
	DbgPrint("AttansicL1Ethernet::getPacketBufferConstraints\n");
	constraints->alignStart = kIOPacketBufferAlign4;
	constraints->alignLength = kIOPacketBufferAlign4;
}

UInt32 AttansicL1Ethernet::outputPacket(mbuf_t m, void *prm)
{
	UInt32 buf_len;
	atl1_tpd_ring *tpd_ring = &atl1Adapter_.tpd_ring;
	UInt32 tpd_next_to_use = tpd_ring->next_to_use;
	tx_packet_desc *tpd = ATL1_TPD_DESC(&atl1Adapter_.tpd_ring, tpd_next_to_use);
	atl1_buffer *buffer_info = &tpd_ring->buffer_info[tpd_next_to_use];

	//return kIOReturnOutputDropped; // TMP!

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
	
	DbgPrint("AttansicL1Ethernet::outputPacket, length %d tp_next_to_use=%d\n", buf_len, tpd_next_to_use);
	//IOSleep(100);
	
	UInt8 *data_ptr = (UInt8 *)buffer_info->memDesc->getBytesNoCopy();
	UInt32 pkt_snd_len = 0;
	mbuf_t cur_buf = m;

//	UInt8 *tmp = data_ptr; //For debug only
	
	do
	{
		if (mbuf_data(cur_buf))	bcopy(mbuf_data(cur_buf), data_ptr, mbuf_len(cur_buf));
		data_ptr += mbuf_len(cur_buf);
		pkt_snd_len += mbuf_len(cur_buf);
	}
	while(((cur_buf = mbuf_next(cur_buf)) != NULL) && ((pkt_snd_len + mbuf_len(cur_buf)) <= buf_len));
	buf_len = pkt_snd_len;
	
	//if (buf_len >= 20) PrintPacket(tmp);	//debug only
	
	tpd->desc.csum.csumpu = (OSSwapHostToLittleInt16((UInt16)buf_len) & CSUM_PARAM_BUFLEN_MASK) << CSUM_PARAM_BUFLEN_SHIFT;
	tpd->desc.csum.csumpl |= (CSUM_PARAM_EOP_MASK << CSUM_PARAM_EOP_SHIFT);
	//tpd->desc.tso.tso |= TSO_PARAM_EOP_MASK;
	if (++tpd_next_to_use >= tpd_ring->count) tpd_next_to_use = 0;
	tpd_ring->next_to_use = tpd_next_to_use;

	atl1UpdateMailbox();
	OSSynchronizeIO();
	
	freePacket(m);
	return kIOReturnOutputSuccess;
}

IOReturn AttansicL1Ethernet::registerWithPolicyMaker(IOService *policyMaker)
{
	DbgPrint("AttansicL1Ethernet::registerWithPolicyMaker\n");
	
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

IOReturn AttansicL1Ethernet::setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker)
{
	DbgPrint("AttansicL1Ethernet::setPowerState\n");
	//TO-DO: Add power state support
	return IOPMAckImplied;
}

IOReturn AttansicL1Ethernet::setPromiscuousMode(bool enabled)
{
	DbgPrint("AttansicL1Ethernet::setPromiscuousMode, %d\n", enabled);
	UInt32 ctrl = regRead32(REG_MAC_CTRL);
	if (enabled)
	{
		ctrl |= MAC_CTRL_PROMIS_EN;
	}
	else
	{
		ctrl &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
	}
	
	regWrite32(REG_MAC_CTRL, ctrl);
	
	//Clear the old settings from the multicast hash table
	regWrite32(REG_RX_HASH_TABLE, 0);	
	regWrite32(REG_RX_HASH_TABLE + (1 << 2), 0);
	return kIOReturnSuccess;
}

IOReturn AttansicL1Ethernet::setMulticastMode(bool enabled)
{
	DbgPrint("AttansicL1Ethernet::setMulticastMode, %d\n", enabled);
	UInt32 ctrl = regRead32(REG_MAC_CTRL);
	if (enabled)
	{
		//Clear the old settings from the multicast hash table
		regWrite32(REG_RX_HASH_TABLE, 0);	
		regWrite32(REG_RX_HASH_TABLE + (1 << 2), 0);
		ctrl |= MAC_CTRL_MC_ALL_EN;
		ctrl &= ~MAC_CTRL_PROMIS_EN;
	}
	else
	{
		ctrl &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
	}
	
	regWrite32(REG_MAC_CTRL, ctrl);
	
	return kIOReturnSuccess;
}

IOReturn AttansicL1Ethernet::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
	DbgPrint("AttansicL1Ethernet::setMulticastList\n");

	UInt32 hash_value;
	for(int i = 0; i < count; i++, addrs++)
	{
		hash_value = atl1HashMcAddr(addrs->bytes);
		atl1HashSet(hash_value);
	}
		
	return kIOReturnSuccess;
}

#pragma mark -
#pragma mark - Attansic L1 Adapter & Hw methods
#pragma mark -

UInt16 AttansicL1Ethernet::regRead16(UInt32 offset)
{
	if (!mmr_base) return 0xFFFF;
	return OSReadLittleInt16(mmr_base, offset);
}
	
UInt32 AttansicL1Ethernet::regRead32(UInt32 offset)
{
	if (!mmr_base) return 0xFFFFFFFF;
	return OSReadLittleInt32(mmr_base, offset);
}

void AttansicL1Ethernet::regWrite16(UInt32 offset, UInt16 data)
{
	if (!mmr_base) return ;
	OSWriteLittleInt16(mmr_base, offset, data);
}

void AttansicL1Ethernet::regWrite32(UInt32 offset, UInt32 data)
{
	if (!mmr_base) return ;
	OSWriteLittleInt32(mmr_base, offset, data);
}

int AttansicL1Ethernet::mdio_read(int phy_id, int reg_num)
{
	UInt16 result;
	
	atl1ReadPhyReg(reg_num & 0x1f, &result);
	
	return result;
}

void AttansicL1Ethernet::mdio_write(int phy_id, int reg_num, int val)
{
	atl1WritePhyReg(reg_num, val);
}

bool AttansicL1Ethernet::atl1Probe()
{
	pciDev_->setBusMasterEnable(true);
	pciDev_->setMemoryEnable(true);
	pciDev_->setIOEnable(false);
	
	vendorId_ = pciDev_->configRead16(kIOPCIConfigVendorID);
	deviceId_ = pciDev_->configRead16(kIOPCIConfigDeviceID);
	
	DbgPrint("Vendor ID %x, device ID %x\n", vendorId_, deviceId_);
	DbgPrint("MMR0 address %x\n", pciDev_->configRead32(kIOPCIConfigBaseAddress0));
	
	pciDev_->enablePCIPowerManagement();
	
	
	hw_addr = pciDev_->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
	if (hw_addr == NULL)
	{
		ErrPrint("Couldn't map io regs\n");
		return false;
	}
	else
	{
		DbgPrint("Memory mapped at bus address %x, virtual address %x, length %d\n", hw_addr->getPhysicalAddress(), 
					hw_addr->getVirtualAddress(), hw_addr->getLength());
	}
	
	hw_addr->retain();
	
	mmr_base = reinterpret_cast<char *>(hw_addr->getVirtualAddress());
	

	DbgPrint("REG_VPD_CAP = %x\n", OSReadLittleInt32(mmr_base, REG_VPD_CAP));	
	DbgPrint("REG_PCIE_CAP_LIST = %x\n", OSReadLittleInt32(mmr_base, REG_PCIE_CAP_LIST));
	DbgPrint("REG_MASTER_CTRL = %x\n", OSReadLittleInt32(mmr_base, REG_MASTER_CTRL));
	DbgPrint("REG_PHY_STATUS = %x\n", OSReadLittleInt32(mmr_base, REG_PHY_STATUS));
	
	atl1Hw_.dev_rev = regRead16(REG_MASTER_CTRL + 2);
	atl1Adapter_.rfd_ring.count = ATL1_DEFAULT_RFD;
	atl1Adapter_.tpd_ring.count = ATL1_DEFAULT_TPD;
	atl1Adapter_.rrd_ring.count = ATL1_DEFAULT_RFD + ATL1_DEFAULT_TPD;


	/*
	 * patch for some L1 of old version,
	 * the final version of L1 may not need these
	 * patches 
	 */
	/* atl1_pcie_patch(adapter); */

	/* really reset GPHY core */
	regWrite16(REG_GPHY_ENABLE, 0);
	
	/* 
	 * reset the controller to
	 * put the device in a known good starting state
	 */
	if (atl1ResetHw())
	{
		ErrPrint("Couldn't reset hardware\n");
		//return false;			//TO-DO: Uncomment
	}
	 
	if (atl1InitHw())
	{
		ErrPrint("Couldn't init hardware\n");
		//return false;
	}
	 
	atl1PciePatch();
	
	return true;
}


bool AttansicL1Ethernet::atl1SoftwareInit()
{
	atl1Hw_.revision_id = pciDev_->configRead8(kIOPCIConfigRevisionID);
	
	atl1Hw_.max_frame_size = MAXIMUM_ETHERNET_FRAME_SIZE;
	atl1Hw_.min_frame_size = MINIMUM_ETHERNET_FRAME_SIZE;	
	
	atl1Adapter_.wol = 0;
	atl1Adapter_.rx_buffer_len = (atl1Hw_.max_frame_size + 7) & ~7;
	atl1Adapter_.ict = 50000;	/* 100ms */
	atl1Adapter_.link_speed = SPEED_0;	/* hardware init */
	atl1Adapter_.link_duplex = FULL_DUPLEX;
	
	atl1Hw_.phy_configured = false;
	atl1Hw_.preamble_len = 7;
	atl1Hw_.ipgt = 0x60;
	atl1Hw_.min_ifg = 0x50;
	atl1Hw_.ipgr1 = 0x40;
	atl1Hw_.ipgr2 = 0x60;
	atl1Hw_.max_retry = 0xf;
	atl1Hw_.lcol = 0x37;
	atl1Hw_.jam_ipg = 7;
	atl1Hw_.rfd_burst = 8;
	atl1Hw_.rrd_burst = 8;
	atl1Hw_.rfd_fetch_gap = 1;
	atl1Hw_.rx_jumbo_th = atl1Adapter_.rx_buffer_len / 8;
	atl1Hw_.rx_jumbo_lkah = 1;
	atl1Hw_.rrd_ret_timer = 16;
	atl1Hw_.tpd_burst = 4;
	atl1Hw_.tpd_fetch_th = 16;
	atl1Hw_.txf_burst = 0x100;
	atl1Hw_.tx_jumbo_task_th = (atl1Hw_.max_frame_size + 7) >> 3;
	atl1Hw_.tpd_fetch_gap = 1;
	atl1Hw_.rcb_value = atl1_rcb_64;
	atl1Hw_.dma_ord = atl1_dma_ord_enh;
	atl1Hw_.dmar_block = atl1_dma_req_256;
	atl1Hw_.dmaw_block = atl1_dma_req_256;
	atl1Hw_.cmb_rrd = 4;
	atl1Hw_.cmb_tpd = 4;
	atl1Hw_.cmb_rx_timer = 1;	/* about 2us */
	atl1Hw_.cmb_tx_timer = 1;	/* about 2us */
	atl1Hw_.smb_timer = 100000;	/* about 200ms */
	
	atl1Adapter_.irq_sem = 0;
	atl1Adapter_.lock = IOSimpleLockAlloc();
	atl1Adapter_.mb_lock = IOSimpleLockAlloc();
	
	return true;
}

void AttansicL1Ethernet::atl1SoftwareFree()
{
	if (atl1Adapter_.lock)
	{
		IOSimpleLockFree(atl1Adapter_.lock);
		atl1Adapter_.lock = NULL;
	}
	
	if (atl1Adapter_.mb_lock)
	{
		IOSimpleLockFree(atl1Adapter_.mb_lock);
		atl1Adapter_.mb_lock = NULL;
	}
}

/*
 * Reset the transmit and receive units; mask and clear all interrupts.
 * hw - Struct containing variables accessed by shared code
 * return : ATL1_SUCCESS  or  idle status (if error)
 */
SInt32 AttansicL1Ethernet::atl1ResetHw()
{
	UInt32 icr;

	/* 
	 * Clear Interrupt mask to stop board from generating
	 * interrupts & Clear any pending interrupt events 
	 */
	/*
	 * regWrite32(REG_IMR, 0);
	 * regWrite32(REG_ISR, 0xffffffff);
	 */

	/*
	 * Issue Soft Reset to the MAC.  This will reset the chip's
	 * transmit, receive, DMA.  It will not effect
	 * the current PCI configuration.  The global reset bit is self-
	 * clearing, and should clear within a microsecond.
	 */
	regWrite32(REG_MASTER_CTRL, MASTER_CTRL_SOFT_RST);
	regRead32(REG_MASTER_CTRL);
	
	regWrite16(REG_GPHY_ENABLE, 1);
	regRead16(REG_GPHY_ENABLE);

	IOSleep(1);		/* delay about 1ms */

	/* Wait at least 10ms for All module to be Idle */
	for (int i = 0; i < 10; i++) 
	{
		icr = regRead32(REG_IDLE_STATUS);
		if (!icr) break;
		IOSleep(1);		/* delay 1 ms */
	}

	if (icr) 
	{
		DbgPrint("icr = %x\n", icr); 
		return icr;
	}

	return ATL1_SUCCESS;
}

/* function about EEPROM
 *
 * check_eeprom_exist
 * return 0 if eeprom exist
 */
SInt32 AttansicL1Ethernet::atl1ChechEepromExist()
{
	UInt32 value;
	
	value = regRead32(REG_SPI_FLASH_CTRL);
	if (value & SPI_FLASH_CTRL_EN_VPD) 
	{
		value &= ~SPI_FLASH_CTRL_EN_VPD;
		regWrite32(REG_SPI_FLASH_CTRL, value);
	}

	value = regRead16(REG_PCIE_CAP_LIST);
	return ((value & 0xFF00) == 0x6C00) ? 0 : 1;
}

bool AttansicL1Ethernet::atl1ReadEeprom(UInt32 offset, UInt32 *value) 
{
	UInt32 control;

	if (offset & 3) return false;	/* address do not align */

	regWrite32(REG_VPD_DATA, 0);
	control = (offset & VPD_CAP_VPD_ADDR_MASK) << VPD_CAP_VPD_ADDR_SHIFT;
	regWrite32(REG_VPD_CAP, control);
	regRead32(REG_VPD_CAP);

	for (int i = 0; i < 10; i++) 
	{
		IOSleep(2);
		control = regRead32(REG_VPD_CAP);
		if (control & VPD_CAP_VPD_FLAG)	break;
	}
	if (control & VPD_CAP_VPD_FLAG) 
	{
		*value = regRead32(REG_VPD_DATA);
		return true;
	}
	return false;		/* timeout */
}

/*
 * Reads the value from a PHY register
 * hw - Struct containing variables accessed by shared code
 * reg_addr - address of the PHY register to read
 */
SInt32 AttansicL1Ethernet::atl1ReadPhyReg(UInt16 reg_addr, UInt16 *phy_data)
{
	UInt32 val;

	val = ((UInt32) (reg_addr & MDIO_REG_ADDR_MASK)) << MDIO_REG_ADDR_SHIFT |
	    	MDIO_START | MDIO_SUP_PREAMBLE | MDIO_RW | MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;
		
	regWrite32(REG_MDIO_CTRL, val);
	regRead32(REG_MDIO_CTRL);
	
	for (int i = 0; i < MDIO_WAIT_TIMES; i++) 
	{
		IODelay(2);
		val = regRead32(REG_MDIO_CTRL);
		if (!(val & (MDIO_START | MDIO_BUSY))) break;
	}
	if (!(val & (MDIO_START | MDIO_BUSY))) 
	{
		*phy_data = (UInt16)val;
		return ATL1_SUCCESS;
	}
	
	return ATL1_ERR_PHY;
}

bool AttansicL1Ethernet::atl1SpiRead(UInt32 addr, UInt32 *buf)
{
	UInt32 value;

	regWrite32(REG_SPI_DATA, 0);
	regWrite32(REG_SPI_ADDR, addr);

	value = SPI_FLASH_CTRL_WAIT_READY | (CUSTOM_SPI_CS_SETUP & SPI_FLASH_CTRL_CS_SETUP_MASK) <<
	    SPI_FLASH_CTRL_CS_SETUP_SHIFT | (CUSTOM_SPI_CLK_HI & SPI_FLASH_CTRL_CLK_HI_MASK) <<
	    SPI_FLASH_CTRL_CLK_HI_SHIFT | (CUSTOM_SPI_CLK_LO & SPI_FLASH_CTRL_CLK_LO_MASK) <<
	    SPI_FLASH_CTRL_CLK_LO_SHIFT | (CUSTOM_SPI_CS_HOLD & SPI_FLASH_CTRL_CS_HOLD_MASK) <<
	    SPI_FLASH_CTRL_CS_HOLD_SHIFT | (CUSTOM_SPI_CS_HI & SPI_FLASH_CTRL_CS_HI_MASK) <<
	    SPI_FLASH_CTRL_CS_HI_SHIFT | (1 & SPI_FLASH_CTRL_INS_MASK) << SPI_FLASH_CTRL_INS_SHIFT;

	regWrite32(REG_SPI_FLASH_CTRL, value);

	value |= SPI_FLASH_CTRL_START;
	regWrite32(REG_SPI_FLASH_CTRL, value);
	regRead32(REG_SPI_FLASH_CTRL);

	for(int i = 0; i < 10; i++) 
	{
		IODelay(1);	/* 1ms */
		value = regRead32(REG_SPI_FLASH_CTRL);
		if (!(value & SPI_FLASH_CTRL_START)) break;
	}

	if (value & SPI_FLASH_CTRL_START) return false;

	*buf = regRead32(REG_SPI_DATA);

	return true;
}

/*
 * get_permanent_address
 * return 0 if get valid mac address, 
 */
SInt32 AttansicL1Ethernet::atl1GetPermanentAddress()
{
	DbgPrint("AttansicL1Ethernet::atl1GetPermanentAddress\n");
	UInt32 addr[2];
	UInt32 i, control;
	UInt16 reg;
	UInt8 eth_addr[ETH_ALEN];
	bool key_valid;

	if (is_valid_ether_addr(atl1Hw_.perm_mac_addr))	return 0;

	/* init */
	addr[0] = addr[1] = 0;

	if (!atl1ChechEepromExist()) 
	{	/* eeprom exist */
		DbgPrint("EEPROM exists\n");
		reg = 0;
		key_valid = false;
		/* Read out all EEPROM content */
		i = 0;
		while (1) 
		{
			//if (atl1_read_eeprom(hw, i + 0x100, &control)) 
			if (atl1ReadEeprom(i + 0x100, &control))
			{
				if (key_valid) 
				{
					DbgPrint("Eeprom KEY reg=0x%X control=0x%X\n", reg, control);
					if (reg == REG_MAC_STA_ADDR) addr[0] = control;
					else if (reg == (REG_MAC_STA_ADDR + 4)) addr[1] = control;
					key_valid = false;
				} 
				else if ((control & 0xff) == 0x5A) 
				{
					DbgPrint("Eeprom control 0x%X\n", control);
					key_valid = true;
					reg = (UInt16) (control >> 16);
				} 
				else break;	/* assume data end while encount an invalid KEYWORD */
			} 
			else break;	/* read error */
			i += 4;
		}

/*
 * The following 2 lines are the Attansic originals.  Saving for posterity.
 *		*(u32 *) & eth_addr[2] = LONGSWAP(addr[0]);
 *		*(u16 *) & eth_addr[0] = SHORTSWAP(*(u16 *) & addr[1]);
 */
		*(UInt32 *) & eth_addr[2] =  OSSwapInt32(addr[0]);
		*(UInt16 *) & eth_addr[0] = OSSwapInt16(*(UInt16 *) & addr[1]);

		if (is_valid_ether_addr(eth_addr)) 
		{
			memcpy(atl1Hw_.perm_mac_addr, eth_addr, ETH_ALEN);
			return 0;
		}
		else
		{
			ErrPrint("Mac address through EEPROM is invalid\n");
		}
		
		//return 1;
	}

	/* see if SPI FLAGS exist ? */
	addr[0] = addr[1] = 0;
	control = reg = 0;
	key_valid = false;
	i = 0;
	while(1) 
	{
		if (atl1SpiRead(i +0x1f000, &control))
		{
			if (key_valid) 
			{
				if (reg == REG_MAC_STA_ADDR) addr[0] = control;
				else if (reg == (REG_MAC_STA_ADDR + 4)) addr[1] = control;
				key_valid = false;
			} 
			else if ((control & 0xff) == 0x5A) 
			{
				key_valid = true;
				reg = (UInt16) (control >> 16);
			} 
			else break;	/* data end */
		} 
		else break;	/* read error */
		i += 4;
	}

/*
 * The following 2 lines are the Attansic originals.  Saving for posterity.
 *	*(u32 *) & eth_addr[2] = LONGSWAP(addr[0]);
 *	*(u16 *) & eth_addr[0] = SHORTSWAP(*(u16 *) & addr[1]);
 */
	*(UInt32 *) & eth_addr[2] = OSSwapInt32(addr[0]);
	*(UInt16 *) & eth_addr[0] = OSSwapInt16(*(UInt16 *) & addr[1]);
	
	if (is_valid_ether_addr(eth_addr)) 
	{
		memcpy(atl1Hw_.perm_mac_addr, eth_addr, ETH_ALEN);
		return 0;
	}
	else
	{
		ErrPrint("Mac address through SPI is invalid\n");
		/*
		* On some motherboards, the MAC address is written by the
		* BIOS directly to the MAC register during POST, and is
		* not stored in eeprom. If all else thus far has failed
		* to fetch the permanent MAC address, try reading it directly.
		*/
		addr[0] = regRead32(REG_MAC_STA_ADDR);
		addr[1] = regRead16((REG_MAC_STA_ADDR + 4));
		*(UInt32 *) & eth_addr[2] = OSSwapInt32(addr[0]);
		*(UInt16 *) & eth_addr[0] = OSSwapInt16(*(UInt16 *) & addr[1]);
		if (is_valid_ether_addr(eth_addr)) {
			memcpy(atl1Hw_.perm_mac_addr, eth_addr, ETH_ALEN);
			return 0;
		}
	}
	
	return 1;
}

/*
 * Hashes an address to determine its location in the multicast table
 * hw - Struct containing variables accessed by shared code
 * mc_addr - the multicast address to hash
 *
 * atl1_hash_mc_addr
 *  purpose
 *      set hash value for a multicast address
 *      hash calcu processing :
 *          1. calcu 32bit CRC for multicast address
 *          2. reverse crc with MSB to LSB
 */
UInt32 AttansicL1Ethernet::atl1HashMcAddr(UInt8 *mc_addr)
{
	UInt32 crc32, value = 0;
	int i;

	crc32 = ether_crc_le(6, mc_addr);
	crc32 = ~crc32;
	for (i = 0; i < 32; i++)
		value |= (((crc32 >> i) & 1) << (31 - i));

	return value;
}

/*
 * Sets the bit in the multicast table corresponding to the hash value.
 * hw - Struct containing variables accessed by shared code
 * hash_value - Multicast address hash value
 */
void AttansicL1Ethernet::atl1HashSet(UInt32 hash_value)
{
	UInt32 hash_bit, hash_reg;
	UInt32 mta;

	/*
	 * The HASH Table  is a register array of 2 32-bit registers.
	 * It is treated like an array of 64 bits.  We want to set
	 * bit BitArray[hash_value]. So we figure out what register
	 * the bit is in, read it, OR in the new bit, then write
	 * back the new value.  The register is determined by the
	 * upper 7 bits of the hash value and the bit within that
	 * register are determined by the lower 5 bits of the value.
	 */
	hash_reg = (hash_value >> 31) & 0x1;
	hash_bit = (hash_value >> 26) & 0x1F;
	mta = regRead32(REG_RX_HASH_TABLE + (hash_reg << 2));
	mta |= (1 << hash_bit);
	regWrite32(REG_RX_HASH_TABLE + (hash_reg << 2), mta);
}


/*
 * Writes a value to a PHY register
 * hw - Struct containing variables accessed by shared code
 * reg_addr - address of the PHY register to write
 * data - data to write to the PHY
 */
SInt32 AttansicL1Ethernet::atl1WritePhyReg(UInt32 reg_addr, UInt16 phy_data)
{
	UInt32 val;

	val = ((UInt32) (phy_data & MDIO_DATA_MASK)) << MDIO_DATA_SHIFT | (reg_addr & MDIO_REG_ADDR_MASK) << 
			MDIO_REG_ADDR_SHIFT | MDIO_SUP_PREAMBLE | MDIO_START | MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;
	regWrite32(REG_MDIO_CTRL, val);
	regRead32(REG_MDIO_CTRL);

	for (int i = 0; i < MDIO_WAIT_TIMES; i++) 
	{
		IODelay(2);
		val = regRead32(REG_MDIO_CTRL);
		if (!(val & (MDIO_START | MDIO_BUSY))) break;
	}

	if (!(val & (MDIO_START | MDIO_BUSY))) return ATL1_SUCCESS;

	return ATL1_ERR_PHY;
}

/*
 * Make L001's PHY out of Power Saving State (bug)
 * hw - Struct containing variables accessed by shared code
 * when power on, L001's PHY always on Power saving State
 * (Gigabit Link forbidden)
 */
SInt32 AttansicL1Ethernet::atl1PhyLeavePowerSaving()
{
	SInt32 ret;
	ret = atl1WritePhyReg(29, 0x0029);
	if (ret) return ret;
	return atl1WritePhyReg(30, 0);
}


/*
 * Resets the PHY and make all config validate
 * hw - Struct containing variables accessed by shared code
 *
 * Sets bit 15 and 12 of the MII Control regiser (for F001 bug)
 */
SInt32 AttansicL1Ethernet::atl1PhyReset()
{
	SInt32 ret_val;
	UInt16 phy_data;

	DbgPrint("AttansicL1Ethernet::atl1PhyReset with media_type=%d\n", atl1Hw_.media_type);

	switch (atl1Hw_.media_type) 
	{
	case MEDIA_TYPE_AUTO_SENSOR:
		phy_data = MII_CR_RESET | MII_CR_AUTO_NEG_EN | MII_CR_RESTART_AUTO_NEG;
		break;
	case MEDIA_TYPE_1000M_FULL:
		phy_data = MII_CR_RESET | MII_CR_SPEED_1000 | MII_CR_FULL_DUPLEX;
		break;
	case MEDIA_TYPE_100M_FULL:
		phy_data = MII_CR_FULL_DUPLEX | MII_CR_SPEED_100 | MII_CR_RESET;
		break;
	case MEDIA_TYPE_100M_HALF:
		phy_data = MII_CR_SPEED_100 | MII_CR_RESET;
		break;
	case MEDIA_TYPE_10M_FULL:
		phy_data = MII_CR_FULL_DUPLEX | MII_CR_SPEED_10 | MII_CR_RESET;
		break;
	default:	/* MEDIA_TYPE_10M_HALF: */
		phy_data = MII_CR_SPEED_10 | MII_CR_RESET;
		break;
	}

	ret_val = atl1WritePhyReg(MII_BMCR, phy_data);
	if (ret_val) 
	{
		UInt32 val;
		/* pcie serdes link may be down! */
		DbgPrint("Autoneg caused pcie phy link down\n");

		for (int i = 0; i < 25; i++) 
		{
			IOSleep(1);
			val = regRead32(REG_MDIO_CTRL);
			if (!(val & (MDIO_START | MDIO_BUSY))) break;
		}

		if ((val & (MDIO_START | MDIO_BUSY)) != 0) 
		{
			DbgPrint("pcie link down at least for 25ms\n");
			return ret_val;
		}
	}
	return ATL1_SUCCESS;
}


/*
 * Configures PHY autoneg and flow control advertisement settings
 * hw - Struct containing variables accessed by shared code
 */
SInt32 AttansicL1Ethernet::atl1PhySetupAutonegAdv()
{
	SInt32 ret_val;
	SInt16 mii_autoneg_adv_reg;
	SInt16 mii_1000t_ctrl_reg;

	/* Read the MII Auto-Neg Advertisement Register (Address 4). */
	mii_autoneg_adv_reg = MII_AR_DEFAULT_CAP_MASK;

	/* Read the MII 1000Base-T Control Register (Address 9). */
	mii_1000t_ctrl_reg = MII_AT001_CR_1000T_DEFAULT_CAP_MASK;

	/*
	 * First we clear all the 10/100 mb speed bits in the Auto-Neg
	 * Advertisement Register (Address 4) and the 1000 mb speed bits in
	 * the  1000Base-T Control Register (Address 9).
	 */
	mii_autoneg_adv_reg &= ~MII_AR_SPEED_MASK;
	mii_1000t_ctrl_reg &= ~MII_AT001_CR_1000T_SPEED_MASK;

	/*
	 * Need to parse media_type  and set up
	 * the appropriate PHY registers.
	 */
	switch (atl1Hw_.media_type) 
	{
	case MEDIA_TYPE_AUTO_SENSOR:
		mii_autoneg_adv_reg |= (MII_AR_10T_HD_CAPS | MII_AR_10T_FD_CAPS | MII_AR_100TX_HD_CAPS |
								MII_AR_100TX_FD_CAPS);
		mii_1000t_ctrl_reg |= MII_AT001_CR_1000T_FD_CAPS;
		break;

	case MEDIA_TYPE_1000M_FULL:
		mii_1000t_ctrl_reg |= MII_AT001_CR_1000T_FD_CAPS;
		break;

	case MEDIA_TYPE_100M_FULL:
		mii_autoneg_adv_reg |= MII_AR_100TX_FD_CAPS;
		break;

	case MEDIA_TYPE_100M_HALF:
		mii_autoneg_adv_reg |= MII_AR_100TX_HD_CAPS;
		break;

	case MEDIA_TYPE_10M_FULL:
		mii_autoneg_adv_reg |= MII_AR_10T_FD_CAPS;
		break;

	default:
		mii_autoneg_adv_reg |= MII_AR_10T_HD_CAPS;
		break;
	}

	/* flow control fixed to enable all */
	mii_autoneg_adv_reg |= (MII_AR_ASM_DIR | MII_AR_PAUSE /* | MII_AR_SELECTOR_FIELD */);
	DbgPrint("atl1PhySetupAutonegAdv() autosensing caps mii_autoneg_adv_reg=0x%02X mii_1000t_ctrl_reg=0x%02X\n",
				mii_autoneg_adv_reg, mii_1000t_ctrl_reg);

	atl1Hw_.mii_autoneg_adv_reg = mii_autoneg_adv_reg;
	atl1Hw_.mii_1000t_ctrl_reg = mii_1000t_ctrl_reg;

	ret_val = atl1WritePhyReg(MII_ADVERTISE, mii_autoneg_adv_reg);
	if (ret_val) return ret_val;

	ret_val = atl1WritePhyReg(MII_AT001_CR, mii_1000t_ctrl_reg);
	if (ret_val) return ret_val;
	
	DbgPrint("atl1PhySetupAutonegAdv() successfull\n");

	return ATL1_SUCCESS;
}


/*
 * Configures link settings.
 * Assumes the hardware has previously been reset and the
 * transmitter and receiver are not enabled.
 */
SInt32 AttansicL1Ethernet::atl1SetupLink()
{
	SInt32 ret_val;

	/*
	 * Options:
	 *  PHY will advertise value(s) parsed from
	 *  autoneg_advertised and fc
	 *  no matter what autoneg is , We will not wait link result.
	 */
	ret_val = atl1PhySetupAutonegAdv();
	if (ret_val) 
	{
		ErrPrint("setting up autonegotiation\n");
		return ret_val;
	}
	/* SW.Reset , En-Auto-Neg if needed */
	ret_val = atl1PhyReset();
	if (ret_val) 
	{
		ErrPrint("resetting the phy\n");
		return ret_val;
	}
	atl1Hw_. phy_configured = true;
	return ret_val;
}

struct atl1_spi_flash_dev flash_table[] = {
/*	MFR_NAME  WRSR  READ  PRGM  WREN  WRDI  RDSR  RDID  SECTOR_ERASE CHIP_ERASE */
	{"Atmel", 0x00, 0x03, 0x02, 0x06, 0x04, 0x05, 0x15, 0x52,        0x62},
	{"SST",   0x01, 0x03, 0x02, 0x06, 0x04, 0x05, 0x90, 0x20,        0x60},
	{"ST",    0x01, 0x03, 0x02, 0x06, 0x04, 0x05, 0xAB, 0xD8,        0xC7},
};

//static void atl1_init_flash_opcode(struct atl1_hw *hw)
void AttansicL1Ethernet::atl1InitFlashOpcode()
{
	if (atl1Hw_.flash_vendor >= sizeof(flash_table) / sizeof(flash_table[0]))
		atl1Hw_.flash_vendor = 0;	/* ATMEL */

	/* Init OP table */
	pciDev_->ioWrite8(REG_SPI_FLASH_OP_PROGRAM, flash_table[atl1Hw_.flash_vendor].cmd_program);	
	pciDev_->ioWrite8(REG_SPI_FLASH_OP_SC_ERASE, flash_table[atl1Hw_.flash_vendor].cmd_sector_erase);
	pciDev_->ioWrite8(REG_SPI_FLASH_OP_CHIP_ERASE, flash_table[atl1Hw_.flash_vendor].cmd_chip_erase);	
	pciDev_->ioWrite8(REG_SPI_FLASH_OP_RDID, flash_table[atl1Hw_.flash_vendor].cmd_rdid);
	pciDev_->ioWrite8(REG_SPI_FLASH_OP_WREN, flash_table[atl1Hw_.flash_vendor].cmd_wren);	
	pciDev_->ioWrite8(REG_SPI_FLASH_OP_RDSR, flash_table[atl1Hw_.flash_vendor].cmd_rdsr);	
	pciDev_->ioWrite8(REG_SPI_FLASH_OP_WRSR, flash_table[atl1Hw_.flash_vendor].cmd_wrsr);	
	pciDev_->ioWrite8(REG_SPI_FLASH_OP_READ, flash_table[atl1Hw_.flash_vendor].cmd_read);
}

/*
 * Performs basic configuration of the adapter.
 * hw - Struct containing variables accessed by shared code
 * Assumes that the controller has previously been reset and is in a
 * post-reset uninitialized state. Initializes multicast table, 
 * and  Calls routines to setup link
 * Leaves the transmit and receive units disabled and uninitialized.
 */
SInt32 AttansicL1Ethernet::atl1InitHw()
{
	UInt32 ret_val = 0;

	/* Zero out the Multicast HASH table */
	pciDev_->ioWrite8(REG_RX_HASH_TABLE, 0);
	/* clear the old settings from the multicast hash table */
	pciDev_->ioWrite8(REG_RX_HASH_TABLE + (1 << 2), 0);

	atl1InitFlashOpcode();

	if (!atl1Hw_.phy_configured) 
	{
		/* enable GPHY LinkChange Interrupt */
		ret_val = atl1WritePhyReg(18, 0xC00);
		
		if (ret_val) return ret_val;
		/* make PHY out of power-saving state */
		ret_val = atl1PhyLeavePowerSaving();
		
		if (ret_val) return ret_val;
		/* Call a subroutine to configure the link */
		ret_val = atl1SetupLink();
	}
	
	return ret_val;
}

/*
 * Detects the current speed and duplex settings of the hardware.
 * speed - Speed of the connection
 * duplex - Duplex setting of the connection
 */
SInt32 AttansicL1Ethernet::atl1GetSpeedAndDuplex(UInt16 *speed, UInt16 *duplex)
{
	SInt32 ret_val;
	UInt16 phy_data;

	ret_val = atl1ReadPhyReg(MII_AT001_PSSR, &phy_data);
	if (ret_val) return ret_val;

	if (!(phy_data & MII_AT001_PSSR_SPD_DPLX_RESOLVED))
	{
		DbgPrint("atl1GetSpeedAndDuplex: AT001 duplex&speed is unresolved, possible media failure\n");
		return ATL1_ERR_PHY_RES;
	}

	switch (phy_data & MII_AT001_PSSR_SPEED) 
	{
	case MII_AT001_PSSR_1000MBS:
		*speed = SPEED_1000;
		break;
	case MII_AT001_PSSR_100MBS:
		*speed = SPEED_100;
		break;
	case MII_AT001_PSSR_10MBS:
		*speed = SPEED_10;
		break;
	default:
		ErrPrint("getting speed\n");
		return ATL1_ERR_PHY_SPEED;
	}
	
	if (phy_data & MII_AT001_PSSR_DPLX) *duplex = FULL_DUPLEX;
	else *duplex = HALF_DUPLEX;

	return ATL1_SUCCESS;
}

void AttansicL1Ethernet::atl1GetAndUpdateLinkStatus()
{
		UInt16 speed, duplex;
		UInt32 currentMediumIndex = MEDIUM_INDEX_AUTO;
		
		if(atl1GetSpeedAndDuplex(&speed, &duplex) == ATL1_SUCCESS)
		{ 
			DbgPrint("Link is active, speed %d, duplex %d\n", speed, duplex);
			

			if(speed == SPEED_10 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10FD;
			if(speed == SPEED_100 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100FD;
			if(speed == SPEED_1000 && duplex == FULL_DUPLEX) currentMediumIndex = MEDIUM_INDEX_1000FD;
			if(speed == SPEED_10 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_10HD;
			if(speed == SPEED_100 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_100HD;
			if(speed == SPEED_1000 && duplex == HALF_DUPLEX) currentMediumIndex = MEDIUM_INDEX_1000HD;		

			setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid, mediumTable[currentMediumIndex], speed * MBit, NULL);
			
			atl1Adapter_.link_speed = speed;
			atl1Adapter_.link_duplex = duplex;
		
			/* if(atl1Hw_.media_type == MEDIA_TYPE_AUTO_SENSOR) */ atl1SetupMacCtrl();
		} else
		{
			DbgPrint("Link is down\n");
			setLinkStatus(kIONetworkLinkValid, NULL, 0, NULL);
		}
}



void AttansicL1Ethernet::atl1SetMacAddr()
{
	UInt32 value;
	/*
	 * 00-0B-6A-F6-00-DC
	 * 0:  6AF600DC   1: 000B
	 * low dword
	 */
	value = (((UInt32) atl1Hw_.mac_addr[2]) << 24) | (((UInt32)atl1Hw_.mac_addr[3]) << 16) |
	    (((UInt32)atl1Hw_.mac_addr[4]) << 8) | (((UInt32)atl1Hw_.mac_addr[5]));
		
	regWrite32(REG_MAC_STA_ADDR, value);
	/* high dword */
	value = (((UInt32) atl1Hw_.mac_addr[0]) << 8) | (((UInt32) atl1Hw_.mac_addr[1]));
	regWrite32(REG_MAC_STA_ADDR + (1 << 2), value);
}

/*
 * atl1_pcie_patch - Patch for PCIE module
 */
void AttansicL1Ethernet::atl1PciePatch()
{
	UInt32 value;
	value = 0x6500;
	regWrite32(0x12FC, value);
	/* pcie flow control mode change */
	value = regRead32(0x1008);
	value |= 0x8000;
	regWrite32(0x1008, value);
}

void AttansicL1Ethernet::atl1InterruptsEnable()
{
	regWrite32(REG_IMR, IMR_NORMAL_MASK);
}

void AttansicL1Ethernet::atl1InterruptsDisable()
{
	//OSIncrementAtomic(&atl1Adapter_.irq_sem);
	regWrite32(REG_IMR, 0);
	regRead32(REG_IMR);
}

UInt32 AttansicL1Ethernet::atl1Configure()
{
	UInt32 value = 0;
	DbgPrint("AttansicL1Ethernet::atl1Configure()\n");
	/* clear interrupt status */
	regWrite32(REG_ISR, 0xffffffff);
	
	/* set MAC Address */
	/*
	value = (((UInt32)atl1Hw_.mac_addr[2]) << 24) |
		(((UInt32) atl1Hw_.mac_addr[3]) << 16) |
		(((UInt32) atl1Hw_.mac_addr[4]) << 8) |
		(((UInt32) atl1Hw_.mac_addr[5]));
	regWrite32(REG_MAC_STA_ADDR, value);
	value = (((UInt32)atl1Hw_.mac_addr[0]) << 8) | (((UInt32)atl1Hw_.mac_addr[1]));
	regWrite32(REG_MAC_STA_ADDR + 4, value);
	*/
	/* tx / rx ring */

	DbgPrint("Writing ring dma registers tpd=%X rfd=%X rrd=%X cmb=%X smb=%X\n",
			atl1Adapter_.tpd_ring.dma, atl1Adapter_.rfd_ring.dma,
			atl1Adapter_.rrd_ring.dma, atl1Adapter_.cmb.dma,
			atl1Adapter_.smb.dma);

	/* HI base address */
	regWrite32(REG_DESC_BASE_ADDR_HI, 
			(UInt32)((atl1Adapter_.tpd_ring.dma & 0xffffffff00000000ULL) >> 32));	
			
	/* LO base address */
	regWrite32(REG_DESC_RFD_ADDR_LO, 
			(UInt32)(atl1Adapter_.rfd_ring.dma & 0x00000000ffffffffULL));
		
	regWrite32(REG_DESC_RRD_ADDR_LO, 
			(UInt32)(atl1Adapter_.rrd_ring.dma & 0x00000000ffffffffULL));
	
	regWrite32(REG_DESC_TPD_ADDR_LO, 
			(UInt32)(atl1Adapter_.tpd_ring.dma & 0x00000000ffffffffULL));
	
	regWrite32(REG_DESC_CMB_ADDR_LO, 
			(UInt32)(atl1Adapter_.cmb.dma & 0x00000000ffffffffULL));
		
	regWrite32(REG_DESC_SMB_ADDR_LO, 
			(UInt32)(atl1Adapter_.smb.dma & 0x00000000ffffffffULL));
			
	/* element count */
	value = atl1Adapter_.rrd_ring.count;
	value <<= 16;
	value += atl1Adapter_.rfd_ring.count;
	regWrite32(REG_DESC_RFD_RRD_RING_SIZE, value);
	regWrite32(REG_DESC_TPD_RING_SIZE, atl1Adapter_.tpd_ring.count);
	
	/* Load Ptr */
	regWrite32(REG_LOAD_PTR, 1);
	
	/* config Mailbox */
	value = ((atl1Adapter_.tpd_ring.next_to_use & MB_TPD_PROD_INDX_MASK) << MB_TPD_PROD_INDX_SHIFT) |
	    ((atl1Adapter_.rrd_ring.next_to_clean & MB_RRD_CONS_INDX_MASK) << MB_RRD_CONS_INDX_SHIFT) |
	    ((atl1Adapter_.rfd_ring.next_to_use & MB_RFD_PROD_INDX_MASK) << MB_RFD_PROD_INDX_SHIFT);
		  
	regWrite32(REG_MAILBOX, value);
	
	/* config IPG/IFG */
	value = (((UInt32)atl1Hw_.ipgt & MAC_IPG_IFG_IPGT_MASK) << MAC_IPG_IFG_IPGT_SHIFT) |
	    (((UInt32)atl1Hw_.min_ifg & MAC_IPG_IFG_MIFG_MASK) << MAC_IPG_IFG_MIFG_SHIFT) |
	    (((UInt32)atl1Hw_.ipgr1 & MAC_IPG_IFG_IPGR1_MASK) << MAC_IPG_IFG_IPGR1_SHIFT) |
	    (((UInt32)atl1Hw_.ipgr2 & MAC_IPG_IFG_IPGR2_MASK) << MAC_IPG_IFG_IPGR2_SHIFT);

	regWrite32(REG_MAC_IPG_IFG, value);
	
	/* config  Half-Duplex Control */
	value = ((UInt32)atl1Hw_.lcol & MAC_HALF_DUPLX_CTRL_LCOL_MASK) |
	    (((UInt32)atl1Hw_.max_retry & MAC_HALF_DUPLX_CTRL_RETRY_MASK) << MAC_HALF_DUPLX_CTRL_RETRY_SHIFT) |
	    MAC_HALF_DUPLX_CTRL_EXC_DEF_EN | (0xa << MAC_HALF_DUPLX_CTRL_ABEBT_SHIFT) |
	    (((UInt32)atl1Hw_.jam_ipg & MAC_HALF_DUPLX_CTRL_JAMIPG_MASK) << MAC_HALF_DUPLX_CTRL_JAMIPG_SHIFT);
		 
	regWrite32(REG_MAC_HALF_DUPLX_CTRL, value);
	
	/* set Interrupt Moderator Timer */
	regWrite16(REG_IRQ_MODU_TIMER_INIT, atl1Adapter_.imt);
	regWrite32(REG_MASTER_CTRL, MASTER_CTRL_ITIMER_EN);

	/* set Interrupt Clear Timer */
	regWrite16(REG_CMBDISDMA_TIMER, atl1Adapter_.ict);

	/* set MTU, 4 : VLAN */
	regWrite32(REG_MTU, atl1Hw_.max_frame_size + 4);
	
	/* jumbo size & rrd retirement timer */
	value = (((UInt32)atl1Hw_.rx_jumbo_th & RXQ_JMBOSZ_TH_MASK) << RXQ_JMBOSZ_TH_SHIFT) |
	    (((UInt32)atl1Hw_.rx_jumbo_lkah & RXQ_JMBO_LKAH_MASK) << RXQ_JMBO_LKAH_SHIFT) |
	    (((UInt32)atl1Hw_.rrd_ret_timer & RXQ_RRD_TIMER_MASK) << RXQ_RRD_TIMER_SHIFT);
		
	regWrite32(REG_RXQ_JMBOSZ_RRDTIM, value);
	
	/* Flow Control */
	switch (atl1Hw_.dev_rev) 
	{
	case 0x8001:
	case 0x9001:
	case 0x9002:
	case 0x9003:
		setFlowControlOld();
		break;
	default:
		setFlowControlNew();
		break;
	}
	
	/* config TXQ */
	value = (((UInt32)atl1Hw_.tpd_burst & TXQ_CTRL_TPD_BURST_NUM_MASK) << TXQ_CTRL_TPD_BURST_NUM_SHIFT) |
	    (((UInt32)atl1Hw_.txf_burst & TXQ_CTRL_TXF_BURST_NUM_MASK) << TXQ_CTRL_TXF_BURST_NUM_SHIFT) |
	    (((UInt32)atl1Hw_.tpd_fetch_th & TXQ_CTRL_TPD_FETCH_TH_MASK) << TXQ_CTRL_TPD_FETCH_TH_SHIFT) | TXQ_CTRL_ENH_MODE | TXQ_CTRL_EN;

	regWrite32(REG_TXQ_CTRL, value);
	
	/* min tpd fetch gap & tx jumbo packet size threshold for taskoffload */
	value = (((UInt32)atl1Hw_.tx_jumbo_task_th & TX_JUMBO_TASK_TH_MASK) << TX_JUMBO_TASK_TH_SHIFT) |
	    (((UInt32)atl1Hw_.tpd_fetch_gap & TX_TPD_MIN_IPG_MASK) << TX_TPD_MIN_IPG_SHIFT);
	
	regWrite32(REG_TX_JUMBO_TASK_TH_TPD_IPG, value);
	
	/* config RXQ */
	value = (((UInt32)atl1Hw_.rfd_burst & RXQ_CTRL_RFD_BURST_NUM_MASK) << RXQ_CTRL_RFD_BURST_NUM_SHIFT) |
	    (((UInt32)atl1Hw_.rrd_burst & RXQ_CTRL_RRD_BURST_THRESH_MASK) << RXQ_CTRL_RRD_BURST_THRESH_SHIFT) |
	    (((UInt32)atl1Hw_.rfd_fetch_gap & RXQ_CTRL_RFD_PREF_MIN_IPG_MASK)  << RXQ_CTRL_RFD_PREF_MIN_IPG_SHIFT) |
	    RXQ_CTRL_CUT_THRU_EN | RXQ_CTRL_EN;

	regWrite32(REG_RXQ_CTRL, value);
	
	/* config DMA Engine */
	value = ((((UInt32)atl1Hw_.dmar_block) & DMA_CTRL_DMAR_BURST_LEN_MASK) << DMA_CTRL_DMAR_BURST_LEN_SHIFT) |
	    ((((UInt32)atl1Hw_.dmaw_block) & DMA_CTRL_DMAR_BURST_LEN_MASK) << DMA_CTRL_DMAR_BURST_LEN_SHIFT) |
	    DMA_CTRL_DMAR_EN | DMA_CTRL_DMAW_EN;
	
	value |= (UInt32)atl1Hw_.dma_ord;
	
	if (atl1_rcb_128 == atl1Hw_.rcb_value) value |= DMA_CTRL_RCB_VALUE;
	regWrite32(REG_DMA_CTRL, value);
	
	/* config CMB / SMB */
	value = atl1Hw_.cmb_rrd | ((UInt32) atl1Hw_.cmb_tpd << 16);
	regWrite32(REG_CMB_WRITE_TH, value);
	value = atl1Hw_.cmb_rx_timer | ((UInt32) atl1Hw_.cmb_tx_timer << 16);
	regWrite32(REG_CMB_WRITE_TIMER, value);
	regWrite32(REG_SMB_TIMER, atl1Hw_.smb_timer);

	/* --- enable CMB / SMB */
	value = CSMB_CTRL_CMB_EN | CSMB_CTRL_SMB_EN;
	regWrite32(REG_CSMB_CTRL, value);
	
	//value = ioread32(adapter->hw.hw_addr + REG_ISR);
	value = regRead32(REG_ISR);
	if ((value & ISR_PHY_LINKDOWN) != 0) value = 1;	/* config failed */
	else value = 0;

	/* clear all interrupt status */
	regWrite32(REG_ISR, 0x3fffffff);
	regWrite32(REG_ISR, 0);
	
	return value;	
}

void AttansicL1Ethernet::setFlowControlOld()
{
	UInt32 hi, lo, value;

	/* RFD Flow Control */
	value = atl1Adapter_.rfd_ring.count;
	hi = value / 16;
	if (hi < 2)
		hi = 2;
	lo = value * 7 / 8;

	value = ((hi & RXQ_RXF_PAUSE_TH_HI_MASK) << RXQ_RXF_PAUSE_TH_HI_SHIFT) |
	    ((lo & RXQ_RXF_PAUSE_TH_LO_MASK) << RXQ_RXF_PAUSE_TH_LO_SHIFT);

	regWrite32(REG_RXQ_RXF_PAUSE_THRESH, value);

	/* RRD Flow Control */
	value = atl1Adapter_.rrd_ring.count;
	lo = value / 16;
	hi = value * 7 / 8;
	if (lo < 2)
		lo = 2;
	value = ((hi & RXQ_RRD_PAUSE_TH_HI_MASK) << RXQ_RRD_PAUSE_TH_HI_SHIFT) |
	    ((lo & RXQ_RRD_PAUSE_TH_LO_MASK) << RXQ_RRD_PAUSE_TH_LO_SHIFT);
	regWrite32(REG_RXQ_RRD_PAUSE_THRESH, value);
}

void AttansicL1Ethernet::setFlowControlNew()
{
	UInt32 hi, lo, value;

	/* RXF Flow Control */
	value = regRead32(REG_SRAM_RXF_LEN);
	lo = value / 16;
	if (lo < 192) lo = 192;
	hi = value * 7 / 8;
	if (hi < lo) hi = lo + 16;
	value = ((hi & RXQ_RXF_PAUSE_TH_HI_MASK) << RXQ_RXF_PAUSE_TH_HI_SHIFT) |
	    ((lo & RXQ_RXF_PAUSE_TH_LO_MASK) << RXQ_RXF_PAUSE_TH_LO_SHIFT);

	regWrite32(REG_RXQ_RXF_PAUSE_THRESH, value);

	/* RRD Flow Control */
	value = regRead32(REG_SRAM_RRD_LEN);
	
	lo = value / 8;
	hi = value * 7 / 8;
	if (lo < 2) lo = 2;
	if (hi < lo) hi = lo + 3;
	value = ((hi & RXQ_RRD_PAUSE_TH_HI_MASK) << RXQ_RRD_PAUSE_TH_HI_SHIFT) |
	    ((lo & RXQ_RRD_PAUSE_TH_LO_MASK) << RXQ_RRD_PAUSE_TH_LO_SHIFT);
		
	regWrite32(REG_RXQ_RRD_PAUSE_THRESH, value);
}

void AttansicL1Ethernet::atl1SetupMacCtrl()
{
	UInt32 value;
	/* Config MAC CTRL Register */
	value = MAC_CTRL_TX_EN | MAC_CTRL_RX_EN;
	/* duplex */
	if (FULL_DUPLEX == atl1Adapter_.link_duplex) value |= MAC_CTRL_DUPLX;
	/* speed */
	value |= ((UInt32) ((SPEED_1000 == atl1Adapter_.link_speed) ?
				MAC_CTRL_SPEED_1000 : MAC_CTRL_SPEED_10_100) << MAC_CTRL_SPEED_SHIFT);
				
	/* flow control */
	value |= (MAC_CTRL_TX_FLOW | MAC_CTRL_RX_FLOW);
	/* PAD & CRC */
	value |= (MAC_CTRL_ADD_CRC | MAC_CTRL_PAD);
	/* preamble length */
	value |= (((UInt32) atl1Hw_.preamble_len
		   & MAC_CTRL_PRMLEN_MASK) << MAC_CTRL_PRMLEN_SHIFT);

	/* rx checksum
	   if (adapter->rx_csum)
	   value |= MAC_CTRL_RX_CHKSUM_EN;
	 */
	/* filter mode */
	value |= MAC_CTRL_BC_EN;
	/* value |= MAC_CTRL_LOOPBACK; */
	regWrite32(REG_MAC_CTRL, value);
}

void AttansicL1Ethernet::atl1UpdateMailbox()
{
	UInt32 tpd_next_to_use;
	UInt32 rfd_next_to_use;
	UInt32 rrd_next_to_clean;
	UInt32 value;
	
	tpd_next_to_use = atl1Adapter_.tpd_ring.next_to_use;
	rfd_next_to_use = atl1Adapter_.rfd_ring.next_to_use;
	rrd_next_to_clean = atl1Adapter_.rrd_ring.next_to_clean;
	
	value = ((rfd_next_to_use & MB_RFD_PROD_INDX_MASK) << MB_RFD_PROD_INDX_SHIFT) |
		((rrd_next_to_clean & MB_RRD_CONS_INDX_MASK) << MB_RRD_CONS_INDX_SHIFT) |
		((tpd_next_to_use & MB_TPD_PROD_INDX_MASK) << MB_TPD_PROD_INDX_SHIFT);
		
	regWrite32(REG_MAILBOX, value);
}

