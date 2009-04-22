#ifndef _ATTANSICL1ETHERNET_H_
#define _ATTANSICL1ETHERNET_H_

#include <IOKit/IOLib.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOEthernetInterface.h>
#include <IOKit/network/IOGatedOutputQueue.h>
#include <IOKit/pci/IOPCIDevice.h>

#include "AttansicL1Hw.h"
#include "AttansicL1Params.h"

enum
{
	MEDIUM_INDEX_10HD	= 0,
	MEDIUM_INDEX_10FD	= 1,
	MEDIUM_INDEX_100HD	= 2,
	MEDIUM_INDEX_100FD	= 3, 
	MEDIUM_INDEX_1000HD = 4,
	MEDIUM_INDEX_1000FD = 5,
	MEDIUM_INDEX_AUTO	= 6,
	MEDIUM_INDEX_COUNT	= 7
};

#define kTransmitQueueCapacity			384

class AttansicL1Ethernet : public IOEthernetController
{
	OSDeclareDefaultStructors(AttansicL1Ethernet)
public:
	virtual bool				init(OSDictionary *properties);
	virtual void				free();
	virtual bool				start(IOService *provider);
	virtual void				stop(IOService *provider);
	
	UInt16						regRead16(UInt32 offset);
	UInt32						regRead32(UInt32 offset);
	
	void						regWrite16(UInt32 offset, UInt16 data);
	void						regWrite32(UInt32 offset, UInt32 data);
	
	virtual IOReturn			enable(IONetworkInterface *netif);
    virtual IOReturn			disable(IONetworkInterface *netif);
	
	virtual bool				configureInterface(IONetworkInterface *netif);
    virtual const OSString		*newVendorString() const;
    virtual const OSString		*newModelString() const;
	virtual IOReturn			selectMedium(const IONetworkMedium *medium);
	
	virtual IOReturn			getHardwareAddress(IOEthernetAddress *addr);
	virtual IOReturn			setHardwareAddress(const IOEthernetAddress *addr); 
	
    virtual void				getPacketBufferConstraints(IOPacketBufferConstraints *constraints) const;
	virtual UInt32				outputPacket(mbuf_t m, void *param);

    virtual IOReturn			registerWithPolicyMaker(IOService *policyMaker);
    virtual IOReturn			setPowerState(unsigned long powerStateOrdinal, IOService *policyMaker);
	
    virtual IOReturn			setPromiscuousMode(bool enabled);
    virtual IOReturn			setMulticastMode(bool enabled);
    virtual IOReturn			setMulticastList(IOEthernetAddress *addrs, UInt32 count);
private:
	virtual bool				OSAddNetworkMedium(UInt32 type, UInt32 bps, UInt32 index);
	virtual IOOutputQueue		*createOutputQueue(); 
	
	virtual bool				atl1AllocateDescriptors();
	virtual void				atl1FreeDescriptors();
	virtual UInt16				atl1AllocRxBuffers();
	virtual bool				atl1AllocTxBuffers();
	virtual void				atl1CleanRxRing();
	virtual void				atl1CleanTxRing();	
	
	virtual void				atl1RxInterrupt();
	virtual void				atl1TxInterrupt();
	void						atl1InterruptHandler(OSObject *client, IOInterruptEventSource *src, int count);

	virtual int					mdio_read(int phy_id, int reg_num);
	virtual void				mdio_write(int phy_id, int reg_num, int val);

	virtual bool				atl1Probe();
	virtual bool				atl1SoftwareInit();
	virtual void				atl1SoftwareFree();
	
	//Hardware functions
	virtual SInt32				atl1ResetHw();
	virtual SInt32				atl1ChechEepromExist();
	virtual bool				atl1ReadEeprom(UInt32 offset, UInt32 *value);
	virtual SInt32				atl1ReadPhyReg(UInt16 reg_addr, UInt16 *phy_data);
	virtual bool				atl1SpiRead(UInt32 addr, UInt32 *buf);
	virtual SInt32				atl1GetPermanentAddress();
	virtual UInt32				atl1HashMcAddr(UInt8 *mc_addr);
	virtual void				atl1HashSet(UInt32 hash_value);
	virtual SInt32				atl1WritePhyReg(UInt32 reg_addr, UInt16 phy_data);
	virtual SInt32				atl1PhyLeavePowerSaving();
	virtual SInt32				atl1PhyReset();
	virtual SInt32				atl1PhySetupAutonegAdv();
	virtual SInt32				atl1SetupLink();
	virtual void				atl1InitFlashOpcode();
	virtual SInt32				atl1InitHw();
	virtual SInt32				atl1GetSpeedAndDuplex(UInt16 *speed, UInt16 *duplex);
	virtual void				atl1GetAndUpdateLinkStatus();
	virtual void				atl1SetMacAddr();
	virtual void				atl1PciePatch();
	
	virtual void				atl1InterruptsEnable();
	virtual void				atl1InterruptsDisable();
	virtual UInt32				atl1Configure();
	virtual void				setFlowControlOld();
	virtual void				setFlowControlNew();
	virtual void				atl1SetupMacCtrl();
	
	virtual void				atl1UpdateMailbox();

private:
	//Private fields, w/o methods
	AttansicL1Hardware			atl1Hw_;
	AttansicL1Adapter			atl1Adapter_;
	
	bool						descMemAllocated_;
	UInt16						vendorId_, deviceId_;
	IOMemoryMap					*hw_addr;
	char						*mmr_base;
	IOPCIDevice					*pciDev_;
	IOEthernetInterface			*netIface_;
	IONetworkStats				*netStats_;
	IOEthernetStats				*etherStats_;
	IOWorkLoop					*workLoop_;
	IOOutputQueue				*transmitQueue_;
	OSDictionary				*mediumDict;
	const IONetworkMedium		*mediumTable[MEDIUM_INDEX_COUNT];
	IOInterruptEventSource		*intSource_;
};

#endif
