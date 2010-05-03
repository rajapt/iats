/*
 *  at_osdep.h-- OS definitions
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

 
#ifndef _ATHEROS_OS_DEP_H_
#define _ATHEROS_OS_DEP_H_

#include <IOKit/IOLib.h>
#include <IOKit/IOTypes.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOBufferMemoryDescriptor.h>

#define u8		UInt8
#define u16		UInt16
#define u32		unsigned int
#define u64		UInt64
#define s32		signed int

//#define spinlock_t	IOSimpleLock * 
#define dma_addr_t  IOPhysicalAddress
#define atomic_t    volatile signed int

#define usec_delay(x) IODelay(x)
#define msec_delay(x) IOSleep(x)

/*********************************************************************************************/
#if defined(DEBUG)
	#define DbgPrint(arg...)	IOLog("[AtherosL1Ethernet] " arg)
#else
	#define DbgPrint(arg...)
#endif

#define ErrPrint(arg...)	IOLog("[AtherosL1Ethernet] Error: " arg)

#define DEBUGFUNC(arg...)  DbgPrint( arg )
#define DEBUGOUT(arg...)   DbgPrint( arg)

#define DEBUGOUT1(arg...)  DbgPrint( arg )
#define AT_ERR(arg...)  ErrPrint( arg )

/*********************************************************************************************/
#define AT_WRITE_REG(a, reg, value)  { if( (a)->mmr_base ) { OSWriteLittleInt32((a)->mmr_base, reg, value);} }

#define AT_WRITE_FLUSH(a)  (((a)->mmr_base ) ? OSReadLittleInt32((a)->mmr_base, 0) : 0xFFFFFFFF)

#define AT_READ_REG(a, reg)   (((a)->mmr_base ) ? OSReadLittleInt32((a)->mmr_base, reg) : 0xFFFFFFFF)
       
#define AT_WRITE_REGB(a, reg, value) { if( (a)->mmr_base ) { *(volatile uint8_t *)((uintptr_t)(a)->mmr_base + reg) = value; } }

//#define AT_READ_REGB(a, reg) (\
//    readb((a)->hw_addr + reg))

#define AT_WRITE_REGW(a, reg, value)   { if( (a)->mmr_base ) { OSWriteLittleInt16((a)->mmr_base, reg, value);} }
    
#define AT_READ_REGW(a, reg)  (((a)->mmr_base ) ? OSReadLittleInt16((a)->mmr_base, reg) : 0xFFFF)

#define AT_WRITE_REG_ARRAY(a, reg, offset, value)  { if( (a)->mmr_base ) { OSWriteLittleInt32((a)->mmr_base, ((reg) + ((offset) << 2)) , value);} }
	
#define AT_READ_REG_ARRAY(a, reg, offset)    (((a)->mmr_base ) ? OSReadLittleInt32((a)->mmr_base, ((reg) + ((offset) << 2)) ) : 0xFFFFFFFF)

/*********************************************************************************************/
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

/*********************************************************************************************/
static inline void atomic_set(atomic_t *v, int i)
{
     *v = i;
}

#define atomic_read(v)          (*(v))

#define set_bit(nr,addr)   ( *(addr) |= (1 << (nr)) ) 
#define clear_bit(nr,addr) ( *(addr) &= ~(1 << (nr)))
#define test_bit(nr,addr)  ( (*(addr)>>(nr)) & 0x01 ) 


#endif//_ATHEROS_OS_DEP_H_

