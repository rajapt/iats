/*
 * linux/mii.h: definitions for MII-compatible transceivers
 * Originally drivers/net/sunhme.h.
 *
 * Copyright (C) 1996, 1999, 2001 David S. Miller (davem@redhat.com)
 */

#ifndef __LINUX_MII_H__
#define __LINUX_MII_H__


/* Generic MII registers. */

#define MII_BMCR            0x00        /* Basic mode control register */
#define MII_BMSR            0x01        /* Basic mode status register  */
#define MII_PHYSID1         0x02        /* PHYS ID 1                   */
#define MII_PHYSID2         0x03        /* PHYS ID 2                   */
#define MII_ADVERTISE       0x04        /* Advertisement control reg   */
#define MII_LPA             0x05        /* Link partner ability reg    */
#define MII_EXPANSION       0x06        /* Expansion register          */
#define MII_CTRL1000        0x09        /* 1000BASE-T control          */
#define MII_STAT1000        0x0a        /* 1000BASE-T status           */
#define MII_ESTATUS         0x0f        /* Extended Status */
#define MII_DCOUNTER        0x12        /* Disconnect counter          */
#define MII_FCSCOUNTER      0x13        /* False carrier counter       */
#define MII_NWAYTEST        0x14        /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER     0x15        /* Receive error counter       */
#define MII_SREVISION       0x16        /* Silicon revision            */
#define MII_RESV1           0x17        /* Reserved...                 */
#define MII_LBRERROR        0x18        /* Lpback, rx, bypass error    */
#define MII_PHYADDR         0x19        /* PHY address                 */
#define MII_RESV2           0x1a        /* Reserved...                 */
#define MII_TPISTATUS       0x1b        /* TPI status for 10mbps       */
#define MII_NCONFIG         0x1c        /* Network interface config    */

/* Link partner ability register. */
#define LPA_SLCT                0x001f  /* Same as advertise selector  */
#define LPA_10HALF              0x0020  /* Can do 10mbps half-duplex   */
#define LPA_10FULL              0x0040  /* Can do 10mbps full-duplex   */
#define LPA_100HALF             0x0080  /* Can do 100mbps half-duplex  */
#define LPA_100FULL             0x0100  /* Can do 100mbps full-duplex  */
#define LPA_100BASE4            0x0200  /* Can do 100mbps 4k packets   */
#define LPA_RESV                0x1c00  /* Unused...                   */
#define LPA_RFAULT              0x2000  /* Link partner faulted        */
#define LPA_LPACK               0x4000  /* Link partner acked us       */
#define LPA_NPAGE               0x8000  /* Next page bit               */
 
#define LPA_DUPLEX              (LPA_10FULL | LPA_100FULL)
#define LPA_100                 (LPA_100FULL | LPA_100HALF | LPA_100BASE4)

#endif /* __LINUX_MII_H__ */
