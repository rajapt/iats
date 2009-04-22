#ifndef _ATTANSICL1DEBUG_H_
#define _ATTANSICL1DEBUG_H_

#include <IOKit/IOLib.h>

#if defined(DEBUG)

#define DbgPrint(arg...)		IOLog("[AttansicL1Ethernet] " arg)

#else

#define DbgPrint(arg...)	

#endif

#define ErrPrint(arg...)		IOLog("[AttansicL1Ethernet] Error: " arg)

#endif