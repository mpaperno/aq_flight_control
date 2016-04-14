/*
    This file is part of the HILSim For AutoQuad library.

    HILSim is licensed for personal hobby use only.  All other users,
    including, but not limited to, commercial, professional,
    university/college, or government, require a separate license agreement.
    Contact http://www.WorldDesign.com for details.

    Copyright 2015-2016 Maxim Paperno.  All Rights Reserved.
*/

#ifndef _hilSim_h
#define _hilSim_h

#include "stdint.h"

#ifdef HAS_HIL_SIM_MP
#define hilSimAvailable()	(1)
extern void hilSimInit(void);
extern void hilSimTick(uint32_t loop);
#else
#define hilSimAvailable()	(0)
#define hilSimInit()
#define hilSimTick(X)
#endif

#endif /* _hilSim_h */
