// Using this board file with AQ M4 boards <= rev 5 (NOT r6, aka. v2) enables voltage sensing on expansion port pin J2-5 (PA6)

#if BOARD_REVISION == 3
    #include "board_m4_r3.h"
#elif BOARD_REVISION == 4 || BOARD_REVISION == 5
    #include "board_m4_r5.h"
#elif BOARD_REVISION == 6
    //#include "board_m4_r6.h"
    #error "Can't be used with M4 r6"
#endif

// disable SPI I/O because we need PA6
#undef SPI_SPI1_CLOCK
// make sure CYRF doesn't try to init SPI1
#ifdef CYRF_SPI
#undef CYRF_SPI
#endif

// set up voltage divider (already set up in board_m4_r6)
// these values are known valid for:
//   - aBUGSworstnightmare BLDC expansion boards 0r1-3
#ifndef ANALOG_EXT_VOLT_RTOP
#define ANALOG_EXT_VOLT_RTOP	10.0f
#define ANALOG_EXT_VOLT_RBOT	1.2f
#endif

// set up input pin (PA6)
#undef ANALOG_CHANNEL_EXT_VOLT
#define	ANALOG_CHANNEL_EXT_VOLT	    ADC_Channel_6
