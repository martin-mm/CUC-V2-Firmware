// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        CleaningUnit.h
//! \brief       Common definitions for the cleaning unit project
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _CLEANINGUNIT_H_
#define _CLEANINGUNIT_H_

#include "base.h"

//#define TEST_MODE 1

// Define board version and their specificities
//#define BOARD_VERSION 10
#define BOARD_VERSION 11
#if BOARD_VERSION == 10
#define HallSensorInput CaptureInput
#endif
#if BOARD_VERSION == 11
#define HallSensorInput 	DigitalInput
#define FlowMeterInput 	DigitalInput
#endif

// Define lift's motor version and their specificities
//#define LIFTMOTOR_VERSION 1  //AHC 12V 0 390 201 989
#define LIFTMOTOR_VERSION 2  //AHC 12V 0 390 203 687
#if LIFTMOTOR_VERSION == 1
	 #define CNTS_PER_MM 11
#endif
#if LIFTMOTOR_VERSION == 2
	 #define CNTS_PER_MM 8
#endif

#endif // _CLEANINGUNIT_H_
