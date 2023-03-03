// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        IO.h
//! \brief       Defines a set of classes encapsulating Input and Output of the LPC1768 micro controller
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _IO_H_
#define _IO_H_

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "Base.h"
#include "EventSource.h"
#include "board-DigIO.h"

/*
#define  BOARD_N_SECURITY_IN        20
#define  BOARD_N_SECURITY_OUT       4

#define GP_Recover_Safety				0		// Output
#define GP_TEST_MUX_A0					1		// Output
#define GP_TEST_MUX_EN					2		// Output
#define GP_Test_ARM						3		// Input
#define GP_Test_EM_Stop					4		// Input
#define GP_Test_Rec_Safety				5		// Input
#define GP_Test_ANT_OK					6		// Input
#define GP_Test_Bumpers					7		// Input
#define GP_LED2							8		// Output
#define GP_ENA_10V						9		// Output
#define GP_PGOOD_5V						10		// Input
#define GP_LED0							11		// Output
#define GP_SAFETY_ChkInLog				12		// Output
#define GP_ENA_5V							13		// Output
#define GP_LED1							14		// Output
#define GP_CHARGE_LED_R					15		// Output
#define GP_CHARGE_LED_G					16		// Output
#define GP_ANT_OK							17		// Input
#define GP_CHARGE_LED_Y					18		// Output
#define GP_FAN								19		// Output
#define GP_SUCT_nFAULT					20		// Input
#define GP_PUMP2_nSLEEP					21		// Output
#define GP_BRUSH_nSLEEP					22		// Output
#define GP_BRUSH_nFAULT					23		// Input
#define GP_FLOOR3							24		// Input
#define GP_FLOOR1							25		// Input
#define GP_BUMPER1						26		// Input
#define GP_LIFT_SUC_nSLEEP				27		// Output
#define GP_BUMPER0						28		// Input
#define GP_BUMPER3						29		// Input
#define GP_BUMPER2						30		// Input
#define GP_FLOOR0							31		// Input
#define GP_PUMP2_nFAULT					32		// Input
#define GP_SUCT_nSLEEP					33		// Output
#define GP_LIFT_BR_nSLEEP				34		// Output
#define GP_PUMP1_nFAULT					35		// Input
#define GP_LIFT_BR_nFAULT				36		// Input
#define GP_PUMP1_nSLEEP					37		// Output
#define GP_LIFT_SUC_nFAULT				38		// Input
#define GP_FLOOR2							39		// Input
#define GP_PGOOD_10V						40		// Input
#define GP_PGOOD_3V3						41		// Input
#define GP_USB_V_Bus						42		// Input
#define GP_ARM_OK							43		// Output
#define GP_TEST_MUX_A2					44		// Output
#define GP_TEST_MUX_A1					45		// Output
#define GP_Safety_Chk_Out_Ant			46		// Input
#define GP_T_24V_Safety					47		// Input
#define GP_WDOG_TRIGGER					48		// Output
#define GP_LIFT_BRUSH_HALL				49		// Input
#define GP_LIFT_SUCT_HALL				50		// Input

#define GP_IN_BUMPER0					0
#define GP_IN_BUMPER1					1
#define GP_IN_BUMPER2					2
#define GP_IN_BUMPER3					3
#define GP_IN_FLOOR0						4
#define GP_IN_FLOOR1						5
#define GP_IN_FLOOR2						6
#define GP_IN_FLOOR3						7
#define GP_IN_BRUSH_nFAULT				8
#define GP_IN_SUCT_nFAULT				9
#define GP_IN_LIFT_BR_nFAULT			10
#define GP_IN_LIFT_SUC_nFAULT			11
#define GP_IN_PUMP1_nFAULT				12
#define GP_IN_PUMP2_nFAULT				13
#define GP_IN_PGOOD_3V3					14
#define GP_IN_PGOOD_5					15
#define GP_IN_PGOOD_10V					16
#define GP_IN_ANT_OK						17
#define GP_IN_Safety_Chk_Out_Ant		18
#define GP_IN_T_24V_Safety				19
#define GP_IN_Test_ANT_OK				20
#define GP_IN_Test_ARM					21
#define GP_IN_Test_Bumpers				22
#define GP_IN_Test_EM_Stop				23
#define GP_IN_Test_Rec_Safety			24
#define GP_IN_USB_V_Bus					25
#define GP_IN_LIFT_BRUSH_HALL			26
#define GP_IN_LIFT_SUCT_HALL			27

#define GP_OUT_LED0						0
#define GP_OUT_LED1						1
#define GP_OUT_LED2						2
#define GP_OUT_CHARGE_LED_G			3
#define GP_OUT_CHARGE_LED_Y			4
#define GP_OUT_CHARGE_LED_R			5
#define GP_OUT_ENA_5V					6
#define GP_OUT_ENA_10V					7
#define GP_OUT_FAN						8
#define GP_OUT_BRUSH_nSLEEP			9
#define GP_OUT_SUCT_nSLEEP				10
#define GP_OUT_LIFT_BR_nSLEEP			11
#define GP_OUT_LIFT_SUC_nSLEEP		12
#define GP_OUT_PUMP1_nSLEEP			13
#define GP_OUT_PUMP2_nSLEEP			14
#define GP_OUT_Recover_Safety			15
#define GP_OUT_SAFETY_ChkInLog		16
#define GP_OUT_ARM_OK					17
#define GP_OUT_TEST_MUX_A0				18
#define GP_OUT_TEST_MUX_A1				19
#define GP_OUT_TEST_MUX_A2				20
#define GP_OUT_TEST_MUX_EN				21
#define GP_OUT_WDOG_TRIGGER			22
*/

typedef struct {
	PORT_Type *Base;
	GPIO_Type *GPIO;
	uint8_t Offset;
	uint8_t InitialState;
	bool HasInterrupt;
	int GPIO_index; 
	gpio_pin_direction_t Direction;
	port_pin_config_t Config;
} mGPIOattrib_t;

typedef struct {
	PORT_Type *PORT;
	GPIO_Type *GPIO;
	uint8_t Offset;
} mGPIO_IO_t;

typedef struct {
	port_interrupt_t 	IntMode;
	bool 					IntState;
} mGPIO_Interrupt_t;

#define NB_MAX_INPUTS	BOARD_N_GPIO_IN

// ----------------------------------------------------------------------------
//! \brief Describe falling or rising edge type of interrupt
typedef enum {
	EEdge_Rising = 0,
	EEdge_Falling = 1,
	EEdge_Both = 2
} EEdge;

// ----------------------------------------------------------------------------
//! \brief List types of modes available for IO pins.
typedef enum {
	EPinMode_NoPull 	= 0x00,
	EPinMode_PullUp 	= 0x01,
	EPinMode_PullDown = 0x02
} EPinMode;

//! \brief List types of additional options available for IO pins.
typedef enum {
	EPinOption_None 	= 0x00,
	EPinOption_ODE 	= 0x01,
	EPinOption_DSE 	= 0x02,
	EPinOption_SLEW 	= 0x04
} EPinOption;

#if defined(__cplusplus)

// ----------------------------------------------------------------------------
//! \class      DigitalIOBase
//! \brief      Base class for all digital inputs and outputs
class DigitalIOBase
{
public:
	DigitalIOBase(unsigned PinID);
    //! \cond 
	virtual ~DigitalIOBase(void) {}
	HideDefaultMethods(DigitalIOBase);
    //! \endcond 
	const char * GetName(void);

protected:
	uint8_t m_nPinId;
	const strGPIOattrib_t	*m_GPIO_ptr;
	bool							m_Valid;
	unsigned						m_PinID;
	port_interrupt_t			m_mode;
	bool ConfigPin(EPinMode _eMode,EPinOption _eOption);
};

// ----------------------------------------------------------------------------
//! \class      DigitalInput
//! \brief      Encapsulate a digital input
class DigitalInput : public DigitalIOBase, 
                     public EventSource
{
private:
	static DigitalInput* 	m_Inputs[NB_MAX_INPUTS];
	EEdge							m_InterruptEdge;	
	bool							m_isInverted;
	static bool RegisterInput(DigitalInput *_pInput);
public:
	DigitalInput(uint8_t _nPinId, bool isInverted = false, EPinMode _eMode = EPinMode_NoPull, EPinOption _eOption = EPinOption_None);
    //! \cond 
	virtual ~DigitalInput(void) {}
	HideDefaultMethods(DigitalInput);
    //! \endcond 
public:
	bool Read(void);
	bool EnableInterrupt(EEdge _eEdge);
	bool DisableInterrupt(EEdge _eEdge);
	bool ClearInterrupt(void);
	virtual void HandleInterrupt(EEdge _eEdge);
public:
	static bool DispatchInterrupt(PORT_Type *PortBase,uint8_t PinOffset);
};

// ----------------------------------------------------------------------------
//! \class      DigitalOutput
//! \brief      Encapsulate a digital output
class DigitalOutput	: public DigitalIOBase
{
private:
	bool							m_isInverted;

public:
	DigitalOutput(uint8_t _nPinId, bool isInverted = false, EPinOption _PinOption = EPinOption_None);
    //! \cond 
	virtual ~DigitalOutput(void) {}
public:
	HideDefaultMethods(DigitalOutput);
    //! \endcond 

public:
	bool Set(void);
	bool Reset(void);
	bool Read(void);
	bool Toggle(void);
};

extern "C" void PORT_IRQHandler(PORT_Type *port,unsigned IRQmask);

#else

#include <stdint.h>
#include <stdbool.h>

extern void PORT_IRQHandler(PORT_Type *port,unsigned IRQmask);

#endif

#endif // _IO_H_
