#ifndef _BOARD_DIGIO_H_
#define _BOARD_DIGIO_H_

#include <stdint.h>

#include "fsl_port.h"
#include "fsl_gpio.h"

#define	PORTA_USED						1
#define	PORTB_USED						1
#define	PORTC_USED						1
#define	PORTD_USED						1
#define	PORTE_USED						1

#define	BOARD_N_LED						6

#define  BOARD_N_GPIO            	62
#define  BOARD_N_GPIO_IN            33
#define  BOARD_N_GPIO_OUT           29

#define  BOARD_N_SECURITY_IN        20
#define  BOARD_N_SECURITY_OUT       4

// PORT A
#define GP_Recover_Safety				0		// Output
#define GP_END_SW1						1		// Input
#define GP_END_SW2						2		// Input
#define GP_TEST_MUX_A0					3		// Output
#define GP_FLOW_METER					4		// Input
#define GP_TEST_MUX_EN					5		// Output
#define GP_END_SW3						6		// Input
#define GP_END_SW4						7		// Input
#define GP_Test_ARM						8		// Input
#define GP_VB_ENABLE1					9		// Output
#define GP_Test_EM_Stop					10		// Input
#define GP_Test_Rec_Safety				11		// Input
#define GP_Test_ANT_OK					12		// Input
#define GP_Test_Bumpers					13		// Input
#define GP_VB_ENABLE2					14		// Output
#define GP_LED2							15		// Output
#define GP_ENA_10V						16		// Output

// PORT B
#define GP_SAFETY_ChkInLog				17		// Output
#define GP_PGOOD_5V						18		// Input
#define GP_ENA_5V							19		// Output
#define GP_LED1							20		// Output
#define GP_LED0							21		// Output
#define GP_CHARGE_VB						22		// Output
#define GP_WDOG_TRIGGER					23		// Output
#define GP_CHARGE_LED_R					24		// Output
#define GP_CHARGE_LED_G					25		// Output
#define GP_ANT_OK							26		// Input
#define GP_CHARGE_LED_Y					27		// Output
#define GP_FAN								28		// Output

// PORT C
#define GP_SUCT_nFAULT					29		// Input
#define GP_PUMP2_nSLEEP					30		// Output
#define GP_BRUSH_nFAULT					31		// Input
#define GP_UNBLOCK_L_BR					32		// Output
#define GP_BRUSH_nSLEEP					33		// Output
#define GP_LIFT_SUCT_HALL_IN			34		// Input
#define GP_LIFT_BRUSH_HALL_IN			35		// Input
#define GP_UNBLOCK_L_SUC				36		// Output
#define GP_FLOOR3							37		// Input

// PORT D
#define GP_FLOOR1							38		// Input
#define GP_BUMPER1						39		// Input
#define GP_VALVE_DOSING_PUMP			40		// Output
#define GP_LIFT_SUC_nSLEEP				41		// Output
#define GP_BUMPER0						42		// Input
#define GP_BUMPER2						43		// Input
#define GP_BUMPER3						44		// Input
#define GP_FLOOR0							45		// Input
#define GP_PUMP2_nFAULT					46		// Input
#define GP_SUCT_nSLEEP					47		// Output
#define GP_LIFT_BR_nSLEEP				48		// Output
#define GP_PUMP1_nFAULT					49		// Input
#define GP_LIFT_BR_nFAULT				50		// Input
#define GP_PUMP1_nSLEEP					51		// Output
#define GP_LIFT_SUC_nFAULT				52		// Input

// PORT E
#define GP_FLOOR2							53		// Input
#define GP_PGOOD_10V						54		// Input
#define GP_PGOOD_3V3						55		// Input
#define GP_USB_V_Bus						56		// Input
#define GP_ARM_OK							57		// Output
#define GP_TEST_MUX_A2					58		// Output
#define GP_TEST_MUX_A1					59		// Output
#define GP_Safety_Chk_Out_Ant			60		// Input
#define GP_T_24V_Safety					61		// Input



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
#define GP_IN_PGOOD_5V					15
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
#define GP_IN_END_SW1					28
#define GP_IN_END_SW2					29
#define GP_IN_END_SW3					30
#define GP_IN_END_SW4					31
#define GP_IN_FLOW_METER				32

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
#define GP_OUT_GP_VALVE_DOSING_PUMP	23
#define GP_OUT_VB_ENABLE1				24
#define GP_OUT_VB_ENABLE2				25
#define GP_OUT_CHARGE_VB				26
#define GP_OUT_UNBLOCK_L_BR			27
#define GP_OUT_UNBLOCK_L_SUC			28

#ifdef CUC_HW_V2
#define GP_LIFT_SUCT_BR_INDEX			11
#define GP_LIFT_SUCT_SUC_INDEX		10
#else
#define GP_LIFT_SUCT_BR_INDEX			17
#define GP_LIFT_SUCT_SUC_INDEX		16
#endif
#define GP_FLOW_METER_INDEX			9

typedef struct {
	PORT_Type 				*Base;
	GPIO_Type 				*GPIO;
	uint8_t 					Offset;
	uint8_t 					InitialState;
	bool 						HasInterrupt;
	bool						IsInverted;
	int 						GPIO_index; 
	gpio_pin_direction_t Direction;
	port_pin_config_t 	Config;
	char						Name[32];
} strGPIOattrib_t;

typedef struct {
	PORT_Type 				*PORT;
	GPIO_Type 				*GPIO;
	uint8_t 					Offset;
} GPIO_IO_t;

typedef struct {
	port_interrupt_t 		IntMode;
	bool 						IntState;
} GPIO_Interrupt_t;

typedef enum {
	eLED0 = 0,
	eLED1,
	eLED2,
	eCHARGE_LED_G,
	eCHARGE_LED_Y,
	eCHARGE_LED_R
} eLED_t;

typedef struct {
	unsigned					period;
	unsigned 				time;
	eLED_t					LED;
	bool						enabled;
	uint8_t					IOport;
} structLED_t;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

void BOARD_setGPIO_PinAttributes(void);
const strGPIOattrib_t *BOARD_getGPIOentry(unsigned index);
const char * BOARD_GetGPIO_Name(int Pin);
bool BOARD_WriteGPIOpin(unsigned index,uint8_t value);
bool BOARD_ReadGPIOpin(unsigned index,uint8_t *value);
bool BOARD_ToggleGPIOpin(unsigned index);
bool BOARD_SetGPIO_Output(unsigned index,uint8_t value);
bool BOARD_GetGPIO_Input(unsigned index,uint8_t *value);
bool BOARD_GetAllGPIO_Inputs(unsigned index,uint32_t *value);
bool BOARD_GetGPIO_Readback(unsigned index,uint8_t *value);
bool BOARD_GetAllGPIO_Readbacks(unsigned index,uint32_t *value);
bool BOARD_ConfigGPI_Interrupt(unsigned index,port_interrupt_t mode);
const strGPIOattrib_t * BOARD_FindPin(PORT_Type *Port,unsigned Offset);
const strGPIOattrib_t * BOARD_GetPinStructPtr(unsigned Pin);

bool BOARD_SetTestMUX(unsigned channel,bool enable);
bool BOARD_GetTestMUX(unsigned *channel,bool *enable);
bool BOARD_GetBumper(unsigned channel);
uint8_t BOARD_GetAllBumpers(void);
bool BOARD_GetFloorSensor(unsigned channel);
bool BOARD_SetAllSecurityOutput(uint32_t value);
uint8_t BOARD_GetAllFloorSensors(void);

void BOARD_SetRecoverSafety(bool value);
bool BOARD_GetRecoverSafety(void);
void BOARD_SetTestMuxA0(bool value);
bool BOARD_GetTestMuxA0(void);
void BOARD_SetTestMuxEN(bool value);
bool BOARD_GetTestMuxEN(void);
bool BOARD_GetTestARM(void);
bool BOARD_GetTestEMstop(void);
bool BOARD_GetTestRecSafety(void);
void BOARD_GetTestANTok(bool value);
bool BOARD_GetTestBumpers(void);
void BOARD_SetLED2(bool value);
bool BOARD_GetLED2(void);
void BOARD_SetENA10V(bool value);
bool BOARD_GetENA10V(void);
bool BOARD_GetPGOOD5V(void);
void BOARD_SetLED0(bool value);
bool BOARD_GetLED0(void);
void BOARD_SetSafetyChkInLog(bool value);
bool BOARD_GetSafetyChkInLog(void);
void BOARD_SetENA5V(bool value);
bool BOARD_GetENA5V(void);
void BOARD_SetLED1(bool value);
bool BOARD_GetLED1(void);
void BOARD_SetChargeLED_R(bool value);
bool BOARD_GetChargeLED_R(void);
void BOARD_SetChargeLED_G(bool value);
bool BOARD_GetChargeLED_G(void);
void BOARD_SetChargeLED_Y(bool value);
bool BOARD_GetChargeLED_Y(void);
bool BOARD_GetANTok(void);
void BOARD_SetFAN(bool value);
bool BOARD_GetFAN(void);
bool BOARD_GetSuct_nFAULT(void);
void BOARD_SetPump1_nSLEEP(bool value);
bool BOARD_GetPump1_nSLEEP(void);
void BOARD_SetPump2_nSLEEP(bool value);
bool BOARD_GetPump2_nSLEEP(void);
void BOARD_SetBrush_nSLEEP(bool value);
bool BOARD_GetBrush_nSLEEP(void);
bool BOARD_GetBrush_nFAULT(void);
bool BOARD_GetFLOOR3(void);
bool BOARD_GetFLOOR1(void);
bool BOARD_GetBUMPER1(void);
void BOARD_SetLiftSuct_nSLEEP(bool value);
bool BOARD_GetLiftSuct_nSLEEP(void);
bool BOARD_GetBUMPER0(void);
bool BOARD_GetBUMPER3(void);
bool BOARD_GetBUMPER2(void);
bool BOARD_GetFLOOR0(void);
bool BOARD_GetPump2_nFAULT(void);
void BOARD_SetSuct_nSLEEP(bool value);
bool BOARD_GetSuct_nSLEEP(void);
void BOARD_SetLiftBR_nSLEEP(bool value);
bool BOARD_GetLiftBR_nSLEEP(void);
bool BOARD_GetPump1_nFAULT(void);
bool BOARD_GetLiftBR_nFAULT(void);
bool BOARD_GetFLOOR2(void);
bool BOARD_GetPGOOD10V(void);
bool BOARD_GetPGOOD3V3(void);
void BOARD_SetARMok(bool value);
bool BOARD_GetARMok(void);
void BOARD_SetTestMuxA1(bool value);
bool BOARD_GetTestMuxA1(void);
void BOARD_SetTestMuxA2(bool value);
bool BOARD_GetTestMuxA2(void);
bool BOARD_GetSafetyChkOutANT(void);
bool BOARD_GetT24_Safety(void);
void BOARD_SetTriggerWatchdog(bool value);
bool BOARD_GetTriggerWatchdog(void);
void BOARD_SetVBenable1(bool value);
bool BOARD_GetVBenable1(void);
void BOARD_SetVBenable2(bool value);
bool BOARD_GetVBenable2(void);
void BOARD_SetChargeVB(bool value);
bool BOARD_GetChargeVB(void);
void BOARD_SetUnblockL_BR(bool value);
bool BOARD_GetUnblockL_BR(void);
void BOARD_SetUnblockL_SUC(bool value);
bool BOARD_GetUnblockL_SUC(void);
bool BOARD_GetEndSwitch1(void);
bool BOARD_GetEndSwitch2(void);
bool BOARD_GetEndSwitch3(void);
bool BOARD_GetEndSwitch4(void);
uint8_t BOARD_GetEndSwitches(void);
void Board_SetAllPWMnSleep(bool value);
void Board_RestorePWMnSleep(uint8_t mask);
uint8_t Board_GetPWMnSleep(void);
uint8_t Board_GetPWMnFault(void);

bool BOARD_GetAllGPIOsignals(uint32_t *result,unsigned size);
bool BOARD_GetSecurityGPIO(unsigned channel,uint8_t *result);
bool BOARD_GetAllSecurityGPIOs(uint32_t *result,unsigned size);
bool BOARD_SetSecurityOutput(unsigned channel,uint8_t value);
void BOARD_Blink_Toggle_LED(void);
void BOARD_Toggle_LED(void);
bool BOARD_EnableLED_Blink(eLED_t led,bool enable,unsigned period);
bool BOARD_Enable_5V(void);
bool BOARD_Enable_10V(void);
bool BOARD_Disable_5V(void);
bool BOARD_Disable_10V(void);
bool BOARD_RegisterGPIO_Callback(int port,int pin,void (*callback)(int));
bool BOARD_EnaHallTestCounters(uint8_t status,uint8_t IRQmask);
bool BOARD_ResetHallTestCounters(uint8_t mask);
bool BOARD_GetHallTestCounters(uint32_t *Cntr_Hall_1,uint32_t *Cntr_Hall_2);
bool BOARD_GetHallTestStatus(uint8_t *status);
bool BOARD_EnaFlowMeterTestCounter(uint8_t status,uint8_t IRQmask);
bool BOARD_ResetFlowMeterTestCounter(void);
bool BOARD_GetFlowMeterTestCounters(uint32_t *Cntr_FlowMeter);
bool BOARD_GetFlowMeterTestStatus(uint8_t *status);

bool BOARD_Enable_Port_IRQ(PORT_Type *port);
bool BOARD_Disable_Port_IRQ(PORT_Type *port);

bool BOARD_InitDigIO(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_DIGIO_H_ */
