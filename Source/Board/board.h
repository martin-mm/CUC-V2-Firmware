#ifndef _BOARD_H_
#define _BOARD_H_

#define BOARD_NAME "CUC"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "clock_config.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_ftm.h"
#include "fsl_uart.h"
#include "fsl_adc16.h"
#include "fsl_dac.h"
#include "fsl_vref.h"
#include "fsl_i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "board-Ana.h"

#if defined (TRACEALYZER ) && (TRACEALYZER != 0)
#include "TracealyzerSetup.h"
#endif


#define CONSOLE_UART             	UART0
#define CONSOLE_UART_BAUDRATE    	115200

//#define dbgprintf							
//#define vdbgprintf						
#define dbgprintf							printf
#define vdbgprintf						vprintf

#define BOARD_HW_CAN0					1
#define BOARD_CAN0_IRQ_ENA				1

#define USE_DRV8701P						0
//#define USE_DRV8701P
//#undef  USE_DRV8701P

#define N_PWM_FTM_TIMER_CHANNELS		3

#define PDB_USED							0
#define PDB_MOD_VALUE					37500000
#define USE_PDB_FOR_PUMPS				0

#define FTM0_USED							1
#define FTM1_USED							1
#define FTM2_USED							1
#define FTM3_USED							1

#define PIT0_USED							1
#define PIT1_USED							1
#define PIT2_USED							0

#define CAN_USED							1

#define CommTask_PRIORITY          	(tskIDLE_PRIORITY + 2)
#define CommTask_STACK_SIZE	      1024

#define PWM_FREQUENCY_FTM0_HZ      	10000
#define PWM_FREQUENCY_FTM1_HZ      	10000
#define PWM_FREQUENCY_FTM3_HZ      	10000
#define PWM_DIVIDER_FTM0				1
#define PWM_DIVIDER_FTM1				1
#define PWM_DIVIDER_FTM3				1
#define TIMER0_FREQUENCY_HZ        	10000
#define TIMER1_FREQUENCY_HZ        	1000
#define TIMER2_FREQUENCY_HZ        	1000
#define TIMER3_FREQUENCY_HZ        	10000

#define PIT0_PERIOD						1				// PIT Timeout in ms
#define PIT1_PERIOD						10				// PIT Timeout in ms
#define PIT2_PERIOD						10				// PIT Timeout in ms

#define PIT1_STARTS_ADC					1

#define PWM_DEADTIME						50				// Deadtime in ns

#define FTM0_INT_PRIORITY           13
#define FTM1_INT_PRIORITY           13
#define FTM2_INT_PRIORITY           13
#define FTM3_INT_PRIORITY           13

#define FTM0_INT_ENA           		1
#define FTM1_INT_ENA           		1
#define FTM2_INT_ENA           		1
#define FTM3_INT_ENA           		1

#define BOARD_PIT0_IRQ_ENA				1
#define BOARD_PIT1_IRQ_ENA				1
#define BOARD_PIT2_IRQ_ENA				0

#define PIT0_INT_PRIORITY           14
#define PIT1_INT_PRIORITY           4
#define PIT2_INT_PRIORITY           4

#define RTC_INT_PRIORITY            5

#define PORTA_INT_PRIORITY          14
#define PORTB_INT_PRIORITY          14
#define PORTC_INT_PRIORITY          14
#define PORTD_INT_PRIORITY          14
#define PORTE_INT_PRIORITY          14

#define  UART0_INT_PRIORITY         11
#define  UART1_INT_PRIORITY         11
#define  UART2_INT_PRIORITY         11
#define  UART3_INT_PRIORITY         11
#define  UART4_INT_PRIORITY         11
#define  UART5_INT_PRIORITY         11

#define N_SUPPLY_RAILS					3

#define RELAY1_VB1_THRESHOLD			30000

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
#define N_PWM_CHANNELS					6
#else
#define N_PWM_CHANNELS					12
#endif
#define N_PWM_CHANNELS_FTM0			4
#define N_PWM_CHANNELS_FTM1			0
#define N_PWM_CHANNELS_FTM3			8
#define N_PWM_CONTROL_CHANNELS		6

#define PWM_CONTROL_BRUSH				0						
#define PWM_CONTROL_SUCT				1						
#define PWM_CONTROL_LIFT_BR			2						
#define PWM_CONTROL_LIFT_SUC			3						
#define PWM_CONTROL_PUMP1				4						
#define PWM_CONTROL_PUMP2				5		

#define I2C_CHANNEL						1

#define CAN_CHANNEL						0

#define RELAY_FLAGS_CHECK_VB1			(1 << 0)
#define RELAY_FLAGS_VB1_LOSS_VB		(1 << 1)

#define RELAY_VB1_STATUS_VB1			(1 << 0)
#define RELAY_VB1_STATUS_NO_CHECK	(1 << 1)

#define USE_ENDSWITCH					0

#define FEED_WD_FTM1						1

#define USE_FLOAT							1
#define USE_IIR_FILTER					1

#define USE_STACK_PROTECTION			1

typedef struct
{
	FTM_Type * const			Timer;			// the timer that controls the PWM
	uint32_t						fTimer;			// Timer Tick Frequency in Hz (equals BusClockFrequency / TimerDivider)
	uint32_t						divider;			// Timer Divider Value (0 .. 7 equals 2^0 .. 2^7)
	uint32_t						nPWMchannels;	// Number of PWM channels
	uint32_t						indexPWMchan;	// index to the first member in the PWMchannel array
} PWM_FTMtimer_t;

typedef struct {
	PWM_FTMtimer_t const * 		TimerCH;			// pointer to the timer channel that controls the PWM (points to an FTMtimer_t struct)
	const uint16_t					Channel;			// The Timer physical PWM channel;
	uint16_t							TimerPeriod;	// The Timer Period (Reload Value) in Timer Ticks;
	uint16_t							ActualPWM;		// The actual PWM value	
	uint8_t							InitialVal;		// Initial Value of the channel (FTMx_OUTINT register)
	uint8_t							TimerIndex;		// Index of the PWM timer that generates the PWM
	ftm_pwm_level_select_t		Polarity;		// Polarity of the channel (FTMx_POL register)
} PWMchannel_t;

typedef enum {
	eDirModeRight = 0,
	eDirModeLeft,
	eDirModeBrake,
	eDirModeHighZ,
	eDirModeSleep
} eDirMode_t;

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)

typedef struct {
	PWMchannel_t 				*	PWMchannel;		// the PWM control
	PORT_Type					*	DirectionPORT;	// the PWM channel 2 of the control
	GPIO_Type					*	DirectionGPIO;	// the PWM channel 2 of the control
	uint8_t							DirectionPIN;	// the PWM channel 2 of the control
	eDirMode_t						DirMode;			// the direction / mode (Left, Right, Brake, HighZ, Sleep) 
	unsigned							pwm;				// the pwm value in %%
	uint8_t							SleepGPIO;		// GPIO of the nSLEEP Signal
	uint8_t							FaultGPIO;		// GPIO of the Fault Signal
	uint8_t							TimerIndex;		// Index of the PWM timer that generates the PWM
	char								Name[16];		// Name of the PWM channel
} PWM_Control_t;

#else

typedef struct {
	PWMchannel_t 				*	PWMchannel1;	// the PWM channel 1 of the control
	PWMchannel_t 				*	PWMchannel2;	// the PWM channel 2 of the control
	eDirMode_t						DirMode;			// the direction / mode (Left, Right, Brake, HighZ, Sleep) 
	unsigned							pwm;				// the pwm value in %%
	uint8_t							SleepGPIO;		// GPIO of the nSLEEP Signal
	uint8_t							FaultGPIO;		// GPIO of the Fault Signal
	uint8_t							TimerIndex;		// Index of the PWM timer that generates the PWM
	char								Name[16];		// Name of the PWM channel
} PWM_Control_t;

#endif

typedef struct {
	uint64_t		ticks_ms;
	uint16_t		ticks_timer;
} SystemTime_t;

typedef struct {
	uint16_t		frequency;			// in Hz
	uint16_t		pulse;				// in ms
	bool			enable;
	uint32_t		period;				// in ms
	uint32_t		period_ctr;			// in ms
	uint32_t		pwm_on;				// in %%
} PumpCtrl_t;

typedef struct{
	uint16_t   	active_dur_p1;		// activation duration pump 1 in ms
	uint16_t   	active_dur_p2;		// activation duration pump 2 in ms
	uint16_t   	pulse_dur_p1;		// pulse duration pump 1 in ms
	uint16_t   	pulse_dur_p2;		// pulse duration pump 2 in ms
	uint16_t   	deactive_dur_p1;	// deactivation duration pump 1 in ms
	uint16_t   	deactive_dur_p2;	// deactivation duration pump 2 in ms
	uint32_t		period;				// total period overall
	uint32_t		wait;					// Rest Pweriod
	uint32_t		period_ctr;			// period counter (in ms)
	uint16_t		PWM_on;				// PWM Pn-Value
	bool			ctrl_by_cl_mgr;	
	bool			PumpCtrlByClMgr;	// true if pump controlled by cleaning manager
	bool 			ValveCtrlByClMgr;	// true valve controlled by cleaning manager
} PumpCtrlClMgr_t;

typedef struct {
	int			frequency;			// Clock Frequency of the PDB in Hz
	int			prescaler;			// Prescaler of the PDB
	int			multiplier;			// Multiplier of the PDB
} PDB_PumpCtrl_Config_t;

extern TaskHandle_t TaskComm;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

	bool BOARD_InitFTM0(void);
	bool BOARD_InitFTM1(void);
	bool BOARD_InitFTM2(void);
	bool BOARD_InitFTM3(void);
	bool BOARD_InitPIT(void);
	void BOARD_InitVREF(void);
	int BOARD_InitCAN(unsigned channel);
	bool BOARD_Init_WDOG(void);
	// void FTM0_IRQHandler(void);
	// void FTM1_IRQHandler(void);
	// void FTM2_IRQHandler(void);
	// void FTM3_IRQHandler(void);
	// void PIT0_IRQHandler(void);
	// void PIT1_IRQHandler(void);
	// void ADC0_IRQHandler(void);
	// void ADC1_IRQHandler(void);
	// void CAN0_ORed_Message_buffer_IRQHandler(void);
	bool BOARD_PowerUpSupplyRails(void);
	bool BOARD_PowerDownSupplyRails(void);
#if USE_FLOAT != 0
	int BOARD_FTM_CalcPrescaler(float TimerFrequency,float ClockSource);
#else
	int BOARD_FTM_CalcPrescaler(uint32_t TimerFrequency,uint32_t ClockSource);
#endif
	bool BOARD_Set_PWM_FTM_Frequency(unsigned channel,uint32_t frequency);
	int BOARD_Get_PWM_FTM_Frequency(unsigned channel);
	int BOARD_Get_PWM_FTM_Period(unsigned channel);
	bool BOARD_Set_FTM_PWM(unsigned channel,unsigned dutycycle);
	int BOARD_Get_FTM_PWM(unsigned channel);
	bool BOARD_Get_FTM_PWM_from_ptr(PWMchannel_t *pwm_channel,unsigned *dutycycle);
	bool BOARD_Get_FTM_PWM_TimerIndex(unsigned channel,uint8_t *index);
	bool BOARD_PITsetPeriod(unsigned channel,unsigned period);
	bool BOARD_PITenableInterrupt(unsigned channel,bool flag);
	bool BOARD_SetPWMControl(unsigned channel,unsigned pwm,eDirMode_t DirMode);
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	bool BOARD_GetPWMControl(unsigned channel,unsigned *pwm,eDirMode_t *DirMode,
		unsigned *dutycycle);
#else
	bool BOARD_GetPWMControl(unsigned channel,unsigned *pwm,eDirMode_t *DirMode,
		unsigned *dutycycle1,unsigned *dutycycle2);
#endif
	bool BOARD_CheckPWMControl(unsigned channel);
	void BOARD_EnablePWM(void);
	void BOARD_DisablePWM(void);
	bool BOARD_EnablePWMchannel(unsigned channel);
	bool BOARD_DisablePWMchannel(unsigned channel);
	bool BOARD_GetPWMchannelStatus(unsigned channel,uint8_t *status);
	bool BOARD_GetPWMdirection(unsigned channel,eDirMode_t *DirMode);
	bool BOARD_GetPWMTimer(unsigned channel,uint8_t *timer);
	const char * BOARD_GetPWMName(unsigned channel);
	PWM_Control_t * BOARD_GetPWM_ptr(unsigned channel);
	bool BOARD_GetPWM_CounterPeriod(unsigned channel,uint32_t *period);
	bool BOARD_GetPWMStatus(unsigned channel,uint32_t *status);
	bool BOARD_getSystemTime(uint64_t *time);
	uint64_t BOARD_getSystemTimeDirect(void);
	void BOARD_SetOwnAddress(uint16_t address);
	uint16_t BOARD_GetOwnAddress(void);
	bool BOARD_Enable_WDOG(void);
	bool BOARD_Disable_WDOG(void);
	uint32_t BOARD_WDOG_getStatus(void);
	bool BOARD_WDOG_clearStatus(uint32_t mask);
	bool BOARD_WDOG_setTimeOut(uint32_t timeout);
	bool BOARD_WDOG_Unlock(void);
	void BOARD_WDOG_Feed(void);
	bool BOARD_Set_Test_Ext_WD(uint8_t flag);
	bool BOARD_SetPumpFreqPulse(uint8_t channel,uint16_t frequency,uint16_t pulselen);
	bool BOARD_GetPumpFreqPulse(uint8_t channel,uint16_t *frequency,uint16_t *pulselen);
	bool BOARD_SetPumpPWM(uint8_t channel,uint16_t PWMvalue);
	bool BOARD_SetWaterPumpParams(uint8_t channel,uint16_t period,uint16_t pulselen,uint16_t PWMvalue);
	bool BOARD_EnablePump(uint8_t channel,bool OnOff);
	bool BOARD_GetPumpStatus(uint8_t channel,bool *OnOff);
	bool BOARD_GetRelayStatus(uint8_t *status,float *VB1_rail);
	bool BOARD_TestRelayStatus(void);
	void HandleRelay1(void);
	void BOARD_Reset_ADC_Trigger(void);
	bool BOARD_getADC_Trigger(void);
	void BOARD_Ena_CUC_IRQ_Callback(bool enable);
	bool BOARD_ClMngr_SetParameters(uint16_t ActDur,uint16_t PulseDur,uint16_t DeactDur,
		uint32_t Period,uint16_t PWM);
	bool BOARD_ClMngr_EnaPumpCtrl(bool Enable);
	bool BOARD_ClMngr_EnaClFuidValveCtrl(bool Enable);
	bool BOARD_isPumpClMgrEnabled(void);
	bool BOARD_isValveClMgrEnabled(void);
	void BOARD_InitDebugConsole(void);
	bool BOARD_Init(void);

#if defined (FTM0_USED) && (FTM0_USED != 0)
	void BOARD_FTM0_start(void);
	void BOARD_FTM0_stop(void);
	void BOARD_FTM0_enable_isr(void);
	void BOARD_FTM0_disable_isr(void);
#endif

#if defined (FTM1_USED) && (FTM1_USED != 0)
	void BOARD_FTM1_start(void);
	void BOARD_FTM1_stop(void);
	void BOARD_FTM1_enable_isr(void);
	void BOARD_FTM1_disable_isr(void);
#endif

#if defined (FTM2_USED) && (FTM2_USED != 0)
	void BOARD_FTM2_start(void);
	void BOARD_FTM2_stop(void);
	void BOARD_FTM2_enable_isr(void);
	void BOARD_FTM2_disable_isr(void);
#endif

#if defined (FTM3_USED) && (FTM3_USED != 0)
	void BOARD_FTM3_start(void);
	void BOARD_FTM3_stop(void);
	void BOARD_FTM3_enable_isr(void);
	void BOARD_FTM3_disable_isr(void);
#endif
	uint64_t Now(void);
	uint64_t DiffTime(unsigned tm);
	void SleepBM(unsigned tm);

	bool CheckVB1_Voltage(uint8_t *vb1_status);
	bool ControlRelay1(bool Status,uint32_t ChargeDel,bool ForceStat);
	bool ControlRelay2(bool Status,bool ForceStat);

	bool BOARD_RegisterPWM_FTM_Callback(void(*IRQHandler)(unsigned channel));
	bool BOARD_RegisterFTM_Callback(uint8_t channel,void(*IRQHandler)(uint64_t value,uint32_t flags));
	bool BOARD_GetStackAndHeapInfo(bool getZIregions,uint32_t *stack_base,uint32_t *stack_size,
			uint32_t *heap_base,uint32_t *heap_size);
	void BOARD_EnaADC_Timer(bool flag);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
