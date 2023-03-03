#include <stdint.h>
#include "board-DigIO.h"
#include "board.h"
#include "fsl_gpio.h"

extern void PORT_IRQHandler(PORT_Type *port,unsigned IRQmask);

#if TRACEALYZER != 0 && (TRC_DIG != 0 || TRC_DIG_ISR != 0)
#include "trcRecorder.h"
#endif

#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
#if defined(PORTA_USED) && (PORTA_USED != 0)
traceHandle PORTA_ISR_Handle;
#endif
#if defined(PORTB_USED) && (PORTB_USED != 0)
traceHandle PORTB_ISR_Handle;
#endif
#if defined(PORTC_USED) && (PORTC_USED != 0)
traceHandle PORTC_ISR_Handle;
#endif
#if defined(PORTD_USED) && (PORTD_USED != 0)
traceHandle PORTD_ISR_Handle;
#endif
#if defined(PORTE_USED) && (PORTE_USED != 0)
traceHandle PORTE_ISR_Handle;
#endif
#endif

static uint8_t mTestHallCounters		= 0;
static uint8_t mTestFlowCounter		= 0;
static uint32_t mHallCounters[2]		= {0,0};
static uint32_t mFlowCounter			= 0;
static uint8_t mIRQmask					= 0;

static structLED_t				gLED_BLINK[BOARD_N_LED] = 
										{
											{
												.LED = eLED0,
												.enabled = false,
												.IOport = GP_LED0,
												.period = 500,	
												.time = 0
											},
											{
												.LED = eLED1,
												.enabled = false,
												.IOport = GP_LED1,
												.period = 500,	
												.time = 0
											},
											{
												.LED = eLED2,
												.enabled = false,
												.IOport = GP_LED2,
												.period = 500,	
												.time = 0
											},
											{
												.LED = eCHARGE_LED_G,
												.enabled = false,
												.IOport = GP_CHARGE_LED_G,
												.period = 500,	
												.time = 0
											},
											{
												.LED = eCHARGE_LED_Y,
												.enabled = false,
												.IOport = GP_CHARGE_LED_Y,
												.period = 500,	
												.time = 0
											},
											{
												.LED = eCHARGE_LED_R,
												.enabled = false,
												.IOport = GP_CHARGE_LED_R,
												.period = 500,	
												.time = 0
											}
										};

static const uint8_t				gSecurityInputs[BOARD_N_SECURITY_IN] = 
										{
											GP_ANT_OK,
											GP_ARM_OK,
											GP_BUMPER0,
											GP_BUMPER1,
											GP_BUMPER2,
											GP_BUMPER3,
											GP_FLOOR0,
											GP_FLOOR1,
											GP_FLOOR2,
											GP_FLOOR3,
											GP_Recover_Safety,
											GP_SAFETY_ChkInLog,
											GP_Safety_Chk_Out_Ant,
											GP_T_24V_Safety,
											GP_Test_ANT_OK,
											GP_Test_ARM,
											GP_Test_Bumpers,
											GP_Test_EM_Stop,
											GP_Test_Rec_Safety,
											GP_WDOG_TRIGGER,
										};

static const uint8_t				gSecurityOutputs[BOARD_N_SECURITY_OUT] = 
										{
											GP_ARM_OK,
											GP_Recover_Safety,
											GP_SAFETY_ChkInLog,
											GP_WDOG_TRIGGER
										};

static const uint8_t				gMapInput[BOARD_N_GPIO_IN] =
										{	
											GP_BUMPER0,
											GP_BUMPER1,
											GP_BUMPER2,
											GP_BUMPER3,
											GP_FLOOR0,
											GP_FLOOR1,
											GP_FLOOR2,
											GP_FLOOR3,
											GP_BRUSH_nFAULT,
											GP_SUCT_nFAULT,
											GP_LIFT_BR_nFAULT,
											GP_LIFT_SUC_nFAULT,
											GP_PUMP1_nFAULT,
											GP_PUMP2_nFAULT,
											GP_PGOOD_3V3,
											GP_PGOOD_5V,
											GP_PGOOD_10V,
											GP_ANT_OK,
											GP_Safety_Chk_Out_Ant,
											GP_T_24V_Safety,
											GP_Test_ANT_OK,
											GP_Test_ARM,
											GP_Test_Bumpers,
											GP_Test_EM_Stop,
											GP_Test_Rec_Safety,
											GP_USB_V_Bus,
											GP_LIFT_BRUSH_HALL_IN,
											GP_LIFT_SUCT_HALL_IN,
											GP_END_SW1,
											GP_END_SW2,
											GP_END_SW3,
											GP_END_SW4,
											GP_FLOW_METER
										};

static const uint8_t				gMapOutput[BOARD_N_GPIO_OUT] =
										{	
											GP_LED0,
											GP_LED1,
											GP_LED2,
											GP_CHARGE_LED_G,
											GP_CHARGE_LED_Y,
											GP_CHARGE_LED_R,
											GP_ENA_5V,
											GP_ENA_10V,
											GP_FAN,
											GP_BRUSH_nSLEEP,
											GP_SUCT_nSLEEP,
											GP_LIFT_BR_nSLEEP,
											GP_LIFT_SUC_nSLEEP,
											GP_PUMP1_nSLEEP,
											GP_PUMP2_nSLEEP,
											GP_Recover_Safety,
											GP_SAFETY_ChkInLog,
											GP_ARM_OK,
											GP_TEST_MUX_A0,
											GP_TEST_MUX_A1,
											GP_TEST_MUX_A2,
											GP_TEST_MUX_EN,
											GP_WDOG_TRIGGER,
											GP_VALVE_DOSING_PUMP,
											GP_VB_ENABLE1,
											GP_VB_ENABLE2,
											GP_CHARGE_VB,
											GP_UNBLOCK_L_BR,
											GP_UNBLOCK_L_SUC
										};

static GPIO_Interrupt_t			gInputInterrupt[BOARD_N_GPIO_IN] = 
										{
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											},
											{
												.IntMode = kPORT_InterruptOrDMADisabled,
												.IntState = false
											}
										};
										
const strGPIOattrib_t sGPIOconfig[BOARD_N_GPIO] = {
   // PORT A
   // MCU-GPIO [0], PTA5, Output, Recover-Safety
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 5,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_Recover_Safety,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Recover-Safety"
   },
	// MCU-GPIO [1], PTA6, Input, END-SW1
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 6,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_END_SW1,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "EndSwitch 1"
   },
   // MCU-GPIO [2], PTA7, Input, END-SW2
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 7,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_END_SW2,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "EndSwitch 2"
   },
   // MCU-GPIO [3], PTA8, Output, Test-Mux-A1
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 8,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_TEST_MUX_A0,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-Mux-A1"
   },
   // MCU-GPIO [4], PTA9, Input, FlowMeter
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 9,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_FLOW_METER,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Flow Meter"
   },
#if defined CUC_HW_V2 || defined CUC_HW_HALL_IOS_V2
   // MCU-GPIO [5], PTC16, Output, Test-Mux-En
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 16,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 1,
		.HasInterrupt = false,
	   .IsInverted = true,
		.GPIO_index = GP_OUT_TEST_MUX_EN,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-Mux-En"
   },
   // MCU-GPIO [6], PTC17, Input, END-SW3
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 17,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_END_SW3,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "EndSwitch 3"
   },
#else
   // MCU-GPIO [5], PTA10, Output, Test-Mux-En
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 10,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 1,
		.HasInterrupt = false,
	   .IsInverted = true,
		.GPIO_index = GP_OUT_TEST_MUX_EN,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-Mux-En"
   },
   // MCU-GPIO [6], PTA11, Input, END-SW3
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 11,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_END_SW3,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "EndSwitch 3"
   },
#endif
   // MCU-GPIO [7], PTA14, Input, END-SW4
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 14,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_END_SW4,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "EndSwitch 4"
   },
   // MCU-GPIO [8], PTA15, Input, Test-Arm
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 15,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_Test_ARM,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-Arm"
   },
   // MCU-GPIO [9], PTA16, Output, VB-Enable1
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 16,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_VB_ENABLE1,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "VB Enable 1"
   },
   // MCU-GPIO [10], PTA17, Input, Test-EM-Stop
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 17,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = true,
		.GPIO_index = GP_IN_Test_EM_Stop,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-EM-Stop"
   },
   // MCU-GPIO [11], PTA24, Input, Test-Rec-Safety
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 24,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = true,
		.GPIO_index = GP_IN_Test_Rec_Safety,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-Rec-Safety"
   },
   // MCU-GPIO [12], PTA25, Input, Test-ANT-OK
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 25,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = true,
		.GPIO_index = GP_IN_Test_ANT_OK,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-ANT-OK"
   },
   // MCU-GPIO [13], PTA26, Input, Test-Bumpers
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 26,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = true,
		.GPIO_index = GP_IN_Test_Bumpers,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-Bumpers"
   },
   // MCU-GPIO [14], PTA27, Output, VB-Enable2
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 27,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_VB_ENABLE2,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "VB Enable 2"
   },
   // MCU-GPIO [15], PTA28, Output, LED2
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 28,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 1,
	   .IsInverted = true,
		.HasInterrupt = false,
		.GPIO_index = GP_OUT_LED2,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "LED2"
   },
   // MCU-GPIO [16], PTA29, Output, ENA-10V
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 29,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = false,
		.GPIO_index = GP_OUT_ENA_10V,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "ENA-10V"
   },


	// PORT B
   // MCU-GPIO [17], PTB4, Output, Safety-ChkInLog
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 4,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_SAFETY_ChkInLog,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Safety-ChkInLog"
   },
   // MCU-GPIO [18], PTB5, Input, PGOOD-5V
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 5,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_PGOOD_5V,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "PGOOD-5V"
   },
   // MCU-GPIO [19], PTB6, Output, Ena-5V
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 6,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_ENA_5V,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Ena-5V"
   },
   // MCU-GPIO [20], PTB7, Output, LED1
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 7,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 1,
	   .IsInverted = true,
		.HasInterrupt = false,
		.GPIO_index = GP_OUT_LED1,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "LED1"
   },
   // MCU-GPIO [21], PTB8, Output, LED0
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 8,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 1,
		.HasInterrupt = false,
	   .IsInverted = true,
		.GPIO_index = GP_OUT_LED0,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "LED0"
   },
   // MCU-GPIO [22], PTB9, Output, CHARGE-VB
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 9,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = false,
		.GPIO_index = GP_OUT_CHARGE_VB,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Charge VB"
   },
   // MCU-GPIO [23], PTB10, Output, WDOG-TRIG
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 10,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_WDOG_TRIGGER,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Watchdig Trig."
   },
   // MCU-GPIO [24], PTB18, Output, Charge-Led-R
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 18,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 1,
		.HasInterrupt = false,
	   .IsInverted = true,
		.GPIO_index = GP_OUT_CHARGE_LED_R,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Charge-Led-R"
   },
   // MCU-GPIO [25], PTB19, Output, Charge-Led-G
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 19,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 1,
		.HasInterrupt = false,
	   .IsInverted = true,
		.GPIO_index = GP_OUT_CHARGE_LED_G,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Charge-Led-G"
   },
   // MCU-GPIO [26], PTB20, Input, ANT-OK
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 20,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_ANT_OK,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "ANT-OK"
   },
   // MCU-GPIO [27], PTB22, Output, Charge-Led-Y
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 22,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 1,
		.HasInterrupt = false,
	   .IsInverted = true,
		.GPIO_index = GP_OUT_CHARGE_LED_Y,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Charge-Led-Y"
   },
   // MCU-GPIO [28], PTB23, Output, FAN
   {
      .Base = PORTB,
      .GPIO = GPIOB,
      .Offset = 23,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_FAN,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "FAN"
   },


	// PORT C
   // MCU-GPIO [29], PTC0, Input, Suct-nFault
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 0,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_SUCT_nFAULT,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Suct-nFault"
   },
   // MCU-GPIO [30], PTC6, Output, Pump2-nSleep
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 6,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_PUMP2_nSLEEP,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Pump2-nSleep"
   },
   // MCU-GPIO [31], PTC12, Input, Brush-nFault
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 12,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_BRUSH_nFAULT,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Brush-nFault"
   },
   // MCU-GPIO [32], PTC14, Output, UNBLOCK-L-BR
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 14,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_UNBLOCK_L_BR,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Unblock L BR"
   },
   // MCU-GPIO [33], PTC15, Output, Brush-nSleep
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 15,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_BRUSH_nSLEEP,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Brush-nSleep"
   },
#if defined CUC_HW_V2 || defined CUC_HW_HALL_IOS_V2
   // MCU-GPIO [34], PTA10, Input, Lift-Suct-Hall-In
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 10,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_LIFT_SUCT_HALL,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Lift-Suct-Hall-In"
   },
   // MCU-GPIO [35], PTA11, Input, Lift-Brush-Hall-In
   {
      .Base = PORTA,
      .GPIO = GPIOA,
      .Offset = 11,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_LIFT_BRUSH_HALL,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Lift-Brush-Hall-In"
   },
#else
   // MCU-GPIO [34], PTC16, Input, Lift-Suct-Hall-In
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 16,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_LIFT_SUCT_HALL,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Lift-Suct-Hall-In"
   },
   // MCU-GPIO [35], PTC17, Input, Lift-Brush-Hall-In
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 17,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_IN_LIFT_BRUSH_HALL,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Lift-Brush-Hall-In"
   },
#endif
   // MCU-GPIO [36], PTC18, Output, UNBLOCK-L-SUC
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 18,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_UNBLOCK_L_SUC,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Unblock L SUC"
   },
   // MCU-GPIO [37], PTC19, Input, Floor3
   {
      .Base = PORTC,
      .GPIO = GPIOC,
      .Offset = 19,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_FLOOR3,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Floor3"
   },


	// PORT D
   // MCU-GPIO [38], PTD0, Input, Floor1
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 0,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_FLOOR1,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Floor1"
   },
   // MCU-GPIO [39], PTD1, Input, Bumper1
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 1,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_BUMPER1,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Bumper1"
   },
   // MCU-GPIO [40], PTD2, Output, DOSING-PUMP
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 2,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_GP_VALVE_DOSING_PUMP,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Dosing Pump"
   },
	// MCU-GPIO [41], PTD3, Output, Lift-Suc-nSleep
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 3,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_LIFT_SUC_nSLEEP,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Lift-Suc-nSleep"
   },
   // MCU-GPIO [42], PTD4, Input, Bumper0
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 4,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_BUMPER0,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Bumper0"
   },
   // MCU-GPIO [43], PTD5, Input, Bumper2
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 5,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
 		.GPIO_index = GP_IN_BUMPER2,
		.Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Bumper2"
   },
   // MCU-GPIO [44], PTD6, Input, Bumper3
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 6,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_BUMPER3,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Bumper3"
   },
   // MCU-GPIO [45], PTD7, Input, Floor0
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 7,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = true,
		.GPIO_index = GP_IN_FLOOR0,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Floor0"
   },
   // MCU-GPIO [46], PTD8, Input, Pump2-nFault
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 8,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = true,
		.GPIO_index = GP_IN_PUMP2_nFAULT,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Pump2-nFault"
   },
   // MCU-GPIO [47], PTD9, Output, Suct-nSleep
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 9,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = false,
		.GPIO_index = GP_OUT_SUCT_nSLEEP,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Suct-nSleep"
   },
   // MCU-GPIO [48], PTD10, Output, Lift-Br-nSleep
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 10,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_LIFT_BR_nSLEEP,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Lift-Br-nSleep"
   },
   // MCU-GPIO [49], PTD11, Input, Pump1-nFault
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 11,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_PUMP1_nFAULT,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Pump1-nFault"
   },
   // MCU-GPIO [50], PTD12, Input, Lift-Br-nFault
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 12,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_LIFT_BR_nFAULT,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Lift-Br-nFault"
   },
   // MCU-GPIO [51], PTD13, Output, Pump1-nSleep
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 13,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_PUMP1_nSLEEP,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Pump1-nSleep"
   },
   // MCU-GPIO [52], PTD15, Input, Lift-Suc-nFault
   {
      .Base = PORTD,
      .GPIO = GPIOD,
      .Offset = 15,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_LIFT_SUC_nFAULT,
      .Config = {
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Lift-Suc-nFault"
   },


	// PORT E
   // MCU-GPIO [53], PTE0, Input, Floor2
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 0,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
	   .IsInverted = false,
		.HasInterrupt = true,
		.GPIO_index = GP_IN_FLOOR2,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Floor2"
   },
   // MCU-GPIO [54], PTE1, Input, PGOOD-10V
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 1,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_PGOOD_10V,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "PGOOD-10V"
   },
   // MCU-GPIO [55], PTE2, Input, PGOOD-3V3
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 2,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_PGOOD_3V3,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "PGOOD-3V3"
   },
   // MCU-GPIO [56], PTE4, Input, USB-V-Bus
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 4,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_USB_V_Bus,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "USB-V-Bus"
   },
   // MCU-GPIO [57], PTE24, Output, ARM-OK
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 24,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_ARM_OK,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "ARM-OK"
   },
   // MCU-GPIO [58], PTE25, Output, Test-Mux-A3
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 25,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_TEST_MUX_A2,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-Mux-A3"
   },
   // MCU-GPIO [59], PTE26, Output, Test-Mux-A2
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 26,
      .Direction = kGPIO_DigitalOutput,
      .InitialState = 0,
		.HasInterrupt = false,
	   .IsInverted = false,
		.GPIO_index = GP_OUT_TEST_MUX_A1,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Test-Mux-A2"
   },
   // MCU-GPIO [60], PTE27, Input, Safety-Chk-Out-Ant
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 27,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_Safety_Chk_Out_Ant,
      .Config =
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "Safety-Chk-Out-Ant"
   },
   // MCU-GPIO [61], PTE28, Input, T-24V-Safety
   {
      .Base = PORTE,
      .GPIO = GPIOE,
      .Offset = 28,
      .Direction = kGPIO_DigitalInput,
      .InitialState = 0,
		.HasInterrupt = true,
	   .IsInverted = false,
		.GPIO_index = GP_IN_T_24V_Safety,
      .Config = 
		{
			.driveStrength = kPORT_LowDriveStrength,
			.lockRegister = kPORT_UnlockRegister,
			.mux = kPORT_MuxAsGpio,
			.openDrainEnable = kPORT_OpenDrainDisable,
			.passiveFilterEnable = kPORT_PassiveFilterDisable,
			.pullSelect = kPORT_PullDisable,
			.slewRate = kPORT_FastSlewRate
      },
		.Name = "T-24V-Safety"
   }
};

static void InitPin(const strGPIOattrib_t *config)
{
gpio_pin_config_t    cfg;

   cfg.pinDirection = config->Direction;
   cfg.outputLogic = config->InitialState;
   GPIO_PinInit(config->GPIO, config->Offset, &cfg);
}

static void ConfigGPIO(int index)
{
	PORT_SetPinConfig(sGPIOconfig[index].Base, sGPIOconfig[index].Offset, &(sGPIOconfig[index].Config));
	InitPin(&(sGPIOconfig[index]));
}

const strGPIOattrib_t * BOARD_FindPin(PORT_Type *Port,unsigned Offset)
{
	if (Offset >= 32)
		return NULL;
	for(int i = 0;i < BOARD_N_GPIO;i++)
	{
		const strGPIOattrib_t * ptr = &(sGPIOconfig[gMapInput[i]]);
		if (ptr->Base == Port && ptr->Offset == Offset)
			return ptr;
	}
	return NULL;
}

const char * BOARD_GetGPIO_Name(int Pin)
{
	if (Pin >= BOARD_N_GPIO)
		return NULL;
	return sGPIOconfig[Pin].Name;
}

const strGPIOattrib_t * BOARD_GetPinStructPtr(unsigned Pin)
{
	if (Pin >= BOARD_N_GPIO)
		return NULL;
	return &(sGPIOconfig[Pin]);
}

bool BOARD_ConfigGPI_Interrupt(unsigned index,port_interrupt_t mode)
{
	if (index >= BOARD_N_GPIO_IN)
		return false;
	const strGPIOattrib_t *ptr = &(sGPIOconfig[gMapInput[index]]);
	PORT_SetPinInterruptConfig(ptr->Base,ptr->Offset,mode);
	gInputInterrupt[index].IntState = false;
	gInputInterrupt[index].IntMode = mode;
	return true;
}

bool BOARD_ClearGPI_Interrupt(unsigned index)
{
	if (index >= BOARD_N_GPIO_IN)
		return false;
	const strGPIOattrib_t *ptr = &(sGPIOconfig[gMapInput[index]]);
	PORT_ClearPinsInterruptFlags(ptr->Base,ptr->Offset);
	gInputInterrupt[index].IntState = false;
	return true;
}

void BOARD_setGPIO_PinAttributes(void)
{
 int      n;

	n = sizeof(sGPIOconfig) /  sizeof(strGPIOattrib_t);
	for (int i = 0;i < n;i++)
	{
	   PORT_SetPinConfig(sGPIOconfig[i].Base, sGPIOconfig[i].Offset, &(sGPIOconfig[i].Config));
	   InitPin(&(sGPIOconfig[i]));
	}
//	ConfigGPIO(GP_ENA_5V);
//	ConfigGPIO(GP_ENA_10V);
//	ConfigGPIO(GP_LED0);
//	ConfigGPIO(GP_LED1);
//	ConfigGPIO(GP_LED2);
//	ConfigGPIO(GP_CHARGE_LED_G);
//	ConfigGPIO(GP_CHARGE_LED_Y);
//	ConfigGPIO(GP_CHARGE_LED_R);
	
	PORT_SetPinMux(PORTB,17,kPORT_MuxAlt3);      // UART TX
	PORT_SetPinMux(PORTB,16,kPORT_MuxAlt3);      // UART RX
   PORT_SetPinMux(PORTA,12,kPORT_MuxAlt2);   	// CAN0 TX
   PORT_SetPinMux(PORTA,13,kPORT_MuxAlt2);   	// CAN0 RX
   PORT_SetPinMux(PORTC,10,kPORT_MuxAlt2);   	// I2C1 SCL
   PORT_SetPinMux(PORTC,11,kPORT_MuxAlt2);   	// I2C1 SDA

#if defined(USE_DRV8701P) && (USE_DRV8701P != 0)
   PORT_SetPinMux(PORTC,1,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
#else
   PORT_SetPinMux(PORTC,1,kPORT_MuxAlt4);   		// FTM0 CH0
#endif
   PORT_SetPinMux(PORTC,2,kPORT_MuxAlt4);   		// FTM0 CH1
#if defined(USE_DRV8701P) && (USE_DRV8701P != 0)
   PORT_SetPinMux(PORTC,3,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
#else
   PORT_SetPinMux(PORTC,3,kPORT_MuxAlt4);   		// FTM0 CH2
#endif
   PORT_SetPinMux(PORTC,4,kPORT_MuxAlt4);   		// FTM0 CH3
#if defined(USE_DRV8701P) && (USE_DRV8701P != 0)
   PORT_SetPinMux(PORTE,5,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
#else
   PORT_SetPinMux(PORTE,5,kPORT_MuxAlt6);   		// FTM3 CH0
#endif
   PORT_SetPinMux(PORTE,6,kPORT_MuxAlt6);   		// FTM3 CH1
#if defined(USE_DRV8701P) && (USE_DRV8701P != 0)
   PORT_SetPinMux(PORTE,7,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
#else
   PORT_SetPinMux(PORTE,7,kPORT_MuxAlt6);   		// FTM3 CH2
#endif
   PORT_SetPinMux(PORTE,8,kPORT_MuxAlt6);   		// FTM3 CH3
#if defined(USE_DRV8701P) && (USE_DRV8701P != 0)
   PORT_SetPinMux(PORTE,9,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
#else
   PORT_SetPinMux(PORTE,9,kPORT_MuxAlt6);   		// FTM3 CH4
#endif
   PORT_SetPinMux(PORTE,10,kPORT_MuxAlt6);   	// FTM3 CH5
#if defined(USE_DRV8701P) && (USE_DRV8701P != 0)
   PORT_SetPinMux(PORTE,11,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
#else
   PORT_SetPinMux(PORTE,11,kPORT_MuxAlt6);   	// FTM3 CH6
#endif
   PORT_SetPinMux(PORTE,12,kPORT_MuxAlt6);   	// FTM3 CH7

//   PORT_SetPinMux(PORTC,1,kPORT_MuxAlt4);   		// FTM0 CH0
//#ifdef USE_DRV8701P
//   PORT_SetPinMux(PORTC,2,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
//#else
//   PORT_SetPinMux(PORTC,2,kPORT_MuxAlt4);   		// FTM0 CH1
//#endif
//   PORT_SetPinMux(PORTC,3,kPORT_MuxAlt4);   		// FTM0 CH2
//#ifdef USE_DRV8701P
//   PORT_SetPinMux(PORTC,4,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
//#else
//   PORT_SetPinMux(PORTC,4,kPORT_MuxAlt4);   		// FTM0 CH3
//#endif
//   PORT_SetPinMux(PORTE,5,kPORT_MuxAlt6);   		// FTM3 CH0
//#ifdef USE_DRV8701P
//   PORT_SetPinMux(PORTE,6,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
//#else
//   PORT_SetPinMux(PORTE,6,kPORT_MuxAlt6);   		// FTM3 CH1
//#endif
//   PORT_SetPinMux(PORTE,7,kPORT_MuxAlt6);   		// FTM3 CH2
//#ifdef USE_DRV8701P
//   PORT_SetPinMux(PORTE,8,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
//#else
//   PORT_SetPinMux(PORTE,8,kPORT_MuxAlt6);   		// FTM3 CH3
//#endif
//   PORT_SetPinMux(PORTE,9,kPORT_MuxAlt6);   		// FTM3 CH4
//#ifdef USE_DRV8701P
//   PORT_SetPinMux(PORTE,10,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
//#else
//   PORT_SetPinMux(PORTE,10,kPORT_MuxAlt6);   	// FTM3 CH5
//#endif
//   PORT_SetPinMux(PORTE,11,kPORT_MuxAlt6);   	// FTM3 CH6
//#ifdef USE_DRV8701P
//   PORT_SetPinMux(PORTE,12,kPORT_MuxAsGpio);   	// GPIO, controls PWM direction
//#else
//   PORT_SetPinMux(PORTE,12,kPORT_MuxAlt6);   	// FTM3 CH7
//#endif

#if defined(USE_DRV8701P) && (USE_DRV8701P != 0)
gpio_pin_config_t    cfg;

   cfg.pinDirection = kGPIO_DigitalOutput;
   cfg.outputLogic = 0;
   GPIO_PinInit(GPIOC,1,&cfg);
   GPIO_PinInit(GPIOC,3,&cfg);
   GPIO_PinInit(GPIOE,5,&cfg);
   GPIO_PinInit(GPIOE,7,&cfg);
   GPIO_PinInit(GPIOE,9,&cfg);
   GPIO_PinInit(GPIOE,11,&cfg);

//gpio_pin_config_t    cfg;

//   cfg.pinDirection = kGPIO_DigitalOutput;
//   cfg.outputLogic = 0;
//   GPIO_PinInit(GPIOC,2,&cfg);
//   GPIO_PinInit(GPIOC,4,&cfg);
//   GPIO_PinInit(GPIOE,6,&cfg);
//   GPIO_PinInit(GPIOE,8,&cfg);
//   GPIO_PinInit(GPIOE,10,&cfg);
//   GPIO_PinInit(GPIOE,12,&cfg);
#endif
}

const strGPIOattrib_t * BOARD_getGPIOentry(unsigned index)
{
	if (index >= BOARD_N_GPIO)
		return NULL;
	else
		return &(sGPIOconfig[index]);
}

bool BOARD_WriteGPIOpin(unsigned index,uint8_t value)
{
	if (index >= BOARD_N_GPIO)
		return false;
	if (sGPIOconfig[index].Direction != kGPIO_DigitalOutput)
		return false;
	if (sGPIOconfig[index].IsInverted)
		value ^= 1;
	GPIO_PinWrite(sGPIOconfig[index].GPIO,sGPIOconfig[index].Offset,value & 0x01); 
	return true;
}
	
bool BOARD_ReadGPIOpin(unsigned index,uint8_t *value)
{
	if (index >= BOARD_N_GPIO)
		return false;
	*value = GPIO_PinRead(sGPIOconfig[index].GPIO,sGPIOconfig[index].Offset); 
	if (sGPIOconfig[index].IsInverted)
		*value ^= 1;
	return true;
}
	
bool BOARD_ToggleGPIOpin(unsigned index)
{
	if (index >= BOARD_N_GPIO)
		return false;
	if (sGPIOconfig[index].Direction != kGPIO_DigitalOutput)
		return false;
	GPIO_PortToggle(sGPIOconfig[index].GPIO,sGPIOconfig[index].Offset); 
	return true;
}
	
bool BOARD_SetGPIO_Output(unsigned index,uint8_t value)
{
unsigned		k;
	
	if (index >= BOARD_N_GPIO_OUT)
		return false;
	if (sGPIOconfig[index].IsInverted)
		value ^= 1;
	k = gMapOutput[index];
	GPIO_PinWrite(sGPIOconfig[k].GPIO,sGPIOconfig[k].Offset,value & 0x01); 
	return true;
}

bool BOARD_GetGPIO_Input(unsigned index,uint8_t *value)
{
unsigned		k;
	
	if (index >= BOARD_N_GPIO_IN)
		return false;
	k = gMapInput[index];
	*value = GPIO_PinRead(sGPIOconfig[k].GPIO,sGPIOconfig[k].Offset);
	if (sGPIOconfig[index].IsInverted)
		*value ^= 1;
	return true;
}

bool BOARD_GetAllGPIO_Inputs(unsigned index,uint32_t *value)
{
int			i;
unsigned		k;
	
	*value = 0;
	for (i = 0;i < BOARD_N_GPIO_IN;i++)
	{
		k = gMapInput[i];
		uint8_t result = GPIO_PinRead(sGPIOconfig[k].GPIO,sGPIOconfig[k].Offset);
		if (sGPIOconfig[index].IsInverted)
			result ^= 1;
		*value |= result << i;
	}
	return true;
}

bool BOARD_GetGPIO_Readback(unsigned index,uint8_t *value)
{
unsigned		k;
	
	if (index >= BOARD_N_GPIO_OUT)
		return false;
	k = gMapOutput[index];
	*value = GPIO_PinRead(sGPIOconfig[k].GPIO,sGPIOconfig[k].Offset);
	if (sGPIOconfig[index].IsInverted)
		*value ^= 1;
	return true;
}

bool BOARD_GetAllGPIO_Readbacks(unsigned index,uint32_t *value)
{
int			i;
unsigned		k;
	
	*value = 0;
	for (i = 0;i < BOARD_N_GPIO_OUT;i++)
	{
		k = gMapOutput[i];
		uint8_t result = GPIO_PinRead(sGPIOconfig[k].GPIO,sGPIOconfig[k].Offset);
		if (sGPIOconfig[index].IsInverted)
			result ^= 1;
		*value |= result << i;
	}
	return true;
}

bool BOARD_GetAllGPIOsignals(uint32_t *result,unsigned size)
{
int			i;
unsigned		k;
	
	if (size < 4)
		return false;
	for (i = 0;i < 4;i++)
		result[i] = 0;
	for (i = 0;i < BOARD_N_GPIO_IN;i++)
	{
		k = gMapInput[i];
		uint8_t value = GPIO_PinRead(sGPIOconfig[k].GPIO,sGPIOconfig[k].Offset);
		if (sGPIOconfig[k].IsInverted)
			value ^= 1;
		result[i / 32] |= value << (i % 32);
	}
	for (i = 0;i < BOARD_N_GPIO_OUT;i++)
	{
		k = gMapOutput[i];
		uint8_t value = GPIO_PinRead(sGPIOconfig[k].GPIO,sGPIOconfig[k].Offset);
		if (sGPIOconfig[k].IsInverted)
			value ^= 1;
		result[1 / 32 + 2] |= value << (i % 32);
	}
	return true;
}

bool BOARD_GetSecurityGPIO(unsigned channel,uint8_t *result)
{
const strGPIOattrib_t	*ptr;

	if (channel >= BOARD_N_SECURITY_IN)
		return false;
	ptr = &(sGPIOconfig[gSecurityInputs[channel]]);
	if (ptr->IsInverted)
		*result = GPIO_PinRead(ptr->GPIO,ptr->Offset) ^ 1;
	else
		*result = GPIO_PinRead(ptr->GPIO,ptr->Offset);
	return true;
}

bool BOARD_GetAllSecurityGPIOs(uint32_t *result,unsigned size)
{
const strGPIOattrib_t	*ptr;

	if (size < 2)
		return false;
	result[0] = 0;
	for (int i = 0;i < BOARD_N_SECURITY_IN && i < 32;i++)
	{
		ptr = &(sGPIOconfig[gSecurityInputs[i]]);
		if (ptr->IsInverted)
			result[0] |= (GPIO_PinRead(ptr->GPIO,ptr->Offset) ^ 1) << i; 
		else
			result[0] |= GPIO_PinRead(ptr->GPIO,ptr->Offset) << i; 
	}
	result[1] = 0;
	for (int i = 0;i < BOARD_N_SECURITY_OUT && i < 32;i++)
	{
		ptr = &(sGPIOconfig[gSecurityOutputs[i]]);
		if (ptr->IsInverted)
			result[1] |= (GPIO_PinRead(ptr->GPIO,ptr->Offset) ^ 1) << i; 
		else
			result[1] |= GPIO_PinRead(ptr->GPIO,ptr->Offset) << i; 
	}
	return true;
}

bool BOARD_SetSecurityOutput(unsigned channel,uint8_t value)
{
const strGPIOattrib_t	*ptr;
	
	if (channel >= BOARD_N_SECURITY_OUT)
		return false;
	ptr = &(sGPIOconfig[gSecurityOutputs[channel]]);
	if (ptr->IsInverted)
		GPIO_PinWrite(ptr->GPIO,ptr->Offset,(value & 0x01) ^ 1); 
	else
		GPIO_PinWrite(ptr->GPIO,ptr->Offset,value & 0x01); 
	return true;
}

bool BOARD_SetAllSecurityOutput(uint32_t value)
{
const strGPIOattrib_t	*ptr;
	
	for (int i = 0;i < BOARD_N_SECURITY_OUT;i++)
	{
		ptr = &(sGPIOconfig[gSecurityOutputs[i]]);
		if (ptr->IsInverted)
			GPIO_PinWrite(ptr->GPIO,ptr->Offset,((value >> i) & 0x01) ^ 1); 
		else
			GPIO_PinWrite(ptr->GPIO,ptr->Offset,(value >> i) & 0x01); 
	}
	return true;
}

bool BOARD_SetTestMUX(unsigned channel,bool enable)
{
	if (channel >= 8)
		return false;
	GPIO_PinWrite(sGPIOconfig[GP_TEST_MUX_A0].GPIO, sGPIOconfig[GP_TEST_MUX_A0].Offset,(channel >> 0) & 0x01);
	GPIO_PinWrite(sGPIOconfig[GP_TEST_MUX_A1].GPIO, sGPIOconfig[GP_TEST_MUX_A1].Offset,(channel >> 1) & 0x01);
	GPIO_PinWrite(sGPIOconfig[GP_TEST_MUX_A2].GPIO, sGPIOconfig[GP_TEST_MUX_A2].Offset,(channel >> 2) & 0x01);
	GPIO_PinWrite(sGPIOconfig[GP_TEST_MUX_EN].GPIO, sGPIOconfig[GP_TEST_MUX_EN].Offset,enable ? 0x00 : 0x01);
	return true;
}

bool BOARD_GetTestMUX(unsigned *channel,bool *enable)
{
	*channel = 0;
	*channel |= (((uint8_t)GPIO_PinRead(sGPIOconfig[GP_TEST_MUX_A0].GPIO, sGPIOconfig[GP_TEST_MUX_A0].Offset)) & 0x01) << 0;
	*channel |= (((uint8_t)GPIO_PinRead(sGPIOconfig[GP_TEST_MUX_A1].GPIO, sGPIOconfig[GP_TEST_MUX_A1].Offset)) & 0x01) << 1;
	*channel |= (((uint8_t)GPIO_PinRead(sGPIOconfig[GP_TEST_MUX_A2].GPIO, sGPIOconfig[GP_TEST_MUX_A2].Offset)) & 0x01) << 2;
	*enable = GPIO_PinRead(sGPIOconfig[GP_TEST_MUX_EN].GPIO, sGPIOconfig[GP_TEST_MUX_EN].Offset) == 0 ? true : false;
	return true;
}

bool BOARD_GetBumper(unsigned channel)
{
	switch (channel)
	{
		case 0:
			return GPIO_PinRead(sGPIOconfig[GP_BUMPER0].GPIO, sGPIOconfig[GP_BUMPER0].Offset) ? true : false;
		case 1:
			return GPIO_PinRead(sGPIOconfig[GP_BUMPER1].GPIO, sGPIOconfig[GP_BUMPER1].Offset) ? true : false;
		case 2:
			return GPIO_PinRead(sGPIOconfig[GP_BUMPER2].GPIO, sGPIOconfig[GP_BUMPER2].Offset) ? true : false;
		case 3:
			return GPIO_PinRead(sGPIOconfig[GP_BUMPER3].GPIO, sGPIOconfig[GP_BUMPER3].Offset) ? true : false;
		default:
			return true;	// a wrong channel will be reported as if the bumper is active
	}
}

uint8_t BOARD_GetAllBumpers(void)
{
uint8_t		result = 0;
	
	result |= (GPIO_PinRead(sGPIOconfig[GP_BUMPER0].GPIO, sGPIOconfig[GP_BUMPER0].Offset) << 0) & 0x01;
	result |= (GPIO_PinRead(sGPIOconfig[GP_BUMPER1].GPIO, sGPIOconfig[GP_BUMPER1].Offset) << 1) & 0x01;
	result |= (GPIO_PinRead(sGPIOconfig[GP_BUMPER2].GPIO, sGPIOconfig[GP_BUMPER2].Offset) << 2) & 0x01;
	result |= (GPIO_PinRead(sGPIOconfig[GP_BUMPER3].GPIO, sGPIOconfig[GP_BUMPER3].Offset) << 3) & 0x01;
	return result;
}

bool BOARD_GetFloorSensor(unsigned channel)
{
	switch (channel)
	{
		case 0:
			return GPIO_PinRead(sGPIOconfig[GP_FLOOR0].GPIO, sGPIOconfig[GP_FLOOR0].Offset) ? true : false;
		case 1:
			return GPIO_PinRead(sGPIOconfig[GP_FLOOR1].GPIO, sGPIOconfig[GP_FLOOR1].Offset) ? true : false;
		case 2:
			return GPIO_PinRead(sGPIOconfig[GP_FLOOR2].GPIO, sGPIOconfig[GP_FLOOR2].Offset) ? true : false;
		case 3:
			return GPIO_PinRead(sGPIOconfig[GP_FLOOR3].GPIO, sGPIOconfig[GP_FLOOR3].Offset) ? true : false;
		default:
			return true;	// a wrong channel will be reported as if the floor sensor is active
	}
}

uint8_t BOARD_GetAllFloorSensors(void)
{
uint8_t		result = 0;
	
	result |= (GPIO_PinRead(sGPIOconfig[GP_BUMPER0].GPIO, sGPIOconfig[GP_BUMPER0].Offset) << 0) & 0x01;
	result |= (GPIO_PinRead(sGPIOconfig[GP_BUMPER1].GPIO, sGPIOconfig[GP_BUMPER1].Offset) << 1) & 0x01;
	result |= (GPIO_PinRead(sGPIOconfig[GP_BUMPER2].GPIO, sGPIOconfig[GP_BUMPER2].Offset) << 2) & 0x01;
	result |= (GPIO_PinRead(sGPIOconfig[GP_BUMPER3].GPIO, sGPIOconfig[GP_BUMPER3].Offset) << 3) & 0x01;
	return result;
}

void BOARD_SetRecoverSafety(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_Recover_Safety].GPIO, sGPIOconfig[GP_Recover_Safety].Offset,value ? 1 : 0);	
}

bool BOARD_GetRecoverSafety(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_Recover_Safety].GPIO, sGPIOconfig[GP_Recover_Safety].Offset) ? true : false;
}

void BOARD_SetTestMuxA0(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_TEST_MUX_A0].GPIO, sGPIOconfig[GP_TEST_MUX_A0].Offset,value ? 1 : 0);	
}

bool BOARD_GetTestMuxA0(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_TEST_MUX_A0].GPIO, sGPIOconfig[GP_TEST_MUX_A0].Offset) ? true : false;
}

void BOARD_SetTestMuxEN(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_TEST_MUX_EN].GPIO, sGPIOconfig[GP_TEST_MUX_EN].Offset,value ? 1 : 0);	
}

bool BOARD_GetTestMuxEN(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_TEST_MUX_EN].GPIO, sGPIOconfig[GP_TEST_MUX_EN].Offset) ? true : false;
}

bool BOARD_GetTestARM(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_Test_ARM].GPIO, sGPIOconfig[GP_Test_ARM].Offset) ? true : false;
}

bool BOARD_GetTestEMstop(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_Test_EM_Stop].GPIO, sGPIOconfig[GP_Test_EM_Stop].Offset) ? true : false;
}

bool BOARD_GetTestRecSafety(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_Test_Rec_Safety].GPIO, sGPIOconfig[GP_Test_Rec_Safety].Offset) ? true : false;
}

void BOARD_GetTestANTok(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_Test_ANT_OK].GPIO, sGPIOconfig[GP_Test_ANT_OK].Offset,value ? 1 : 0);	
}

bool BOARD_GetTestBumpers(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_Test_Bumpers].GPIO, sGPIOconfig[GP_Test_Bumpers].Offset) ? true : false;
}

void BOARD_SetLED2(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_LED2].GPIO, sGPIOconfig[GP_LED2].Offset,value ? 1 : 0);	
}

bool BOARD_GetLED2(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_LED2].GPIO, sGPIOconfig[GP_LED2].Offset) ? true : false;
}

void BOARD_SetENA10V(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_ENA_10V].GPIO, sGPIOconfig[GP_ENA_10V].Offset,value ? 1 : 0);	
}

bool BOARD_GetENA10V(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_ENA_10V].GPIO, sGPIOconfig[GP_ENA_10V].Offset) ? true : false;
}

bool BOARD_GetPGOOD5V(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_PGOOD_5V].GPIO, sGPIOconfig[GP_PGOOD_5V].Offset) ? true : false;
}

void BOARD_SetLED0(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_LED0].GPIO, sGPIOconfig[GP_LED0].Offset,value ? 1 : 0);	
}

bool BOARD_GetLED0(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_LED0].GPIO, sGPIOconfig[GP_LED0].Offset) ? true : false;
}

void BOARD_SetSafetyChkInLog(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_SAFETY_ChkInLog].GPIO, sGPIOconfig[GP_SAFETY_ChkInLog].Offset,value ? 1 : 0);	
}

bool BOARD_GetSafetyChkInLog(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_SAFETY_ChkInLog].GPIO, sGPIOconfig[GP_SAFETY_ChkInLog].Offset) ? true : false;
}

void BOARD_SetENA5V(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_ENA_5V].GPIO, sGPIOconfig[GP_ENA_5V].Offset,value ? 1 : 0);	
}

bool BOARD_GetENA5V(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_ENA_5V].GPIO, sGPIOconfig[GP_ENA_5V].Offset) ? true : false;
}

void BOARD_SetLED1(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_LED1].GPIO, sGPIOconfig[GP_LED1].Offset,value ? 1 : 0);	
}

bool BOARD_GetLED1(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_LED1].GPIO, sGPIOconfig[GP_LED1].Offset) ? true : false;
}

void BOARD_SetChargeLED_R(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_CHARGE_LED_R].GPIO, sGPIOconfig[GP_CHARGE_LED_R].Offset,value ? 1 : 0);	
}

bool BOARD_GetChargeLED_R(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_CHARGE_LED_R].GPIO, sGPIOconfig[GP_CHARGE_LED_R].Offset) ? true : false;
}

void BOARD_SetChargeLED_G(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_CHARGE_LED_G].GPIO, sGPIOconfig[GP_CHARGE_LED_G].Offset,value ? 1 : 0);	
}

bool BOARD_GetChargeLED_G(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_CHARGE_LED_G].GPIO, sGPIOconfig[GP_CHARGE_LED_G].Offset) ? true : false;
}

void BOARD_SetChargeLED_Y(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_CHARGE_LED_Y].GPIO, sGPIOconfig[GP_CHARGE_LED_Y].Offset,value ? 1 : 0);	
}

bool BOARD_GetChargeLED_Y(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_CHARGE_LED_Y].GPIO, sGPIOconfig[GP_CHARGE_LED_Y].Offset) ? true : false;
}

bool BOARD_GetANTok(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_ANT_OK].GPIO, sGPIOconfig[GP_ANT_OK].Offset) ? true : false;
}

void BOARD_SetFAN(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_FAN].GPIO, sGPIOconfig[GP_FAN].Offset,value ? 1 : 0);	
}

bool BOARD_GetFAN(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_FAN].GPIO, sGPIOconfig[GP_FAN].Offset) ? true : false;
}

bool BOARD_GetSuct_nFAULT(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_SUCT_nFAULT].GPIO, sGPIOconfig[GP_SUCT_nFAULT].Offset) ? true : false;
}

void BOARD_SetPump1_nSLEEP(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_PUMP1_nSLEEP].GPIO, sGPIOconfig[GP_PUMP1_nSLEEP].Offset,value ? 1 : 0);	
}

bool BOARD_GetPump1_nSLEEP(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_PUMP1_nSLEEP].GPIO, sGPIOconfig[GP_PUMP1_nSLEEP].Offset) ? true : false;
}

void BOARD_SetPump2_nSLEEP(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_PUMP2_nSLEEP].GPIO, sGPIOconfig[GP_PUMP2_nSLEEP].Offset,value ? 1 : 0);	
}

bool BOARD_GetPump2_nSLEEP(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_PUMP2_nSLEEP].GPIO, sGPIOconfig[GP_PUMP2_nSLEEP].Offset) ? true : false;
}

void BOARD_SetBrush_nSLEEP(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_BRUSH_nSLEEP].GPIO, sGPIOconfig[GP_BRUSH_nSLEEP].Offset,value ? 1 : 0);	
}

bool BOARD_GetBrush_nSLEEP(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_BRUSH_nSLEEP].GPIO, sGPIOconfig[GP_BRUSH_nSLEEP].Offset) ? true : false;
}

bool BOARD_GetBrush_nFAULT(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_BRUSH_nFAULT].GPIO, sGPIOconfig[GP_BRUSH_nFAULT].Offset) ? true : false;
}

bool BOARD_GetFLOOR3(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_FLOOR3].GPIO, sGPIOconfig[GP_FLOOR3].Offset) ? true : false;
}

bool BOARD_GetFLOOR1(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_FLOOR1].GPIO, sGPIOconfig[GP_FLOOR1].Offset) ? true : false;
}

bool BOARD_GetBUMPER1(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_BUMPER1].GPIO, sGPIOconfig[GP_BUMPER1].Offset) ? true : false;
}

void BOARD_SetLiftSuct_nSLEEP(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_LIFT_SUC_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_SUC_nSLEEP].Offset,value ? 1 : 0);	
}

bool BOARD_GetLiftSuct_nSLEEP(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_LIFT_SUC_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_SUC_nSLEEP].Offset) ? true : false;
}

bool BOARD_GetBUMPER0(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_BUMPER0].GPIO, sGPIOconfig[GP_BUMPER0].Offset) ? true : false;
}

bool BOARD_GetBUMPER3(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_BUMPER3].GPIO, sGPIOconfig[GP_BUMPER3].Offset) ? true : false;
}

bool BOARD_GetBUMPER2(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_BUMPER2].GPIO, sGPIOconfig[GP_BUMPER2].Offset) ? true : false;
}

bool BOARD_GetFLOOR0(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_FLOOR0].GPIO, sGPIOconfig[GP_FLOOR0].Offset) ? true : false;
}

bool BOARD_GetPump2_nFAULT(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_PUMP2_nFAULT].GPIO, sGPIOconfig[GP_PUMP2_nFAULT].Offset) ? true : false;
}

void BOARD_SetSuct_nSLEEP(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_SUCT_nSLEEP].GPIO, sGPIOconfig[GP_SUCT_nSLEEP].Offset,value ? 1 : 0);	
}

bool BOARD_GetSuct_nSLEEP(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_SUCT_nSLEEP].GPIO, sGPIOconfig[GP_SUCT_nSLEEP].Offset) ? true : false;
}

void BOARD_SetLiftBR_nSLEEP(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_LIFT_BR_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_BR_nSLEEP].Offset,value ? 1 : 0);	
}

bool BOARD_GetLiftBR_nSLEEP(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_LIFT_BR_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_BR_nSLEEP].Offset) ? true : false;
}

bool BOARD_GetPump1_nFAULT(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_PUMP1_nFAULT].GPIO, sGPIOconfig[GP_PUMP1_nFAULT].Offset) ? true : false;
}

bool BOARD_GetLiftBR_nFAULT(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_LIFT_BR_nFAULT].GPIO, sGPIOconfig[GP_LIFT_BR_nFAULT].Offset) ? true : false;
}

bool BOARD_GetFLOOR2(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_FLOOR2].GPIO, sGPIOconfig[GP_FLOOR2].Offset) ? true : false;
}

bool BOARD_GetPGOOD10V(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_PGOOD_10V].GPIO, sGPIOconfig[GP_PGOOD_10V].Offset) ? true : false;
}

bool BOARD_GetPGOOD3V3(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_PGOOD_3V3].GPIO, sGPIOconfig[GP_PGOOD_3V3].Offset) ? true : false;
}

void BOARD_SetARMok(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_ARM_OK].GPIO, sGPIOconfig[GP_ARM_OK].Offset,value ? 1 : 0);	
}

bool BOARD_GetARMok(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_ARM_OK].GPIO, sGPIOconfig[GP_ARM_OK].Offset) ? true : false;
}

void BOARD_SetTestMuxA1(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_TEST_MUX_A1].GPIO, sGPIOconfig[GP_TEST_MUX_A1].Offset,value ? 1 : 0);	
}

bool BOARD_GetTestMuxA1(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_TEST_MUX_A1].GPIO, sGPIOconfig[GP_TEST_MUX_A1].Offset) ? true : false;
}

void BOARD_SetTestMuxA2(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_TEST_MUX_A2].GPIO, sGPIOconfig[GP_TEST_MUX_A2].Offset,value ? 1 : 0);	
}

bool BOARD_GetTestMuxA2(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_TEST_MUX_A2].GPIO, sGPIOconfig[GP_TEST_MUX_A2].Offset) ? true : false;
}

bool BOARD_GetSafetyChkOutANT(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_Safety_Chk_Out_Ant].GPIO, sGPIOconfig[GP_Safety_Chk_Out_Ant].Offset) ? true : false;
}

bool BOARD_GetT24_Safety(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_T_24V_Safety].GPIO, sGPIOconfig[GP_T_24V_Safety].Offset) ? true : false;
}

void BOARD_SetTriggerWatchdog(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_WDOG_TRIGGER].GPIO, sGPIOconfig[GP_WDOG_TRIGGER].Offset,value ? 1 : 0);
}

bool BOARD_GetTriggerWatchdog(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_WDOG_TRIGGER].GPIO, sGPIOconfig[GP_WDOG_TRIGGER].Offset) ? true : false;
}

void BOARD_SetVBenable1(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_VB_ENABLE1].GPIO, sGPIOconfig[GP_VB_ENABLE1].Offset,value ? 1 : 0);
}

bool BOARD_GetVBenable1(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_VB_ENABLE1].GPIO, sGPIOconfig[GP_VB_ENABLE1].Offset) ? true : false;
}

void BOARD_SetVBenable2(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_VB_ENABLE2].GPIO, sGPIOconfig[GP_VB_ENABLE2].Offset,value ? 1 : 0);
}

bool BOARD_GetVBenable2(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_VB_ENABLE2].GPIO, sGPIOconfig[GP_VB_ENABLE2].Offset) ? true : false;
}

void BOARD_SetChargeVB(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_CHARGE_VB].GPIO, sGPIOconfig[GP_CHARGE_VB].Offset,value ? 1 : 0);
}

bool BOARD_GetChargeVB(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_CHARGE_VB].GPIO, sGPIOconfig[GP_CHARGE_VB].Offset) ? true : false;
}

void BOARD_SetUnblockL_BR(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_UNBLOCK_L_BR].GPIO, sGPIOconfig[GP_UNBLOCK_L_BR].Offset,value ? 1 : 0);
}

bool BOARD_GetUnblockL_BR(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_UNBLOCK_L_BR].GPIO, sGPIOconfig[GP_UNBLOCK_L_BR].Offset) ? true : false;
}

void BOARD_SetUnblockL_SUC(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_UNBLOCK_L_SUC].GPIO, sGPIOconfig[GP_UNBLOCK_L_SUC].Offset,value ? 1 : 0);
}

bool BOARD_GetUnblockL_SUC(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_UNBLOCK_L_SUC].GPIO, sGPIOconfig[GP_UNBLOCK_L_SUC].Offset) ? true : false;
}

bool BOARD_GetEndSwitch1(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_IN_END_SW1].GPIO, sGPIOconfig[GP_IN_END_SW1].Offset) ? true : false;
}

bool BOARD_GetEndSwitch2(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_IN_END_SW2].GPIO, sGPIOconfig[GP_IN_END_SW2].Offset) ? true : false;
}

bool BOARD_GetEndSwitch3(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_IN_END_SW3].GPIO, sGPIOconfig[GP_IN_END_SW3].Offset) ? true : false;
}

bool BOARD_GetEndSwitch4(void)
{
	return GPIO_PinRead(sGPIOconfig[GP_IN_END_SW4].GPIO, sGPIOconfig[GP_IN_END_SW4].Offset) ? true : false;
}

uint8_t BOARD_GetEndSwitches(void)
{
uint8_t		ret;
	
	ret = GPIO_PinRead(sGPIOconfig[GP_IN_END_SW4].GPIO, sGPIOconfig[GP_IN_END_SW1].Offset) ? (1 << 0) : 0;
	ret += GPIO_PinRead(sGPIOconfig[GP_IN_END_SW4].GPIO, sGPIOconfig[GP_IN_END_SW2].Offset) ? (1 << 1) : 0;
	ret += GPIO_PinRead(sGPIOconfig[GP_IN_END_SW4].GPIO, sGPIOconfig[GP_IN_END_SW3].Offset) ? (1 << 2) : 0;
	ret += GPIO_PinRead(sGPIOconfig[GP_IN_END_SW4].GPIO, sGPIOconfig[GP_IN_END_SW4].Offset) ? (1 << 3) : 0;
	return ret;
}

void Board_SetAllPWMnSleep(bool value)
{
	GPIO_PinWrite(sGPIOconfig[GP_BRUSH_nSLEEP].GPIO, sGPIOconfig[GP_BRUSH_nSLEEP].Offset,value ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_SUCT_nSLEEP].GPIO, sGPIOconfig[GP_SUCT_nSLEEP].Offset,value ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_LIFT_BR_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_BR_nSLEEP].Offset,value ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_LIFT_SUC_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_SUC_nSLEEP].Offset,value ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_PUMP1_nSLEEP].GPIO, sGPIOconfig[GP_PUMP1_nSLEEP].Offset,value ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_PUMP2_nSLEEP].GPIO, sGPIOconfig[GP_PUMP2_nSLEEP].Offset,value ? 1 : 0);	
}

void Board_RestorePWMnSleep(uint8_t mask)
{
	GPIO_PinWrite(sGPIOconfig[GP_BRUSH_nSLEEP].GPIO, sGPIOconfig[GP_BRUSH_nSLEEP].Offset,(mask & (1 << 0)) != 0 ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_SUCT_nSLEEP].GPIO, sGPIOconfig[GP_SUCT_nSLEEP].Offset,(mask & (1 << 1)) != 0 ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_LIFT_BR_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_BR_nSLEEP].Offset,(mask & (1 << 2)) != 0 ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_LIFT_SUC_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_SUC_nSLEEP].Offset,(mask & (1 << 3)) != 0 ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_PUMP1_nSLEEP].GPIO, sGPIOconfig[GP_PUMP1_nSLEEP].Offset,(mask & (1 << 4)) != 0 ? 1 : 0);	
	GPIO_PinWrite(sGPIOconfig[GP_PUMP2_nSLEEP].GPIO, sGPIOconfig[GP_PUMP2_nSLEEP].Offset,(mask & (1 << 5)) != 0 ? 1 : 0);	
}

uint8_t Board_GetPWMnSleep(void)
{
uint8_t	value = 0;
	
	value |= GPIO_PinRead(sGPIOconfig[GP_BRUSH_nSLEEP].GPIO, sGPIOconfig[GP_BRUSH_nSLEEP].Offset) ? (1 << 0) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_SUCT_nSLEEP].GPIO, sGPIOconfig[GP_SUCT_nSLEEP].Offset) ? (1 << 1) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_LIFT_BR_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_BR_nSLEEP].Offset) ? (1 << 2) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_LIFT_SUC_nSLEEP].GPIO, sGPIOconfig[GP_LIFT_SUC_nSLEEP].Offset) ? (1 << 3) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_PUMP1_nSLEEP].GPIO, sGPIOconfig[GP_PUMP1_nSLEEP].Offset) ? (1 << 4) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_PUMP2_nSLEEP].GPIO, sGPIOconfig[GP_PUMP2_nSLEEP].Offset) ? (1 << 5) : 0;
	return value;
}

uint8_t Board_GetPWMnFault(void)
{
uint8_t	value = 0;
	
	value |= GPIO_PinRead(sGPIOconfig[GP_BRUSH_nFAULT].GPIO, sGPIOconfig[GP_BRUSH_nFAULT].Offset) ? (1 << 0) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_SUCT_nFAULT].GPIO, sGPIOconfig[GP_SUCT_nFAULT].Offset) ? (1 << 1) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_LIFT_BR_nFAULT].GPIO, sGPIOconfig[GP_LIFT_BR_nFAULT].Offset) ? (1 << 2) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_LIFT_SUC_nFAULT].GPIO, sGPIOconfig[GP_LIFT_SUC_nFAULT].Offset) ? (1 << 3) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_PUMP1_nFAULT].GPIO, sGPIOconfig[GP_PUMP1_nFAULT].Offset) ? (1 << 4) : 0;
	value |= GPIO_PinRead(sGPIOconfig[GP_PUMP2_nFAULT].GPIO, sGPIOconfig[GP_PUMP2_nFAULT].Offset) ? (1 << 5) : 0;
	return value;
}

void BOARD_Toggle_LED(void)
{
	for (int i = 0;i < BOARD_N_LED;i++)
	{
		if (gLED_BLINK[i].enabled)
		{
			if (gLED_BLINK[i].time >= gLED_BLINK[i].period)
			{
				GPIO_PortToggle(sGPIOconfig[gLED_BLINK[i].IOport].GPIO,1 << sGPIOconfig[gLED_BLINK[i].IOport].Offset);
				gLED_BLINK[i].time = 0;
			}
			else
			{
				gLED_BLINK[i].time++;
			}
		}
	}
}

bool BOARD_EnableLED_Blink(eLED_t led,bool enable,unsigned period)
{
	if ((unsigned)led >= BOARD_N_LED)
		return false;
	gLED_BLINK[(unsigned)led].enabled = enable;
	gLED_BLINK[(unsigned)led].period = period / PIT1_PERIOD / 1;
	return true;
}

bool BOARD_Enable_5V(void)
{
	GPIO_PinWrite(sGPIOconfig[GP_ENA_5V].GPIO, sGPIOconfig[GP_ENA_5V].Offset,1);
	return true;
}

bool BOARD_Enable_10V(void)
{
	GPIO_PinWrite(sGPIOconfig[GP_ENA_10V].GPIO, sGPIOconfig[GP_ENA_10V].Offset,1);
	return true;
}

bool BOARD_Disable_5V(void)
{
	GPIO_PinWrite(sGPIOconfig[GP_ENA_5V].GPIO, sGPIOconfig[GP_ENA_5V].Offset,0);
	return true;
}

bool BOARD_Disable_10V(void)
{
	GPIO_PinWrite(sGPIOconfig[GP_ENA_10V].GPIO, sGPIOconfig[GP_ENA_10V].Offset,0);
	return true;
}


bool BOARD_RegisterGPIO_Callback(int port,int pin,void (*callback)(int))
{
	return true;
}

bool BOARD_Enable_Port_IRQ(PORT_Type *port)
{
	if (port == PORTA)
		NVIC_EnableIRQ(PORTA_IRQn);
	else if (port == PORTB)
		NVIC_EnableIRQ(PORTB_IRQn);
	else if (port == PORTC)
		NVIC_EnableIRQ(PORTC_IRQn);
	else if (port == PORTD)
		NVIC_EnableIRQ(PORTD_IRQn);
	else if (port == PORTE)
		NVIC_EnableIRQ(PORTE_IRQn);
	else
		return false;
	return true;
}
	
bool BOARD_Disable_Port_IRQ(PORT_Type *port)
{
	if (port == PORTA)
		NVIC_DisableIRQ(PORTA_IRQn);
	else if (port == PORTB)
		NVIC_DisableIRQ(PORTB_IRQn);
	else if (port == PORTC)
		NVIC_DisableIRQ(PORTC_IRQn);
	else if (port == PORTD)
		NVIC_DisableIRQ(PORTD_IRQn);
	else if (port == PORTE)
		NVIC_DisableIRQ(PORTE_IRQn);
	else
		return false;
	return true;
}

bool BOARD_EnaHallTestCounters(uint8_t status,uint8_t IRQmask)
{
	mTestHallCounters = status & 0x03;
	mIRQmask &= ~(3 << 0);
	mIRQmask |= IRQmask & (3 << 0);
	if ((IRQmask & (3 << 0)) != 0)
#if defined CUC_HW_V2 || defined CUC_HW_HALL_IOS_V2
		BOARD_Enable_Port_IRQ(PORTA);
#else
		BOARD_Enable_Port_IRQ(PORTC);
#endif
	if ((IRQmask & (1 << 0)) != 0)
	{
		if (!BOARD_ConfigGPI_Interrupt(GP_IN_LIFT_BRUSH_HALL,kPORT_InterruptRisingEdge))
			return false;
	}
	if ((IRQmask & (1 << 1)) != 0)
	{
		if (!BOARD_ConfigGPI_Interrupt(GP_IN_LIFT_SUCT_HALL,kPORT_InterruptRisingEdge))
			return false;
	}
	return true;
}
	
bool BOARD_EnaFlowMeterTestCounter(uint8_t status,uint8_t IRQmask)
{
	mTestFlowCounter = status & 0x01;
	mIRQmask &= ~(1 << 2);
	mIRQmask |= (IRQmask & 1) << 2;
	if ((IRQmask & (1 << 0)) != 0)
	{
		BOARD_Enable_Port_IRQ(PORTA);
		if (!BOARD_ConfigGPI_Interrupt(GP_IN_FLOW_METER,kPORT_InterruptRisingEdge))
			return false;
	}
	return true;
}
	
bool BOARD_ResetHallTestCounters(uint8_t mask)
{
	if ((mask & (1 << 0)) != 0)
		mHallCounters[0] = 0;
	if ((mask & (1 << 1)) != 0)
		mHallCounters[1] = 0;
	return true;
}

bool BOARD_ResetFlowMeterTestCounter(void)
{
	mFlowCounter = 0;
	return true;
}

bool BOARD_GetHallTestCounters(uint32_t *Cntr_Hall_1,uint32_t *Cntr_Hall_2)
{
	*Cntr_Hall_1 = mHallCounters[0];
	*Cntr_Hall_2 = mHallCounters[1];
	return true;
}

bool BOARD_GetFlowMeterTestCounters(uint32_t *Cntr_FlowMeter)
{
	*Cntr_FlowMeter = mFlowCounter;
	return true;
}

bool BOARD_GetHallTestStatus(uint8_t *status)
{
	*status = mTestHallCounters & 0x03;
	*status += (mIRQmask & 0x03) << 2;
	return true;
}

bool BOARD_GetFlowMeterTestStatus(uint8_t *status)
{
	*status = 	mTestFlowCounter & 0x01;
	*status += ((mIRQmask >> 2) & 0x01) << 2;
	return true;
}

static inline void BOARD_CountTestCounters(uint32_t IRQmask)
{
	if ((IRQmask & (1 << GP_LIFT_SUCT_BR_INDEX)) != 0 &&
			(mTestHallCounters & (1 << 0)) != 0)
		mHallCounters[0]++;
	if ((IRQmask & (1 << GP_LIFT_SUCT_SUC_INDEX)) != 0 &&
			(mTestHallCounters & (1 << 1)) != 0)
		mHallCounters[1]++;
}

static inline void BOARD_CountFlowMeterTestCounter(uint32_t IRQmask)
{
	if ((IRQmask & (1 << GP_FLOW_METER_INDEX)) != 0 &&
			(mTestFlowCounter & (1 << 0)) != 0)
		mFlowCounter++;
}

void PORTA_IRQHandler(void)
{
uint32_t IRQmask = PORTA->ISFR;

#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISRBegin(PORTA_ISR_Handle); 
#endif
	PORTA->ISFR = IRQmask;	// Resets the Interrupt
	if (mTestFlowCounter)
		BOARD_CountFlowMeterTestCounter(IRQmask);
	else
		if (!mTestHallCounters)
			PORT_IRQHandler(PORTA,IRQmask);
#if defined CUC_HW_V2 || defined CUC_HW_HALL_IOS_V2
	if (mTestHallCounters)
		BOARD_CountTestCounters(IRQmask);
	else
		if (!mTestFlowCounter)
			PORT_IRQHandler(PORTA,IRQmask);
#endif
#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void PORTB_IRQHandler(void)
{
uint32_t IRQmask = PORTB->ISFR;

#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISRBegin(PORTB_ISR_Handle); 
#endif
	PORTB->ISFR = IRQmask;	// Resets the Interrupt
	PORT_IRQHandler(PORTB,IRQmask);
#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void PORTC_IRQHandler(void)
{
uint32_t IRQmask = PORTC->ISFR;

#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISRBegin(PORTC_ISR_Handle); 
#endif
	PORTC->ISFR = IRQmask;	// Resets the Interrupt
#if !defined CUC_HW_V2 && !defined CUC_HW_HALL_IOS_V2
	if (mTestHallCounters)
		BOARD_CountTestCounters(IRQmask);
	else
		PORT_IRQHandler(PORTC,IRQmask);
#endif
#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void PORTD_IRQHandler(void)
{
uint32_t IRQmask = PORTD->ISFR;

#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISRBegin(PORTD_ISR_Handle); 
#endif
	PORTD->ISFR = IRQmask;	// Resets the Interrupt
	PORT_IRQHandler(PORTD,IRQmask);
#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void PORTE_IRQHandler(void)
{
uint32_t IRQmask = PORTE->ISFR;

#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISRBegin(PORTE_ISR_Handle); 
#endif
	PORTE->ISFR = IRQmask;	// Resets the Interrupt
	PORT_IRQHandler(PORTE,IRQmask);
#if TRACEALYZER != 0 && TRC_DIG_ISR != 0
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

static bool BOARD_InitTracealyzer(void)
{
#if TRACEALYZER != 0 && TRC_DIG != 0
#if defined(PORTA_USED) && (PORTA_USED != 0)
	PORTA_ISR_Handle = xTraceSetISRProperties("PORTA ISR", PORTA_INT_PRIORITY);
#endif
#if defined(PORTB_USED) && (PORTB_USED != 0)
	PORTB_ISR_Handle = xTraceSetISRProperties("PORTB ISR", PORTB_INT_PRIORITY);
#endif
#if defined(PORTC_USED) && (PORTC_USED != 0)
	PORTC_ISR_Handle = xTraceSetISRProperties("PORTC ISR", PORTC_INT_PRIORITY);
#endif
#if defined(PORTD_USED) && (PORTD_USED != 0)
	PORTD_ISR_Handle = xTraceSetISRProperties("PORTD ISR", PORTD_INT_PRIORITY);
#endif
#if defined(PORTE_USED) && (PORTE_USED != 0)
	PORTE_ISR_Handle = xTraceSetISRProperties("PORTE ISR", PORTE_INT_PRIORITY);
#endif
#endif
	return true;
}

bool BOARD_InitDigIO(void)
{
	BOARD_InitTracealyzer();
	return true;
}
