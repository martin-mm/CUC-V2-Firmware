#include <stdint.h>
#include "board.h"
#include "board-DigIO.h"
#include "board-Ana.h"
#include "uart.h"
#include "CAN.h"
#include "I2C.h"
#include "EEPROM.h"
#include "crc.h"
#include "fsl_wdog.h"

#if (TRACEALYZER != 0) && ((TRC_BOARD != 0) || (TRC_BOARD_ISR != 0))
#include "trcRecorder.h"
#endif

#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
#if defined(FTM0_USED) && (FTM0_USED != 0)
traceHandle FTM0_ISR_Handle;
#endif
#if defined(FTM1_USED) && (FTM1_USED != 0)
traceHandle FTM1_ISR_Handle;
#endif
#if defined(FTM2_USED) && (FTM2_USED != 0)
traceHandle FTM2_ISR_Handle;
#endif
#if defined(FTM3_USED) && (FTM3_USED != 0)
traceHandle FTM3_ISR_Handle;
#endif
#if defined(PDB_USED) && (PDB_USED != 0)
traceHandle PDB_ISR_Handle;
#endif
#if defined(PIT0_USED) && (PIT0_USED != 0)
traceHandle PIT0_ISR_Handle;
#endif
#if defined(PIT1_USED) && (PIT1_USED != 0)
traceHandle PIT1_ISR_Handle;
#endif
#if defined(PIT2_USED) && (PIT2_USED != 0)
traceHandle PIT2_ISR_Handle ;
#endif
#if defined(CAN_USED) && (CAN_USED != 0)
traceHandle CAN_ISR_Handle;
traceHandle CAN_ERR_ISR_Handle;
#endif
#endif

#if (TRACEALYZER != 0) && (TRC_BOARD != 0)
static traceString 		trcBoard;
#endif

static void (*PWM_IRQHandler)(unsigned channel) = NULL;
static void (*TIMER0_IRQHandler)(uint64_t value,uint32_t flags) = NULL;
static void (*TIMER1_IRQHandler)(uint64_t value,uint32_t flags) = NULL;

static volatile uint64_t		JiffyCntr = 0;
static volatile SystemTime_t	mSystemTime = {
	.ticks_ms = 0,
	.ticks_timer = 0
};
static volatile uint32_t		FTM1_counter = 0;
static volatile uint32_t		FTM2_counter = 0;
static volatile bool				Disable_CUC_IRQ_Callback = false;

extern uint32_t 					Image$$ARM_LIB_HEAP$$Base;
extern uint32_t 					Image$$ARM_LIB_HEAP$$Length;
extern uint32_t 					Image$$ARM_LIB_STACK$$Base;
extern uint32_t 					Image$$ARM_LIB_STACK$$Length;

extern uint32_t 					Image$$ARM_LIB_HEAP$$ZI$$Base;
extern uint32_t 					Image$$ARM_LIB_HEAP$$ZI$$Length;
extern uint32_t 					Image$$ARM_LIB_STACK$$ZI$$Base;
extern uint32_t 					Image$$ARM_LIB_STACK$$ZI$$Length;

static uint16_t            	DeviceOwnAddress = 0;
static volatile bool				ADCtrigger = false;
static bool							gTestWD = false;
static bool							RelayStatus1 = false,RelayStatus2 = false;
static bool							OldRelay1Status = false,OldRelay2Status = false;
static bool							ForceStatus1 = false;
static bool							ForceStatus2 = false;
static bool							CheckVB1 = true;
static uint16_t					ChargeDelay = 2000 / 50;
static uint8_t						RelayFlags = 0;

static bool							m_EnaADC_Timer = false;

static volatile SystemTime_t	SystemTime = {
	.ticks_ms = 0,
	.ticks_timer = 0
};

static const PWM_FTMtimer_t		FTMtimer[N_PWM_FTM_TIMER_CHANNELS] =
{
	// Channel 0
	{
		.Timer = FTM0,
		.fTimer = PWM_FREQUENCY_FTM0_HZ,
		.divider = PWM_DIVIDER_FTM0,
		.nPWMchannels = N_PWM_CHANNELS_FTM0,
		.indexPWMchan = 0
	},
	// Channel 1
	{
		.Timer = FTM3,
		.fTimer = PWM_FREQUENCY_FTM3_HZ,
		.divider = PWM_DIVIDER_FTM3,
		.nPWMchannels = N_PWM_CHANNELS_FTM3,
		.indexPWMchan = 4
	},
	// Channel 2
	{
		.Timer = FTM1,
		.fTimer = PWM_FREQUENCY_FTM1_HZ,
		.divider = PWM_DIVIDER_FTM1,
		.nPWMchannels = N_PWM_CHANNELS_FTM1,
		.indexPWMchan = 4
	}
};

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)

static PWMchannel_t				gPWMchannel[N_PWM_CHANNELS] =
{
	// Channel 0 - BRUSH
	{
		.TimerCH = &(FTMtimer[0]),
		.Channel = 1,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 0,
		.Polarity = kFTM_LowTrue
	},
	// Channel 2 - SUCT
	{
		.TimerCH = &(FTMtimer[0]),
		.Channel = 3,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 0,
		.Polarity = kFTM_LowTrue
	},
	// Channel 4 - LIFT-BR
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 1,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 6 - LIFT-SUCT
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 3,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 8 - PUMP1
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 5,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_HighTrue
	},
	// Channel 10 - PUMP2
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 7,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_HighTrue
	}
};

#else

static PWMchannel_t				gPWMchannel[N_PWM_CHANNELS] =
{
	// Channel 0 - BRUSH 1
	{
		.TimerCH = &(FTMtimer[0]),
		.Channel = 0,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 0,
		.Polarity = kFTM_LowTrue
	},
	// Channel 1 - BRUSH 2
	{
		.TimerCH = &(FTMtimer[0]),
		.Channel = 1,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 0,
		.Polarity = kFTM_LowTrue
	},
	// Channel 2 - SUCT 1
	{
		.TimerCH = &(FTMtimer[0]),
		.Channel = 2,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 0,
		.Polarity = kFTM_LowTrue
	},
	// Channel 3 - SUCT 2
	{
		.TimerCH = &(FTMtimer[0]),
		.Channel = 3,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 0,
		.Polarity = kFTM_LowTrue
	},
	// Channel 4 - LIFT-BR 1
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 0,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 5 - LIFT-BR 2
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 1,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 6 - LIFT-SUCT 1
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 2,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 7 - LIFT-SUCT 2
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 3,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 8 - PUMP1 1
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 4,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 9 - PUMP1 2
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 5,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 10 - PUMP2 1
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 6,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	},
	// Channel 11 - PUMP2 2
	{
		.TimerCH = &(FTMtimer[1]),
		.Channel = 7,
		.TimerPeriod = 0,
		.ActualPWM = 0,
		.InitialVal = 1,
		.TimerIndex = 1,
		.Polarity = kFTM_LowTrue
	}
};

#endif

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)

static PWM_Control_t		gPWM_Control[N_PWM_CONTROL_CHANNELS] =
{
	// BRUSH
	{
		.PWMchannel = 	&(gPWMchannel[0]),
		.DirectionPORT = PORTC,
		.DirectionGPIO = GPIOC,
		.DirectionPIN = 1,
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_BRUSH_nSLEEP,
		.FaultGPIO = GP_BRUSH_nFAULT,
		.TimerIndex = 0,
		.Name ="BRUSH"
	},
	// SUCT
	{
		.PWMchannel = &(gPWMchannel[1]),
		.DirectionPORT = PORTC,
		.DirectionGPIO = GPIOC,
		.DirectionPIN = 3,
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_SUCT_nSLEEP,
		.FaultGPIO = GP_SUCT_nFAULT,
		.TimerIndex = 0,
		.Name ="SUCTION"
	},
	// LIFT-BR
	{
		.PWMchannel = &(gPWMchannel[2]),
		.DirectionPORT = PORTE,
		.DirectionGPIO = GPIOE,
		.DirectionPIN = 5,
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_LIFT_BR_nSLEEP,
		.FaultGPIO = GP_LIFT_BR_nFAULT,
		.TimerIndex = 1,
		.Name ="LIFT-BRUSH"
	},
	// LIFT-SUC
	{
		.PWMchannel = &(gPWMchannel[3]),
		.DirectionPORT = PORTE,
		.DirectionGPIO = GPIOE,
		.DirectionPIN = 7,
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_LIFT_SUC_nSLEEP,
		.FaultGPIO = GP_LIFT_SUC_nFAULT,
		.TimerIndex = 1,
		.Name ="LIFT-SUCT"
	},
	// PUMP1
	{
		.PWMchannel = &(gPWMchannel[4]),
		.DirectionPORT = PORTE,
		.DirectionGPIO = GPIOE,
		.DirectionPIN = 9,
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_PUMP1_nSLEEP,
		.FaultGPIO = GP_PUMP1_nFAULT,
		.TimerIndex = 1,
		.Name ="PUMP1"
	},
	// PUMP2
	{
		.PWMchannel = &(gPWMchannel[5]),
		.DirectionPORT = PORTE,
		.DirectionGPIO = GPIOE,
		.DirectionPIN = 11,
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_PUMP2_nSLEEP,
		.FaultGPIO = GP_PUMP2_nFAULT,
		.TimerIndex = 1,
		.Name ="PUMP2"
	},
};

#else

static PWM_Control_t		gPWM_Control[N_PWM_CONTROL_CHANNELS] =
{
	// BRUSH
	{
		.PWMchannel1 = &(gPWMchannel[0]),
		.PWMchannel2 = &(gPWMchannel[1]),
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_BRUSH_nSLEEP,
		.FaultGPIO = GP_BRUSH_nFAULT,
		.TimerIndex = 0,
		.Name ="BRUSH"
	},
	// SUCT
	{
		.PWMchannel1 = &(gPWMchannel[2]),
		.PWMchannel2 = &(gPWMchannel[3]),
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_SUCT_nSLEEP,
		.FaultGPIO = GP_SUCT_nFAULT,
		.TimerIndex = 0,
		.Name ="SUCTION"
	},
	// LIFT-BR
	{
		.PWMchannel1 = &(gPWMchannel[4]),
		.PWMchannel2 = &(gPWMchannel[5]),
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_LIFT_BR_nSLEEP,
		.FaultGPIO = GP_LIFT_BR_nFAULT,
		.TimerIndex = 1,
		.Name ="LIFT-BRUSH"
	},
	// LIFT-SUC
	{
		.PWMchannel1 = &(gPWMchannel[6]),
		.PWMchannel2 = &(gPWMchannel[7]),
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_LIFT_SUC_nSLEEP,
		.FaultGPIO = GP_LIFT_SUC_nFAULT,
		.TimerIndex = 1,
		.Name ="LIFT-SUCT"
	},
	// PUMP1
	{
		.PWMchannel1 = &(gPWMchannel[8]),
		.PWMchannel2 = &(gPWMchannel[9]),
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_PUMP1_nSLEEP,
		.FaultGPIO = GP_PUMP1_nFAULT,
		.TimerIndex = 1,
		.Name ="PUMP1"
	},
	// PUMP2
	{
		.PWMchannel1 = &(gPWMchannel[10]),
		.PWMchannel2 = &(gPWMchannel[11]),
		.DirMode = eDirModeSleep,
		.pwm = 0,
		.SleepGPIO = GP_PUMP2_nSLEEP,
		.FaultGPIO = GP_PUMP2_nFAULT,
		.TimerIndex = 1,
		.Name ="PUMP2"
	},
};

#endif

static const pit_config_t	pitConfig =
{
	.enableRunInDebug = false
};

static const i2c_master_config_t i2c_master_config[2] =
{
	{
		.enableMaster = true,			   // enables master configuration
		.baudRate_Bps = I2C0_BAUD_RATE,	// baudrate in bps
		.glitchFilterWidth = 0			   // glitch filter value
	},
	{
		.enableMaster = true,			   // enables master configuration
		.baudRate_Bps = I2C1_BAUD_RATE,	// baudrate in bps
		.glitchFilterWidth = 0			   // glitch filter value
	},
};

static const wdog_config_t	wdog_config = 
{
	.enableWdog = false,
	.clockSource = kWDOG_LpoClockSource,
	.prescaler = kWDOG_ClockPrescalerDivide1,
	.workMode = {
		.enableWait = true,
		.enableStop = false,
		.enableDebug = false	
	},
	.enableUpdate = true,
	.enableInterrupt = false,
	.enableWindowMode = false,
	.windowValue = 0,
	.timeoutValue = 1000
};

#if defined (PDB_USED ) && (PDB_USED != 0)
static const pdb_config_t	pdb_config =
{
	.loadValueMode = kPDB_LoadValueImmediately,
	.prescalerDivider = kPDB_PrescalerDivider128,
	.dividerMultiplicationFactor = kPDB_DividerMultiplicationFactor10,
	.triggerInputSource = kPDB_TriggerSoftware,
	.enableContinuousMode = false
};	

static PDB_PumpCtrl_Config_t PDB_PumpCtrl_Config =
{
	.frequency = 48000000,
	.prescaler = 128,
	.multiplier = 10
};

static int	PDB_CalcFactor = 1;
#endif

static PumpCtrl_t		PumpCtrl[2] =
{
	{
		.frequency = 1,
		.pulse = 100,
		.enable = false,
		.period = 1000,
		.period_ctr = 0,
		.pwm_on = 1000
	},
	{
		.frequency = 1,
		.pulse = 100,
		.enable = false,
		.period = 1000,
		.period_ctr = 0,
		.pwm_on = 1000
	}
};

static PumpCtrlClMgr_t		PumpClMgrCtrl =
{
	.active_dur_p1 = 1000,
	.active_dur_p2 = 1000,
	.pulse_dur_p1 = 0,
	.pulse_dur_p2 = 0,
	.deactive_dur_p1 = 1000,
	.deactive_dur_p2 = 1000,
	.period = 10000,
	.wait = 0,
	.period_ctr = 0,
	.PWM_on = 0,
	.PumpCtrlByClMgr = true,	
	.ValveCtrlByClMgr = true	
};

#if USE_FLOAT != 0
int BOARD_FTM_CalcPrescaler(float TimerFrequency,float ClockSource)
{
int	k = 0;
	
	if (TimerFrequency == 0)
		return -1;
	while (((uint32_t)(ClockSource / (TimerFrequency * (1 << k) + 0.5F))) >= 65536)
		k++;
	if ((1 << k) > 128)
		return -1;
	else
		return k;
}
#else
int BOARD_FTM_CalcPrescaler(uint32_t TimerFrequency,uint32_t ClockSource)
{
int	k = 0;
	
	if (TimerFrequency == 0)
		return -1;
	while (ClockSource / ((uint64_t)TimerFrequency * (1 << k)) >= 65536)
		k++;
	if ((1 << k) > 128)
		return -1;
	else
		return k;
}
#endif

bool BOARD_InitFTM0(void)
{
ftm_config_t 						ftmInfo;
ftm_chnl_pwm_signal_param_t 	ftmParam[4];
// int									prescale;

	FTM_GetDefaultConfig(&ftmInfo);
//   ftmInfo.pwmSyncMode = FTM_SYNC_SWSYNC_MASK;
   ftmInfo.bdmMode = kFTM_BdmMode_3;

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
   ftmParam[0].chnlNumber = kFTM_Chnl_0;
	ftmParam[0].level = kFTM_NoPwmSignal;
	ftmParam[0].dutyCyclePercent = 0;
	ftmParam[0].firstEdgeDelayPercent = 0U;
   ftmParam[1].chnlNumber = kFTM_Chnl_1;
	ftmParam[1].level = gPWMchannel[0].Polarity;
	ftmParam[1].dutyCyclePercent = 0;
	ftmParam[1].firstEdgeDelayPercent = 0U;
   ftmParam[2].chnlNumber = kFTM_Chnl_2;
	ftmParam[2].level = kFTM_NoPwmSignal;
	ftmParam[2].dutyCyclePercent = 0;
	ftmParam[2].firstEdgeDelayPercent = 0U;
   ftmParam[3].chnlNumber = kFTM_Chnl_3;
	ftmParam[3].level = gPWMchannel[1].Polarity;
	ftmParam[3].dutyCyclePercent = 0;
	ftmParam[3].firstEdgeDelayPercent = 0U;

/*	ftmParam[0].chnlNumber = kFTM_Chnl_0;
	ftmParam[0].level = gPWMchannel[0].Polarity;
	ftmParam[0].dutyCyclePercent = 0;
	ftmParam[0].firstEdgeDelayPercent = 0U;
   ftmParam[1].chnlNumber = kFTM_Chnl_1;
	ftmParam[1].level = kFTM_NoPwmSignal;
	ftmParam[1].dutyCyclePercent = 0;
	ftmParam[1].firstEdgeDelayPercent = 0U;
   ftmParam[2].chnlNumber = kFTM_Chnl_2;
	ftmParam[2].level = gPWMchannel[1].Polarity;
	ftmParam[2].dutyCyclePercent = 0;
	ftmParam[2].firstEdgeDelayPercent = 0U;
   ftmParam[3].chnlNumber = kFTM_Chnl_3;
	ftmParam[3].level = kFTM_NoPwmSignal;
	ftmParam[3].dutyCyclePercent = 0;
	ftmParam[3].firstEdgeDelayPercent = 0U; */
#else
   ftmParam[0].chnlNumber = kFTM_Chnl_0;
	ftmParam[0].level = gPWMchannel[0].Polarity;
	ftmParam[0].dutyCyclePercent = 0;
	ftmParam[0].firstEdgeDelayPercent = 0U;
   ftmParam[1].chnlNumber = kFTM_Chnl_1;
	ftmParam[1].level = gPWMchannel[1].Polarity;
	ftmParam[1].dutyCyclePercent = 0;
	ftmParam[1].firstEdgeDelayPercent = 0U;
   ftmParam[2].chnlNumber = kFTM_Chnl_2;
	ftmParam[2].level = gPWMchannel[2].Polarity;
	ftmParam[2].dutyCyclePercent = 0;
	ftmParam[2].firstEdgeDelayPercent = 0U;
   ftmParam[3].chnlNumber = kFTM_Chnl_3;
	ftmParam[3].level = gPWMchannel[3].Polarity;
	ftmParam[3].dutyCyclePercent = 0;
	ftmParam[3].firstEdgeDelayPercent = 0U;
#endif
	
//	prescale = BOARD_FTM_CalcPrescaler(PWM_FREQUENCY_FTM0_HZ,CLOCK_GetFreq(kCLOCK_BusClk));
//	if (prescale < 0)
//		return false;
//	ftmInfo.prescale = (ftm_clock_prescale_t)prescale;
	ftmInfo.prescale = (ftm_clock_prescale_t)(FTMtimer[0].divider);

	FTM_Init(FTM0, &ftmInfo);
   FTM_SetupPwm(FTM0,ftmParam,4U,kFTM_EdgeAlignedPwm,FTMtimer[0].fTimer,CLOCK_GetFreq(kCLOCK_BusClk));

	FTM_StartTimer(FTM0, kFTM_SystemClock);

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_0,kFTM_NoPwmSignal);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_1,ftmParam[1].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_2,kFTM_NoPwmSignal);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_3,ftmParam[3].level);

/*	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_0,ftmParam[0].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_1,kFTM_NoPwmSignal);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_2,ftmParam[2].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_3,kFTM_NoPwmSignal); */
#else
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_0,ftmParam[0].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_1,ftmParam[1].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_2,ftmParam[2].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM0,kFTM_Chnl_3,ftmParam[3].level);
#endif
	
	gPWMchannel[0].TimerPeriod = FTM0->MOD;	
	gPWMchannel[1].TimerPeriod = FTM0->MOD;	
	gPWMchannel[2].TimerPeriod = FTM0->MOD;	
	gPWMchannel[3].TimerPeriod = FTM0->MOD;	
	return true;
}

bool BOARD_InitFTM1(void)
{
	return true;
}

bool BOARD_InitFTM2(void)
{
   return true;
}

bool BOARD_InitFTM3(void)
{
ftm_config_t 						ftmInfo;
ftm_chnl_pwm_signal_param_t 	ftmParam[8];
// int									prescale;

	FTM_GetDefaultConfig(&ftmInfo);
//   ftmInfo.pwmSyncMode = FTM_SYNC_SWSYNC_MASK;
	ftmInfo.bdmMode = kFTM_BdmMode_3;

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	ftmParam[0].chnlNumber = kFTM_Chnl_0;
	ftmParam[0].level = kFTM_NoPwmSignal;
	ftmParam[0].dutyCyclePercent = 0;
	ftmParam[0].firstEdgeDelayPercent = 0U;
	ftmParam[1].chnlNumber = kFTM_Chnl_1;
	ftmParam[1].level = gPWMchannel[2].Polarity;
	ftmParam[1].dutyCyclePercent = 0;
	ftmParam[1].firstEdgeDelayPercent = 0U;
	ftmParam[2].chnlNumber = kFTM_Chnl_2;
	ftmParam[2].level = kFTM_NoPwmSignal;
	ftmParam[2].dutyCyclePercent = 0;
	ftmParam[2].firstEdgeDelayPercent = 0U;
	ftmParam[3].chnlNumber = kFTM_Chnl_3;
	ftmParam[3].level = gPWMchannel[3].Polarity;
	ftmParam[3].dutyCyclePercent = 0;
	ftmParam[3].firstEdgeDelayPercent = 0U;
	ftmParam[4].chnlNumber = kFTM_Chnl_4;
	ftmParam[4].level = kFTM_NoPwmSignal;
	ftmParam[4].dutyCyclePercent = 0;
	ftmParam[4].firstEdgeDelayPercent = 0U;
	ftmParam[5].chnlNumber = kFTM_Chnl_5;
	ftmParam[5].level = gPWMchannel[4].Polarity;
	ftmParam[5].dutyCyclePercent = 0;
	ftmParam[5].firstEdgeDelayPercent = 0U;
	ftmParam[6].chnlNumber = kFTM_Chnl_6;
	ftmParam[6].level = kFTM_NoPwmSignal;
	ftmParam[6].dutyCyclePercent = 0;
	ftmParam[6].firstEdgeDelayPercent = 0U;
	ftmParam[7].chnlNumber = kFTM_Chnl_7;
	ftmParam[7].level = gPWMchannel[5].Polarity;
	ftmParam[7].dutyCyclePercent = 0;
	ftmParam[7].firstEdgeDelayPercent = 0U;

/*	ftmParam[0].chnlNumber = kFTM_Chnl_0;
	ftmParam[0].level = gPWMchannel[2].Polarity;
	ftmParam[0].dutyCyclePercent = 0;
	ftmParam[0].firstEdgeDelayPercent = 0U;
	ftmParam[1].chnlNumber = kFTM_Chnl_1;
	ftmParam[1].level = kFTM_NoPwmSignal;
	ftmParam[1].dutyCyclePercent = 0;
	ftmParam[1].firstEdgeDelayPercent = 0U;
	ftmParam[2].chnlNumber = kFTM_Chnl_2;
	ftmParam[2].level = gPWMchannel[3].Polarity;
	ftmParam[2].dutyCyclePercent = 0;
	ftmParam[2].firstEdgeDelayPercent = 0U;
	ftmParam[3].chnlNumber = kFTM_Chnl_3;
	ftmParam[3].level = kFTM_NoPwmSignal;
	ftmParam[3].dutyCyclePercent = 0;
	ftmParam[3].firstEdgeDelayPercent = 0U;
	ftmParam[4].chnlNumber = kFTM_Chnl_4;
	ftmParam[4].level = gPWMchannel[4].Polarity;
	ftmParam[4].dutyCyclePercent = 0;
	ftmParam[4].firstEdgeDelayPercent = 0U;
	ftmParam[5].chnlNumber = kFTM_Chnl_5;
	ftmParam[5].level = kFTM_NoPwmSignal;
	ftmParam[5].dutyCyclePercent = 0;
	ftmParam[5].firstEdgeDelayPercent = 0U;
	ftmParam[6].chnlNumber = kFTM_Chnl_6;
	ftmParam[6].level = gPWMchannel[5].Polarity;
	ftmParam[6].dutyCyclePercent = 0;
	ftmParam[6].firstEdgeDelayPercent = 0U;
	ftmParam[7].chnlNumber = kFTM_Chnl_7;
	ftmParam[7].level = kFTM_NoPwmSignal;
	ftmParam[7].dutyCyclePercent = 0;
	ftmParam[7].firstEdgeDelayPercent = 0U; */
#else
	ftmParam[0].chnlNumber = kFTM_Chnl_0;
	ftmParam[0].level = gPWMchannel[4].Polarity;
	ftmParam[0].dutyCyclePercent = 0;
	ftmParam[0].firstEdgeDelayPercent = 0U;
	ftmParam[1].chnlNumber = kFTM_Chnl_1;
	ftmParam[1].level = gPWMchannel[5].Polarity;
	ftmParam[1].dutyCyclePercent = 0;
	ftmParam[1].firstEdgeDelayPercent = 0U;
	ftmParam[2].chnlNumber = kFTM_Chnl_2;
	ftmParam[2].level = gPWMchannel[6].Polarity;
	ftmParam[2].dutyCyclePercent = 0;
	ftmParam[2].firstEdgeDelayPercent = 0U;
	ftmParam[3].chnlNumber = kFTM_Chnl_3;
	ftmParam[3].level = gPWMchannel[7].Polarity;
	ftmParam[3].dutyCyclePercent = 0;
	ftmParam[3].firstEdgeDelayPercent = 0U;
	ftmParam[4].chnlNumber = kFTM_Chnl_4;
	ftmParam[4].level = gPWMchannel[8].Polarity;
	ftmParam[4].dutyCyclePercent = 0;
	ftmParam[4].firstEdgeDelayPercent = 0U;
	ftmParam[5].chnlNumber = kFTM_Chnl_5;
	ftmParam[5].level = gPWMchannel[9].Polarity;
	ftmParam[5].dutyCyclePercent = 0;
	ftmParam[5].firstEdgeDelayPercent = 0U;
	ftmParam[6].chnlNumber = kFTM_Chnl_6;
	ftmParam[6].level = gPWMchannel[10].Polarity;
	ftmParam[6].dutyCyclePercent = 0;
	ftmParam[6].firstEdgeDelayPercent = 0U;
	ftmParam[7].chnlNumber = kFTM_Chnl_7;
	ftmParam[7].level = gPWMchannel[11].Polarity;
	ftmParam[7].dutyCyclePercent = 0;
	ftmParam[7].firstEdgeDelayPercent = 0U;
#endif

//	prescale = BOARD_FTM_CalcPrescaler(PWM_FREQUENCY_FTM3_HZ,CLOCK_GetFreq(kCLOCK_BusClk));
//	if (prescale < 0)
//		return false;
//	ftmInfo.prescale = (ftm_clock_prescale_t)prescale;
	ftmInfo.prescale = (ftm_clock_prescale_t)(FTMtimer[1].divider);

	FTM_Init(FTM3, &ftmInfo);
	FTM_SetupPwm(FTM3,ftmParam,8U,kFTM_EdgeAlignedPwm,FTMtimer[1].fTimer,CLOCK_GetFreq(kCLOCK_BusClk));

	FTM_StartTimer(FTM3, kFTM_SystemClock);

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_0,0);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_1,ftmParam[0].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_2,0);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_3,ftmParam[2].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_4,0);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_5,ftmParam[4].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_6,0);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_7,ftmParam[6].level);

/*	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_0,ftmParam[0].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_1,0);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_2,ftmParam[2].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_3,0);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_4,ftmParam[4].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_5,0);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_6,ftmParam[6].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_7,0); */
#else
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_0,ftmParam[0].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_1,ftmParam[1].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_2,ftmParam[2].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_3,ftmParam[3].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_4,ftmParam[4].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_5,ftmParam[5].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_6,ftmParam[6].level);
	FTM_UpdateChnlEdgeLevelSelect(FTM3,kFTM_Chnl_7,ftmParam[7].level);
#endif

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	gPWMchannel[2].TimerPeriod = FTM3->MOD;	
	gPWMchannel[3].TimerPeriod = FTM3->MOD;	
	gPWMchannel[4].TimerPeriod = FTM3->MOD;	
	gPWMchannel[5].TimerPeriod = FTM3->MOD;	
#else
	gPWMchannel[4].TimerPeriod = FTM3->MOD;	
	gPWMchannel[5].TimerPeriod = FTM3->MOD;	
	gPWMchannel[6].TimerPeriod = FTM3->MOD;	
	gPWMchannel[7].TimerPeriod = FTM3->MOD;	
	gPWMchannel[8].TimerPeriod = FTM3->MOD;	
	gPWMchannel[9].TimerPeriod = FTM3->MOD;	
	gPWMchannel[10].TimerPeriod = FTM3->MOD;	
	gPWMchannel[11].TimerPeriod = FTM3->MOD;	
#endif

	return true;
}

#if defined (FTM0_USED ) && (FTM0_USED != 0)
void BOARD_FTM0_start(void)
{
	FTM_StartTimer(FTM0, kFTM_SystemClock);
}

void BOARD_FTM0_stop(void)
{
	FTM_StartTimer(FTM0, kFTM_SystemClock);
}

void BOARD_FTM0_enable_isr(void)
{
   NVIC_SetPriority(FTM0_IRQn,FTM0_INT_PRIORITY);
   NVIC_EnableIRQ(FTM0_IRQn);
   FTM_EnableInterrupts(FTM0,kFTM_TimeOverflowInterruptEnable);
}

void BOARD_FTM0_disable_isr(void)
{
   NVIC_DisableIRQ(FTM0_IRQn);
   FTM_DisableInterrupts(FTM0,kFTM_TimeOverflowInterruptEnable);
}
#endif

#if defined (FTM1_USED ) && (FTM1_USED != 0)
void BOARD_FTM1_start(void)
{
	FTM_StartTimer(FTM1, kFTM_SystemClock);
}

void BOARD_FTM1_stop(void)
{
	FTM_StartTimer(FTM1, kFTM_SystemClock);
}

void BOARD_FTM1_enable_isr(void)
{
   NVIC_SetPriority(FTM1_IRQn,FTM1_INT_PRIORITY);
   NVIC_EnableIRQ(FTM1_IRQn);
   FTM_EnableInterrupts(FTM1,kFTM_TimeOverflowInterruptEnable);
}

void BOARD_FTM1_disable_isr(void)
{
   NVIC_DisableIRQ(FTM1_IRQn);
   FTM_DisableInterrupts(FTM1,kFTM_TimeOverflowInterruptEnable);
}
#endif

#if defined (FTM2_USED ) && (FTM2_USED != 0)
void BOARD_FTM2_start(void)
{
	FTM_StartTimer(FTM2, kFTM_SystemClock);
}

void BOARD_FTM2_stop(void)
{
	FTM_StartTimer(FTM2, kFTM_SystemClock);
}

void BOARD_FTM2_enable_isr(void)
{
   NVIC_SetPriority(FTM2_IRQn,FTM2_INT_PRIORITY);
   NVIC_EnableIRQ(FTM2_IRQn);
   FTM_EnableInterrupts(FTM2,kFTM_TimeOverflowInterruptEnable);
}

void BOARD_FTM2_disable_isr(void)
{
   NVIC_DisableIRQ(FTM2_IRQn);
   FTM_DisableInterrupts(FTM2,kFTM_TimeOverflowInterruptEnable);
}
#endif

#if defined (FTM3_USED ) && (FTM3_USED != 0)
void BOARD_FTM3_start(void)
{
	FTM_StartTimer(FTM3, kFTM_SystemClock);
}

void BOARD_FTM3_stop(void)
{
	FTM_StartTimer(FTM3, kFTM_SystemClock);
}

void BOARD_FTM3_enable_isr(void)
{
   NVIC_SetPriority(FTM3_IRQn,FTM3_INT_PRIORITY);
   NVIC_EnableIRQ(FTM3_IRQn);
   FTM_EnableInterrupts(FTM3,kFTM_TimeOverflowInterruptEnable);
}

void BOARD_FTM3_disable_isr(void)
{
   NVIC_DisableIRQ(FTM3_IRQn);
   FTM_DisableInterrupts(FTM3,kFTM_TimeOverflowInterruptEnable);
}
#endif

bool BOARD_InitPIT(void)
{
	PIT_Init(PIT,&pitConfig);
	PIT_SetTimerPeriod(PIT,kPIT_Chnl_0,MSEC_TO_COUNT(PIT0_PERIOD,CLOCK_GetBusClkFreq()) - 1);
	PIT_SetTimerPeriod(PIT,kPIT_Chnl_1,MSEC_TO_COUNT(PIT1_PERIOD,CLOCK_GetBusClkFreq()) - 1);
	PIT_SetTimerPeriod(PIT,kPIT_Chnl_2,0xFFFFFFFF);    // Full resolution
#if defined (PIT0_USED ) && (PIT0_USED != 0)
#if BOARD_PIT0_IRQ_ENA
	NVIC_SetPriority(PIT0_IRQn,PIT0_INT_PRIORITY);
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0,kPIT_TimerInterruptEnable);
#endif
#endif
#if defined (PIT1_USED ) && (PIT1_USED != 0)
#if BOARD_PIT1_IRQ_ENA
	NVIC_SetPriority(PIT1_IRQn,PIT1_INT_PRIORITY);
	PIT_EnableInterrupts(PIT, kPIT_Chnl_1,kPIT_TimerInterruptEnable);
#endif
#endif
#if defined (PIT0_USED ) && (PIT0_USED != 0)
#if BOARD_PIT0_IRQ_ENA
	EnableIRQ(PIT0_IRQn);
#endif
#endif
#if defined (PIT1_USED ) && (PIT1_USED != 0)
#if BOARD_PIT1_IRQ_ENA
	EnableIRQ(PIT1_IRQn);
#endif
#endif
#if defined (PIT2_USED ) && (PIT2_USED != 0)
#if BOARD_PIT2_IRQ_ENA
	EnableIRQ(PIT2_IRQn);
#endif
#endif
#if defined (PIT0_USED ) && (PIT0_USED != 0)
	PIT_StartTimer(PIT, kPIT_Chnl_0);
#endif
#if defined (PIT1_USED ) && (PIT1_USED != 0)
	PIT_StartTimer(PIT, kPIT_Chnl_1);
#endif
#if defined (PIT2_USED ) && (PIT2_USED != 0)
   PIT_StartTimer(PIT, kPIT_Chnl_2);
#endif
	return true;
}

static void BOARD_InitUART0(void)
{
UART_descriptor_t    *desc;

	desc = GetUARTdescriptorFromPtr(UART0);
	if (desc)
	{
		desc->baudrate = CONSOLE_UART_BAUDRATE;
		UART_InitIRQ(desc,desc->IRQ_enabled);
		UART_InitUart(desc);
	}
}

bool BOARD_PITsetPeriod(unsigned channel,unsigned period)
{
	switch(channel)
	{
#if defined (PIT0_USED ) && (PIT0_USED != 0)
#if BOARD_PIT0_IRQ_ENA
		case 0:
			PIT_SetTimerPeriod(PIT,kPIT_Chnl_0,MSEC_TO_COUNT(PIT0_PERIOD,CLOCK_GetBusClkFreq()) - 1);
			break;
#endif
#endif
#if defined (PIT1_USED ) && (PIT1_USED != 0)
#if BOARD_PIT1_IRQ_ENA
		case 1:
			PIT_SetTimerPeriod(PIT,kPIT_Chnl_1,MSEC_TO_COUNT(PIT0_PERIOD,CLOCK_GetBusClkFreq()) - 1);
			break;
#endif
#endif
#if defined (PIT2_USED ) && (PIT2_USED != 0)
#if BOARD_PIT2_IRQ_ENA
		case 2:
			PIT_SetTimerPeriod(PIT,kPIT_Chnl_2,MSEC_TO_COUNT(PIT0_PERIOD,CLOCK_GetBusClkFreq()) - 1);
			break;
#endif
#endif
		default:
			return false;
	}
	return true;
}

bool BOARD_PITenableInterrupt(unsigned channel,bool flag)
{
	switch(channel)
	{
#if defined (PIT0_USED ) && (PIT0_USED != 0)
#if BOARD_PIT0_IRQ_ENA
		case 0:
			if (flag)
				PIT_EnableInterrupts(PIT,kPIT_Chnl_0,kPIT_TimerInterruptEnable);
			else
				PIT_DisableInterrupts(PIT,kPIT_Chnl_0,kPIT_TimerInterruptEnable);
			break;
#endif
#endif
#if defined (PIT1_USED ) && (PIT1_USED != 0)
#if BOARD_PIT1_IRQ_ENA
		case 1:
			if (flag)
				PIT_EnableInterrupts(PIT,kPIT_Chnl_1,kPIT_TimerInterruptEnable);
			else
				PIT_DisableInterrupts(PIT,kPIT_Chnl_1,kPIT_TimerInterruptEnable);
			break;
#endif
#endif
#if defined (PIT2_USED ) && (PIT2_USED != 0)
#if BOARD_PIT2_IRQ_ENA
		case 2:
			if (flag)
				PIT_EnableInterrupts(PIT,kPIT_Chnl_2,kPIT_TimerInterruptEnable);
			else
				PIT_DisableInterrupts(PIT,kPIT_Chnl_2,kPIT_TimerInterruptEnable);
			break;
#endif
#endif
		default:
			return false;
	}
	return true;
}

static bool BOARD_InitI2C(int channel)
{
	switch  (channel)
	{
		case 0:
			I2C_MasterInit(I2C0, &(i2c_master_config[0]),CLOCK_GetFreq(I2C0_CLK_SRC));
			// Enable I2C0 master interfaces
			I2C_Enable(I2C0,true);
			return true;
		case 1:
			I2C_MasterInit(I2C1, &(i2c_master_config[1]),CLOCK_GetFreq(I2C1_CLK_SRC));
			// Enable I2C0 master interfaces
			I2C_Enable(I2C1,true);
		default:
			return false;
	}
}

int BOARD_InitCAN(unsigned channel)
{
   if (channel >= CAN_NR_IF)
      return 0;
   return CAN_init(channel);
}

bool BOARD_Init_WDOG(void)
{
	WDOG_Init(WDOG,&wdog_config);
	return true;
}

bool BOARD_Enable_WDOG(void)
{
	WDOG_Enable(WDOG);
	return true;
}

bool BOARD_Disable_WDOG(void)
{
	WDOG_Disable(WDOG);
	return true;
}

uint32_t BOARD_WDOG_getStatus(void)
{
	return WDOG_GetStatusFlags(WDOG);
}

bool BOARD_WDOG_clearStatus(uint32_t mask)
{
	WDOG_ClearStatusFlags(WDOG,mask);
	return true;
}

bool BOARD_WDOG_setTimeOut(uint32_t timeout)
{
	WDOG_SetTimeoutValue(WDOG,timeout);
	return true;
}

bool BOARD_WDOG_Unlock(void)
{
	WDOG_Unlock(WDOG);
	return true;
}

void BOARD_WDOG_Feed(void)
{
	WDOG_Refresh(WDOG);
}

static void BOARD_setClockAttributes(void)
{
   // Turn on the ADC0 and ADC1 clocks
   CLOCK_EnableClock(kCLOCK_Adc0);
   CLOCK_EnableClock(kCLOCK_Adc1);
   // Turn on the PDB clock
   CLOCK_EnableClock(kCLOCK_Pdb0);
   // Turn on the VREF clocks
   CLOCK_EnableClock(kCLOCK_Vref0);
   // Turn on the DAC clocks
   CLOCK_EnableClock(kCLOCK_Dac0);
   // Turn on the PORT clocks
   CLOCK_EnableClock(kCLOCK_PortA);
   CLOCK_EnableClock(kCLOCK_PortB);
   CLOCK_EnableClock(kCLOCK_PortC);
   CLOCK_EnableClock(kCLOCK_PortD);
   CLOCK_EnableClock(kCLOCK_PortE);
   // Turn on the UART clocks
   CLOCK_EnableClock(kCLOCK_Uart1);
   // Turn on the I2C clocks
   CLOCK_EnableClock(kCLOCK_I2c0);
   // Turn on the SPI clocks
   CLOCK_EnableClock(kCLOCK_Spi0);
   CLOCK_EnableClock(kCLOCK_Spi1);
   // Turn on the CRC clocks
   CLOCK_EnableClock(kCLOCK_Crc0);
   CLOCK_EnableClock(kCLOCK_Pit0);
   // Turn on the FlexTimer clocks
   CLOCK_EnableClock(kCLOCK_Ftm0);
   CLOCK_EnableClock(kCLOCK_Ftm1);
   CLOCK_EnableClock(kCLOCK_Ftm2);
   CLOCK_EnableClock(kCLOCK_Ftm3);
   // Turn on the CAN clocks
   CLOCK_EnableClock(kCLOCK_Flexcan0);
   // Turn on the UART0 and UART1 clocks
   CLOCK_EnableClock(kCLOCK_Uart0);
   CLOCK_EnableClock(kCLOCK_Uart1);
}

#if defined (USE_PDB_FOR_PUMPS) && (USE_PDB_FOR_PUMPS != 0) && defined (PDB_USED) && (PDB_USED != 0)
void PumpControlPDB(void)
{
static uint16_t		PumpState = 0;
static uint16_t		PumpStateOld = 0xFF;

	if (PumpClMgrCtrl.ctrl_by_cl_mgr)
	{
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
		pwm_channel1 = &(gPWMchannel[PWM_CONTROL_PUMP1]);
		pwm_channel2 = &(gPWMchannel[PWM_CONTROL_PUMP2]);
#else
		PWMchannel_t * pwm_channel1 = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
		PWMchannel_t * pwm_channel2 = gPWM_Control[PWM_CONTROL_PUMP2].PWMchannel1;
#endif
		if (PumpCtrl[0].enable && PumpCtrl[1].enable)
		{
#if defined (TRACEALYZER ) && (TRACEALYZER != 0) && TRC_BOARD != 0
//			if (PumpStateOld != PumpState)
//			{
//				PumpStateOld = PumpState;
//				vTracePrintF(trcBoard,"State Pumps = %d, Wait %d, Cnt = %d",PumpState,PumpClMgrCtrl.wait,PumpClMgrCtrl.period_ctr);
//			}
#endif
			switch (PumpState)
			{
				case 0:
					pwm_channel1->ActualPWM = 0;
					pwm_channel1->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = 0;
					FTM_SetSoftwareTrigger(pwm_channel1->TimerCH->Timer,true);
					pwm_channel2->ActualPWM = 0;
					pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = 0;
					FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
					BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.active_dur_p1);
					PumpState = 1;
					break;
				case 1:
					if (PumpClMgrCtrl.pulse_dur_p1 > 0)
					{
						pwm_channel1->ActualPWM = PumpClMgrCtrl.PWM_on;
						pwm_channel1->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = PumpClMgrCtrl.PWM_on;
						FTM_SetSoftwareTrigger(pwm_channel1->TimerCH->Timer,true);
						BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.pulse_dur_p1);
						PumpState = 2;
					}
					else
					{
						BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.deactive_dur_p1);
						PumpState = 3;
					}
					break;
				case 2:
					pwm_channel1->ActualPWM = 0;
					pwm_channel1->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = 0;
					FTM_SetSoftwareTrigger(pwm_channel1->TimerCH->Timer,true);
					BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.deactive_dur_p1);
					PumpState = 3;
					break;
				case 3:
					BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.active_dur_p2);
					PumpState = 4;
					break;
				case 4:
					if (PumpClMgrCtrl.pulse_dur_p2 > 0)
					{
						pwm_channel2->ActualPWM = PumpClMgrCtrl.PWM_on;
						pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel2->Channel].CnV = PumpClMgrCtrl.PWM_on;
						FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
						BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.pulse_dur_p2);
						PumpState = 5;
					}
					else
					{
						BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.deactive_dur_p2);
						PumpState = 6;
					}
					break;
				case 5:
					pwm_channel2->ActualPWM = 0;
					pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel2->Channel].CnV = 0;
					FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
					BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.deactive_dur_p2);
					PumpState = 6;
					break;
				case 6:
					pwm_channel2->ActualPWM = 0;
					pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel2->Channel].CnV = 0;
					FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
					BOARD_LoadAndTriggerPDB(PumpClMgrCtrl.wait);
					PumpState = 0;
					break;
			}
		}
		else
		{
			if (pwm_channel1->ActualPWM != 0)
			{
				pwm_channel1->ActualPWM = 0;
				pwm_channel1->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = 0;
				FTM_SetSoftwareTrigger(pwm_channel1->TimerCH->Timer,true);
			}
			if (pwm_channel2->ActualPWM != 0)
			{
				pwm_channel2->ActualPWM = 0;
				pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel2->Channel].CnV = 0;
				FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
			}
		}
	}
}

#else

void PumpControlPIT(void)
{
static uint16_t		PumpState = 0;
#if defined (TRACEALYZER ) && (TRACEALYZER != 0) && TRC_BOARD != 0
static uint16_t		PumpStateOld = 0xFF;
#endif
PWMchannel_t			*pwm_channel1;
PWMchannel_t			*pwm_channel2;

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	pwm_channel1 = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
	pwm_channel2 = gPWM_Control[PWM_CONTROL_PUMP2].PWMchannel1;
#else
	pwm_channel1 = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
	pwm_channel2 = gPWM_Control[PWM_CONTROL_PUMP2].PWMchannel1;
#endif
	if (PumpCtrl[0].enable && PumpCtrl[1].enable)
	{
#if defined (TRACEALYZER ) && (TRACEALYZER != 0) && TRC_BOARD != 0
		if (PumpStateOld != PumpState)
		{
			PumpStateOld = PumpState;
			vTracePrintF(trcBoard,"State Pumps = %d, Wait %d, Cnt = %d",PumpState,PumpClMgrCtrl.wait,PumpClMgrCtrl.period_ctr);
		}
#endif
		switch (PumpState)
		{
			case 0:
				PumpClMgrCtrl.period_ctr = 0;
				pwm_channel1->ActualPWM = 0;
				pwm_channel1->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = 0;
				FTM_SetSoftwareTrigger(pwm_channel1->TimerCH->Timer,true);
				pwm_channel2->ActualPWM = 0;
				pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = 0;
				FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
				PumpState = 1;
				break;
			case 1:
				if (PumpClMgrCtrl.period_ctr >= PumpClMgrCtrl.active_dur_p1)
				{
					if (PumpClMgrCtrl.pulse_dur_p1 > 0)
					{
						PumpClMgrCtrl.period_ctr = 0;
						pwm_channel1->ActualPWM = PumpClMgrCtrl.PWM_on;
						pwm_channel1->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = PumpClMgrCtrl.PWM_on;
						FTM_SetSoftwareTrigger(pwm_channel1->TimerCH->Timer,true);
						PumpState = 2;
					}
				}
				else
					PumpClMgrCtrl.period_ctr++;
				break;
			case 2:
				if (PumpClMgrCtrl.period_ctr >= PumpClMgrCtrl.pulse_dur_p1)
				{
					PumpClMgrCtrl.period_ctr = 0;
					pwm_channel1->ActualPWM = 0;
					pwm_channel1->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = 0;
					FTM_SetSoftwareTrigger(pwm_channel1->TimerCH->Timer,true);
					PumpState = 3;
				}
				else
					PumpClMgrCtrl.period_ctr++;
				break;
			case 3:
				if (PumpClMgrCtrl.period_ctr >= PumpClMgrCtrl.deactive_dur_p1)
				{
					PumpClMgrCtrl.period_ctr = 0;
					PumpState = 4;
				}
				else
					PumpClMgrCtrl.period_ctr++;
				break;
			case 4:
				if (PumpClMgrCtrl.period_ctr >= PumpClMgrCtrl.active_dur_p2)
				{
					PumpClMgrCtrl.period_ctr = 0;
					if (PumpClMgrCtrl.pulse_dur_p2 > 0)
					{
						pwm_channel2->ActualPWM = PumpClMgrCtrl.PWM_on;
						pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel2->Channel].CnV = PumpClMgrCtrl.PWM_on;
						FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
					}
					PumpState = 5;
				}
				else
					PumpClMgrCtrl.period_ctr++;
				break;
			case 5:
				if (PumpClMgrCtrl.period_ctr >= PumpClMgrCtrl.pulse_dur_p2)
				{
					PumpClMgrCtrl.period_ctr = 0;
					pwm_channel2->ActualPWM = 0;
					pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel2->Channel].CnV = 0;
					FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
					PumpState = 6;
				}
				else
					PumpClMgrCtrl.period_ctr++;
				break;
			case 6:
				if (PumpClMgrCtrl.period_ctr >= PumpClMgrCtrl.deactive_dur_p2)
				{
					PumpClMgrCtrl.period_ctr = 0;
					pwm_channel2->ActualPWM = 0;
					pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel2->Channel].CnV = 0;
					FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
					PumpState = 7;
				}
				else
					PumpClMgrCtrl.period_ctr++;
				break;
			case 7:
				if (PumpClMgrCtrl.period_ctr >= PumpClMgrCtrl.wait)
				{
					PumpClMgrCtrl.period_ctr = 0;
					PumpState = 0;
				}
				else
					PumpClMgrCtrl.period_ctr++;
				break;
		}
	}
}
#endif

#if defined (FTM0_USED ) && (FTM0_USED != 0)
/*!
 ******************************************************************************
 *	Flextimer 0 interrupt routine
 ******************************************************************************
*/
void FTM0_IRQHandler(void)
{
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISRBegin(FTM0_ISR_Handle); 
#endif
   FTM0->STATUS = 0;
   FTM0->SC &= ~FTM_SC_TOF_MASK;
	if (PWM_IRQHandler != NULL && !Disable_CUC_IRQ_Callback)
		PWM_IRQHandler(0);
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if defined (FTM1_USED ) && (FTM1_USED != 0)
/*!
 ******************************************************************************
 *	Flextimer 1 interrupt routine
 ******************************************************************************
*/
void FTM1_IRQHandler(void)
{
uint32_t		status1,status2;
	
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISRBegin(FTM1_ISR_Handle); 
#endif
	status1 = FTM1->SC;
	status2 = FTM1->STATUS;
	FTM1->SC &= ~FTM_SC_TOF_MASK;
	FTM1->STATUS = 0;
	if ((status2 & FTM_STATUS_CH0F_MASK) != 0 && !Disable_CUC_IRQ_Callback)
	{
		if (TIMER0_IRQHandler != NULL)
		{
			uint64_t	value = FTM1->CONTROLS[0].CnV;
			value += FTM1_counter;
			TIMER0_IRQHandler(value,(1 << 0));
		}
	}
	if ((status2 & FTM_STATUS_CH1F_MASK) != 0)
	{
		if (TIMER0_IRQHandler != 0 && !Disable_CUC_IRQ_Callback)
		{
			uint64_t	value = FTM1->CONTROLS[0].CnV;
			value += FTM1_counter;
			TIMER0_IRQHandler(value,(1 << 1));
		}
	}
	if ((status1 | FTM_SC_TOF_MASK) != 0)
	{
		FTM1_counter++;
		if (TIMER0_IRQHandler != NULL && !Disable_CUC_IRQ_Callback)
		{
			uint64_t value = FTM1->CNT;
			value += FTM1_counter << 16;
			TIMER0_IRQHandler(value,(1U << 31));
		}
	}
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if defined (FTM2_USED ) && (FTM2_USED != 0)
/*!
 ******************************************************************************
 *	Flextimer 2 interrupt routine
 ******************************************************************************
*/
void FTM2_IRQHandler(void)
{
uint32_t		status1,status2;
	
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISRBegin(FTM2_ISR_Handle); 
#endif
	status1 = FTM2->SC;
	status2 = FTM2->STATUS;
	FTM2->SC &= ~FTM_SC_TOF_MASK;
	FTM2->STATUS = 0;
	if ((status2 & FTM_STATUS_CH0F_MASK) != 0)
	{
		if (TIMER1_IRQHandler != 0 && !Disable_CUC_IRQ_Callback)
		{
			uint64_t	value = FTM2->CONTROLS[0].CnV;
			value += FTM2_counter;
			TIMER1_IRQHandler(value,(1 << 0));
		}
	}
	if ((status2 & FTM_STATUS_CH1F_MASK) != 0)
	{
		if (TIMER1_IRQHandler != 0 && !Disable_CUC_IRQ_Callback)
		{
			uint64_t	value = FTM2->CONTROLS[0].CnV;
			value += FTM2_counter;
			TIMER1_IRQHandler(value,(1 << 1));
		}
	}
	if ((status1 | FTM_SC_TOF_MASK) != 0)
	{
		FTM2_counter++;
		if (TIMER1_IRQHandler != NULL && !Disable_CUC_IRQ_Callback)
		{
			uint64_t value = FTM2->CNT;
			value += FTM2_counter << 16;
			TIMER1_IRQHandler(value,(1U << 31));
		}
	}
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if defined (FTM3_USED ) && (FTM3_USED != 0)
/*!
 ******************************************************************************
 *	Flextimer 3 interrupt routine
 ******************************************************************************
*/
void FTM3_IRQHandler(void)
{
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISRBegin(FTM3_ISR_Handle); 
#endif
   FTM3->SC &= ~FTM_SC_TOF_MASK;

#if PIT1_STARTS_ADC == 0
	if (m_EnaADC_Timer)
	{
		if (getADC0convStatus())
		{
			ADC0copyChannels();
	//		ADC0convStart();
		}
		if (getADC1convStatus())
		{
	//		ADC1convStart();
			ADC1copyChannels();
		}
		ADC0convStart();
		ADC1convStart();
		ADCtrigger = true;
	}
#endif
	if (PWM_IRQHandler != NULL && !Disable_CUC_IRQ_Callback)
		PWM_IRQHandler(1);
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if (PDB_USED != 0 && USE_PDB_FOR_PUMPS != 0)
/*!
 ******************************************************************************
 *	Programmable Delay Clock (PDB) interrupt routine
 ******************************************************************************
*/
void PDB0_IRQHandler(void)
{
// volatile uint16_t	   dummy;
// PWMchannel_t			*pwm_channel;
// static uint16_t		PumpState = 0,PumpStateOld = 0xFF;

#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISRBegin(PDB_ISR_Handle);
#endif
	PDB0->SC &= ~PDB_SC_PDBIF_MASK;
	PumpControlPDB();
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if defined (PIT0_USED ) && (PIT0_USED != 0)
/*!
 ******************************************************************************
 *	Period Timer 0 interrupt routine
 ******************************************************************************
*/
void PIT0_IRQHandler(void)
{
volatile uint16_t	   dummy;
PWMchannel_t			*pwm_channel;
static bool				bCntrlUsed = false;

#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISRBegin(PIT0_ISR_Handle); 
#endif
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
	JiffyCntr++;
#if !PDB_USED || !USE_PDB_FOR_PUMPS
	if (PumpClMgrCtrl.ctrl_by_cl_mgr)
	{
		bCntrlUsed = true;
		PumpControlPIT();
	}
	else
#endif
	{
		if (bCntrlUsed)
		{
			PWMchannel_t * pwm_channel1 = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
			PWMchannel_t * pwm_channel2 = gPWM_Control[PWM_CONTROL_PUMP2].PWMchannel1;
			if (pwm_channel1->ActualPWM != 0)
			{
				pwm_channel1->ActualPWM = 0;
				pwm_channel1->TimerCH->Timer->CONTROLS[pwm_channel1->Channel].CnV = 0;
				FTM_SetSoftwareTrigger(pwm_channel1->TimerCH->Timer,true);
			}
			if (pwm_channel2->ActualPWM != 0)
			{
				pwm_channel2->ActualPWM = 0;
				pwm_channel2->TimerCH->Timer->CONTROLS[pwm_channel2->Channel].CnV = 0;
				FTM_SetSoftwareTrigger(pwm_channel2->TimerCH->Timer,true);
			}
			bCntrlUsed = false;
		}
		if (PumpCtrl[0].enable && !PumpClMgrCtrl.ctrl_by_cl_mgr)
		{
			if (PumpCtrl[0].period_ctr == PumpCtrl[0].pulse)
			{
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
				pwm_channel = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
#else
				pwm_channel = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
#endif
				pwm_channel->ActualPWM = 0;
				pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV = 0;
				FTM_SetSoftwareTrigger(pwm_channel->TimerCH->Timer,true);
			}
			if (PumpCtrl[0].period_ctr == PumpCtrl[0].period - 1)
			{
				PumpCtrl[0].period_ctr = 0;
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
				pwm_channel = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
#else
				pwm_channel = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
#endif
				pwm_channel->ActualPWM = PumpCtrl[0].pwm_on;
				pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV = 
					pwm_channel->TimerCH->Timer->MOD + 1;
				
				FTM_SetSoftwareTrigger(pwm_channel->TimerCH->Timer,true);
			}
			else
				PumpCtrl[0].period_ctr++;
		}
		if (PumpCtrl[1].enable && !PumpClMgrCtrl.ctrl_by_cl_mgr)
		{
			if (PumpCtrl[1].period_ctr == PumpCtrl[1].pulse)
			{
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
				pwm_channel = gPWM_Control[PWM_CONTROL_PUMP2].PWMchannel1;
#else
				pwm_channel = gPWM_Control[PWM_CONTROL_PUMP2].PWMchannel1;
#endif
				pwm_channel->ActualPWM = 0;
				pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV = 0;
				FTM_SetSoftwareTrigger(pwm_channel->TimerCH->Timer,true);
			}
			if (PumpCtrl[1].period_ctr == PumpCtrl[1].period - 1)
			{
				PumpCtrl[1].period_ctr = 0;
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
				pwm_channel = gPWM_Control[PWM_CONTROL_PUMP2].PWMchannel1;
#else
				pwm_channel = gPWM_Control[PWM_CONTROL_PUMP2].PWMchannel1;
#endif
				pwm_channel->ActualPWM = PumpCtrl[1].pwm_on;
				pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV = 
					pwm_channel->TimerCH->Timer->MOD + 1;
				FTM_SetSoftwareTrigger(pwm_channel->TimerCH->Timer,true);			
			}
			else
				PumpCtrl[1].period_ctr++;
		}
	}
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if defined (PIT1_USED ) && (PIT1_USED != 0)
void PIT1_IRQHandler(void)
{
static unsigned	cnt = 0,cntCheckRelay = 0;
static bool			WD_Trig = false;
BaseType_t        xHigherPriorityTaskWoken = pdFALSE;

#if (TRACEALYZER != 0) && (TRC_BOARD != 0)
	vTraceStoreISRBegin(PIT1_ISR_Handle); 
#endif
	PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
	BOARD_Toggle_LED();
#if PIT1_STARTS_ADC != 0
	if (m_EnaADC_Timer)
	{
		if (getADC0convStatus())
		{
			ADC0copyChannels();
	//		ADC0convStart();
		}
		if (getADC1convStatus())
		{
	//		ADC1convStart();
			ADC1copyChannels();
		}
		ADC0convStart();
		ADC1convStart();
		ADCtrigger = true;
	}
#endif
	if (cnt == 99)
	{
		cnt = 0;
#if defined (FEED_WD_FTM1) && (FEED_WD_FTM1 != 0)
		if (!gTestWD)
			BOARD_SetTriggerWatchdog(WD_Trig);
		WD_Trig = !WD_Trig;
#endif
	}
	else
		cnt++;
	if (cntCheckRelay == 4)
	{
		cntCheckRelay = 0;
		if (TaskComm != NULL)
		{
			xTaskNotifyFromISR(TaskComm,(1 << 2),eSetBits,&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
	else
		cntCheckRelay++;
#if (TRACEALYZER != 0) && (TRC_BOARD != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

#if defined (PIT2_USED ) && (PIT2_USED != 0)
void PIT2_IRQHandler(void)
{
#if (TRACEALYZER != 0) && (TRC_BOARD != 0)
	vTraceStoreISRBegin(PIT2_ISR_Handle); 
#endif
	PIT->CHANNEL[2].TFLG |= PIT_TFLG_TIF_MASK;
#if (TRACEALYZER != 0) && (TRC_BOARD != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

void CAN0_ORed_Message_buffer_IRQHandler(void)
{
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISRBegin(CAN_ISR_Handle); 
#endif
   CAN_MessageIRQhandler(0);
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISREnd(0);
#endif
}

void CAN0_Error_IRQHandler(void)
{
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISRBegin(CAN_ERR_ISR_Handle); 
#endif
	CAN_ErrorIRQhandler(0);
#if (TRACEALYZER != 0) && (TRC_BOARD_ISR != 0)
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

bool BOARD_PowerUpSupplyRails(void)
{
	if (!BOARD_Enable_5V())
		return false;
	if (!BOARD_Enable_10V())
		return false;
	return true;
}

bool BOARD_PowerDownSupplyRails(void)
{
	if (!BOARD_Disable_5V())
		return false;
	if (!BOARD_Disable_10V())
		return false;
	return true;
}

bool BOARD_Set_PWM_FTM_Frequency(unsigned channel,uint32_t frequency)
{
#if USE_FLOAT != 0
float				fBus;
#else
uint32_t			fBus;
#endif
int				prescale = 0;
uint16_t			period;
FTM_Type			*timer;
	
	if (channel >= N_PWM_FTM_TIMER_CHANNELS)
		return false;
	timer = FTMtimer[channel].Timer;
	FTM_StopTimer(timer);
	fBus = CLOCK_GetBusClkFreq();
	prescale = BOARD_FTM_CalcPrescaler(frequency,fBus);
	if (prescale < 0)
		return false;
#if USE_FLOAT != 0
	period = (uint16_t)(fBus / ((prescale + 1) * (float)frequency));
#else
	period = (uint16_t)(((uint64_t)fBus * frequency) / (prescale + 1));
#endif
	FTM_SetTimerPeriod(timer,period);
	timer->SC = FTM_SC_PS(prescale);
	for (int i = FTMtimer[channel].indexPWMchan;i < 
		FTMtimer[channel].nPWMchannels + FTMtimer[channel].indexPWMchan;i++)
	{
		gPWMchannel[i].TimerPeriod = period;
	}
	FTM_StartTimer(timer, kFTM_SystemClock);
	return true;
}

int BOARD_Get_PWM_FTM_Frequency(unsigned channel)
{
#if USE_FLOAT != 0
float			period,fBus;
#else
uint32_t		period,fBus;
#endif
FTM_Type		*timer;
unsigned		divide;
	
	if (channel >= N_PWM_FTM_TIMER_CHANNELS)
		return -1;
	timer = FTMtimer[channel].Timer;
	fBus = CLOCK_GetBusClkFreq();
	period = timer->MOD;
	divide = 1 << (((timer->SC & FTM_SC_PS_MASK)) >> FTM_SC_PS_SHIFT);
#if USE_FLOAT != 0
	return (int)(fBus / (period * divide) + 0.5F);
#else
	return (int)(fBus / (period * divide));
#endif
}

/*!
 *********************************************************************************
 * Gets the PWM period in microseconds
 * \param[in]	channel	PWM Channel [0 .. N_PWM_FTM_TIMER_CHANNELS]
 * \return		PWM period in usec
 *********************************************************************************
*/
int BOARD_Get_PWM_FTM_Period(unsigned channel)
{
#if USE_FLOAT != 0
float			period,fBus;
#else
uint32_t		period,fBus;
#endif
FTM_Type		*timer;
unsigned		divide;
	
	// Period is in us
	if (channel >= N_PWM_FTM_TIMER_CHANNELS)
		return -1;
	timer = FTMtimer[channel].Timer;
	fBus = CLOCK_GetBusClkFreq();
	period = timer->MOD;
	divide = 1 << (((timer->SC & FTM_SC_PS_MASK)) >> FTM_SC_PS_SHIFT);
#if USE_FLOAT != 0
	return (int)(((uint64_t)period * 1000000ULL * divide) / fBus);
#else
	return (int)(((uint64_t)period * 1000000ULL * divide) / fBus);
#endif
}

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)

static bool BOARD_Set_FTM_PWM_And_DIR(unsigned channel,unsigned dutycycle,eDirMode_t direction)
{
uint16_t			pwm_val;
uint16_t			period;
PWM_Control_t	*pwm_control;
	
		if (channel >= N_PWM_CHANNELS || channel >= N_PWM_CONTROL_CHANNELS)
			return false;
		pwm_control = &(gPWM_Control[channel]);
		period = pwm_control->PWMchannel->TimerPeriod;
		if (direction == eDirModeLeft)
			dutycycle = 1000 - dutycycle;
#ifdef USE_FLOAT
		pwm_val = (uint16_t)(period * (float)dutycycle / 1000.0F + 0.5F);
#else
		pwm_val = (uint16_t)((period * dutycycle + 500) / 1000);
#endif
      if (pwm_val >= period)
			pwm_val = period + 1;
		switch (direction)
		{
			case eDirModeRight:
				GPIO_PinWrite(pwm_control->DirectionGPIO,pwm_control->DirectionPIN,1);	
				pwm_control->PWMchannel->TimerCH->Timer->CONTROLS[pwm_control->PWMchannel->Channel].CnV = pwm_val;
				pwm_control->DirMode = direction;
				pwm_control->PWMchannel->ActualPWM = pwm_val;
				BOARD_WriteGPIOpin(pwm_control->SleepGPIO,1);
				break;
			case eDirModeLeft:
				GPIO_PinWrite(pwm_control->DirectionGPIO,pwm_control->DirectionPIN,0);	
				pwm_control->PWMchannel->TimerCH->Timer->CONTROLS[pwm_control->PWMchannel->Channel].CnV = pwm_val;
				pwm_control->DirMode = direction;
				pwm_control->PWMchannel->ActualPWM = pwm_val;
				BOARD_WriteGPIOpin(pwm_control->SleepGPIO,1);
				break;
			case eDirModeBrake:
				GPIO_PinWrite(pwm_control->DirectionGPIO,pwm_control->DirectionPIN,0);	
				pwm_control->DirMode = direction;
				pwm_control->PWMchannel->ActualPWM = 0;
				pwm_control->PWMchannel->TimerCH->Timer->CONTROLS[pwm_control->PWMchannel->Channel].CnV = 0;
				BOARD_WriteGPIOpin(pwm_control->SleepGPIO,1);
				break;
			case eDirModeHighZ:
				GPIO_PinWrite(pwm_control->DirectionGPIO,pwm_control->DirectionPIN,0);	
				pwm_control->PWMchannel->TimerCH->Timer->CONTROLS[pwm_control->PWMchannel->Channel].CnV = 0;
				pwm_control->DirMode = direction;
				pwm_control->PWMchannel->ActualPWM = 0;
				BOARD_WriteGPIOpin(pwm_control->SleepGPIO,0);
				break;
			case eDirModeSleep:
				GPIO_PinWrite(pwm_control->DirectionGPIO,pwm_control->DirectionPIN,0);	
				pwm_control->PWMchannel->TimerCH->Timer->CONTROLS[pwm_control->PWMchannel->Channel].CnV = 0;
				pwm_control->DirMode = direction;
				pwm_control->PWMchannel->ActualPWM = 0;
				BOARD_WriteGPIOpin(pwm_control->SleepGPIO,0);
				break;
			default:
				return false;
		}
		FTM_SetSoftwareTrigger(pwm_control->PWMchannel->TimerCH->Timer,true);
		return true;
}
#endif

bool BOARD_Set_FTM_PWM(unsigned channel,unsigned dutycycle)
{
uint16_t			pwm_val;
uint16_t			period;
PWMchannel_t	*pwm_channel;
	
		if (channel >= N_PWM_CHANNELS)
			return false;
		pwm_channel = &(gPWMchannel[channel]);
		period = pwm_channel->TimerPeriod;
#if USE_FLOAT != 0
		pwm_val = (uint16_t)(period * (float)dutycycle / 1000.0F + 0.5F);
#else
		pwm_val = (uint16_t)(((uint64_t)period * dutycycle) / 1000);
#endif
      if (pwm_val >= period)
			pwm_val = period + 1;
      pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV = pwm_val;
		FTM_SetSoftwareTrigger(pwm_channel->TimerCH->Timer,true);
		return true;
}

static bool BOARD_Set_FTM_PWM_from_ptr(PWMchannel_t *pwm_channel,unsigned dutycycle)
{
uint16_t			pwm_val;
uint16_t			period;
	
		if (pwm_channel == NULL)
			return false;
		period = pwm_channel->TimerPeriod;
#if USE_FLOAT != 0
		pwm_val = (uint16_t)(period * (float)dutycycle / 1000.0F + 0.5F);
#else
		pwm_val = (uint16_t)(((uint64_t)period * dutycycle) / 1000);
#endif
      if (pwm_val >= period)
			pwm_val = period + 1;
      pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV = pwm_val;
		FTM_SetSoftwareTrigger(pwm_channel->TimerCH->Timer,true);
		return true;
}

int BOARD_Get_FTM_PWM(unsigned channel)
{
float				pwm_val;
float				period;
int				dutycycle;
PWMchannel_t	*pwm_channel;

		if (channel >= N_PWM_CHANNELS)
			return -1;
		pwm_channel = &(gPWMchannel[channel]);
		period = pwm_channel->TimerPeriod;
		pwm_val = pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV;
		dutycycle = (int)(pwm_val / period * 1000.0F + 0.5F);
		return dutycycle;
}

bool BOARD_Get_FTM_PWM_from_ptr(PWMchannel_t *pwm_channel,unsigned *dutycycle)
{
#if USE_FLOAT != 0
float				pwm_val;
float				period;
#else
uint32_t			pwm_val;
uint32_t			period;
#endif

		if (pwm_channel == NULL)
			return false;
		period = pwm_channel->TimerPeriod;
		pwm_val = pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV;
#if USE_FLOAT != 0
		*dutycycle = (int)(pwm_val / period * 1000.0F + 0.5F);
#else
		*dutycycle = (int)(((uint32_t)pwm_val * 1000) / period);
#endif
		return true;
}

bool BOARD_Get_FTM_PWM_TimerIndex(unsigned channel,uint8_t *index)
{
	if (channel >= N_PWM_CHANNELS)
		return false;
	*index = gPWMchannel[channel].TimerIndex;
	return true;
}

bool BOARD_Set_Test_Ext_WD(uint8_t flag)
{
	gTestWD = flag;
	return true;
}

#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)

bool BOARD_SetPWMControl(unsigned channel,unsigned pwm,eDirMode_t DirMode)
{
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	return BOARD_Set_FTM_PWM_And_DIR(channel,pwm,DirMode);	
}

bool BOARD_GetPWMControl(unsigned channel,unsigned *pwm,eDirMode_t *DirMode,
	unsigned *dutycycle)
{
PWMchannel_t	*pwm_ch;

	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	*pwm = gPWM_Control[channel].pwm;
	*DirMode = gPWM_Control[channel].DirMode;
	pwm_ch = gPWM_Control[channel].PWMchannel;
	if (!BOARD_Get_FTM_PWM_from_ptr(pwm_ch,dutycycle))
		return false;
	return true;
}

bool BOARD_CheckPWMControl(unsigned channel)
{
PWM_Control_t	*pwm_ch;
eDirMode_t  	DirMode;
unsigned			pwm;
uint8_t			DirState;
float				period;
float				pwm_val;
int				dutycycle;
uint8_t			pin;
	
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	pwm_ch = &(gPWM_Control[channel]);
	pwm = pwm_ch->pwm;
	DirMode = gPWM_Control[channel].DirMode;
	if (!BOARD_ReadGPIOpin(pwm_ch->SleepGPIO,&pin))
		return false;
	DirState = GPIO_PinRead(pwm_ch->DirectionGPIO,pwm_ch->DirectionPIN);
	if (DirMode == eDirModeRight)
	{
		period = pwm_ch->PWMchannel->TimerCH->Timer->MOD;
		pwm_val = pwm_ch->PWMchannel->TimerCH->Timer->CONTROLS->CnV;
		if (pwm_val > period)
			pwm_val = period;
		dutycycle = (int)(pwm_val * 1000.0F / period + 0.5F);
      if (dutycycle == 0 && pwm > 1)
			return false;
		else
			if (dutycycle <= pwm - 1 || dutycycle >= pwm - 1)
				return false;
		if (DirState == 0)
			return false;
		if (pin == 0)
			return false;
		return true;
	}
	else
		if (DirMode == eDirModeLeft)
		{
			period = pwm_ch->PWMchannel->TimerCH->Timer->MOD;
			pwm_val = pwm_ch->PWMchannel->TimerCH->Timer->CONTROLS->CnV;
			if (pwm_val > period)
				pwm_val = period;
			dutycycle = (int)(pwm_val * 1000.0F / period + 0.5F);
			if (dutycycle == 0 && pwm > 1)
				return false;
			else
				if (dutycycle <= pwm - 1 || dutycycle >= pwm - 1)
					return false;
			if (DirState != 0)
				return false;
			if (pin == 0)
				return false;
			return true;
		}
		else
			if (DirMode == eDirModeBrake)
			{
				if (pwm_ch->PWMchannel->TimerCH->Timer->CONTROLS->CnV != 0)
					return false;
				if (DirState != 0)
					return false;
				if (pin == 0)
					return false;
				return true;
			}
			else
				if (DirMode == eDirModeHighZ)
				{
					period = pwm_ch->PWMchannel->TimerCH->Timer->MOD;
					if (pwm_ch->PWMchannel->TimerCH->Timer->CONTROLS->CnV <= period)
						return false;
					if (DirState != 0)
						return false;
					if (pin == 0)
						return false;
					return true;
				}
				else
					if (DirMode == eDirModeSleep)
					{
						if (pin != 0)
							return false;
						return true;
					}
	return false;
}

#else

bool BOARD_SetPWMControl(unsigned channel,unsigned pwm,eDirMode_t DirMode)
{
PWMchannel_t	*pwm_ch1,*pwm_ch2;
#if defined (USE_ENDSWITCH) && (USE_ENDSWITCH != 0)
bool				bEndSw = false;
#endif
	
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
#if defined (USE_ENDSWITCH) && (USE_ENDSWITCH != 0)
	if (channel == PWM_CONTROL_LIFT_BR)
	{
		if (DirMode == eDirModeLeft)
		{
			if (!BOARD_GetEndSwitch1())
				bEndSw = true;
		}
	}
#endif
	pwm_ch1 = gPWM_Control[channel].PWMchannel1;
	pwm_ch2 = gPWM_Control[channel].PWMchannel2;
	if (DirMode == eDirModeRight)
	{
		gPWM_Control[channel].pwm = pwm;
		gPWM_Control[channel].DirMode = DirMode;
		pwm_ch1->ActualPWM = pwm;
		pwm_ch2->ActualPWM = 0;
		BOARD_Set_FTM_PWM_from_ptr(pwm_ch1,pwm);
		BOARD_Set_FTM_PWM_from_ptr(pwm_ch2,0);
		BOARD_WriteGPIOpin(gPWM_Control[channel].SleepGPIO,1);
		return true;
	}
	else
		if (DirMode == eDirModeLeft)
		{
#if defined (USE_ENDSWITCH) && (USE_ENDSWITCH != 0)
			if (bEndSw)
			{
				gPWM_Control[channel].pwm = 0;
				gPWM_Control[channel].DirMode = DirMode;
				pwm_ch1->ActualPWM = 0;
				pwm_ch2->ActualPWM = 0;
				BOARD_Set_FTM_PWM_from_ptr(pwm_ch1,0);
				BOARD_Set_FTM_PWM_from_ptr(pwm_ch2,0);
				BOARD_WriteGPIOpin(gPWM_Control[channel].SleepGPIO,0);
			}
			else
			{
#endif
				gPWM_Control[channel].pwm = pwm;
				gPWM_Control[channel].DirMode = DirMode;
				pwm_ch1->ActualPWM = 0;
				pwm_ch2->ActualPWM = pwm;
				BOARD_Set_FTM_PWM_from_ptr(pwm_ch1,0);
				BOARD_Set_FTM_PWM_from_ptr(pwm_ch2,pwm);
				if (channel == 4)
					channel = 4;
				BOARD_WriteGPIOpin(gPWM_Control[channel].SleepGPIO,1);
#if defined (USE_ENDSWITCH) && (USE_ENDSWITCH != 0)
			}
#endif
			return true;
		}
		else
			if (DirMode == eDirModeBrake)
			{
				gPWM_Control[channel].DirMode = DirMode;
				pwm_ch1->ActualPWM = 0;
				pwm_ch2->ActualPWM = 0;
				BOARD_Set_FTM_PWM_from_ptr(pwm_ch1,0);
				BOARD_Set_FTM_PWM_from_ptr(pwm_ch2,0);
				BOARD_WriteGPIOpin(gPWM_Control[channel].SleepGPIO,1);
				return true;
			}
			else
				if (DirMode == eDirModeHighZ)
				{
					gPWM_Control[channel].DirMode = DirMode;
					pwm_ch1->ActualPWM = 1;
					pwm_ch2->ActualPWM = 1;
					BOARD_Set_FTM_PWM_from_ptr(pwm_ch1,1000);
					BOARD_Set_FTM_PWM_from_ptr(pwm_ch2,1000);
					BOARD_WriteGPIOpin(gPWM_Control[channel].SleepGPIO,1);
					return true;
				}
				else
					if (DirMode == eDirModeSleep)
					{
						gPWM_Control[channel].DirMode = eDirModeSleep;
//						BOARD_Set_FTM_PWM_from_ptr(pwm_ch1,1000);
//						BOARD_Set_FTM_PWM_from_ptr(pwm_ch2,1000);
						BOARD_WriteGPIOpin(gPWM_Control[channel].SleepGPIO,0);
						return true;
					}
	return false;
}

bool BOARD_GetPWMControl(unsigned channel,unsigned *pwm,eDirMode_t *DirMode,
	unsigned *dutycycle1,unsigned *dutycycle2)
{
PWMchannel_t	*pwm_ch1,*pwm_ch2;

	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	*pwm = gPWM_Control[channel].pwm;
	*DirMode = gPWM_Control[channel].DirMode;
	pwm_ch1 = gPWM_Control[channel].PWMchannel1;
	pwm_ch2 = gPWM_Control[channel].PWMchannel2;
	if (!BOARD_Get_FTM_PWM_from_ptr(pwm_ch1,dutycycle1))
		return false;
	if (!BOARD_Get_FTM_PWM_from_ptr(pwm_ch2,dutycycle2))
		return false;
	return true;
}

bool BOARD_GetPWMdirection(unsigned channel,eDirMode_t *DirMode)
{
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	*DirMode = gPWM_Control[channel].DirMode;
	return true;
}

/*!
 ******************************************************************************
 *	Check the PWM Controller of a channel
 * \param[in]     channel  	PWM channel (0 .. N_PWM_CONTROL_CHANNELS - 1)
 * \return			true if success, false else
 ******************************************************************************
*/
bool BOARD_CheckPWMControl(unsigned channel)
{
PWMchannel_t	*pwm_ch1,*pwm_ch2;
eDirMode_t  	DirMode;
unsigned			pwm;
float				period;
float				pwm_val;
int				dutycycle;
uint8_t			pin;
	
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	pwm_ch1 = gPWM_Control[channel].PWMchannel1;
	pwm_ch2 = gPWM_Control[channel].PWMchannel2;
	pwm = gPWM_Control[channel].pwm;
	DirMode = gPWM_Control[channel].DirMode;
	if (!BOARD_ReadGPIOpin(gPWM_Control[channel].SleepGPIO,&pin))
		return false;
	if (DirMode == eDirModeRight)
	{
		period = pwm_ch1->TimerCH->Timer->MOD;
		pwm_val = pwm_ch1->TimerCH->Timer->CONTROLS->CnV;
		if (pwm_val > period)
			pwm_val = period;
		dutycycle = (int)(pwm_val * 1000.0F / period + 0.5F);
      if (dutycycle == 0 && pwm > 1)
			return false;
		else
			if (dutycycle <= pwm - 1 || dutycycle >= pwm - 1)
				return false;
		if (pwm_ch2->TimerCH->Timer->CONTROLS->CnV != 0)
			return false;
		if (pin == 0)
			return false;
		return true;
	}
	else
		if (DirMode == eDirModeLeft)
		{
			period = pwm_ch2->TimerCH->Timer->MOD;
			pwm_val = pwm_ch2->TimerCH->Timer->CONTROLS->CnV;
			if (pwm_val > period)
				pwm_val = period;
			dutycycle = (int)(pwm_val * 1000.0F / period + 0.5F);
			if (dutycycle == 0 && pwm > 1)
				return false;
			else
				if (dutycycle <= pwm - 1 || dutycycle >= pwm - 1)
					return false;
			if (pwm_ch1->TimerCH->Timer->CONTROLS->CnV != 0)
				return false;
			if (pin == 0)
				return false;
			return true;
		}
		else
			if (DirMode == eDirModeBrake)
			{
				if (pwm_ch1->TimerCH->Timer->CONTROLS->CnV != 0)
					return false;
				if (pwm_ch2->TimerCH->Timer->CONTROLS->CnV != 0)
					return false;
				if (pin == 0)
					return false;
				return true;
			}
			else
				if (DirMode == eDirModeHighZ)
				{
					period = pwm_ch2->TimerCH->Timer->MOD;
					if (pwm_ch1->TimerCH->Timer->CONTROLS->CnV <= period)
						return false;
					if (pwm_ch2->TimerCH->Timer->CONTROLS->CnV <= period)
						return false;
					if (pin == 0)
						return false;
					return true;
				}
				else
					if (DirMode == eDirModeSleep)
					{
						if (pin != 0)
							return false;
						return true;
					}
	return false;
}

#endif

/*!
 ******************************************************************************
 *	Get the Status of a channel
 * \param[in]     channel  	PWM channel (0 .. N_PWM_CONTROL_CHANNELS - 1)
 * \param[out]    status  		Status of the channel
 * \return			true if success, false else
 ******************************************************************************
*/
bool BOARD_GetPWMStatus(unsigned channel,uint32_t *status)
{
uint8_t				value;

	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	if (!BOARD_ReadGPIOpin(gPWM_Control[channel].FaultGPIO,&value))
		return false;
	*status = value == 0 ? 1 : 0;
	return true;
}

/*!
 ******************************************************************************
 *	Get the Index of the PWM Timer of a channel
 * \param[in]     channel  	PWM channel (0 .. N_PWM_CONTROL_CHANNELS - 1)
 * \param[out]    timer  		Index of the Timer
 * \return			true if success, false else
 ******************************************************************************
*/
bool BOARD_GetPWMTimer(unsigned channel,uint8_t *timer)
{
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	*timer = gPWM_Control[channel].TimerIndex;
	return true;
}

/*!
 ******************************************************************************
 *	Gets the Name of a PWM channel
 * \param[in]     channel  	PWM channel (0 .. N_PWM_CONTROL_CHANNELS - 1)
 * \return			Name of the channel if success, NULL else
 ******************************************************************************
*/
const char * BOARD_GetPWMName(unsigned channel)
{
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return NULL;
	return gPWM_Control[channel].Name;
}

/*!
 ******************************************************************************
 *	Enables all PWM channels
 * \return			true if success, false else
 ******************************************************************************
*/
void BOARD_EnablePWM(void)
{
	for (int i = 0;i < N_PWM_CONTROL_CHANNELS;i++)
		BOARD_WriteGPIOpin(gPWM_Control[i].SleepGPIO,1);
}

/*!
 ******************************************************************************
 *	Completely disable the PWM (all PWM channels)
 ******************************************************************************
*/
void BOARD_DisablePWM(void)
{
	for (int i = 0;i < N_PWM_CONTROL_CHANNELS;i++)
		BOARD_WriteGPIOpin(gPWM_Control[i].SleepGPIO,0);
}

/*!
 ******************************************************************************
 *	Enables a PWM channel
 * \param[in]     channel  	PWM channel (0 .. N_PWM_CONTROL_CHANNELS - 1)
 * \return			true if success, false else
 ******************************************************************************
*/
bool BOARD_EnablePWMchannel(unsigned channel)
{
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	return BOARD_WriteGPIOpin(gPWM_Control[channel].SleepGPIO,1);
}

/*!
 ******************************************************************************
 *	Dieables a PWM channel
 * \param[in]     channel  	PWM channel (0 .. N_PWM_CONTROL_CHANNELS - 1)
 * \return			true if success, false else
 ******************************************************************************
*/
bool BOARD_DisablePWMchannel(unsigned channel)
{
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	return BOARD_WriteGPIOpin(gPWM_Control[channel].SleepGPIO,0);
}

/*!
 ******************************************************************************
 *	Get the Status of a PWM channel
 * \param[in]     channel  	PWM channel (0 .. N_PWM_CONTROL_CHANNELS - 1)
 * \param[out]    status  		Status of poth PWM Drivers of the channel
 * \return			true if success, false else
 ******************************************************************************
*/
bool BOARD_GetPWMchannelStatus(unsigned channel,uint8_t *status)
{
uint8_t		value;
	
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	if (!BOARD_ReadGPIOpin(gPWM_Control[channel].SleepGPIO,&value))
		return false;
	*status = value ? (1 << 0) : (0 << 0);
	if (!BOARD_ReadGPIOpin(gPWM_Control[channel].FaultGPIO,&value))
		return false;
	*status |= value ? (1 << 1) : (0 << 1);
	return true;
}

PWM_Control_t * BOARD_GetPWM_ptr(unsigned channel)
{
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return NULL;
	else
		return &(gPWM_Control[channel]);
}

bool BOARD_GetPWM_CounterPeriod(unsigned channel,uint32_t *period)
{
	if (channel >= N_PWM_CONTROL_CHANNELS)
		return false;
	else
	{
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
		*period = gPWM_Control[channel].PWMchannel->TimerCH->Timer->MOD;
#else
		*period = gPWM_Control[channel].PWMchannel1->TimerCH->Timer->MOD;
#endif
		return true;
	}
}

/*!
 ******************************************************************************
 *	Sets the frequency (in Hz) and pulse duration (ms) for a pump control
 * \param[in]     channel  		selects the pump
 * \param[in]     frequency  		pump control signal frequency in Hz
 * \param[in]     pulselen  		pump control pulse duration in ms
 * \return			true if success
 ******************************************************************************
*/

bool BOARD_SetPumpFreqPulse(uint8_t channel,uint16_t frequency,uint16_t pulselen)
{
PWMchannel_t	*pwm_ch1;

	if (channel > 1)
		return false;
	if (frequency < 1)
		frequency = 1;
	if (pulselen < 1)
		pulselen = 1;
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	pwm_ch1 = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel;
#else
	pwm_ch1 = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel1;
#endif
	PumpCtrl[channel].frequency = frequency;
	PumpCtrl[channel].pulse = pulselen;
	PumpCtrl[channel].period = 1000 / frequency;
	PumpCtrl[channel].period_ctr = 0;
	if (PumpCtrl[channel].period < 1)
	{
		PumpCtrl[channel].enable = false;
		pwm_ch1->ActualPWM = 0;
		pwm_ch1->TimerCH->Timer->CONTROLS[pwm_ch1->Channel].CnV = 0;
		pwm_ch1->ActualPWM = 0;
		FTM_SetSoftwareTrigger(pwm_ch1->TimerCH->Timer,true);
		return false;
	}
	return true;
}

/*!
 ******************************************************************************
 *	Sets the PWM (on) value for a pump control in %%
 * \param[in]     channel  		selects the pump
 * \param[in]     PWMvalue			PWM value in %% (1 / 1000)
 * \param[in]     pulselen  		pump control pulse duration in ms
 * \return			true if success
 ******************************************************************************
*/

bool BOARD_SetPumpPWM(uint8_t channel,uint16_t PWMvalue)
{
PWMchannel_t	*pwm_ch;

	if (channel > 1)
		return false;
	if (PWMvalue > 1000)
		PWMvalue = 1000;
	pwm_ch = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel1;
	uint32_t period = pwm_ch->TimerPeriod;
	uint32_t pwm_val = (uint16_t)(period * (float)PWMvalue / 1000.0F + 0.5F);
	if (pwm_val >= period)
		pwm_val = period + 1;
//#ifdef USE_DRV8701P
//	pwm_ch1 = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel;
//#else
//	pwm_ch1 = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel1;
//#endif
	PumpCtrl[channel].pwm_on = pwm_val;
	return true;
}

/*!
 ******************************************************************************
 *	Gets the frequency (in Hz) and pulse duration (ms) for a pump control
 * \param[in]     channel  		selects the pump
 * \param[out]    frequency  		pump control signal frequency in Hz
 * \param[out]    pulselen  		pump control pulse duration in ms
 * \return			true if success
 ******************************************************************************
*/

bool BOARD_GetPumpFreqPulse(uint8_t channel,uint16_t *frequency,uint16_t *pulselen)
{
	if (channel > 1)
		return false;
	*frequency = PumpCtrl[channel].frequency;
	*pulselen = PumpCtrl[channel].pulse;
	return true;
}

/*!
 ******************************************************************************
 *	Sets the Waterpump Parameters (period (in us), pulse duration (us) 
 * and PWM value (in 1/1000) for a pump control
 * \param[in]     channel  		selects the pump
 * \param[in]     period  			pump control period in ms
 * \param[in]     pulselen  		pump control pulse duration in ms
 * \param[in]     PWMvalue  		pump control PWM value (in 1/1000 = %%)
 * \return			true if success
 ******************************************************************************
*/

bool BOARD_SetWaterPumpParams(uint8_t channel,uint16_t period,uint16_t pulselen,uint16_t PWMvalue)
{
PWMchannel_t	*pwm_channel;
uint32_t			frequency;

	if (channel > 1)
		return false;
	if (pulselen < 1)
		pulselen = 1;
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	pwm_ch1 = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel;
#else
	pwm_channel = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel1;
#endif
	frequency = 1000000 / period;
	if (frequency < 1)
		frequency = 1;
	uint32_t period_int = pwm_channel->TimerPeriod;
	uint32_t pwm_val = (uint16_t)(period_int * (float)PWMvalue / 1000.0F + 0.5F);
	if (pwm_val >= period_int)
		pwm_val = period + 1;
	PumpCtrl[channel].frequency = frequency;
	PumpCtrl[channel].pulse = pulselen / 1000;
	PumpCtrl[channel].period = period / 1000;
	PumpCtrl[channel].period_ctr = 0;
	PumpCtrl[channel].pwm_on = pwm_val;
	if (PumpCtrl[channel].period < 1)
	{
		PumpCtrl[channel].enable = false;
		pwm_channel->ActualPWM = 0;
		pwm_channel->TimerCH->Timer->CONTROLS[pwm_channel->Channel].CnV = 0;
		pwm_channel->ActualPWM = 0;
		FTM_SetSoftwareTrigger(pwm_channel->TimerCH->Timer,true);
		return false;
	}
	return true;
}

/*!
 ******************************************************************************
 *	Enables or Disables (On/Off) a pump
 * \param[in]     channel  		selects the pump
 * \param[in]     onoff  			Enables or Disables the pump (On/Off)
 * \return			true if success
 ******************************************************************************
*/

bool BOARD_EnablePump(uint8_t channel,bool OnOff)
{
PWMchannel_t	*pwm_ch1;
	
	if (channel > 1)
		return false;
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	pwm_ch1 = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel;
#else
	pwm_ch1 = gPWM_Control[PWM_CONTROL_PUMP1 + channel].PWMchannel1;
#endif
	if (OnOff)
	{
		BOARD_WriteGPIOpin(gPWM_Control[PWM_CONTROL_PUMP1 + channel].SleepGPIO,1);
	}
	else
	{
		pwm_ch1->ActualPWM = 0;
		pwm_ch1->TimerCH->Timer->CONTROLS[pwm_ch1->Channel].CnV = 0;
		pwm_ch1->ActualPWM = 0;
		FTM_SetSoftwareTrigger(pwm_ch1->TimerCH->Timer,true);
		BOARD_WriteGPIOpin(gPWM_Control[PWM_CONTROL_PUMP1 + channel].SleepGPIO,0);
	}
	PumpCtrl[channel].enable = OnOff;
	return true;
}

/*!
 ******************************************************************************
 *	Gets the On/Off-status a pump
 * \param[in]     channel  		selects the pump
 * \param[out]    onoff  			pump On/Off-status
 * \return			true if success
 ******************************************************************************
*/

bool BOARD_GetPumpStatus(uint8_t channel,bool *OnOff)
{
	if (channel > 1)
		return false;
	*OnOff = PumpCtrl[channel].enable;
	return true;
}

/*!
 ******************************************************************************
 *	Register an PWM Timer interrupt callback
 * \param[in]     IRQHandler  	IRQ Handler
 * \return			true if success
 ******************************************************************************
*/
bool BOARD_RegisterPWM_FTM_Callback(void(*IRQHandler)(unsigned channel))
{
   PWM_IRQHandler = IRQHandler;
	return true;
}

/*!
 ******************************************************************************
 *	Register an Timer Overflow interrupt callback
 * \param[in]     IRQHandler  	IRQ Handler
 * \return			true if success
 ******************************************************************************
*/
bool BOARD_RegisterFTM_Callback(uint8_t channel,void(*IRQHandler)(uint64_t value,uint32_t flags))
{
	switch (channel)
	{
		case 0:
			TIMER0_IRQHandler = IRQHandler;
			break;
		case 1:
			TIMER1_IRQHandler = IRQHandler;
			break;
		default:
			return false;
	}	
   return true;
}

/*!
 ******************************************************************************
 *	Enable of Disable the CUC interrupt callbacks
 * \param[in]     IRQHandler  	IRQ Handler
 * \return			true if success
 ******************************************************************************
*/
void BOARD_Ena_CUC_IRQ_Callback(bool enable)
{
	Disable_CUC_IRQ_Callback = !enable;
}

/*!
 ******************************************************************************
 *	Sets the Own Address of the device
 * \param[in]     address     Own Address
 ******************************************************************************
*/
void BOARD_SetOwnAddress(uint16_t address)
{
   DeviceOwnAddress = address;
}

/*!
 ******************************************************************************
 *	Gets the Own Address of the device
 * \return        Own Address of the device
 ******************************************************************************
*/
uint16_t BOARD_GetOwnAddress(void)
{
   return DeviceOwnAddress;
}

void BOARD_InitDebugConsole(void)
{
}

bool BOARD_getSystemTime(uint64_t *time)
{
uint32_t				clock;
static uint64_t	old_time = 0;
	
	// System Time is in ns
	PIT_DisableInterrupts(PIT,kPIT_Chnl_0,kPIT_TimerInterruptEnable);
	SystemTime.ticks_ms = JiffyCntr * PIT0_PERIOD;
	SystemTime.ticks_timer = PIT_GetCurrentTimerCount(PIT,kPIT_Chnl_0);
	clock = CLOCK_GetBusClkFreq();
	*time = (1000000000ULL * SystemTime.ticks_timer) / clock + (uint64_t)(SystemTime.ticks_ms) * 1000000ULL;
	PIT_EnableInterrupts(PIT,kPIT_Chnl_0,kPIT_TimerInterruptEnable);
	// Prevent false time due to pipelining
	if (*time < old_time)
		*time += 1000000ULL;
	old_time = *time;
	return true;
}

/*!
 *********************************************************************************
 * Gets the System Time in nanoseconds. The System Time starts with 0 when the
 * MCU powers up (the 64-Bit Jiffy Counter variable will wrap around in 584 years)
 * The function directly returns the system time without the boolean return value
 * \param[out]		time	System Time in Nanoseconds
 * \return			true if success, false else
 *********************************************************************************
*/
uint64_t BOARD_getSystemTimeDirect(void)
{
uint64_t				clk1_H,clk2_H;
uint64_t				clk1_L,clk2_L;
uint64_t				time;
static uint64_t	old_time = 0;

	// System Time is in ns
	clk1_H = JiffyCntr;
	clk1_L = 0xFFFFFFFFUL - PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_0);
	clk2_L = 0xFFFFFFFFUL - PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_0);
	clk2_H = JiffyCntr;
	/*
	 * We need to detect a wrap-around of the timer to determine if the
	 * Jiffy Counter before of after the wrap around is valid
	 * if clk1_L > clk2_L the timer did wrap-around and clk2_H is valid,
	 * else clk1_L is valid
	 */
	if (clk1_L > clk2_L)
	{
		mSystemTime.ticks_ms = clk2_H * PIT0_PERIOD;
		mSystemTime.ticks_timer = clk2_L;
	}
	else
	{
		mSystemTime.ticks_ms = clk1_H * PIT0_PERIOD;
		mSystemTime.ticks_timer = clk1_L;
	}
	/* calculate the time in nanoseconds */
	time = (1000000000ULL * mSystemTime.ticks_timer) / CLOCK_GetBusClkFreq() +
		   (uint64_t)(mSystemTime.ticks_ms) * 1000000ULL;
	// Prevent false time due to pipelining
	if (time < old_time)
		time += 1000000ULL;
	old_time = time;
	return time;
}

void BOARD_Reset_ADC_Trigger(void)
{
	ADCtrigger = false;
}
	
bool BOARD_getADC_Trigger(void)
{
	return ADCtrigger;
}

uint64_t Now(void)
{
	return JiffyCntr;
}

uint64_t DiffTime(unsigned tm)
{
	return JiffyCntr - tm;
}

void SleepBM(unsigned tm)
{
volatile uint64_t	nw = Now();

	while (DiffTime(nw) < tm)
		;
}

bool CheckVB1_Voltage(uint8_t *vb1_status)
{
uint16_t 	value;

	*vb1_status = 0;
	if ((RelayFlags & RELAY_FLAGS_CHECK_VB1) != 0)
	{
		if (BOARD_get_ADC(ADC_VB_SENSE,&value))
		{
			if (value < RELAY1_VB1_THRESHOLD)
			{
				*vb1_status &= ~RELAY_VB1_STATUS_VB1;
			}
			else
			{
				*vb1_status |= RELAY_VB1_STATUS_VB1;
			}
			return true;
		}
		else
			return false;
	}
	else
	{
		*vb1_status |= RELAY_VB1_STATUS_NO_CHECK;
		return true;
	}
}

void HandleRelay1(void)
{
static uint32_t		DelayCounter = 0;
static int				state1 = 0, state2 = 0;
static uint8_t			nSleepStatus = 0;
static uint8_t			nRetry = 0;
static bool				bErrorMax = false;
uint16_t 				value;
uint8_t					VBstate;

	switch (state1)
	{
		case 0:	// Initial State
			if (RelayStatus1 != OldRelay1Status || ForceStatus1)
			{
				DelayCounter = 0;
				ForceStatus1 = false;
				if (RelayStatus1)
				{
					BOARD_SetVBenable1(false);
					BOARD_SetVBenable2(false);
					BOARD_SetChargeVB(false);
					nSleepStatus = Board_GetPWMnSleep();
					Board_SetAllPWMnSleep(false);				
					BOARD_SetVBenable2(false);
					bErrorMax = false;
					RelayFlags |= RELAY_FLAGS_CHECK_VB1;
					nRetry = 0;
					state1 = 1;
				}
				else
				{
					state1 = 10;
				}
			}
			break;
		case 1:	// Switch Relay 1 on - Start
			if (DelayCounter == 10)
			{
//				if (BOARD_GetTestEMstop())
//				{
					BOARD_SetChargeVB(true);
					DelayCounter = 0;
					state1 = 2;
//				}
			}
			else
				DelayCounter++;
			break;
		case 2:	// Switch Relay 1 on - Precharge Capacitors
			if (DelayCounter == ChargeDelay)
			{
				if (!BOARD_get_ADC(ADC_VB_SENSE,&value))
				{
					BOARD_SetChargeVB(false);
					Board_RestorePWMnSleep(nSleepStatus);
					state1 = 0;
				}
				if (value < RELAY1_VB1_THRESHOLD)
				{
					BOARD_SetChargeVB(false);
					Board_RestorePWMnSleep(nSleepStatus);
					state1 = 0;
				}
				BOARD_SetVBenable1(true);
				BOARD_SetChargeVB(false);
				DelayCounter = 0;
				state1 = 3;
			}
			else
				DelayCounter++;
			break;
		case 3:	// Switch Relay 1 on - Done
			if (DelayCounter == 10)
			{
				Board_RestorePWMnSleep(nSleepStatus);
				DelayCounter = 0;
				state1 = 4;
			}
			else
				DelayCounter++;
			break;
		case 4:	// Switch Relay 1 on - Test
			if (DelayCounter == 2)
			{
				if (BOARD_get_ADC(ADC_VB_SENSE,&value))
				{
					if (value < RELAY1_VB1_THRESHOLD)
					{
						OldRelay1Status = false;
						BOARD_SetVBenable1(false);
						DelayCounter = 0;
						state1 = 6;
					}
					else
					{
						OldRelay1Status = true;				
						state1 = 5;
					}
				}
				else
				{
					OldRelay1Status = false;
					BOARD_SetVBenable1(false);
					state1 = 0;
				}
			}
			else
				DelayCounter++;
			break;
		case 5: // Relay 1 is on
			if (!CheckVB1_Voltage(&VBstate))
				state1 = 6;
			else
			{
				if ((VBstate & RELAY_VB1_STATUS_VB1) == 0)
				{
					BOARD_SetVBenable1(false);
					OldRelay1Status = false;
					DelayCounter = 0;
					state1 = 6;
				}
				else
					if (!RelayStatus1)
						state1 = 0;
			}
			break;
		case 6: // Relay 1 on - Error, Retry
			if (nRetry == 5)
			{
				DelayCounter = 0;
				bErrorMax = true;
				state1 = 100;
			}
			else
			{
				if (DelayCounter == 50)
				{
					DelayCounter = 0;
					nRetry++;
					nSleepStatus = Board_GetPWMnSleep();
					state1 = 1;
				}
				else
					DelayCounter++;
			}
			break;
		case 10:	// Switch Relay 1 off - Start
			if (DelayCounter == 10)
			{
				nSleepStatus = Board_GetPWMnSleep();
				Board_SetAllPWMnSleep(false);				
				BOARD_SetChargeVB(false);
				BOARD_SetVBenable2(false);
				DelayCounter = 0;
				state1 = 11;
			}
			else
				DelayCounter++;
			break;
		case 11:	// Switch Relay 1 off - Action
			if (DelayCounter == 2)
			{
				BOARD_SetVBenable1(false);
				DelayCounter = 0;
				state1 = 12;
			}
			else
				DelayCounter++;
			break;
		case 12:	// Switch Relay 1 off - Done
			if (DelayCounter == 2)
			{
				OldRelay1Status = false;
				OldRelay2Status = false;
				Board_RestorePWMnSleep(nSleepStatus);
				state1 = 13;
			}
			else
				DelayCounter++;
			break;
		case 13: // Switch Relay 1 is off
			if (RelayStatus1)
					state1 = 0;
			break;
		case 100:
			if (DelayCounter == 200)
				state1 = 0;
			else
			if (DelayCounter == 200)
			break;
		default:
			BOARD_SetChargeVB(false);
			BOARD_SetVBenable2(false);
			BOARD_SetVBenable1(false);
			OldRelay1Status = false;
			OldRelay2Status = false;
			state1 = 0;
	}
	switch (state2)
	{
		case 0:
			if (RelayStatus2 != OldRelay2Status || ForceStatus2)
			{
				ForceStatus2 = false;
				if (RelayStatus2)
				{
					if (OldRelay1Status)
					{
						state2 = 1;
					}
				}
				else
				{
					state2 = 10;
				}
			}
			break;
		case 1:
			BOARD_SetVBenable2(true);
			OldRelay2Status = true;	
			state2 = 0;
			break;
		case 10:
			BOARD_SetVBenable2(false);
			OldRelay2Status = false;	
			state2 = 0;
			break;
		default:
			BOARD_SetVBenable2(false);
			OldRelay2Status = false;
			state2 = 0;
	}
}

bool ControlRelay1(bool Status,uint32_t ChargeDel,bool ForceStat)
{
	RelayStatus1 = Status;
	ChargeDelay = ChargeDel / 50;
	ForceStatus1 = ForceStat;
	return true;
//uint8_t		status;

//	if (Status != OldRelay1Status || ForceStatus)
//	{
//		CheckVB1 = false;
//		if (Status)
//		{
//			BOARD_SetVBenable1(false);
//			BOARD_SetVBenable2(false);
//			OldRelay2Status = false;
//			SleepBM(500);
//			Board_GetAllPWMnSleepBit(&status);
//			Board_SetAllPWMnSleep(false);				
//			BOARD_SetChargeVB(true);
//			SleepBM(ChargeDelay);
//			BOARD_SetVBenable1(true);
//			BOARD_SetChargeVB(false);
//			SleepBM(500);
//			Board_SetAllPWMnSleepBit(status);
//		}
//		else
//		{
//			Board_SetAllPWMnSleep(false);				
//			BOARD_SetChargeVB(false);
//			BOARD_SetVBenable2(false);
//			OldRelay2Status = false;
//			SleepBM(100);
//			BOARD_SetVBenable1(false);
//			SleepBM(500);
//		}
//		OldRelay1Status = Status;
//		CheckVB1 = true;
//	}
//	return true;
}

bool ControlRelay2(bool Status,bool ForceStat)
{
	RelayStatus2 = Status;
	ForceStatus2 = ForceStat;
	return true;
//	if (RelayStatus2 != OldRelay2Status || ForceStat)
//	{
//		if (OldRelay1Status)
//		{
//			if (Status)
//			{
//				BOARD_SetVBenable2(true);
//			}
//			else
//			{
//				BOARD_SetVBenable2(false);
//			}
//			OldRelay2Status = Status;
//		}
//		CheckVB1 = true;
//	}
//	return true;
}

/*!
 ******************************************************************************
 *	Tests the status of the Relays. If both Relays are enabled and the voltage
 * behind the second Relay is below the threshold then the function signals an
 * error
 * \return			true if the Relay Status matches the Volt. behind the Relay
 ******************************************************************************
*/
bool BOARD_TestRelayStatus(void)
{
uint16_t		value;
	
	if (!BOARD_get_ADC(ADC_VB_24V_RELAY2,&value))
		return false;
	if (value >= RELAY1_VB1_THRESHOLD)
		return true;
	else
		return false;
}

/*!
 ******************************************************************************
 *	Gets the status of the Relays
 * \param[out]    status  			Status of the Relays
 * \param[out]    VB1_rail  		Measured Voltage at the Relays
 * \return			true if success
 ******************************************************************************
*/
bool BOARD_GetRelayStatus(uint8_t *status,float *VB1_rail)
{
uint16_t		value;
	
	if (!BOARD_get_ADC_float(ADC_VB_SENSE,VB1_rail))
		return false;
	if (!BOARD_get_ADC(ADC_VB_SENSE,&value))
		return false;
	*status = BOARD_GetVBenable1() ? (1 << 0) : 0;
	*status |= BOARD_GetVBenable2() ? (1 << 1) : 0;
	*status |= value >= RELAY1_VB1_THRESHOLD ? (1 << 2) : 0;
	return true;
}

/*!
 ******************************************************************************
 *	Sets the Waterpump Parameters for the Cleaning Manager
 * \param[in]     ActDur  			Activation Duration in ms
 * \param[in]     PulseDur  		Pulse Duration in ms
 * \param[in]     DeactDur  		Deactivation Duration in ms
 * \param[in]     Period  			Period in ms
 * \param[in]     PWM  				Pulse-On PWM value in 1/1000
 * \return			true if success
 ******************************************************************************
*/
bool BOARD_ClMngr_SetParameters(uint16_t ActDur,uint16_t PulseDur,
	uint16_t DeactDur,uint32_t Period,uint16_t PWM)
{
int				resttime = Period - (ActDur + PulseDur + DeactDur) * 2;
PWMchannel_t	*pwm_channel;

	if (resttime < 0)
		resttime = 0;
	if (PWM > 1000)
		PWM = 1000;
#if !defined (PDB_USED) || (PDB_USED == 0)
//	ActDur /= 1000;
//	PulseDur /= 1000;
//	DeactDur /= 1000;
#endif
	PumpClMgrCtrl.period = Period;
	PumpClMgrCtrl.wait = resttime; 
	PumpClMgrCtrl.active_dur_p1 = ActDur;
	PumpClMgrCtrl.active_dur_p2 = ActDur;
	PumpClMgrCtrl.pulse_dur_p1 = PulseDur;
	PumpClMgrCtrl.pulse_dur_p2 = PulseDur;
	PumpClMgrCtrl.deactive_dur_p1 = DeactDur;
	PumpClMgrCtrl.deactive_dur_p2 = DeactDur;
//	PumpClMgrCtrl.period_ctr = 0;
	pwm_channel = gPWM_Control[PWM_CONTROL_PUMP1].PWMchannel1;
	uint32_t period_int = pwm_channel->TimerPeriod;
	uint32_t pwm_val = (uint16_t)(period_int * (float)PWM / 1000.0F + 0.5F);
	if (pwm_val >= period_int)
		pwm_val = period_int + 1;
	PumpClMgrCtrl.PWM_on = pwm_val;
	return true;
}

/*!
 ******************************************************************************
 *	Enables or Disables the Dosing Pump Cleaning Manager Control
 * \param[in]     Enable  			if true the Control is enabled, else disabled
 * \return			true if success
 ******************************************************************************
*/
bool BOARD_ClMngr_EnaPumpCtrl(bool Enable)
{
	PumpClMgrCtrl.PumpCtrlByClMgr = Enable;
	return true;
}

/*!
 ******************************************************************************
 *	Enables or Disables the Cleaning Fluid Valve
 * \param[in]     Enable  			if true the Control is enabled, else disabled
 * \return			true if success
 ******************************************************************************
*/
bool BOARD_ClMngr_EnaClFuidValveCtrl(bool Enable)
{
	PumpClMgrCtrl.ValveCtrlByClMgr = Enable;
	return true;
}

/*!
 ******************************************************************************
 *	Gets the Cleaning Fluid Valve Cleaning Manager Control Enable State
 * \return			Enable State
 ******************************************************************************
*/
bool BOARD_isPumpClMgrEnabled(void)
{
	return PumpClMgrCtrl.PumpCtrlByClMgr;
}

/*!
 ******************************************************************************
 *	Gets the Cleaning Fluid Valve Control Enable State
 * \return			Enable State
 ******************************************************************************
*/
bool BOARD_isValveClMgrEnabled(void)
{
	return PumpClMgrCtrl.PumpCtrlByClMgr;
}

/*!
 ******************************************************************************
 *	Initializes the PDB (Programmable Delay Block)
 *	Loads the Modulo Value of the PDB (Programmable Delay Block).
 * The Modulo Value is limited by the PDB resolution and the Mod Reg Size
 * (Fbus / (PDBpredivider * PDBmultiplier)
 * \param[in]		modval		Mod value in us
 * \return			true if succeeded, false else
 ******************************************************************************
*/
bool BOARD_InitPDB(uint32_t modval)
{
#if defined (PDB_USED ) && (PDB_USED != 0)
int		PDBvalue;

	PDB_PumpCtrl_Config.frequency = CLOCK_GetBusClkFreq();
	PDB_CalcFactor = PDB_PumpCtrl_Config.prescaler *
			PDB_PumpCtrl_Config.multiplier * 1000000;
	PDBvalue = (modval * (uint64_t)(PDB_PumpCtrl_Config.frequency) - 2) / PDB_CalcFactor;
	if (PDBvalue < 1)
		PDBvalue = 1;
	if (PDBvalue > 0xFFFF)
		PDBvalue = 0xFFFF;
	// Change pdb_config members if necessary
	PDB_Init(PDB0,&pdb_config);
	PDB_SetModulusValue(PDB0,PDBvalue);
	PDB_DoLoadValues(PDB0);
	while ((PDB_GetStatusFlags(PDB0) & PDB_SC_LDMOD_MASK) != 0)
		;
	// Enable the Interrupt
	NVIC_SetPriority(PDB0_IRQn,PDB_INT_PRIORITY);
   NVIC_EnableIRQ(PDB0_IRQn);
	return true;
#else
	return false;
#endif
}

/*!
 ******************************************************************************
 *	Enables or Disables the PDB (Programmable Delay Block)
 * \param[in]		enable		true = enable, false = disable
 * \return			true if succeeded, false else
 ******************************************************************************
*/
bool BOARD_EnablePDB(bool enable)
{
#if defined (PDB_USED ) && (PDB_USED != 0)
	PDB_Enable(PDB0,enable);
	return true;
#else
	return false;
#endif
}

/*!
 ******************************************************************************
 *	Enables or Disables the PDB (Programmable Delay Block) Interrupt
 * \param[in]		enable		true = enable, false = disable
 * \return			true if succeeded, false else
 ******************************************************************************
*/
bool BOARD_EnablePDBInterrupt(bool enable)
{
#if defined (PDB_USED ) && (PDB_USED != 0)
	if (enable)
	{
		PDB_EnableInterrupts(PDB0,kPDB_DelayInterruptEnable);
	}
	else
	{
		PDB_DisableInterrupts(PDB0,kPDB_DelayInterruptEnable);
	}
	return true;
#else
	return false;
#endif
}

/*!
 ******************************************************************************
 *	Loads the Delay Value of the PDB (Programmable Delay Block).
 * The Delay Value is limited by the PDB resolution and the Mod Reg Size
 * (Fbus / (PDBpredivider * PDBmultiplier) and the Mod Reg Value
 * param[in]		delay in us
 * \return			true if succeeded, false else
 ******************************************************************************
*/
bool BOARD_LoadAndTriggerPDB(uint32_t delay)
{
#if defined (PDB_USED ) && (PDB_USED != 0)
int				PDBvalue;

	PDBvalue = (delay * (uint64_t)(PDB_PumpCtrl_Config.frequency) - 2) / PDB_CalcFactor;
	if (PDBvalue < 1)
		PDBvalue = 1;
	if (PDBvalue > 0xFFFF)
		PDBvalue = 0xFFFF;
   PDB0->IDLY = PDBvalue;
	PDB0->SC |= PDB_SC_LDOK_MASK;
	while ((PDB0->SC & PDB_SC_LDMOD_MASK) != 0);
	PDB0->SC |= PDB_SC_SWTRIG_MASK;
	return true;
#else
	return false;
#endif
}

/*!
 ******************************************************************************
 *	Gets the Stack and Heap region info
 * \param[in]     getZIregions  	if true the ZI regions are used
 * \param[out]    onoff  			pump On/Off-status
 * \return			true if success
 ******************************************************************************
*/
bool BOARD_GetStackAndHeapInfo(bool getZIregions,uint32_t *stack_base,uint32_t *stack_size,
		uint32_t *heap_base,uint32_t *heap_size)
{
	if (getZIregions)
	{
		*stack_base = (uint32_t)(&Image$$ARM_LIB_STACK$$ZI$$Base);
		*stack_size = (uint32_t)(&Image$$ARM_LIB_STACK$$ZI$$Length);
		*heap_base = (uint32_t)(&Image$$ARM_LIB_HEAP$$ZI$$Base);
		*heap_size = (uint32_t)(&Image$$ARM_LIB_HEAP$$ZI$$Length);
	}
	else
	{
		*stack_base = (uint32_t)(&Image$$ARM_LIB_STACK$$Base);
		*stack_size = (uint32_t)(&Image$$ARM_LIB_STACK$$Length);
		*heap_base = (uint32_t)(&Image$$ARM_LIB_HEAP$$Base);
		*heap_size = (uint32_t)(&Image$$ARM_LIB_HEAP$$Length);
	}
	return true;
}

void BOARD_EnaADC_Timer(bool flag)
{
	m_EnaADC_Timer = flag;
}

#if (TRACEALYZER != 0) && (TRC_BOARD != 0)
static bool BOARD_InitTracealyzer(void)
{
#if defined (FTM0_USED ) && (FTM0_USED != 0)
	FTM0_ISR_Handle = xTraceSetISRProperties("FTM0 ISR", FTM0_INT_PRIORITY);
#endif
#if defined (FTM1_USED ) && (FTM1_USED != 0)
	FTM1_ISR_Handle = xTraceSetISRProperties("FTM1 ISR", FTM1_INT_PRIORITY);
#endif
#if defined (FTM2_USED ) && (FTM2_USED != 0)
	FTM2_ISR_Handle = xTraceSetISRProperties("FTM2 ISR", FTM2_INT_PRIORITY);
#endif
#if defined (FTM3_USED ) && (FTM3_USED != 0)
	FTM3_ISR_Handle = xTraceSetISRProperties("FTM3 ISR", FTM3_INT_PRIORITY);
#endif
#if defined (PDB_USED ) && (PDB_USED != 0)
	PDB_ISR_Handle = xTraceSetISRProperties("PDB ISR", PDB_INT_PRIORITY);
#endif
#if defined (PIT0_USED ) && (PIT0_USED != 0)
	PIT0_ISR_Handle = xTraceSetISRProperties("PIT0 ISR", PIT0_INT_PRIORITY);
#endif
#if defined (PIT1_USED ) && (PIT1_USED != 0)
	PIT1_ISR_Handle = xTraceSetISRProperties("PIT1 ISR", PIT1_INT_PRIORITY);
#endif
#if defined (PIT2_USED ) && (PIT2_USED != 0)
	PIT2_ISR_Handle = xTraceSetISRProperties("PIT2 ISR", PIT2_INT_PRIORITY);
#endif
#if defined (CAN_USED ) && (CAN_USED != 0)
	CAN_ISR_Handle = xTraceSetISRProperties("CAN ISR", PIT1_INT_PRIORITY);
	CAN_ERR_ISR_Handle = xTraceSetISRProperties("CAN ERR ISR", PIT1_INT_PRIORITY);
#endif
	return true;
}
#endif

/*!
 ******************************************************************************
 *	Initializes the Board
 * \return			true if success
 ******************************************************************************
*/
bool BOARD_Init(void)
{
	BOARD_setClockAttributes();
	BOARD_setGPIO_PinAttributes();
	BOARD_InitDigIO();
	BOARD_InitUART0();
	BOARD_InitPIT();
	BOARD_InitFTM0();
	BOARD_InitFTM1();
	BOARD_InitFTM2();
	BOARD_InitFTM3();
//	BOARD_FTM0_enable_isr();
//	BOARD_FTM1_enable_isr();
//	BOARD_FTM2_enable_isr();
//	BOARD_FTM3_enable_isr();
	BOARD_InitVREF();
	BOARD_InitADC();
//	BOARD_InitDAC();
	CRC_init();
	BOARD_InitCAN(CAN_CHANNEL);
	BOARD_InitI2C(I2C_CHANNEL);
	I2C_Bus_Init();
	EEPROM_Initialize();
	uint8_t data;
#if defined (PDB_USED ) && (PDB_USED != 0)
	BOARD_InitPDB(PDB_MOD_VALUE);
#endif
	BOARD_PowerUpSupplyRails();
#if (TRACEALYZER != 0) && (TRC_BOARD != 0)
	BOARD_InitTracealyzer();
#endif
#if (TRACEALYZER != 0) && (TRC_BOARD != 0)
	trcBoard = xTraceRegisterString("BOARD");
#endif
	return true;
}
