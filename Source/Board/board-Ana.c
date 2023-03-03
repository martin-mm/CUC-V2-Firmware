#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "board-Ana.h"

#if TRACEALYZER != 0 && (TRC_ANA != 0 || TRC_ANA_CHANNEL != 0 || TRC_ANA_ISR != 0)
#include "trcRecorder.h"
#endif

#if TRACEALYZER != 0 && TRC_ANA_ISR
#if defined(ADC0_USED) && (ADC0_USED != 0)
#if TRC_ANA_ISR != 0 
traceHandle ADC0_ISR_Handle;
#endif
#endif
#if defined(ADC1_USED) && (ADC1_USED != 0)
#if TRC_ANA_ISR != 0 
traceHandle ADC1_ISR_Handle;
#endif
#endif
#endif

static volatile int				gCnt0 = 0;
static volatile int				gCnt1 = 0;

static volatile bool				ADC0copySema = false;
static volatile bool				ADC1copySema = false;

static uint16_t		      	ADC_conversion_temp_value[BOARD_ADC_NumberOfChannels];
//static uint16_t		      	ADC_conversion_temp_value_F[BOARD_ADC_NumberOfChannels];
static uint16_t		      	ADC_conversion_value[BOARD_ADC_NumberOfChannels];
static uint16_t		      	ADC_conv_offset_calib[BOARD_ADC_NumberOfChannels];
//static uint16_t		      	ADC_conversion_value_F[BOARD_ADC_NumberOfChannels];
static volatile int		   	gADC0channel_ctr = 0;
static volatile int		   	gADC1channel_ctr = 0;
static volatile bool				gADC0allConvDone = false;
static volatile bool				gADC1allConvDone = false;
#if TRACEALYZER != 0 && (TRC_ANA != 0 || TRC_ANA_CHANNEL != 0)
static traceString 				adc_CH0;
static traceString 				adc_CH1;
#endif

static const vref_config_t	VrefConfigStruct =
{
    .bufferMode = kVREF_ModeHighPowerBuffer
};

static volatile ADC_Conversion_Ptr_t 	ADC_Conversion_ADC0_Ptr[ADC0_N_CHANNEL] =
{
	{
		.value_ptr = &(ADC_conversion_temp_value[2]),
		.copy_value_ptr = &(ADC_conversion_value[2]),
		.adc_mux = 0,
		.adc_channel = ADC_CH2
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[3]),
		.copy_value_ptr = &(ADC_conversion_value[3]),
		.adc_mux = 0,
		.adc_channel = ADC_CH3
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[4]),
		.copy_value_ptr = &(ADC_conversion_value[4]),
		.adc_mux = 0,
		.adc_channel = ADC_CH4
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[5]),
		.copy_value_ptr = &(ADC_conversion_value[5]),
		.adc_mux = 0,
		.adc_channel = ADC_CH5
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[6]),
		.copy_value_ptr = &(ADC_conversion_value[6]),
		.adc_mux = 0,
		.adc_channel = ADC_CH6
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[9]),
		.copy_value_ptr = &(ADC_conversion_value[9]),
		.adc_mux = 0,
		.adc_channel = ADC_CH9
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[10]),
		.copy_value_ptr = &(ADC_conversion_value[10]),
		.adc_mux = 0,
		.adc_channel = ADC_CH10
	},
#ifdef CUC_HW_V2
	{
		.value_ptr = &(ADC_conversion_temp_value[11]),
		.copy_value_ptr = &(ADC_conversion_value[11]),
		.adc_mux = 0,
		.adc_channel = ADC_CH11
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[12]),
		.copy_value_ptr = &(ADC_conversion_value[12]),
		.adc_mux = 0,
		.adc_channel = ADC_CH12
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[13]),
		.copy_value_ptr = &(ADC_conversion_value[13]),
		.adc_mux = 0,
		.adc_channel = ADC_CH13
	}
#else
	{
		.value_ptr = &(ADC_conversion_temp_value[11]),
		.copy_value_ptr = &(ADC_conversion_value[11]),
		.adc_mux = 0,
		.adc_channel = ADC_CH11
	}
#endif
};

static volatile ADC_Conversion_Ptr_t 	ADC_Conversion_ADC1_Ptr[ADC1_N_CHANNEL] =
{
	{
		.value_ptr = &(ADC_conversion_temp_value[0]),
		.copy_value_ptr = &(ADC_conversion_value[0]),
		.adc_mux = 0,
		.adc_channel = ADC_CH0
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[1]),
		.copy_value_ptr = &(ADC_conversion_value[1]),
		.adc_mux = 0,
		.adc_channel = ADC_CH1
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[7]),
		.copy_value_ptr = &(ADC_conversion_value[7]),
		.adc_mux = 0,
		.adc_channel = ADC_CH7
	},
	{
		.value_ptr = &(ADC_conversion_temp_value[8]),
		.copy_value_ptr = &(ADC_conversion_value[8]),
		.adc_mux = 0,
		.adc_channel = ADC_CH8
	},
#ifdef CUC_HW_V2
	{
		.value_ptr = &(ADC_conversion_temp_value[14]),
		.copy_value_ptr = &(ADC_conversion_value[14]),
		.adc_mux = 0,
		.adc_channel = ADC_TEMP_SENSOR
	}
#else
	{
		.value_ptr = &(ADC_conversion_temp_value[12]),
		.copy_value_ptr = &(ADC_conversion_value[12]),
		.adc_mux = 0,
		.adc_channel = ADC_TEMP_SENSOR
	}
#endif
};

static adc16_board_channel_t		adc16BoardChannel[BOARD_ADC_NumberOfChannels] =
{
	{
		.ADC = ADC_CH0_ADC,			// [0] ADC1-DM1 = LIFT-SUC-CUR
		.ChannelConfig =
		{
				.channelNumber = ADC_CH0,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0008332F,			// Unit is A
		.offset = 30036.9F,			// Offset
		.calib_offset = true,		// calibrate offset 
		.calib_gain = false,		// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH1_ADC,			// [1] ADC1-DP0 = LIFT-BR-CUR
		.ChannelConfig =
		{
				.channelNumber = ADC_CH1,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0008332F,			// Unit is A
		.offset = 30036.9F,			// Offset
		.calib_offset = true,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH2_ADC,			// [2] ADC0-DM1 = PUMP-CUR
		.ChannelConfig =
		{
				.channelNumber = ADC_CH2,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0008332F,			// Unit is A
		.offset = 30036.9F,			// Offset
		.calib_offset = true,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH3_ADC,			// [3] ADC0-DP1 = BRUSH-CUR
		.ChannelConfig =
		{
				.channelNumber = ADC_CH3,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0008332F,			// Unit is A
		.offset = 30036.9F,			// Offset
		.calib_offset = true,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH4_ADC,			// [4] ADC0-DM0 = SUCT-CUR
		.ChannelConfig =
		{
				.channelNumber = ADC_CH4,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0008332F,			// Unit is A
		.offset = 30036.9F,			// Offset
		.calib_offset = true,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH5_ADC,			// [5] ADC0-SE22 = VB-SAFETY
		.ChannelConfig =
		{
				.channelNumber = ADC_CH5,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0005127031F,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH6_ADC,			// [6] ADC0-SE21 = VB-SENSE
		.ChannelConfig =
		{
				.channelNumber = ADC_CH6,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0005127031F,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH7_ADC,			// [7] ADC1-DP1 = 3V3-RAIL
		.ChannelConfig =
		{
				.channelNumber = ADC_CH7,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0000549325F,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH8_ADC,			// [8] ADC1-DM0 = 5V-RAIL
		.ChannelConfig =
		{
				.channelNumber = ADC_CH8,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0000842298F,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH9_ADC,			// [9] ADC0-DP0 = 10V-RAIL
		.ChannelConfig =
		{
				.channelNumber = ADC_CH9,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0001684596,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH10_ADC,			// [10] ADC0-SE12 = VB-OUT
		.ChannelConfig =
		{
				.channelNumber = ADC_CH10,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0005127031F,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH11_ADC,			// [11] ADC0-SE13 = VB-OUT-SW
		.ChannelConfig =
		{
				.channelNumber = ADC_CH11,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0005127031F,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
#ifdef CUC_HW_V2
	{
		.ADC = ADC_CH12_ADC,			// [12] ADC0-SE8 = VB_24V_A_KEY_SNS
		.ChannelConfig =
		{
				.channelNumber = ADC_CH12,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0005127031F,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADC_CH13_ADC,			// [13] ADC0-SE9 = VB_24V_RELAY2
		.ChannelConfig =
		{
				.channelNumber = ADC_CH13,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.0005127031F,		// Unit is V
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = true,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	},
	{
		.ADC = ADCTEMP_SENSOR_ADC,	// [14] Internal Temperature Sensor
		.ChannelConfig =
		{
				.channelNumber = ADC_TEMP_SENSOR,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.00001831F,			// Unit is °C
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false,			// calibrate gain 
		.use_hw_average = false,	// use hardware averager
		.hw_average_mode = kADC16_HardwareAverageCount32		// size of hardware average
	}
#else
	{
		.ADC = ADCTEMP_SENSOR_ADC,	// [12] Internal Temperature Sensor
		.ChannelConfig =
		{
				.channelNumber = ADC_TEMP_SENSOR,
				.enableDifferentialConversion = false,
				.enableInterruptOnConversionCompleted = true
		},
		.gain = 0.00001831F,			// Unit is °C
		.offset = 0,					// Offset
		.calib_offset = false,		// calibrate offset 
		.calib_gain = false			// calibrate gain 
	}
#endif
};

static const adc16_config_t				adc16ConfigStruct =
{
	.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt,
	.clockSource = kADC16_ClockSourceAlt0,
	.enableAsynchronousClock = false,
	.clockDivider = kADC16_ClockDivider8,
	.resolution = kADC16_ResolutionSE16Bit,
	.longSampleMode = kADC16_LongSampleCycle24,
	.enableHighSpeed = false,
	.enableLowPower = false,
	.enableContinuousConversion = false
};

static const char *adc_channel_names[BOARD_ADC_NumberOfChannels] =
{
	"Lift-Suction-Current",
	"Lift-Brush-Current",
	"Pump Current",
	"Brush Current",
	"Suction Current",
	"Safety Voltage Sense",
	"Battery Voltage Sense",
	"3.3V-Rail Voltage",
	"5V-Rail Voltage",
	"10V-Rail Voltage",
	"24V-Out Voltage Sense",
	"24V-Out-SW Voltage Sense",
#ifdef CUC_HW_V2
	"24V_A_Key_SNS Voltage Sense",
	"24V_Relay2 Voltage Sense",
#endif
	"Temperature Sensor"
};

void setADC0channel_ctr(unsigned value)
{
	gADC0channel_ctr = value;
}

unsigned getADC0channel_ctr(void)
{
	return gADC0channel_ctr;
}

void setADC1channel_ctr(unsigned value)
{
	gADC1channel_ctr = value;
}

unsigned getADC1channel_ctr(void)
{
	return gADC1channel_ctr;
}

bool getADC0convStatus(void)
{
	return gADC0allConvDone;
}

bool getADC1convStatus(void)
{
	return gADC1allConvDone;
}

void ADC0convStart(void)
{
volatile uint32_t	   							reg;
volatile register ADC_Conversion_Ptr_t		*ptr;
	
	gADC0channel_ctr = 0;
	gADC0allConvDone = false;
	reg = ADC0->R[0];	
	reg = ADC0->R[1];	
	ptr = &(ADC_Conversion_ADC0_Ptr[gADC0channel_ctr]);
	reg = ADC0->SC1[0];
	reg &= ~ADC_SC1_ADCH_MASK;
	reg |= ADC_SC1_ADCH(ptr->adc_channel);
	if (ptr->adc_mux)
		ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
	else
		ADC0->CFG2 &= ~ADC_CFG2_MUXSEL_MASK;
	ADC0->SC1[0] = reg;
#if TRACEALYZER != 0 && TRC_ANA != 0
	vTracePrintF(adc_CH0,"Trigger");
#endif
}

void ADC1convStart(void)
{
volatile uint32_t	   							reg;
volatile register ADC_Conversion_Ptr_t		*ptr;
	
	gADC1channel_ctr = 0;
	gADC1allConvDone = false;
	reg = ADC1->R[0];	
	reg = ADC1->R[1];	
	ptr = &(ADC_Conversion_ADC1_Ptr[gADC1channel_ctr]);
	reg = ADC1->SC1[0];
	reg &= ~ADC_SC1_ADCH_MASK;
	reg |= ADC_SC1_ADCH(ptr->adc_channel);
	if (ptr->adc_mux)
		ADC1->CFG2 |= ADC_CFG2_MUXSEL_MASK;
	else
		ADC1->CFG2 &= ~ADC_CFG2_MUXSEL_MASK;
	ADC1->SC1[0] = reg;
#if TRACEALYZER != 0 && TRC_ANA != 0
	vTracePrintF(adc_CH1,"Trigger");
#endif
}

void ADC0copyChannels(void)
{
int		i;
	
	// TODO Implement Timeout
	for (i = 0;i < ADC0_N_CHANNEL;i++)
	{
		while (ADC0copySema);
		*(ADC_Conversion_ADC0_Ptr[i].copy_value_ptr) = *(ADC_Conversion_ADC0_Ptr[i].value_ptr);
	}
#if TRACEALYZER != 0 && TRC_ANA != 0
	vTracePrintF(adc_CH0,"Copy");
#endif
}

void ADC1copyChannels(void)
{
int		i;
	
	// TODO Implement Timeout
	for (i = 0;i < ADC1_N_CHANNEL;i++)
	{
		while (ADC1copySema);
		*(ADC_Conversion_ADC1_Ptr[i].copy_value_ptr) = *(ADC_Conversion_ADC1_Ptr[i].value_ptr);
	}
#if TRACEALYZER != 0 && TRC_ANA != 0
	vTracePrintF(adc_CH1,"Copy");
#endif
}

const char * BOARD_getADCchannelName(unsigned channel)
{
	if (channel >= BOARD_ADC_NumberOfChannels)
		return NULL;
	return adc_channel_names[channel];
}

void ADC0_IRQHandler(void)
{
volatile uint32_t						status,value;
register ADC_Conversion_Ptr_t		*ptr;

#if TRACEALYZER != 0 && TRC_ANA_ISR != 0
	vTraceStoreISRBegin(ADC0_ISR_Handle); 
#endif
	status = ADC0->SC1[0];
	if ((status & ADC_SC1_COCO_MASK) != 0)
	{
		if (!gADC0allConvDone)
		{
			ptr = (ADC_Conversion_Ptr_t *)&(ADC_Conversion_ADC0_Ptr[gADC0channel_ctr]);
			ADC0copySema = true;
			*(ptr->value_ptr) = value = ADC0->R[0];
			ADC0copySema = false;
#if TRACEALYZER != 0 && TRC_ANA != 0
			vTracePrintF(adc_CH0,"ADCctr %d, Value %d",gADC0channel_ctr,value);
#endif
#if TRACEALYZER != 0 && TRC_ANA_CHANNEL != 0
			vTracePrintF(adc_CH0,"[%d,%d]",gADC0channel_ctr,value);
#endif			
			gADC0channel_ctr++;
			if (gADC0channel_ctr >= ADC0_N_CHANNEL)
			{
				gADC0allConvDone = true;
				gADC0channel_ctr = 0;
			}
			else
			{
				ptr = (ADC_Conversion_Ptr_t *)&(ADC_Conversion_ADC0_Ptr[gADC0channel_ctr]);
				status &= ~ADC_SC1_ADCH_MASK;
				status |= ADC_SC1_ADCH(ptr->adc_channel);
				if (ptr->adc_mux)
					ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
				else
					ADC0->CFG2 &= ~ADC_CFG2_MUXSEL_MASK;
				ADC0->SC1[0] = status;
			}
		}
	}
	else
	{
#if TRACEALYZER != 0 && TRC_ANA != 0
		vTracePrintF(adc_CH0,"Status = %u",status);
#endif
	}
	gCnt0++;
#if TRACEALYZER != 0 && TRC_ANA_ISR != 0
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void ADC1_IRQHandler(void)
{
volatile uint32_t						status,value;
register ADC_Conversion_Ptr_t		*ptr;

#if TRACEALYZER != 0 && TRC_ANA_ISR != 0
	vTraceStoreISRBegin(ADC1_ISR_Handle); 
#endif
	status = ADC1->SC1[0];
	if ((status & ADC_SC1_COCO_MASK) != 0)
	{
		if (!gADC1allConvDone)
		{
			ptr = (ADC_Conversion_Ptr_t *)&(ADC_Conversion_ADC1_Ptr[gADC1channel_ctr]);
			if ((uint32_t)(ptr->value_ptr) < 0x20000)
				return;
			ADC1copySema = true;
			*(ptr->value_ptr) = value = ADC1->R[0];
			ADC1copySema = false;
#if TRACEALYZER != 0 && TRC_ANA != 0
			vTracePrintF(adc_CH1,"ADCctr %d, Value %d",gADC1channel_ctr,value);
#endif
#if TRACEALYZER != 0 && TRC_ANA_CHANNEL != 0
			vTracePrintF(adc_CH1,"[%d,%d]",gADC1channel_ctr,value);
#endif			
			gADC1channel_ctr++;
			if (gADC1channel_ctr >= ADC1_N_CHANNEL)
			{
				gADC1allConvDone = true;
				gADC1channel_ctr = 0;
			}
			else
			{
				ptr = (ADC_Conversion_Ptr_t *)&(ADC_Conversion_ADC1_Ptr[gADC1channel_ctr]);
				status &= ~ADC_SC1_ADCH_MASK;
				status |= ADC_SC1_ADCH(ptr->adc_channel);
				if (ptr->adc_mux)
					ADC1->CFG2 |= ADC_CFG2_MUXSEL_MASK;
				else
					ADC1->CFG2 &= ~ADC_CFG2_MUXSEL_MASK;
				ADC1->SC1[0] = status;
			}
		}
	}
	else
	{
#if TRACEALYZER != 0 && TRC_ANA != 0
		vTracePrintF("adc_CH1","Status = %u",status);
#endif
	}
	gCnt1++;
#if TRACEALYZER != 0 && TRC_ANA_ISR != 0
	vTraceStoreISREnd(0);
#endif
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void BOARD_InitVREF(void)
{
	// init reference
	VREF_Init(VREF,&VrefConfigStruct);
}

static bool BOARD_InitTracealyzer(void)
{
#if TRACEALYZER != 0 && TRC_ANA_ISR != 0
#if ADC0_USED != 0
	ADC0_ISR_Handle = xTraceSetISRProperties("ADC0 ISR", ADC0_INT_PRIORITY);
#endif
#if ADC1_USED != 0
	ADC1_ISR_Handle = xTraceSetISRProperties("ADC1 ISR", ADC1_INT_PRIORITY);
#endif
#endif
#if TRACEALYZER != 0 && (TRC_ANA != 0 || TRC_ANA_CHANNEL)
#if ADC0_USED != 0
	adc_CH0 = xTraceRegisterString("ADC0");
#endif
#if ADC0_USED != 0
	adc_CH1 = xTraceRegisterString("ADC1");
#endif
#endif
	return true;
}

int BOARD_InitADC(void)
{
int      ret = 1;

	for (int i = 0;i < BOARD_ADC_NumberOfChannels;i++)
	{
		ADC_conversion_value[i] = 0;
		ADC_conversion_temp_value[i] = 0;
		ADC_conv_offset_calib[i] = 0;
//		ADC_conversion_value_F[i] = 0.0;
	}
	// update configuration
	ADC16_Init(ADC0,&adc16ConfigStruct);
	ADC16_Init(ADC1,&adc16ConfigStruct);

	// do self calibration
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	if (ADC16_DoAutoCalibration(ADC0) != kStatus_Success)
	{
		ret = 0;
	}
	if (ADC16_DoAutoCalibration(ADC1) != kStatus_Success)
	{
		ret = 0;
	}
#endif
	// set channel mux mode
	ADC16_SetChannelMuxMode(ADC0,kADC16_ChannelMuxA);
	ADC16_SetChannelMuxMode(ADC1,kADC16_ChannelMuxA);

	// update channel config
	for (int i = 0;i < BOARD_ADC_NumberOfChannels;i++)
	{
		ADC16_SetChannelConfig(adc16BoardChannel[i].ADC, BOARD_ADC_ChannelGroup,
                             &(adc16BoardChannel[i].ChannelConfig));
	}
	BOARD_ADC_InitAverager(-1);
	// Select PIT0 as external trigger;
//	SIM->SOPT7 &= ~(SIM_SOPT7_ADC0TRGSEL_MASK | SIM_SOPT7_ADC0TRGSEL_MASK);
//	SIM->SOPT7 |= SIM_SOPT7_ADC0ALTTRGEN_MASK | SIM_SOPT7_ADC0ALTTRGEN_MASK | SIM_SOPT7_ADC0TRGSEL(4);
//	ADC16_EnableHardwareTrigger(ADC0, true);
	ADC16_EnableHardwareTrigger(ADC0, false);
	EnableIRQ(ADC0_IRQn);

//	SIM->SOPT7 &= ~(SIM_SOPT7_ADC1TRGSEL_MASK | SIM_SOPT7_ADC1TRGSEL_MASK);
//	SIM->SOPT7 |= SIM_SOPT7_ADC1ALTTRGEN_MASK | SIM_SOPT7_ADC1ALTTRGEN_MASK | SIM_SOPT7_ADC1TRGSEL(4);
//	ADC16_EnableHardwareTrigger(ADC0, true);
	ADC16_EnableHardwareTrigger(ADC0, false);
	EnableIRQ(ADC1_IRQn);

	BOARD_InitTracealyzer();

   return ret;
}

/*!
 ******************************************************************************
 *	Initializes the hardware Average Function of the ADCs
 * \return        true if success, false else
 ******************************************************************************
*/
bool BOARD_ADC_InitAverager(int samples)
{
adc16_hardware_average_mode_t		nSamples;
	
	if (samples != 0)
	{
		switch (samples)
		{
			case 0:
				nSamples = kADC16_HardwareAverageDisabled;
				break;
			case 4:
				nSamples = kADC16_HardwareAverageCount4;
				break;
			case 8:
				nSamples = kADC16_HardwareAverageCount8;
				break;
			case 16:
				nSamples = kADC16_HardwareAverageCount16;
				break;
			case 32:
				nSamples = kADC16_HardwareAverageCount32;
				break;
			default:
				nSamples = kADC16_HardwareAverageDisabled;
		}
	}
	for (int i = 0;i < BOARD_ADC_NumberOfChannels;i++)
	{
		if (adc16BoardChannel[i].hw_average_mode)
		{
			if (samples < 0)
				ADC16_SetHardwareAverage(adc16BoardChannel[i].ADC,adc16BoardChannel[i].hw_average_mode);
			else
				ADC16_SetHardwareAverage(adc16BoardChannel[i].ADC,nSamples);
		}
	}
	return true;
}

/*!
 ******************************************************************************
 *	Gets the actual value of an ADC channel
 * \param[in]		channel 	Selected channel
 * \param[out]		value 	Actual value of the selected ADC channel
 * \return        true if success, false else
 ******************************************************************************
*/
bool BOARD_get_ADC(uint8_t channel,uint16_t *value)
{

	if (channel >= BOARD_ADC_NumberOfChannels)
		return false;
   *value =  ADC_conversion_value[channel];
   return true;
}

/*!
 ******************************************************************************
 *	Gets the actual value of an ADC channel as a float value
 * \param[in]		channel 	Selected channel
 * \param[out]		value 	Actual value of the selected ADC channel
 * \return        true if success, false else
 ******************************************************************************
*/
bool BOARD_get_ADC_float(uint8_t channel,float *value)
{

	if (channel >= BOARD_ADC_NumberOfChannels)
		return false;
	if (channel == ADC_TEMP_SENSOR_CHANNEL)
	{
		float Vtemp = ADC_conversion_value[channel] * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
		*value = 25.0F - (Vtemp - ADC_VTEMP25) / ADC_VTEMP_SLOPE;
	}
	else
	{
		float		temp = 
		*value =  (ADC_conversion_value[channel] -
				adc16BoardChannel[channel].offset) *
				adc16BoardChannel[channel].gain;
		if (channel == ADC_TEMP_SENSE)
			*value = 25.0F - (*value - ADC_VTEMP25) / ADC_VTEMP_SLOPE;
	}
   return true;
}

/*!
 ******************************************************************************
 *	Gets the ADC clock frequency
 * \param[in]		ADC 		ADC device
 * \return        ADC Clock Frequency in Hz
 ******************************************************************************
*/
uint32_t BOARD_get_ADC_clock(ADC_Type *ADC)
{
uint32_t		clock;
uint32_t		divider;
	
   clock = CLOCK_GetCoreSysClkFreq();
	divider = 1U << ((ADC->CFG1 & ADC_CFG1_ADICLK_MASK) >> ADC_CFG1_ADICLK_SHIFT);
	return clock / divider;
}

/*!
 ******************************************************************************
 *	Calibrates the offset of an ADC channel
 * \param[in]		channel 	Selected channel
 * \return        true if success, false else
 ******************************************************************************
*/
bool BOARD_calib_ADC_channel_offset(uint8_t channel)
{
unsigned		sum;
uint16_t		value;
	
	if (channel >= BOARD_ADC_NumberOfChannels)
		return false;
	sum = 0;
	BOARD_Reset_ADC_Trigger();
	while (!BOARD_getADC_Trigger())
		;
	BOARD_Reset_ADC_Trigger();
	for (int i = 0;i < 100; i++)
	{
		while (!BOARD_getADC_Trigger())
			;
		if (!BOARD_get_ADC(channel,&value))
			return false;
		sum += value;
	}
	ADC_conv_offset_calib[channel] = sum / 100;
	if (adc16BoardChannel[channel].calib_offset)
		adc16BoardChannel[channel].offset = sum / 100.0F;
	return true;
}

/*!
 ******************************************************************************
 *	Gets the offset calibration value of an ADC channel
 * \param[in]		channel 	Selected channel
 * \return        offset if success, 0 else
 ******************************************************************************
*/
unsigned BOARD_getOffsetCalibValue(uint8_t channel)
{
	if (channel >= BOARD_ADC_NumberOfChannels)
		return 0;
	return ADC_conv_offset_calib[channel];
}

/*!
 ******************************************************************************
 *	Gets the value and the offset calibration value of an ADC channel
 * \param[in]		channel 	Selected channel
 * \param[out]		value 	Value
 * \param[out]		offset 	Calibrated offset
 * \return        true if success, false else
 ******************************************************************************
*/
bool BOARD_getValueAndOffset(uint8_t channel,uint16_t *value,uint16_t *offset)
{
	if (channel >= BOARD_ADC_NumberOfChannels)
		return false;
	*offset = ADC_conv_offset_calib[channel];
   *value =  ADC_conversion_value[channel];
	return true;
}
