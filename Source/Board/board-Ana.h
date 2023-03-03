#ifndef _BOARD_ANA_H_
#define _BOARD_ANA_H_

#include <stdint.h>

#include "fsl_port.h"
#include "fsl_adc16.h"
#include "fsl_vref.h"

#define ADC0_USED							1
#define ADC1_USED							1

#define ADC0_INT_PRIORITY         	9
#define ADC1_INT_PRIORITY         	9


#ifdef CUC_HW_V2
#define BOARD_ADC_NumberOfChannels	15
#define BOARD_ADC_ChannelGroup		0
#define ADC0_N_CHANNEL	            10
#define ADC1_N_CHANNEL	            5
#else
#define BOARD_ADC_NumberOfChannels	13
#define BOARD_ADC_ChannelGroup		0
#define ADC0_N_CHANNEL	            8
#define ADC1_N_CHANNEL	            5
#endif

#ifdef CUC_HW_V2
#define ADC_TEMP_SENSOR_CHANNEL	   14
#else
#define ADC_TEMP_SENSOR_CHANNEL	   12
#endif

#define ADC_REF_VOLTAGE	   			1.2
#define ADC_RESOLUTION_BIT				16
#define ADC_MAX_VALUE					65535.0F
#define ADC_VTEMP25						0.716F
#define ADC_VTEMP_SLOPE					0.00162F

#define ADC_REF_mV						1200
#define ADC_MAX_VAL						65535			// 2^16 - 1

#define ADC_LIFT_SUCT_CUR				0
#define ADC_LIFT_BR_CUR					1
#define ADC_PUMP_CUR						2
#define ADC_BRUSH_CUR					3
#define ADC_SUCT_CUR						4
#define ADC_VB_SAFETY					5
#define ADC_VB_SENSE						6
#define ADC_3V3_RAIL						7
#define ADC_5V_RAIL						8
#define ADC_10V_RAIL						9
#define ADC_VB_OUT						10
#define ADC_VB_SW_OUT					11
#ifdef CUC_HW_V2
#define ADC_VB_24V_KEY					12
#define ADC_VB_24V_RELAY2				13
#define ADC_TEMP_SENSE					14
#else
#define ADC_TEMP_SENSE					12
#endif

#define ADC_CH0			            0x14     // ADC1_DM1		LIFT_SUC_CUR			ADC-IN0
#define ADC_CH1			            0x00     // ADC1_DP0		LIFT_BR_CUR				ADC-IN1
#define ADC_CH2			            0x14     // ADC0_DM1		PUMP_CUR					ADC-IN2
#define ADC_CH3			            0x01     // ADC0_DP1		BRUSH_CUR				ADC-IN3
#define ADC_CH4			            0x13     // ADC0_DM0		SUCT_CUR					ADC-IN4
#define ADC_CH5			            0x16     // ADC0_SE22	VB_24V_SAFETY			ADC-IN5
#define ADC_CH6			            0x15     // ADC0_SE21	VB_SENSE					ADC-IN6
#define ADC_CH7			            0x01     // ADC1_DP1		3V3-Rail					ADC-IN7
#define ADC_CH8			            0x13     // ADC1_DM0		5V-Rail					ADC-IN9
#define ADC_CH9			            0x00     // ADC0_DP0		10V-Rail					ADC-IN8
#define ADC_CH10			            0x0D     // ADC0_SE13	VB_24V_OUT				ADC-IN10
#define ADC_CH11			            0x0C     // ADC0_SE12	VB_24V_SW_OUT			ADC-IN11
#ifdef CUC_HW_V2
#define ADC_CH12			            0x08     // ADC0_SE8		VB_24V_A_KEY_SNS 		ADC-IN13
#define ADC_CH13			            0x09     // ADC0_SE9		VB_24V_RELAY2			ADC-IN12
#endif
#define ADC_TEMP_SENSOR			      0x1A     // Internal Temperature Sensor

#define ADC_CH0_ADC		            ADC1
#define ADC_CH1_ADC		            ADC1
#define ADC_CH2_ADC		            ADC0
#define ADC_CH3_ADC		            ADC0
#define ADC_CH4_ADC		            ADC0
#define ADC_CH5_ADC		            ADC0
#define ADC_CH6_ADC		            ADC0
#define ADC_CH7_ADC		            ADC1
#define ADC_CH8_ADC		            ADC1
#define ADC_CH9_ADC		            ADC0
#define ADC_CH10_ADC		            ADC0
#define ADC_CH11_ADC		            ADC0
#ifdef CUC_HW_V2
#define ADC_CH12_ADC		            ADC0
#define ADC_CH13_ADC		            ADC0
#endif
#define ADCTEMP_SENSOR_ADC		      ADC1

typedef struct {
	uint16_t *value_ptr;
	uint16_t *copy_value_ptr;
	uint8_t adc_channel;
	uint8_t adc_mux;
} ADC_Conversion_Ptr_t;

typedef struct {
	adc16_channel_config_t ChannelConfig;		// Channel Configuration
	ADC_Type 												*ADC;				// Device ADC
	float 													gain;					// gain;
	float 													offset;				// offset
	bool 														calib_offset;		// calibrate offset 
	bool 														calib_gain;			// calibrate gain 
	bool 														use_hw_average;	// use hardware average;
	adc16_hardware_average_mode_t 	hw_average_mode;	// size of hardware average
} adc16_board_channel_t;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

void setADC0channel_ctr(unsigned value);
unsigned getADC0channel_ctr(void);
void setADC1channel_ctr(unsigned value);
unsigned getADC1channel_ctr(void);
bool getADC0convStatus(void);
bool getADC1convStatus(void);
void ADC0convStart(void);
void ADC1convStart(void);
void ADC0copyChannels(void);
void ADC1copyChannels(void);
void BOARD_InitVREF(void);
int BOARD_InitADC(void);
const char * BOARD_getADCchannelName(unsigned channel);
bool BOARD_get_ADC(uint8_t channel,uint16_t *value);
bool BOARD_get_ADC_float(uint8_t channel,float *value);
uint32_t BOARD_get_ADC_clock(ADC_Type *ADC);
bool BOARD_calib_ADC_channel_offset(uint8_t channel);
unsigned BOARD_getOffsetCalibValue(uint8_t channel);
bool BOARD_getValueAndOffset(uint8_t channel,uint16_t *value,uint16_t *offset);
bool BOARD_ADC_InitAverager(int samples);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_ANA_H_ */
