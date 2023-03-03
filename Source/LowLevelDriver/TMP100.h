#ifndef _TMP100_H_
#define _TMP100_H_

#include "board.h"
#include "I2C.h"

#define N_TMP100							2
#define TMP100_1_8BIT_ADDR				0x90
#define TMP100_2_8BIT_ADDR				0x94

#define TMP100_PTR_REG_ADDR			0

#define TMP100_TEMP_REG					0
#define TMP100_CONFIG_REG				1
#define TMP100_TLOW_REG					2
#define TMP100_THIGH_REG				3

#define TEMPERATURE_RESOLUTION		0.0625F
#define TEMPERATURE_RESOLUTION_INT	625

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

bool TMP100_ReadTempReg(uint8_t channel,int *result);
bool TMP100_WriteConfReg(uint8_t channel,uint8_t value);
bool TMP100_ReadConfReg(uint8_t channel,uint8_t *result);
bool TMP100_WriteLowLimitTeg(uint8_t channel,uint16_t value);
bool TMP100_ReadLowLimitTeg(uint8_t channel,uint8_t *result);
bool TMP100_WriteHighLimitTeg(uint8_t channel,uint16_t value);
bool TMP100_ReadHighLimitTeg(uint8_t channel,uint8_t *result);
bool TMP100_ReadTempReg(uint8_t channel,int *result);
#if USE_FLOAT != 0
bool TMP100_GetTemperature(uint8_t channel,float *result);
#else
bool TMP100_GetTemperature(uint8_t channel,int32_t *result);
#endif
	
#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _TMP100_H_ */
