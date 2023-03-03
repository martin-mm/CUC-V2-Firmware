#include "TMP100.h"

static bool TMP100_WriteRegister(uint8_t channel,uint8_t SubAddr,uint8_t *values,unsigned size)
{
uint8_t	SlaveAddr;
	
	switch (channel)
	{
		case 0:
			SlaveAddr = TMP100_1_8BIT_ADDR;
			break;
		case 1:
			SlaveAddr = TMP100_2_8BIT_ADDR;
			break;
		default:
			return false;
	}
	if (SubAddr > 3)
		return false;
	if (size < 1 || size > 2)
		return false;
	return I2C_WriteI2C(SlaveAddr,SubAddr,1,values,size);
}

static bool TMP100_ReadRegister(uint8_t channel,uint8_t SubAddr,uint8_t *result,unsigned size)
{
uint8_t	SlaveAddr;
	
	switch (channel)
	{
		case 0:
			SlaveAddr = TMP100_1_8BIT_ADDR;
			break;
		case 1:
			SlaveAddr = TMP100_2_8BIT_ADDR;
			break;
		default:
			return false;
	}
	if (SubAddr > 3)
		return false;
	if (size < 1 || size > 2)
		return false;
	return I2C_ReadI2C(SlaveAddr,SubAddr,1,result,size);
}

bool TMP100_ReadTempReg(uint8_t channel,int *result)
{
uint8_t		reg_val[2];
	
	if (!TMP100_ReadRegister(channel,TMP100_TEMP_REG,reg_val,sizeof(reg_val)))
		return false;
	*result = (((uint16_t)(reg_val[0])) << 4) + (((uint16_t)(reg_val[1])) >> 4);
	return true;
}

bool TMP100_WriteConfReg(uint8_t channel,uint8_t value)
{
	return TMP100_WriteRegister(channel,TMP100_CONFIG_REG,&value,sizeof(value));
}

bool TMP100_ReadConfReg(uint8_t channel,uint8_t *result)
{
	return TMP100_ReadRegister(channel,TMP100_CONFIG_REG,result,1);
}

bool TMP100_WriteLowLimitTeg(uint8_t channel,uint16_t value)
{
uint8_t	buf[2];
	
	buf[0] = value << 4;
	buf[1] = (value & 0x0F) << 4;
	return TMP100_WriteRegister(channel,TMP100_TLOW_REG,buf,sizeof(buf));
}

bool TMP100_ReadLowLimitTeg(uint8_t channel,uint8_t *result)
{
uint8_t		reg_val[2];
	
	if (!TMP100_ReadRegister(channel,TMP100_TLOW_REG,reg_val,sizeof(reg_val)))
		return false;
	*result = (((uint16_t)(reg_val[0])) << 4) + (((uint16_t)(reg_val[1])) >> 4);
	return true;
}

bool TMP100_WriteHighLimitTeg(uint8_t channel,uint16_t value)
{
uint8_t	buf[2];
	
	buf[0] = value << 4;
	buf[1] = (value & 0x0F) << 4;
	return TMP100_WriteRegister(channel,TMP100_THIGH_REG,buf,sizeof(buf));
}

bool TMP100_ReadHighLimitTeg(uint8_t channel,uint8_t *result)
{
uint8_t		reg_val[2];
	
	if (!TMP100_ReadRegister(channel,TMP100_THIGH_REG,reg_val,sizeof(reg_val)))
		return false;
	*result = (((uint16_t)(reg_val[0])) << 4) + (((uint16_t)(reg_val[1])) >> 4);
	return true;
}

#if USE_FLOAT != 0
bool TMP100_GetTemperature(uint8_t channel,float *result)
{
int		value;
	
	if (!TMP100_ReadTempReg(channel,&value))
		return false;
	*result = (float)value * TEMPERATURE_RESOLUTION;
	return true;
}
#else
bool TMP100_GetTemperature(uint8_t channel,int32_t *result)
{
int32_t	value;
	
	if (!TMP100_ReadTempReg(channel,&value))
		return false;
	*result = (value * TEMPERATURE_RESOLUTION_INT) / 100;		// in 0.1°C
	return true;
}
#endif