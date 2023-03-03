/*
 * EEPROMhandler.c
 *
 *  Created on: Dec 28, 2021
 *      Author: martin
 */

#include "EEPROMhandler.h"
#include "EEPROM.h"
#include "board.h"
#include "crc.h"
#include "Misc.h"

bool EEPROM_FillVersionStruct(EEPROM_UVersionEntry_t *result,uint8_t *data,int size)
{
uint16_t		crc;
	
	if (size != sizeof(EEPROM_VersionEntry_t))
		return false;
	memcpy(result->ByteVect,data,size);
	CRC_Config(0x1021, 0, 0, 0, 0);
	crc = CRC_Cal_16(0, result->ByteVect, size - 2, 0);
	result->Entry.crc = crc;
	return true;
}

bool EEPROM_FillEntryStruct(EEPROM_UParamEntry_t *result,uint8_t *data,int size)
{
uint16_t		crc;
	
	if (size != sizeof(EEPROM_ParamEntry_t))
		return false;
	memcpy(result->ByteVect,data,size);
	CRC_Config(0x1021, 0, 0, 0, 0);
	crc = CRC_Cal_16(0, result->ByteVect, size - 2, 0);
	result->Entry.crc = crc;
	return true;
}

bool EEPROM_MakeVersionStruct(EEPROM_UVersionEntry_t *result,uint8_t Major,uint8_t Minor,
		uint8_t Build, char * Serial,char *Vendor)
{
uint16_t		crc;

	if (strlen(Serial) > 9)
		return false;
	if (strlen(Vendor) > 15)
		return false;
	strcpy(result->Entry.Serial,Serial);
	strcpy(result->Entry.Vendor,Vendor);
	result->Entry.Major = Major;
	result->Entry.Minor = Minor;
	result->Entry.Build = Build;
	result->Entry.Reserved = 0;
	CRC_Config(0x1021, 0, 0, 0, 0);
	crc = CRC_Cal_16(0, result->ByteVect, sizeof(EEPROM_VersionEntry_t) - 2, 0);
	result->Entry.crc = crc;
	return true;
}

bool EEPROM_MakeParameterStruct(EEPROM_UParamEntry_t *result,uint16_t ID,uint32_t Type,
		uint64_t Value,char *Name)
{
uint16_t		crc;

	if (strlen(Name) > 15)
		return false;
	strcpy(result->Entry.Name,Name);
	result->Entry.ID = ID;
	result->Entry.Type = Type;
	result->Entry.Value = Value;
	CRC_Config(0x1021, 0, 0, 0, 0);
	crc = CRC_Cal_16(0, result->ByteVect, sizeof(EEPROM_VersionEntry_t) - 2, 0);
	result->Entry.crc = crc;
	return true;
}

bool EEPROM_GetVersionStruct(EEPROM_UVersionEntry_t *data,uint8_t *Major,uint8_t *Minor,
		uint8_t *Build, char *Serial,char *Vendor)
{
uint16_t		crc;

	CRC_Config(0x1021, 0, 0, 0, 0);
	crc = CRC_Cal_16(0, data->ByteVect, sizeof(EEPROM_VersionEntry_t) - 2, 0);
	if (crc != data->Entry.crc)
		return false;
	strcpy(Serial,data->Entry.Serial);
	strcpy(Vendor,data->Entry.Vendor);
	*Major = data->Entry.Major;
	*Minor = data->Entry.Minor;
	*Build = data->Entry.Build;
	return true;
}

bool EEPROM_GetParameterStruct(EEPROM_UParamEntry_t *data,uint16_t *ID,uint32_t *Type,
		uint64_t *Value,char *Name)
{
uint16_t		crc;

	CRC_Config(0x1021, 0, 0, 0, 0);
	crc = CRC_Cal_16(0, data->ByteVect, sizeof(EEPROM_VersionEntry_t) - 2, 0);
	if (crc != data->Entry.crc)
		return false;
	strcpy(Name,data->Entry.Name);
	*ID = data->Entry.ID;
	*Type = data->Entry.Type;
	*Value = data->Entry.Value;
	return true;
}

bool EEPROM_WriteStruct(uint16_t address,uint8_t *data,unsigned size)
{
	if (size != 32)
		return false;
	return EEPROM_WriteBlock(EERPROM_I2C_ADDRESS,address,data,size);
}

bool EEPROM_ReadStruct(uint16_t address,uint8_t *data,unsigned size)
{
	if (size != 32)
		return false;
	return EEPROM_ReadBlock(EERPROM_I2C_ADDRESS,address,data,size);
}

bool EEPROM_WriteVersionStruct(EEPROM_UVersionEntry_t * VersionStruct)
{
	return EEPROM_WriteBlock(EERPROM_I2C_ADDRESS,EEPROM_VERSION_STR_ADDR,
		VersionStruct->ByteVect,sizeof(VersionStruct->ByteVect));
}

bool EEPROM_ReadVersionStruct(EEPROM_UVersionEntry_t * VersionStruct)
{
	return EEPROM_ReadBlock(EERPROM_I2C_ADDRESS,EEPROM_VERSION_STR_ADDR,
		VersionStruct->ByteVect,sizeof(VersionStruct->ByteVect));
}

bool EEPROM_WriteParamStruct(uint16_t address,EEPROM_UParamEntry_t * ParamStruct)
{
	return EEPROM_WriteBlock(EERPROM_I2C_ADDRESS,address,
		ParamStruct->ByteVect,sizeof(ParamStruct->ByteVect));
}

bool EEPROM_ReadParamStruct(uint16_t address,EEPROM_UParamEntry_t * ParamStruct)
{
	return EEPROM_ReadBlock(EERPROM_I2C_ADDRESS,address,ParamStruct->ByteVect,
		sizeof(ParamStruct->ByteVect));
}

bool EEPROM_WriteMagicWord(void)
{
uint32_t		MagicWord = EEPROM_MAGIC_WORD;
	
	return EEPROM_WriteBlock(EERPROM_I2C_ADDRESS,EEPROM_MAGIC_WORD_ADDR,
		(uint8_t *)(&MagicWord),sizeof(MagicWord));
}

uint32_t EEPROM_ReadMagicWord(void)
{
uint32_t		MagicWord;
	
	if (EEPROM_ReadBlock(EERPROM_I2C_ADDRESS,EEPROM_MAGIC_WORD_ADDR,
		(uint8_t *)(&MagicWord),sizeof(MagicWord)))
		return MagicWord;
	else
		return 0;
}

bool EEPROM_CheckMagicWord(void)
{
uint32_t		MagicWord;
	
	if (EEPROM_ReadBlock(EERPROM_I2C_ADDRESS,EEPROM_MAGIC_WORD_ADDR,
		(uint8_t *)(&MagicWord),sizeof(MagicWord)))
		return MagicWord == EEPROM_MAGIC_WORD;
	else
		return false;
}

bool EEPROM_WriteParamStructCount(uint32_t count)
{
	return EEPROM_WriteBlock(EERPROM_I2C_ADDRESS,EEPROM_PARAM_COUNT_ADDR,
		(uint8_t *)(&count),sizeof(count));
}

int EEPROM_ReadParamStructCount(void)
{
uint32_t count;
	
	if (EEPROM_ReadBlock(EERPROM_I2C_ADDRESS,EEPROM_PARAM_COUNT_ADDR,
		(uint8_t *)(&count),sizeof(count)))
		return count;
	else
		return -1;
}

bool EEPROM_ClearParamStructCount(void)
{
	return EEPROM_WriteParamStructCount(0);
}

int EEPROM_WriteNextParamStruct(EEPROM_UParamEntry_t * ParamStruct)
{
	int count = EEPROM_ReadParamStructCount();
	if (count == -1)
		return -1;
	uint16_t address = EEPROM_PARAM_START_ADDR + count
		* sizeof(ParamStruct);
	if (!EEPROM_WriteParamStruct(address,ParamStruct))
		return -1;
	count++;
	if (!EEPROM_WriteParamStructCount(count))
		return -1;
	return count;
}

bool EEPROM_ReadParamStructAtPos(uint32_t position,EEPROM_UParamEntry_t * ParamStruct)
{
	int count = EEPROM_ReadParamStructCount();
	if (count == -1)
		return false;
	if (position > count)
		return false;
	uint16_t address = EEPROM_PARAM_START_ADDR + position
		* sizeof(ParamStruct);
	return EEPROM_ReadParamStruct(address,ParamStruct);
}

bool EEPROM_WriteParamStructAtPos(uint32_t position,EEPROM_UParamEntry_t * ParamStruct)
{
	int count = EEPROM_ReadParamStructCount();
	if (count == -1)
		return false;
	if (position > count + 1)
		return false;
	uint16_t address = EEPROM_PARAM_START_ADDR + position
		* sizeof(ParamStruct);
	if (!EEPROM_WriteParamStruct(address,ParamStruct))
		return false;
	if (position > count)
		return EEPROM_WriteParamStructCount(position);
	return true;
}

bool EEPROM_InitializeEEPROM(void)
{
	if (!EEPROM_WriteMagicWord())
		return false;
	if (!EEPROM_CheckMagicWord())
		return false;
	return EEPROM_WriteParamStructCount(0);
}
