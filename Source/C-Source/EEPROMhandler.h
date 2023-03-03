/*
 * EEPROMhandler.h
 *
 *  Created on: Dec 28, 2021
 *      Author: martin
 */

#ifndef EEPROMHANDLER_H_
#define EEPROMHANDLER_H_

#include <stdint.h>
#include <stdbool.h>

#define	EEPROM_MAGIC_WORD					0xA54179F4
#define	EEPROM_MAGIC_WORD_ADDR			0x0000

#define	EEPROM_VERSION_STR_ADDR			0x0004
#define	EEPROM_PARAM_COUNT_ADDR			(EEPROM_VERSION_STR_ADDR + sizeof(EEPROM_VersionEntry_t))
#define	EEPROM_PARAM_START_ADDR			0x0100

#pragma push
#pragma pack(1)

typedef struct
{
	uint32_t		Type;
	uint64_t		Value;
	uint16_t		ID;
	char			Name[16];
	uint16_t		crc;
} EEPROM_ParamEntry_t;

typedef struct
{
	uint8_t		Major;
	uint8_t		Minor;
	uint8_t		Build;
	uint8_t		Reserved;
	char			Serial[16];
	char			Vendor[10];
	uint16_t		crc;
} EEPROM_VersionEntry_t;

typedef union
{
	EEPROM_ParamEntry_t  	Entry;
	uint8_t						ByteVect[32];
} EEPROM_UParamEntry_t;

typedef union
{
	EEPROM_VersionEntry_t  	Entry;
	uint8_t						ByteVect[32];
} EEPROM_UVersionEntry_t;
#pragma pop

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

bool EEPROM_FillVersionStruct(EEPROM_UVersionEntry_t *result,uint8_t *data,int size);
bool EEPROM_FillEntryStruct(EEPROM_UParamEntry_t *result,uint8_t *data,int size);
bool EEPROM_MakeVersionStruct(EEPROM_UVersionEntry_t *result,uint8_t Major,uint8_t Minor,
		uint8_t Build, char * Serial,char *Vendor);
bool EEPROM_MakeParameterStruct(EEPROM_UParamEntry_t *result,uint16_t ID,uint32_t Type,
		uint64_t Value,char *Name);
bool EEPROM_GetVersionStruct(EEPROM_UVersionEntry_t *data,uint8_t *Major,uint8_t *Minor,
		uint8_t *Build, char *Serial,char *Vendor);
bool EEPROM_GetParameterStruct(EEPROM_UParamEntry_t *data,uint16_t *ID,uint32_t *Type,
		uint64_t *Value,char *Name);
bool EEPROM_WriteStruct(uint16_t address,uint8_t *data,unsigned size);
bool EEPROM_ReadStruct(uint16_t address,uint8_t *data,unsigned size);
bool EEPROM_WriteVersionStruct(EEPROM_UVersionEntry_t * VersionStruct);
bool EEPROM_ReadVersionStruct(EEPROM_UVersionEntry_t * VersionStruct);
bool EEPROM_WriteParamStruct(uint16_t address,EEPROM_UParamEntry_t * ParamStruct);
bool EEPROM_ReadParamStruct(uint16_t address,EEPROM_UParamEntry_t * ParamStruct);
bool EEPROM_WriteMagicWord(void);
uint32_t EEPROM_ReadMagicWord(void);
bool EEPROM_CheckMagicWord(void);
bool EEPROM_WriteParamStructCount(uint32_t count);
int EEPROM_ReadParamStructCount(void);
bool EEPROM_ClearParamStructCount(void);
int EEPROM_WriteNextParamStruct(EEPROM_UParamEntry_t * ParamStruct);
bool EEPROM_ReadParamStructAtPos(uint32_t position,EEPROM_UParamEntry_t * ParamStruct);
bool EEPROM_WriteParamStructAtPos(uint32_t position,EEPROM_UParamEntry_t * ParamStruct);
bool EEPROM_InitializeEEPROM(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */
	
#endif // EEPROMHANDLER_H_
