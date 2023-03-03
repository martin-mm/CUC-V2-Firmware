/*
 * EEPROM.h
 *
 *  Created on: Apr 21, 2021
 *      Author: martin
 */

#ifndef HL_DRIVERS_EEPROM_H_
#define HL_DRIVERS_EEPROM_H_

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "fsl_i2c.h"

#define EEPROM_N_DEVICES		1			//!<1 Chip in one Device

#define EEPROM_I2C_Base			I2C1
#define EEPROM_I2C_Base_Index	1
#define EEPROM_MAX_WRITE_TIME	10			//!< EEPROM write cycle last max. 10ms
#define EEPROM_PAGE_SIZE		256		//!< EEPROM page size
#define EERPROM_I2C_BUS			1			//!< I2C Bus of EEPROM

#define EERPROM_I2C_ADDRESS	0xA8		//!< I2C Slave addres of the EEPROM

typedef struct
{
uint32_t	magic_no;
uint32_t	index;
uint32_t	timestamp;
uint32_t	type;
uint8_t	data[16];
} EEPROM_Entry_t;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

int EEPROM_Initialize(void);
bool EEPROM_WriteByte(uint8_t slave_address,uint16_t address,uint8_t data);
bool EEPROM_WriteBlock(uint8_t slave_address,uint16_t address,uint8_t *data,int len);
bool EEPROM_ReadByte(uint8_t slave_address,uint16_t address,uint8_t *data);
bool EEPROM_ReadBlock(uint8_t slave_address,uint16_t address,uint8_t *data,int len);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* HL_DRIVERS_EEPROM_H_ */
