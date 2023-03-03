/*
 * EEPROM.c
 *
 *  Created on: Apr 21, 2021
 *      Author: martin
 */

#include "EEPROM.h"
#include "board.h"
#include "common.h"
#include "I2C.h"
#include "fsl_i2c.h"

/*!
 *********************************************************************************
 * Initializes the EEPROM
 * \return		1 if no error, else 0
 *********************************************************************************
*/
int EEPROM_Initialize(void)
{
	return(1);
}

/*!
*********************************************************************************
* Wait until the EEPROM has accomplished writing a byte
* \param[in] slave_address I2C-Address of the EEPROM
* \return true if success, else false
*********************************************************************************
*/
static status_t EEPROM_WaitUntilWriteDone(uint8_t slave_address) {
I2C_status_t 	status;

	if ((status = I2C_Bus_InitI2Ctransfer(EEPROM_I2C_Base,slave_address)) != I2C_Stat_Success)
	{
		I2C_Bus_MasterStop(EEPROM_I2C_Base);
		if (status == I2C_Stat_Nak)
			status = I2C_Stat_Addr_Nak;
		return status;
	}
	I2C_Bus_MasterStop(EEPROM_I2C_Base);
	return I2C_Stat_Success;
}

/*!
 *********************************************************************************
 * Writes a byte to the EEPROM
 * \param[in] 	slave_address 	slave address of I2C device
 * \param[in]	address			16Bit address of device register
 * \param[in]	data			data that shall be written
 * \return		true if success, else false
 *********************************************************************************
*/
bool EEPROM_WriteByte(uint8_t slave_address,uint16_t address,uint8_t data)
{
status_t status;
uint64_t time_now;


   volatile int i = 100;
   while(i--);


   if (!I2C_WriteI2Cbyte(EEPROM_I2C_Base_Index,slave_address,address,sizeof(address),data))
	   return false;

   time_now = BOARD_getSystemTimeDirect();
   do {
	   status = EEPROM_WaitUntilWriteDone(slave_address);
	   if (status != I2C_Stat_Success && status != I2C_Stat_Addr_Nak) {
		   return false;
	   }
	   if ((time_now + 10000000ULL) < BOARD_getSystemTimeDirect()) {
		   return false;
	   }
   } while (status != I2C_Stat_Success);

   return true;
}

/*!
 *********************************************************************************
 * Writes a byte block to the EEPROM
 * \param[in] 	slave_address 	slave address of I2C device
 * \param[in]	address			16Bit address of device register
 * \param[in]	data				pointer to the data vector
 * \param[in]	len				length of data vector
 * \return		true if success, else false
 *********************************************************************************
*/
bool EEPROM_WriteBlock(uint8_t slave_address,uint16_t address,uint8_t *data,int len)
{
status_t 	status;
uint64_t 	time_now;
uint16_t		start_addr;

   volatile int i = 100;
   while(i--);

	start_addr = address % EEPROM_PAGE_SIZE;
	while (len > 0)
	{
		if (start_addr + len > EEPROM_PAGE_SIZE)
		{
			if (!I2C_WriteI2Cchannel(EEPROM_I2C_Base_Index,slave_address,start_addr,sizeof(address),data,EEPROM_PAGE_SIZE - start_addr))
			{
				return false;
			}
			start_addr = 0;
			len -= EEPROM_PAGE_SIZE - start_addr;
		}
		else
		{
			if (!I2C_WriteI2Cchannel(EEPROM_I2C_Base_Index,slave_address,start_addr,sizeof(address),data,len))
			{
				return false;
			}
			len = 0; 
		}
		time_now = BOARD_getSystemTimeDirect();
		do {
			status = EEPROM_WaitUntilWriteDone(slave_address);
			if (status != I2C_Stat_Success && status != I2C_Stat_Addr_Nak) {
				return false;
			}
			if ((time_now + 10000000ULL) < BOARD_getSystemTimeDirect()) {
				return false;
			}
		} while (status != I2C_Stat_Success);
	}
   return true;
}

/*!
 *********************************************************************************
 * Reads a byte from the I2C Bus
 * \param[in] 	slave_address 	slave address of I2C device
 * \param[in]	address			16Bit address of device register
 * \param[out]	data			pointer to the data vector
 * \return		true if success, else false
 *********************************************************************************
*/
bool EEPROM_ReadByte(uint8_t slave_address,uint16_t address,uint8_t *data)
{
   volatile int i = 100;
   while(i--);

   return I2C_ReadI2Cbyte(EEPROM_I2C_Base_Index, slave_address, address, sizeof(address), data);
}

/*!
 *********************************************************************************
 * Reads a block from the I2C Bus
 * \param[in] 	slave_address 	slave address of I2C device
 * \param[in]	address			16Bit address of device register
 * \param[out]	data			pointer to the data vector
 * \param[in]	len				length of data vector
 * \return		true if success, else false
 *********************************************************************************
*/
bool EEPROM_ReadBlock(uint8_t slave_address,uint16_t address,uint8_t *data,int len)
{
   volatile int i = 100;
   while(i--);

   return I2C_ReadI2Cchannel(EEPROM_I2C_Base_Index, slave_address,address,sizeof(address),data,len);
}
