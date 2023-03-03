#include "FRAM.h"

/*!
 *********************************************************************************
 * Initializes the FRAM
 * \return		1 if no error, else 0
 *********************************************************************************
*/
int InitializeFRAM(void)
{
	return 1;
}

/*!
 *********************************************************************************
 * Writes a byte vector to the I2C Bus
 * \param[in] 	slave_address 	slave address of I2C device
 * \param[in]	address			16Bit address of device register
 * \param[in]	data				pointer to the data vector
 * \param[in]	len				length of data vector
 * \return		1 if success, else 0
 *********************************************************************************
*/
int FRAM_WriteI2C(uint8_t slave_address,uint16_t address,uint8_t *data,int len)
{
   volatile int i = 100;
   while(i--);

   i2c_master_transfer_t      xfer;
   
   xfer.slaveAddress = slave_address >> 1;
   xfer.direction = kI2C_Write;
   xfer.flags = kI2C_TransferDefaultFlag;
   xfer.subaddress = address;
   xfer.subaddressSize = sizeof(address);
   xfer.data = data;
   xfer.dataSize = len;
   if (I2C_MasterTransferBlocking(FRAM_I2C_Base, &xfer) == kStatus_Success)
      return 1;
   else
      return 0;
}

/*!
 *********************************************************************************
 * Reads a byte vector from the I2C Bus
 * \param[in] 	slave_address 	slave address of I2C device
 * \param[in]	address			16Bit address of device register
 * \param[out]	data				pointer to the data vector
 * \param[in]	len				length of data vector
 * \return		1 if success, else 0
 *********************************************************************************
*/
int FRAM_ReadI2C(uint8_t slave_address,uint16_t address,uint8_t *data,int len)
{
   volatile int i = 100;
   while(i--);
 
   i2c_master_transfer_t      xfer;
   
   xfer.slaveAddress = slave_address >> 1;
   xfer.direction = kI2C_Read;
   xfer.flags = kI2C_TransferDefaultFlag;
   xfer.subaddress = address;
   xfer.subaddressSize = sizeof(address);
   xfer.data = data;
   xfer.dataSize = len;
   if (I2C_MasterTransferBlocking(FRAM_I2C_Base, &xfer) == kStatus_Success)
      return 1;
   else
      return 0;
}

/*!
 *********************************************************************************
 * Writes a buffer to the FRAM
 * \param[in] 	address 		address in FRAM
 * \param[in]	buf			buffer that should be written
 * \param[in]	len			length of buffer in bytes
 * \return		1 if no error, else 0
 *********************************************************************************
*/
int WriteFRAM(uint16_t address,void *buf,int len)
{
	return FRAM_WriteI2C(FRAM_address,address,(uint8_t *)buf,len);
}

/*!
 *********************************************************************************
 * Reads into a buffer to the FRAM
 * \param[in] 	address 		address in FRAM
 * \param[out]	buf			buffer that holds the received data
 * \param[in]	len			length of buffer in bytes
 * \return		1 if no error, else 0
 *********************************************************************************
*/
int ReadFRAM(uint16_t address,void *buf,int len)
{
	return FRAM_ReadI2C(FRAM_address,address,(uint8_t *)buf,len);
}
