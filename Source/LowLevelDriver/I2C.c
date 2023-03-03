/*
 * I2C.c
 *
 *  Created on: Jan 1, 2021
 *      Author: martin
 */

#include "I2C.h"

#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t	mI2C_Sema = NULL;

/*!
 *********************************************************************************
 * Writes a byte vector to the I2C Bus
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size			subaddress size of device register
 * \param[in]	data				pointer to the data vector
 * \param[in]	len				length of data vector
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_WriteI2C(uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint8_t *data,int len)
{
	return I2C_WriteI2Cchannel(I2C_CHANNEL,slave_address,sub_address,sub_size,data,len);
//   volatile int i = 100;
//   while(i--);

//   i2c_master_transfer_t      xfer;

//   xfer.slaveAddress = slave_address >> 1;
//   xfer.direction = kI2C_Write;
//   xfer.flags = kI2C_TransferDefaultFlag;
//   xfer.subaddress = sub_address;
//   xfer.subaddressSize = sub_size;
//   xfer.data = data;
//   xfer.dataSize = len;
//   if (I2C_MasterTransferBlocking(I2C1, &xfer) == kStatus_Success)
//      return true;
//   else
//      return false;
}


/*!
 *********************************************************************************
 * Reads a byte vector from the I2C Bus
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size			subaddress size of device register
 * \param[out]	data				pointer to the data vector
 * \param[in]	len				length of data vector
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_ReadI2C(uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint8_t *data,int len)
{
	return I2C_ReadI2Cchannel(I2C_CHANNEL,slave_address,sub_address,sub_size,data,len);
//   volatile int i = 100;
//   while(i--);

//   i2c_master_transfer_t      xfer;

//   xfer.slaveAddress = slave_address >> 1;
//   xfer.direction = kI2C_Read;
//   xfer.flags = kI2C_TransferDefaultFlag;
//   xfer.subaddress = sub_address;
//   xfer.subaddressSize = sub_size;
//   xfer.data = data;
//   xfer.dataSize = len;
//   if (I2C_MasterTransferBlocking(I2C1, &xfer) == kStatus_Success)
//      return true;
//   else
//      return false;
}

// -------------------------------------------------------------------------------
// -------------------------------------------------------------------------------

/*!
 *********************************************************************************
 * Initializes the I2C Bus Driver
 * \return		true if success, false else
 *********************************************************************************
*/
bool I2C_Bus_Init(void)
{
	mI2C_Sema = xSemaphoreCreateBinary();
	if (mI2C_Sema != NULL) {
		xSemaphoreGive(mI2C_Sema);
		return true;
	} else {
		return false;
	}
}

/*!
 *********************************************************************************
 * Write blocking to the I2C Bus
 * \param[in]	base 			I2C Channel
 * \param[in]	TXdata 			Data that shall be written
 * \param[in]	TXsize 			Length of the data
 * \param[in]	NoStop 			Set to true if no stop bit shall be issued
 * \return		0 if success, an error code else
 *********************************************************************************
*/
I2C_status_t I2C_Bus_MasterWriteBlocking(I2C_Type *base, const uint8_t *TXdata, unsigned TXsize, bool NoStop)
{
I2C_status_t 	status;

	if ((status = I2C_Bus_WaitUntilTransferComplete(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
		return status;
    base->S = I2C_S_IICIF_MASK;
    base->C1 |= I2C_C1_TX_MASK;			// This is an I2C TX data case

    while (TXsize--)
    {
        base->D = *TXdata++;			// Send a Byte

    	//if ((status = I2C_Bus_WaitUntilTransferComplete(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
    	//	return status;
    	if ((status = I2C_Bus_WaitUntilInterruptOccurs(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
    	    return status;

    	//for (int i = 0; i < 100; i++)
    	//	__NOP();


        // Check for NAK or Arbitration Lost
        if (base->S & I2C_S_ARBL_MASK)
        {
            base->S = I2C_S_ARBL_MASK;
            I2C_Bus_MasterStop(base);
            return I2C_Stat_ArbitrationLost;
        }
        if (base->S & I2C_S_RXAK_MASK)
        {
            base->S = I2C_S_RXAK_MASK;
            I2C_Bus_MasterStop(base);
            return I2C_Stat_Nak;
        }
    }
    base->S = I2C_S_IICIF_MASK;

    if (NoStop)
    	return I2C_Stat_Success;
    else
    	/* Send stop. */
		return I2C_Bus_MasterStop(base);
}

/*!
 *********************************************************************************
 * Read blocking from the I2C Bus
 * \param[in]	base 			I2C Channel
 * \param[out]	RXdata 			Data that has been read
 * \param[in]	RXsize 			Length of the data
 * \param[in]	NoStop 			Set to true if no stop bit shall be issued
 * \return		0 if success, an error code else
 *********************************************************************************
*/
I2C_status_t I2C_Bus_MasterReadBlocking(I2C_Type *base, uint8_t *RXdata, unsigned RXsize, bool NoStop)
{
I2C_status_t 		status;
volatile uint8_t 	dummy;

	if ((status = I2C_Bus_WaitUntilTransferComplete(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
		return status;
    base->S = I2C_S_IICIF_MASK;

    dummy = I2C_C1_TX_MASK;

    base->C1 &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);			// This is an I2C RX data case

    if (RXsize == 1) {				// If RXsize == 1 then the next read access issues a NAK
    	/* Issue NACK on read. */
    	base->C1 |= I2C_C1_TXAK_MASK;
    }



    dummy = base->D;		// Perform a dummy read

    while (RXsize)
    {
    	// Wait for Interrupt of dummy read
    	if ((status = I2C_Bus_WaitUntilInterruptOccurs(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success) {
    		I2C_Bus_MasterStop(base);
    		return status;
    	}
    	base->S = I2C_S_IICIF_MASK;

    	RXsize--;

    	if (RXsize == 0) {
    		/* Change direction to Tx to avoid extra clocks. */
			base->C1 |= I2C_C1_TX_MASK;
	        /* Read from the data register. */
	        //*RXdata = base->D;
    	} else if (RXsize == 1) {
			/* Issue NACK on read. */
			base->C1 |= I2C_C1_TXAK_MASK;
    	}
		/* Read from the data register. */
		*RXdata++ = base->D;
    }

    if (!NoStop) {
		/* Issue STOP command before reading last byte. */
		return I2C_Bus_MasterStop(base);
	}
    return I2C_Stat_Success;
}

/*!
 *********************************************************************************
 * Transfer data to the I2C Bus
 * \param[in]	base 			I2C Channel
 * \param[in]	address 		I2C Slave Address
 * \param[in]	TXdata 			Data that shall be transferred
 * \param[in]	TXsize 			Length of the data
 * \return		0 if success, an error code else
 *********************************************************************************
*/
I2C_status_t I2C_WriteTransferBlocking(I2C_Type *base,uint8_t address,uint8_t *TXdata,unsigned TXsize)
{
I2C_status_t 	status;

	if ((status = I2C_Bus_InitI2Ctransfer(base,address)) != I2C_Stat_Success)
		return status;
    if (TXsize > 0)			// if there is data to send
        return I2C_Bus_MasterWriteBlocking(base,TXdata,TXsize,false);	// Send the Data
    else			// else send a stop condition
    	return I2C_Bus_MasterStop(base);								// Issue a STOP condition
}

/*!
 *********************************************************************************
 * Transfer data from the I2C Bus
 * \param[in]	base 			I2C Channel
 * \param[in]	address 		I2C Slave Address
 * \param[in]	TXdata 			Data that shall be transferred to the I2C Bus
 * \param[in]	TXsize 			Length of the data
 * \param[out]	RXdata 			Data that shall be transferred from the I2C Bus
 * \param[in]	RXsize 			Length of the data
 * \return		0 if success, an error code else
 *********************************************************************************
*/
I2C_status_t I2C_ReadTransferBlocking(I2C_Type *base,uint8_t address,uint8_t *RXdata,unsigned RXsize)
{
I2C_status_t 	status;

	if ((status = I2C_Bus_InitI2Ctransfer(base,address)) != I2C_Stat_Success)
		return status;
    if (RXsize > 0)		// if there is data to receive
        /* Read Data. */
        return I2C_Bus_MasterReadBlocking(base,RXdata,RXsize,false);	// Receive the Data
    else
    	/* STOP condition */
    	return I2C_Bus_MasterStop(base);								// Issue a STOP condition
}

/*!
 *********************************************************************************
 * Transfer data to and from the I2C Bus (write and read after repeated start)
 * \param[in]	base 			I2C Channel
 * \param[in]	address 		I2C Slave Address
 * \param[out]	RXdata 			Data that shall be transfered
 * \param[in]	RXsize 			Length of the data
 * \return		0 if success, an error code else
 *********************************************************************************
*/
I2C_status_t I2C_ReadWriteTransfer(I2C_Type *base,uint8_t address,uint8_t *TXdata,
		unsigned TXsize,uint8_t *RXdata,unsigned RXsize)
{
I2C_status_t 	status;

	if ((status = I2C_Bus_InitI2Ctransfer(base,address)) != I2C_Stat_Success)
		return status;

	if (TXsize > 0)
		// Write data without a STOP condition
		if ((status = I2C_Bus_MasterWriteBlocking(base,TXdata,TXsize,true)) != I2C_Stat_Success)
			return status;
	if ((status = I2C_Bus_MasterRepeatedStart(base)) != I2C_Stat_Success)
		return status;
    if ((status = I2C_Bus_SendAddress(base,address,false)) != I2C_Stat_Success)
    	return status;
    if ((status = I2C_Bus_CheckClearErrors(base,base->S)) != I2C_Stat_Success)
    	return status;
    else
    	return I2C_Bus_MasterReadBlocking(base,RXdata,RXsize,false);
}

/*!
 *********************************************************************************
 * Transfer data to the I2C Bus with subaddress
 * \param[in]	base 			I2C Channel
 * \param[in]	address 		I2C Slave Address
 * \param[in]	SubAddress 		I2C SubAddress
 * \param[in]	SubAddressSize 	size of I2C I2C SubAddress
 * \param[in]	TXdata 			Data that shall be transferred
 * \param[in]	TXsize 			Length of the data
 * \return		0 if success, an error code else
 *********************************************************************************
*/
I2C_status_t I2C_WriteTransferSubBlocking(I2C_Type *base,uint8_t address,uint32_t SubAddress,
		unsigned SubAddressSize,uint8_t *TXdata,unsigned TXsize)
{
I2C_status_t 	status;

	// Wait until the I2C Bus is free
	if ((status = I2C_Bus_WaitUntilTransferComplete(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
		return status;
	// Clear all Error Flags
	I2C_Bus_ClearFlags(base, I2C_CLEAR_FLAGS);
	// Issue a START condition
	if ((status = I2C_Bus_MasterStart(base)) != I2C_Stat_Success)
		return status;
	// Send the I2C Slave Address
	if ((status = I2C_Bus_SendAddress(base,address,false)) != I2C_Stat_Success)
		return status;
	// Check the Error Status
	if ((status = I2C_Bus_CheckClearErrors(base,base->S)) != I2C_Stat_Success)
		return status;

	if ((status = I2C_Bus_SendSubAddress(base,SubAddress,SubAddressSize)) != I2C_Stat_Success)
		return status;

    if (TXsize > 0)
        return I2C_Bus_MasterWriteBlocking(base,TXdata,TXsize,false);
    else
    	return I2C_Bus_MasterStop(base);
}

/*!
 *********************************************************************************
 * Transfer data from the I2C Bus with subaddress
 * \param[in]	base 			I2C Channel
 * \param[in]	address 		I2C Slave Address
 * \param[in]	SubAddress 		I2C SubAddress
 * \param[in]	SubAddressSize 	size of I2C I2C SubAddress
 * \param[out]	RXdata 			Data that shall be transferred
 * \param[in]	RXsize 			Length of the data
 * \return		0 if success, an error code else
 *********************************************************************************
*/
I2C_status_t I2C_ReadTransferSubBlocking(I2C_Type *base,uint8_t address,uint32_t SubAddress,
		unsigned SubAddressSize,uint8_t *RXdata,unsigned RXsize)
{
I2C_status_t 	status;

	if (RXsize < 1)
		return I2C_Stat_Timeout;

	if ((status = I2C_Bus_InitI2Ctransfer(base,address)) != I2C_Stat_Success)
			return status;

	if ((status = I2C_Bus_SendSubAddress(base,SubAddress,SubAddressSize)) != I2C_Stat_Success)
		return status;

    if ((status = I2C_Bus_MasterRepeatedStart(base)) != I2C_Stat_Success)
    	return status;

    if ((status = I2C_Bus_SendAddress(base,address,true)) != I2C_Stat_Success)
    	return status;

    if (RXsize > 0) {
    	return I2C_Bus_MasterReadBlocking(base,RXdata,RXsize,false);
    } else {
    	return I2C_Bus_MasterStop(base);
    }
}

/*!
 *********************************************************************************
 * Writes a byte vector to the I2C Bus
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size			subaddress size of device register
 * \param[in]	data				pointer to the data vector
 * \param[in]	len				length of data vector
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_WriteI2Cchannel(uint8_t channel, uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint8_t *data,int len)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
   		case 0:
   			port = I2C0;
   			break;
   		case 1:
   			port = I2C1;
   			break;
   		default:
   			return false;
	}

	while(i--);

	result = I2C_WriteTransferSubBlocking(port,slave_address,sub_address,sub_size,data,len) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif

	return result;
}

/*!
 *********************************************************************************
 * Reads a byte vector from the I2C Bus
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size			subaddress size of device register
 * \param[out]	data				pointer to the data vector
 * \param[in]	len				length of data vector
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_ReadI2Cchannel(uint8_t channel, uint8_t slave_address, uint16_t sub_address,uint16_t sub_size,uint8_t *data,int len)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
		case 0:
			port = I2C0;
			break;
		case 1:
			port = I2C1;
			break;
		default:
			return false;
	}

	while(i--);

	result =  I2C_ReadTransferSubBlocking(port,slave_address,sub_address,sub_size,data,len) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
   if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif

	return result;
}

/*!
 *********************************************************************************
 * Reads a byte vector from the I2C Bus after writing a vector
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	TXdata			data to write
 * \param[in]	TXsize			size of the data to write
 * \param[out]	RXdata			data that has been read
 * \param[in]	RXsize			Size of the vector to read
 * \return		true if success, else false
 *********************************************************************************
*/
bool I2C_WriteReadI2C(uint8_t channel, uint8_t slave_address, uint8_t *TXdata,uint16_t TXsize,
		uint8_t *RXdata,uint16_t RXsize)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
		case 0:
			port = I2C0;
			break;
		case 1:
			port = I2C1;
			break;
		default:
			return false;
	}

	while(i--);

	result =  I2C_ReadWriteTransfer(port,slave_address,TXdata,TXsize,RXdata,RXsize) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
  	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif
	return result;
}

/*!
 *********************************************************************************
 * For SMBus transfer data has to be written and the first byte read after
 * write transmission is accomplished gives the number of bytes to read
 * \param[in]		channel 		I2C Channel (0 or 1)
 * \param[in] 		slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]		TXdata			data to write
 * \param[in]		TXsize			size of the data to write
 * \param[out]		RXdata			data that has been read
 * \param[in,out]	RXsize			Size of the vector to read and returns read data size
 * \return			true if success, else false
 *********************************************************************************
*/
bool I2C_WriteReadI2C_SMB(uint8_t channel, uint8_t slave_address, uint8_t *TXdata,uint16_t TXsize,
		uint8_t *RXdata,uint16_t *RXsize)
{
uint8_t 				size;
I2C_status_t 		status;
I2C_Type* 			base;
volatile uint8_t	dummy;
volatile int 		i = 100;
bool					result,semaStatus;

 	semaStatus = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && (mI2C_Sema != NULL);
#if I2C_USE_SEMAPHORE
	if (semaStatus)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
		case 0:
			base = I2C0;
			break;
		case 1:
			base = I2C1;
			break;
		default:
			return false;
	}

	while(i--);


	if ((status = I2C_Bus_InitI2Ctransfer(base,slave_address)) != I2C_Stat_Success)
	{
#if I2C_USE_SEMAPHORE
	  	if (semaStatus)
		{
			xSemaphoreGive(mI2C_Sema);
		}
#endif
		return false;
	}
	if (TXsize > 0)			// if there is data to send
	{
		// Send the Data without STOP condition
		if ((status = I2C_Bus_MasterWriteBlocking(base,TXdata,TXsize,true)) != I2C_Stat_Success)
		{
#if I2C_USE_SEMAPHORE
		  	if (semaStatus)
			{
				xSemaphoreGive(mI2C_Sema);
			}
#endif
			return false;
		}
	}
	if ((status = I2C_Bus_MasterRepeatedStart(base)) != I2C_Stat_Success)
	{
#if I2C_USE_SEMAPHORE
	  	if (semaStatus)
		{
			xSemaphoreGive(mI2C_Sema);
		}
#endif
		return false;
	}
	if ((status = I2C_Bus_SendAddress(base,slave_address+1,false)) != I2C_Stat_Success)
	{
#if I2C_USE_SEMAPHORE
	  	if (semaStatus)
		{
			xSemaphoreGive(mI2C_Sema);
		}
#endif
		return false;
	}

	if ((status = I2C_Bus_CheckClearErrors(base,base->S)) != I2C_Stat_Success)
	{
#if I2C_USE_SEMAPHORE
	  	if (semaStatus)
		{
			xSemaphoreGive(mI2C_Sema);
		}
#endif
		return false;
	}

	if ((status = I2C_Bus_WaitUntilTransferComplete(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
	{
#if I2C_USE_SEMAPHORE
	  	if (semaStatus)
		{
			xSemaphoreGive(mI2C_Sema);
		}
#endif
		return false;
	}

	base->S = I2C_S_IICIF_MASK;

	base->C1 &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);			// This is an I2C RX data case

	if (*RXsize == 0)	// If RXsize == 1 then the next read access issues a NAK
       /* Issue NACK on read. */
		base->C1 |= I2C_C1_TXAK_MASK;

	dummy = base->D;		// Perform a dummy read
	if ((status = I2C_Bus_WaitUntilInterruptOccurs(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
	{
  		I2C_Bus_MasterStop(base);
#if I2C_USE_SEMAPHORE
	  	if (semaStatus)
		{
			xSemaphoreGive(mI2C_Sema);
		}
#endif
		return false;
	}
	base->S = I2C_S_IICIF_MASK;
	size = base->D;
	if (size > *RXsize)
	{
		I2C_Bus_MasterStop(base);
#if I2C_USE_SEMAPHORE
	  	if (semaStatus)
		{
			xSemaphoreGive(mI2C_Sema);
		}
#endif
		return false;
	}
	*RXsize = size;
	if (size == 1)
	{
		/* Issue NACK on read. */
		base->C1 |= I2C_C1_TXAK_MASK;
	}

	while (size)
	{
		// Wait for Interrupt of 2nd read
		if ((status = I2C_Bus_WaitUntilInterruptOccurs(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
		{
			I2C_Bus_MasterStop(base);
#if I2C_USE_SEMAPHORE
		  	if (semaStatus)
			{
				xSemaphoreGive(mI2C_Sema);
			}
#endif
			return false;
		}
		base->S = I2C_S_IICIF_MASK;

		size--;

		if (size == 0) {
			/* Change direction to Tx to avoid extra clocks. */
			base->C1 |= I2C_C1_TX_MASK;
			/* Read from the data register. */
			//*RXdata = base->D;
		} else if (size == 1) {
			/* Issue NACK on read. */
			base->C1 |= I2C_C1_TXAK_MASK;
		}
		/* Read from the data register. */
		*RXdata++ = base->D;
	}

	result = I2C_Bus_MasterStop(base) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
	if (semaStatus)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif
	
	return result;
}

/*!
 *********************************************************************************
 * Writes a byte to the I2C Bus
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size		subaddress size of device register
 * \param[in]	data			byte to write
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_WriteI2Cbyte(uint8_t channel, uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint8_t data)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
   		case 0:
   			port = I2C0;
   			break;
   		case 1:
   			port = I2C1;
   			break;
   		default:
   			return false;
	}

	while(i--);

	result = I2C_WriteTransferSubBlocking(port,slave_address,sub_address,sub_size,&data,1) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif
	
	return result;
}

/*!
 *********************************************************************************
 * Writes a word to the I2C Bus
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size		subaddress size of device register
 * \param[in]	data			word to write
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_WriteI2Cword(uint8_t channel, uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint16_t data)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;
uint8_t			*ptr = (uint8_t *)(&data);

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
		case 0:
			port = I2C0;
			break;
		case 1:
			port = I2C1;
			break;
		default:
			return false;
	}

	while(i--);

	result = I2C_WriteTransferSubBlocking(port,slave_address,sub_address,sub_size,ptr,2) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
 	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif

	return result;
}

/*!
 *********************************************************************************
 * Writes a long word to the I2C Bus
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size		subaddress size of device register
 * \param[in]	data			word to write
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_WriteI2Clword(uint8_t channel, uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint32_t data)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;
uint8_t			*ptr = (uint8_t *)(&data);

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
   		case 0:
   			port = I2C0;
   			break;
   		case 1:
   			port = I2C1;
   			break;
   		default:
   			return false;
	}

	while(i--);

	result = I2C_WriteTransferSubBlocking(port,slave_address,sub_address,sub_size,ptr,4) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
 	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif

	return result;
}

/*!
 *********************************************************************************
 * Reads a byte from the I2C Bus
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size		subaddress size of device register
 * \param[out]	data			pointer to the data that has been read
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_ReadI2Cbyte(uint8_t channel, uint8_t slave_address, uint16_t sub_address,uint16_t sub_size,uint8_t *data)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
		case 0:
			port = I2C0;
			break;
		case 1:
			port = I2C1;
			break;
		default:
			return false;
	}

	while(i--);

	result = I2C_ReadTransferSubBlocking(port,slave_address,sub_address,sub_size,data,1) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
 	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif

	return result;
}

/*!
 *********************************************************************************
 * Reads a word from the I2C Bus
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size		subaddress size of device register
 * \param[out]	data			pointer to the data that has been read
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_ReadI2Cword(uint8_t channel, uint8_t slave_address, uint16_t sub_address,uint16_t sub_size,uint16_t *data)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;
uint8_t			*ptr = (uint8_t *)(&data);

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
		case 0:
			port = I2C0;
			break;
		case 1:
			port = I2C1;
			break;
		default:
			return false;
	}

	while(i--);

	result = I2C_ReadTransferSubBlocking(port,slave_address,sub_address,sub_size,ptr,2) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
 	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif
	
	return result;
}

/*!
 *********************************************************************************
 * Reads a long word from the I2C Bus
 * \param[in]	channel 		I2C Channel (0 or 1)
 * \param[in] 	slave_address 	slave address of I2C device (8Bit-Address)
 * \param[in]	sub_address		subaddress of device register
 * \param[in]	sub_size		subaddress size of device register
 * \param[out]	data			pointer to the data that has been read
 * \return		1 if success, else 0
 *********************************************************************************
*/
bool I2C_ReadI2Clword(uint8_t channel, uint8_t slave_address, uint16_t sub_address,uint16_t sub_size,uint32_t *data)
{
I2C_Type		*port;
volatile int 	i = 100;
bool			result;
uint8_t			*ptr = (uint8_t *)(&data);

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		if (xSemaphoreTake(mI2C_Sema,10 / portTICK_PERIOD_MS) != pdTRUE)
			return false;
	}
#endif
	switch (channel) {
		case 0:
			port = I2C0;
			break;
		case 1:
			port = I2C1;
			break;
		default:
			return false;
	}

	while(i--);

	result = I2C_ReadTransferSubBlocking(port,slave_address,sub_address,sub_size,ptr,4) == I2C_Stat_Success;

#if I2C_USE_SEMAPHORE
	if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && mI2C_Sema != NULL)
	{
		xSemaphoreGive(mI2C_Sema);
	}
#endif
	
	return result;
}
