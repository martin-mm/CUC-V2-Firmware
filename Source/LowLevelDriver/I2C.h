/*
 * I2C.h
 *
 *  Created on: Jan 1, 2021
 *      Author: martin
 */

#ifndef DRIVERS_I2C_H_
#define DRIVERS_I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "Board.h"

#define I2C0_BAUD_RATE				400000
#define I2C1_BAUD_RATE				400000

#define NUM_I2C_CHANNELS			1

#define I2C_BUS_WAIT_TIMEOUT		1

#define I2C_TIMEOUT_MS				10

#define I2C_USE_SEMAPHORE			0

typedef enum
{
	I2C_Stat_Success = 0,
	I2C_Stat_Busy,            	//!< I2C is busy with current transfer
	I2C_Stat_Idle,            	//!< Bus is Idle
	I2C_Stat_Nak,            	//!< NAK received during transfer
	I2C_Stat_ArbitrationLost, 	//!< Arbitration lost during transfer
	I2C_Stat_Timeout,         	//!< Timeout poling status flags
	I2C_Stat_Addr_Nak,        	//!< NAK received during the address probe
	I2C_Stat_Invalid_Argument 	//!< Invalid argument
} I2C_status_t;

#define I2C_CLEAR_FLAGS 	(I2C_S_ARBL_MASK | I2C_S_IICIF_MASK | ((I2C_FLT_STARTF_MASK | I2C_FLT_STOPF_MASK) << 8))

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 *********************************************************************************
 * Waits until the I2C Bus is free or a timeout occurs
 * \param[in]	base 			I2C Channel
 * \param[in] 	timeout_ms 		Timeout in ms
 * \return		true if success, else false
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_WaitUntilBusFree(I2C_Type *base,uint32_t timeout_ms)
{
#if I2C_BUS_WAIT_TIMEOUT != 0
uint64_t 	now = BOARD_getSystemTimeDirect();
uint64_t	timeout_ns = (uint64_t)timeout_ms * 1000000ULL;

	// Wait until bus free
    while ((base->S & I2C_S_BUSY_MASK) != 0)
    {
    	if (BOARD_getSystemTimeDirect() >= now + timeout_ns)
    		return I2C_Stat_Timeout;
    }
    return I2C_Stat_Success;
#else
    // Wait until bus free
    while (base->S & I2C_S_BUSY_MASK)
    {
    }
    return I2C_Stat_Success;
#endif
}

/*!
 *********************************************************************************
 * Waits until an I2C Bus Transfer has been accomplished or a timeout occurs
 * \param[in]	base 			I2C Channel
 * \param[in] 	timeout_ms 		Timeout in ms
 * \return		true if success, else false
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_WaitUntilTransferComplete(I2C_Type *base,uint32_t timeout_ms)
{
uint32_t i = 0;

#if I2C_BUS_WAIT_TIMEOUT != 0
uint64_t 	now = BOARD_getSystemTimeDirect();
uint64_t		timeout_ns = (uint64_t)timeout_ms * 1000000ULL;

	// Wait until transfer complete
    while ((base->S & I2C_S_TCF_MASK) == 0)
    {
    	if (BOARD_getSystemTimeDirect() >= now + timeout_ns)
    		return I2C_Stat_Timeout;
    }
    return I2C_Stat_Success;
#else
    // Wait until transfer complete
    while ((base->S & I2C_S_TCF_MASK) == 0)
    {
    	i++;
    }
    return I2C_Stat_Success;
#endif
}

/*!
 *********************************************************************************
 * Waits until an I2C Bus Transfer Interrupt occurs
 * \param[in]	base 			I2C Channel
 * \param[in] 	timeout_ms 		Timeout in ms
 * \return		true if success, else false
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_WaitUntilInterruptOccurs(I2C_Type *base,uint32_t timeout_ms)
{

#if I2C_BUS_WAIT_TIMEOUT != 0
uint64_t 	now = BOARD_getSystemTimeDirect();
uint64_t	timeout_ns = (uint64_t)timeout_ms * 1000000ULL;

	// Wait until transfer complete
    while ((base->S & I2C_S_IICIF_MASK) == 0)
    {
    	if (BOARD_getSystemTimeDirect() >= now + timeout_ns)
    		return I2C_Stat_Timeout;
    }
    base->S = I2C_S_IICIF_MASK;		// clear the interrupt
    return I2C_Stat_Success;
#else
    // Wait until interrupt occurs
    while ((base->S & I2C_S_IICIF_MASK) == 0);
    base->S = I2C_S_IICIF_MASK;		// clear the interrupt
    return I2C_Stat_Success;
#endif
}

/*!
 *********************************************************************************
 * Gets I2C Bus Status Register
 * \param[in]	base 			I2C Channel
 * \return		I2C Bus Status Register
 *********************************************************************************
*/
static inline uint16_t I2C_Bus_GetStatusFlags(I2C_Type *base)
{
uint16_t 	status;

	status = base->S;
	status |= (uint16_t)(base->FLT & (I2C_FLT_STARTF_MASK | I2C_FLT_STOPF_MASK)) << 8;
    return status;
}

/*!
 *********************************************************************************
 * Issues an I2C Bus Start Condition
 * \param[in]	base 			I2C Channel
 * \return		0 if success, an error code else
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_MasterStart(I2C_Type *base)
{
    uint16_t statusFlags = I2C_Bus_GetStatusFlags(base);

    // Is the Bus busy?/
    if (statusFlags & I2C_S_BUSY_MASK)
        return I2C_Stat_Busy;
    else
    {
    	// Send a START condition
        base->C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;

    	return I2C_Stat_Success;
	}
}

/*!
 *********************************************************************************
 * Issues an I2C Bus Repeated Start Condition
 * \param[in]	base 			I2C Channel
 * \return		0 if success, an error code else
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_MasterRepeatedStart(I2C_Type *base)
{
uint32_t 	status = I2C_Bus_GetStatusFlags(base);
uint8_t 	timeDelay = 6;

    // Is the Bus busy?/
    if ((status & I2C_S_BUSY_MASK) && ((base->C1 & I2C_C1_MST_MASK) == 0))
        return I2C_Stat_Busy;
    else
    {
    	uint8_t saveReg = base->F;
        base->F = saveReg & ~I2C_F_MULT_MASK;

        // send the repeated START condition
        base->C1 |= I2C_C1_RSTA_MASK | I2C_C1_TX_MASK;

        base->F = saveReg;

        // Add delay for the restart condition to occur
        while (timeDelay--)
        {
            __NOP();
        }
        return I2C_Stat_Success;
    }
}

/*!
 *********************************************************************************
 * Issues an I2C Bus Stop Condition
 * \param[in]	base 			I2C Channel
 * \return		0 if success, an error code else
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_MasterStop(I2C_Type *base)
{
	I2C_status_t status;

    // Send a STOP command
    base->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);

    if ((status = I2C_Bus_WaitUntilBusFree(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
    	return status;
    else
    	return I2C_Stat_Success;
}

/*!
 *********************************************************************************
 * Checks and Clears I2C Bus Error Flags
 * \param[in]	base 			I2C Channel
 * \param[in]	status 			Status Flags
 * \return		0 if success, an error code else
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_CheckClearErrors(I2C_Type *base, uint16_t status)
{
    /* Check arbitration lost. */
    if (status & I2C_S_ARBL_MASK)
    {
        /* Clear arbitration lost flag. */
        base->S = I2C_S_ARBL_MASK;
        return I2C_Stat_ArbitrationLost;
    }
    /* Check NAK */
    else
		if (status & I2C_S_RXAK_MASK)
		{
			return I2C_Stat_Nak;
		}
		else
		{
			return I2C_Stat_Success;
		}
}

/*!
 *********************************************************************************
 * Writes the I2C Bus Slave Address
 * \param[in]	base 			I2C Channel
 * \param[in] 	address 		Slave Address
 * \param[in] 	isReadAccess 	Set to "true" if this is a read access
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_SendAddress(I2C_Type *base,uint8_t address,bool isReadAccess)
{
	I2C_status_t status;

    base->D = address | (isReadAccess ? 1 : 0);

    if ((status = I2C_Bus_WaitUntilInterruptOccurs(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
	{
		I2C_Bus_MasterStop(base);
		return status;
	}
    if ((status = I2C_Bus_CheckClearErrors(base,base->S)) != I2C_Stat_Success)
	{
		if (status == I2C_Stat_Nak)
			I2C_Bus_MasterStop(base);
		return status;
	}
    return I2C_Stat_Success;
}

/*!
 *********************************************************************************
 * Clears I2C Bus Error Flags
 * \param[in]	base 			I2C Channel
 * \param[in]	mask 			Masks the bits that have to be cleared
 *********************************************************************************
*/
static inline void I2C_Bus_ClearFlags(I2C_Type *base, uint16_t mask)
{
    base->FLT |= (mask >> 8) & (I2C_FLT_STARTF_MASK | I2C_FLT_STOPF_MASK);
    base->S = mask & 0xFF;
}

/*!
 *********************************************************************************
 * Sends a SubAddress on the I2C-Bus
 * \param[in]	base 				I2C Channel
 * \param[in]	SubAddress 			SubAddress
 * \param[in]	SubAddressSize 		Size of the SubAddress in Bytes (0 .. 4)
 * \return	I2C_Stat_Success if success, error code else
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_SendSubAddress(I2C_Type *base,uint32_t SubAddress,unsigned SubAddressSize)
{
I2C_status_t		status;

	base->C1 |= I2C_C1_TX_MASK;			// This is an I2C TX data case

	while (SubAddressSize > 0)
	{
		SubAddressSize--;
		/* Clear interrupt pending flag. */
		base->S = kI2C_IntPendingFlag;
		base->D = (SubAddress) >> (SubAddressSize << 3);
		if ((status = I2C_Bus_WaitUntilInterruptOccurs(base,I2C_TIMEOUT_MS)) != I2C_Stat_Success)
		{
			I2C_Bus_MasterStop(base);
			return status;
		}
		if ((status = I2C_Bus_CheckClearErrors(base,base->S)) != I2C_Stat_Success)
		{
			if (status == I2C_Stat_Nak)
				I2C_Bus_MasterStop(base);
			return status;
		}
	}

    base->S = kI2C_IntPendingFlag;

    return I2C_Stat_Success;
}

/*!
 *********************************************************************************
 * Initializes an I2C-Transfer by issuing a START condition on the I2C-Bus and
 * \param[in]	base 				I2C Channel
 * \param[in]	address 			I2C Slave Address
 * \param[in]	SubAddressSize 		Size of the SubAddress in Bytes (0 .. 4)
 * \return	I2C_Stat_Success if success, error code else
 *********************************************************************************
*/
static inline I2C_status_t I2C_Bus_InitI2Ctransfer(I2C_Type *base,uint8_t address)
{
I2C_status_t 	status;

	// Wait until the previous transfer is done
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
	//if ((status = I2C_Bus_CheckClearErrors(base,base->S)) != I2C_Stat_Success)
	//	return status;
	return I2C_Stat_Success;
}

bool I2C_WriteI2C(uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint8_t *data,int len);
bool I2C_ReadI2C(uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint8_t *data,int len);

bool I2C_Bus_Init(void);
I2C_status_t I2C_Bus_MasterWriteBlocking(I2C_Type *base, const uint8_t *TXdata, unsigned TXsize, bool NoStop);
I2C_status_t I2C_Bus_MasterReadBlocking(I2C_Type *base, uint8_t *RXdata, unsigned RXsize, bool NoStop);
I2C_status_t I2C_WriteTransferBlocking(I2C_Type *base,uint8_t address,uint8_t *TXdata,unsigned TXsize);
I2C_status_t I2C_ReadTransferBlocking(I2C_Type *base,uint8_t address,uint8_t *RXdata,unsigned RXsize);
I2C_status_t I2C_ReadWriteTransfer(I2C_Type *base,uint8_t address,uint8_t *TXdata,
		unsigned TXsize,uint8_t *RXdata,unsigned RXsize);
I2C_status_t I2C_WriteTransferSubBlocking(I2C_Type *base,uint8_t address,uint32_t SubAddress,
		unsigned SubAddressSize,uint8_t *TXdata,unsigned TXsize);
I2C_status_t I2C_ReadTransferSubBlocking(I2C_Type *base,uint8_t address,uint32_t SubAddress,
		unsigned SubAddressSize,uint8_t *RXdata,unsigned RXsize);
bool I2C_WriteI2Cchannel(uint8_t channel, uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint8_t *data,int len);
bool I2C_ReadI2Cchannel(uint8_t channel, uint8_t slave_address, uint16_t sub_address,uint16_t sub_size,uint8_t *data,int len);
bool I2C_WriteReadI2C(uint8_t channel, uint8_t slave_address, uint8_t *TXdata,uint16_t TXsize,
		uint8_t *RXdata,uint16_t RXsize);
bool I2C_WriteReadI2C_SMB(uint8_t channel, uint8_t slave_address, uint8_t *TXdata,uint16_t TXsize,
		uint8_t *RXdata,uint16_t *RXsize);
bool I2C_WriteI2Cbyte(uint8_t channel, uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint8_t data);
bool I2C_WriteI2Cword(uint8_t channel, uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint16_t data);
bool I2C_WriteI2Clword(uint8_t channel, uint8_t slave_address,uint16_t sub_address,uint16_t sub_size,uint32_t data);
bool I2C_ReadI2Cbyte(uint8_t channel, uint8_t slave_address, uint16_t sub_address,uint16_t sub_size,uint8_t *data);
bool I2C_ReadI2Cword(uint8_t channel, uint8_t slave_address, uint16_t sub_address,uint16_t sub_size,uint16_t *data);
bool I2C_ReadI2Clword(uint8_t channel, uint8_t slave_address, uint16_t sub_address,uint16_t sub_size,uint32_t *data);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* DRIVERS_I2C_H_ */
