// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        CANDriver.h
//! \brief       Defines the class managing a CAN controller
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _CANDRIVER_H_
#define _CANDRIVER_H_

// ----------------------------------------------------------------------------
// Includes
#include "CANDefs.h"
#include "UCDevice.h"

#define CONTROLLER_CAN1   0
#define CONTROLLER_CAN2   1
#define CONTROLLER_NB     2

#define CAN_RX_FIFO_SIZE  CAN_No_ReceiveObjects

/* CAN message object structure                                              */
typedef struct {
  uint32_t id;                 /* 29 bit identifier                               */
  uint8_t  data[8];            /* Data field                                      */
  uint8_t  len;                /* Length of data field in bytes                   */
  uint8_t  ch;                 /* Object channel                                  */
  uint8_t  format;             /* 0 - STANDARD,   1 - EXTENDED IDENTIFIER         */
  uint8_t  type;               /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg;

/*--------------------------- CAN_rx_stats ----------------------------------
 * These functions return statistics of the message queue
*---------------------------------------------------------------------------*/
uint32_t CAN_rx_stats_lost(uint32_t ctrl);
uint32_t CAN_rx_stats_max_size(uint32_t ctrl);
void CAN_rx_stats_lost_incr(uint32_t ctrl);
void CAN_rx_stats_max_size_update(uint32_t ctrl);
void CAN_rx_stats_reset(uint32_t ctrl);

/*--------------------------- CAN_tx_stats ----------------------------------
 * These functions return statistics of the message queue
*---------------------------------------------------------------------------*/
uint32_t CAN_tx_stats_lost(uint32_t ctrl);
uint32_t CAN_tx_stats_max_size(uint32_t ctrl);
void CAN_tx_stats_lost_incr(uint32_t ctrl);
void CAN_tx_stats_max_size_update(uint32_t ctrl);
void CAN_tx_stats_reset(uint32_t ctrl);

// ----------------------------------------------------------------------------
//! \class      CANDriver
//! \brief      Manage a CAN controller
//! \details    This is a wrapper around the low level CAN driver provided by the RTX operating system.
class CANDriver
{
public:
	CANDriver(EDevice_t _eDevice, uint32_t _nBaudrate);
    //! \cond 
	virtual ~CANDriver() {}
	HideDefaultMethods(CANDriver);
    //! \endcond 

public:
	bool Start();
	void SetTimeout(uint8_t timeout);
	
	bool SetCANid(uint32_t ID,uint8_t type = 0);
		
	bool SendRequest(uint32_t id);
	bool SendMessage(uint32_t id);
	bool SendMessage(uint32_t id, uint8_t data);
	bool SendMessage(uint32_t id, uint16_t data);
	bool SendMessage(uint32_t id, uint32_t data);
	bool SendMessage(uint32_t id, uint32_t data1, uint32_t data2);	

	bool ReadMessage(CAN_msg &msg);
	bool ReadMessage(CAN_msg &msg, uint16_t timeout);

	uint32_t GetRxStatsLost();
	uint32_t GetRxStatsMaxSize();
	void ResetRxStats();

	uint32_t GetTxStatsLost();
	uint32_t GetTxStatsMaxSize();
	void ResetTxStats();

	bool AddCANid(uint32_t ID,bool isExtended = false,bool isRemote = false);

private:
	uint8_t m_nController;
	uint16_t m_Timeout;
};

#endif // _CANDRIVER_H_
