/*
 * CAN.h
 *
 *  Created on: Jan 1, 2021
 *      Author: martin
 */

#ifndef DRIVERS_CAN_H_
#define DRIVERS_CAN_H_

#include "fsl_flexcan.h"
#include "board.h"

#define  CAN_NR_IF                     1
#define  CAN_IF_IDENT                  0
#define  CAN_NR_RX_FILTERS             2
#define  CAN_NR_AUX_RX_FILTERS    		4
#define  CAN_NR_RX_FILTERS_TOTAL       (CAN_NR_RX_FILTERS + CAN_NR_AUX_RX_FILTERS)

#define  CAN0_IF_IRQ_PRIO              9
#define  CAN0_IF_ERR_IRQ_PRIO          9
#define  CAN0_BAUDRATE                 125000
#define  CAN0_INDIVIDUAL_MASK          0xFFFFFFFF
#define  CAN0_OWN_ADDRESS              0x00001000

#define  CAN_TX_MAILBOX_INDEX          9
// #define  CAN_RX_MAILBOX_INDEX          10

#define  CAN_RX_DFAULT_MASK            0x3FFFFFFF

#define  CAN_RX_MESSAGE_BUFFER_DEPTH   16

#define  CAN_TX_TIMEOUT                1000
#define  CAN_RX_TIMEOUT                1000

#define CAN0_DEFAULT_EXT_ID            0x100
#define CAN0_RTR                       0
#define CAN0_IDE                       1

#define CAN1_DEFAULT_EXT_ID            0x100
#define CAN1_RTR                       0
#define CAN1_IDE                       1

// CAN Options
#define CAN_OPTIONS_USE_EXT_ID_SHIFT   0
#define CAN_OPTIONS_USE_MASK_SHIFT     1

typedef enum
{
   eCANerr_BusWarn_TX      = (1 << 0),
   eCANerr_BusWarn_RX      = (1 << 1),
   eCANerr_BusOff          = (1 << 2),
   eCANerr_RXfifo_Overflow = (1 << 3),
   eCANerr_RXfifo_Warning  = (1 << 4)
} CANerror_t;

typedef struct
{
   int         CAN_IF_Id;
   CAN_Type    *CAN_IF;
   uint32_t    CAN_Baudrate;
   uint32_t    CAN_ID;
   uint32_t    CAN_ID_RX_mask;
   uint8_t     CAN_HasExtendedID;
   uint8_t     CAN_AcceptRemoteFrame;
   uint8_t     CAN_Use_RX_Mask;
   uint8_t     CAN_N_RX_filters;
	uint8_t		CAN_N_AUX_RX_filters;
   IRQn_Type   CAN_IRQ;
	IRQn_Type	CAN_Error_IRQ;
   uint8_t     CAN_IRQ_Prio;
	uint8_t		CAN_ERR_IRQ_Prio;
   uint32_t    CAN_Error_Status;
   uint32_t    CAN_rxFifoFilter[CAN_NR_RX_FILTERS_TOTAL];
} CANdescriptor_t;

typedef struct
{
   uint32_t    ID;
	bool			isExtID;
   int         len;
   uint8_t     payload[8];
} CANmessage_t;

#define CAN_DESC_ARRAY_SIZE            (CAN_NR_IF * sizeof(CANdescriptor_t))

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

void CAN_RestoreDefaultIFdescriptors(void);
bool CAN_init(unsigned channel);
bool CAN_ChangeBaudrate(unsigned channel,unsigned baudrate);
bool CAN_GetBaudrate(unsigned channel,unsigned *baudrate);
bool CAN_ChangeID(unsigned channel,uint32_t ID,uint8_t type);
bool CAN_GetID(unsigned channel,uint32_t *ID,uint8_t *type);
bool CAN_ChangeAddressMode(unsigned channel,uint8_t isExtended);
bool CAN_ChangeParameters(unsigned channel,unsigned baudrate,uint32_t IDmask,
                         uint32_t ACCmask,uint8_t options);
bool CAN_GetParameters(unsigned channel,unsigned *baudrate,uint32_t *IDmask,
                      uint32_t *ACCmask,uint8_t *options);
void CAN_MessageIRQhandler(unsigned channel);
int CAN_SetRxMessageBuffer(uint32_t address,uint8_t enable);
bool CAN_SendMessage(unsigned channel,uint32_t address,bool IDisExtended,uint8_t *payload,int len);
bool CAN_RequestMessage(unsigned channel,uint32_t address,bool IDisExtended);
int CAN_NumberOfRxMessageAvailable(unsigned channel);
bool CAN_isRxMessageAvailable(unsigned channel);
void CAN_clearRxMessageAvailable(unsigned channel);
bool CAN_getRxMessage(unsigned channel,uint32_t *address,bool *IDisExtended,uint8_t *payload,int *len);
uint32_t CAN_getErrorFlags(unsigned channel,uint8_t clear);
bool CAN_clearErrorFlags(unsigned channel);
bool CAN_SetAcceptanceFilter(unsigned channel,uint32_t mask,uint8_t flag);
bool CAN_GetAcceptanceFilter(unsigned channel,unsigned *ACCmask,uint8_t *flag);
bool CAN_AddMessageBuffer(uint32_t ID,bool useExtendedID,bool isRemoteFrame);
bool CAN_DeleteMessageBuffers(uint8_t ID);
int CAN_getNumberOfAuxMessageBuffers(uint8_t ID);
bool CAN_EnableSelfReception(int ID,bool enable);
void CAN_ErrorIRQhandler(unsigned channel);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* DRIVERS_CAN_H_ */
