/*
 * CAN.c
 *
 *  Created on: Jan 1, 2021
 *      Author: martin
 */

#include "CAN.h"
#include "Misc.h"

#if (TRACEALYZER != 0) && (TRC_CAN != 0)
#include "trcRecorder.h"

static traceString 				trcCAN = NULL;
#endif

#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)

flexcan_config_t           CAN0_Config;

static CANmessage_t        CAN_RXbuffer[CAN_NR_IF][CAN_RX_MESSAGE_BUFFER_DEPTH];
static volatile uint32_t   CAN_Ptr_In_RX[CAN_NR_IF] = {0};
static volatile uint32_t   CAN_Ptr_Out_RX[CAN_NR_IF] = {0};
static volatile uint8_t    CAN_RX_buffer_full[CAN_NR_IF] = {0};

const CANdescriptor_t      CAN_DefaultDescriptor[CAN_NR_IF] = {
                              {
                                 .CAN_IF_Id = 0,
                                 .CAN_IF = CAN0,
                                 .CAN_Baudrate = 125000,
                                 .CAN_ID = 0x100,
                                 .CAN_ID_RX_mask = CAN_RX_DFAULT_MASK,
                                 .CAN_HasExtendedID = 0,
                                 .CAN_AcceptRemoteFrame = 1,
                                 .CAN_Use_RX_Mask = 1,
                                 .CAN_N_RX_filters = 1,
                                 .CAN_N_AUX_RX_filters = 0,
                                 .CAN_IRQ = CAN0_ORed_Message_buffer_IRQn,
											.CAN_Error_IRQ = CAN0_Error_IRQn,
                                 .CAN_IRQ_Prio = CAN0_IF_IRQ_PRIO,
                                 .CAN_ERR_IRQ_Prio = CAN0_IF_ERR_IRQ_PRIO,
                                 .CAN_Error_Status = 0
                              }
                           };

static CANdescriptor_t   	CAN_Descriptor[CAN_NR_IF] = {
                              {
                                 .CAN_IF_Id = 0,
                                 .CAN_IF = CAN0,
                                 .CAN_Baudrate = 125000,
                                 .CAN_ID = 0x100,
                                 .CAN_ID_RX_mask = CAN_RX_DFAULT_MASK,
                                 .CAN_HasExtendedID = 0,
                                 .CAN_AcceptRemoteFrame = 1,
                                 .CAN_Use_RX_Mask = 1,
                                 .CAN_N_RX_filters = 2,
                                 .CAN_N_AUX_RX_filters = 0,
                                 .CAN_IRQ = CAN0_ORed_Message_buffer_IRQn,
											.CAN_Error_IRQ = CAN0_Error_IRQn,
                                 .CAN_IRQ_Prio = CAN0_IF_IRQ_PRIO,
                                 .CAN_ERR_IRQ_Prio = CAN0_IF_ERR_IRQ_PRIO,
                                 .CAN_Error_Status = 0
                              }
                           };

// static flexcan_mb_transfer_t        txXfer, rxXfer;
static volatile bool                txComplete = false;
static volatile bool                rxComplete = false;
// static flexcan_frame_t              frame;
// static uint32_t                     txIdentifier;
// static uint32_t                     rxIdentifier;
static unsigned 							mCAN_Error = 0;

/*!
 ******************************************************************************
 *	Restores the default CAN descriptors
 ******************************************************************************
*/
void CAN_RestoreDefaultIFdescriptors(void)
{
   memcpy(CAN_Descriptor,CAN_DefaultDescriptor,CAN_DESC_ARRAY_SIZE);
}

/*!
 ******************************************************************************
 *	Gets the CAN Interface Pointer from the Interface ID
 * \param[in]     ID       Interface ID
 * \return        Interface Pointer if success, NULL else
 ******************************************************************************
*/
static CAN_Type *CAN_GetIfPtr(unsigned ID)
{
   switch (ID)
   {
      case CAN_IF_IDENT:
         return CAN0;
      default:
         return NULL;
   }
}

/*!
 ******************************************************************************
 *	Gets the CAN Interface Descriptor from the Interface ID
 * \param[in]     ID       Interface ID
 * \return        Interface Pointer if success, NULL else
 ******************************************************************************
*/
static CANdescriptor_t *CAN_GetDescriptorFromID(unsigned ID)
{
	if (ID > 0)
		return NULL;
	return &(CAN_Descriptor[ID]);
}

/*!
 ******************************************************************************
 *	Enters the CAN Interface Freeze Mode
 * \param[in]     CanIF     CAN Interface Pointer
 * \return        Interface Pointer if success, NULL else
 ******************************************************************************
*/
static void CAN_EnterFreezeMode(CAN_Type *CanIF)
{
   CanIF->MCR |= CAN_MCR_HALT_MASK;
   while (!(CanIF->MCR & CAN_MCR_FRZACK_MASK))
   {
   }
}

/*!
 ******************************************************************************
 *	Exits the CAN Interface Freeze Mode
 * \param[in]     CanIF     CAN Interface Pointer
 * \return        Interface Pointer if success, NULL else
 ******************************************************************************
*/
static void CAN_ExitFreezeMode(CAN_Type *CanIF)
{
	CanIF->MCR &= ~CAN_MCR_HALT_MASK;
   while (CanIF->MCR & CAN_MCR_FRZACK_MASK)
   {
   }
}

/*!
 ******************************************************************************
 *	Enables or Disables CAN Interface Self Reception
 * \param[in]     CanIF     CAN Interface Pointer
 * \return        Interface Pointer if success, NULL else
 ******************************************************************************
*/
static void CAN_SetSelfReception(CAN_Type *CanIF,bool enable)
{
	CAN_EnterFreezeMode(CanIF);
	if (enable)
		CanIF->MCR &= ~CAN_MCR_SRXDIS_MASK;
	else
		CanIF->MCR |= CAN_MCR_SRXDIS_MASK;
	CAN_EnterFreezeMode(CanIF);
}

/*!
 ******************************************************************************
 *	Internal CAN Initialize Routine
 * \param[in]     channel     CAN channel
 * \param[in]     enable      if set (!= 0) enables the CAN interface IRQ
 ******************************************************************************
*/
static bool CAN_Initialize(unsigned channel,uint8_t enable)
{
flexcan_config_t           CAN_Config;
flexcan_rx_fifo_config_t   CAN_FIFO_Config;
CANdescriptor_t            *ptr;
uint32_t                   mask;

   if (channel >= CAN_NR_IF)
      return false;
   ptr = &(CAN_Descriptor[channel]);
   ptr->CAN_N_RX_filters = 1;
   if (ptr->CAN_HasExtendedID)
   {
      ptr->CAN_rxFifoFilter[0] = FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(
            ptr->CAN_ID,0,1);
      if (ptr->CAN_AcceptRemoteFrame)
      {
         ptr->CAN_rxFifoFilter[1] = FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(
               ptr->CAN_ID + 1,1,1);
         ptr->CAN_N_RX_filters++;
      }
   }
   else
   {
//    CAN_rxFifoFilter[channel][0] = FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(
//             ptr->CAN_ID,ptr->CAN_AcceptRemoteFrame,0);
      ptr->CAN_rxFifoFilter[0] = FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(
            ptr->CAN_ID,0,1);
      if (ptr->CAN_AcceptRemoteFrame)
      {
         ptr->CAN_rxFifoFilter[1] = FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(
               ptr->CAN_ID,1,0);
         ptr->CAN_N_RX_filters++;
      }
   }

   FLEXCAN_GetDefaultConfig(&CAN_Config);
   CAN_Config.clkSrc = kFLEXCAN_ClkSrcPeri;
   CAN_Config.baudRate = ptr->CAN_Baudrate;
	CAN_Config.timingConfig.preDivider = 24 - 1;
	CAN_Config.timingConfig.propSeg = 2;
	CAN_Config.timingConfig.phaseSeg1 = 4;
	CAN_Config.timingConfig.phaseSeg2 = 6;
	CAN_Config.timingConfig.rJumpwidth = 2;
   CAN_Config.maxMbNum = 16;
   CAN_Config.enableIndividMask = true;
   CAN_Config.enableLoopBack = false;
   CAN_Config.enableSelfWakeup = false;
   FLEXCAN_Init(ptr->CAN_IF,&CAN_Config,CLOCK_GetBusClkFreq());
	CAN_SetSelfReception(ptr->CAN_IF,false);
   CAN_FIFO_Config.idFilterTable = ptr->CAN_rxFifoFilter;
   CAN_FIFO_Config.idFilterType  = kFLEXCAN_RxFifoFilterTypeA;
   CAN_FIFO_Config.idFilterNum   = ptr->CAN_N_RX_filters;
   CAN_FIFO_Config.priority      = kFLEXCAN_RxFifoPrioHigh;
   if (ptr->CAN_Use_RX_Mask)
   {
      if (ptr->CAN_HasExtendedID)
      {
         mask = (ptr->CAN_ID_RX_mask << 1) & 0x3FFFFFFF;
         mask |= (1 << 30);   // Enable Extended ID reception;
      }
      else
      {
         mask = (ptr->CAN_ID_RX_mask << 19) & 0x3FFFFFFF;
      }
      if (ptr->CAN_AcceptRemoteFrame)
         mask |= (1u << 31);
   }
   else
      mask = 0x3FFFFFFF;
   for (int i = 0;i < ptr->CAN_N_RX_filters;i++)
      FLEXCAN_SetRxIndividualMask(ptr->CAN_IF,i,mask);
   FLEXCAN_SetRxFifoConfig(ptr->CAN_IF,&CAN_FIFO_Config,true);
   if (enable)
   {
      NVIC_SetPriority(ptr->CAN_IRQ,ptr->CAN_IRQ_Prio);
      NVIC_EnableIRQ(ptr->CAN_IRQ);
      NVIC_SetPriority(ptr->CAN_Error_IRQ,ptr->CAN_ERR_IRQ_Prio);
      NVIC_EnableIRQ(ptr->CAN_Error_IRQ);
      FLEXCAN_EnableMbInterrupts(ptr->CAN_IF,kFLEXCAN_RxFifoFrameAvlFlag);
		FLEXCAN_EnableInterrupts(ptr->CAN_IF,kFLEXCAN_ErrorInterruptEnable);
      FLEXCAN_Enable(ptr->CAN_IF,true);
   }
   FLEXCAN_ClearMbStatusFlags(ptr->CAN_IF,kFLEXCAN_RxFifoFrameAvlFlag);
   return true;
}

/*!
 ******************************************************************************
 *	CAN Initialize Routine
 * \param[in]     channel     CAN channel
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_init(unsigned channel)
{
#if TRACEALYZER != 0 && TRC_CANNODE != 0
	if (trcCAN != NULL)
		trcCAN = xTraceRegisterString("CAN LL");
#endif

   switch (channel)
   {
      case 0:
#ifndef  BOARD_HW_CAN0
         return false;
#else
#ifdef BOARD_CAN0_IRQ_ENA
         return CAN_Initialize(CAN_IF_IDENT,1);
#else
         return CAN_Initialize(CAN_IF_IDENT,0);
#endif
#endif
      case 1:
#ifndef  BOARD_HW_CAN1
         return false;
#else
#ifdef BOARD_CAN1_IRQ_ENA
         return CAN_Initialize(1,1);
#else
         return CAN_Initialize(1,0);
#endif
#endif
      default:
         return false;
   }
//   FLEXCAN_EnableInterrupts(CAN_IF0,kFLEXCAN_BusOffInterruptEnable |
}

/*!
 ******************************************************************************
 *	Adds a CAN Message Buffer Filter
 * change and then reenabled.
 * \param[in]     ID     				CAN ID
 * \param[in]     useExtendedID    	if true this is an extended ID
 * \param[in]     isRemoteFrame    	if true this is a rempote frame
 * \return        true if success, false else
 ******************************************************************************
*/
bool CAN_AddMessageBuffer(uint32_t ID,bool useExtendedID,bool isRemoteFrame)
{
flexcan_rx_fifo_config_t   CAN_FIFO_Config;
CANdescriptor_t            *ptr;
int								k;

   ptr = &(CAN_Descriptor[CAN_IF_IDENT]);
	if ((k = ptr->CAN_N_AUX_RX_filters) >= CAN_NR_AUX_RX_FILTERS)
		return false;
	k += CAN_NR_RX_FILTERS;
   if (useExtendedID)
   {
      if (isRemoteFrame)
      {
         ptr->CAN_rxFifoFilter[k] = FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(
               ID,1,1);
      }
		else
		{
			ptr->CAN_rxFifoFilter[k] = FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(
					ID,0,1);
		}
   }
   else
   {
      if (isRemoteFrame)
      {
         ptr->CAN_rxFifoFilter[k] = FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(
               ID,1,0);
      }
		else
		{
			ptr->CAN_rxFifoFilter[k] = FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(
					ID,0,0);
		}
   }
   ptr->CAN_N_AUX_RX_filters++;
   CAN_FIFO_Config.idFilterTable = ptr->CAN_rxFifoFilter;
   CAN_FIFO_Config.idFilterType  = kFLEXCAN_RxFifoFilterTypeA;
   CAN_FIFO_Config.idFilterNum   = ptr->CAN_N_AUX_RX_filters + CAN_NR_RX_FILTERS;
   CAN_FIFO_Config.priority      = kFLEXCAN_RxFifoPrioHigh;
   FLEXCAN_SetRxFifoConfig(ptr->CAN_IF,&CAN_FIFO_Config,true);
	return true;
}

/*!
 ******************************************************************************
 *	Deletes all auxilliary CAN Message Buffers
 * change and then reenabled.
 * \param[in]     ID     				CAN ID
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_DeleteMessageBuffers(uint8_t ID)
{
flexcan_rx_fifo_config_t   CAN_FIFO_Config;
CANdescriptor_t            *ptr;

   ptr = &(CAN_Descriptor[CAN_IF_IDENT]);
   ptr->CAN_N_AUX_RX_filters = 0;
   CAN_FIFO_Config.idFilterTable = ptr->CAN_rxFifoFilter;
   CAN_FIFO_Config.idFilterType  = kFLEXCAN_RxFifoFilterTypeA;
   CAN_FIFO_Config.idFilterNum   = ptr->CAN_N_AUX_RX_filters + CAN_NR_RX_FILTERS;
   CAN_FIFO_Config.priority      = kFLEXCAN_RxFifoPrioHigh;
   FLEXCAN_SetRxFifoConfig(ptr->CAN_IF,&CAN_FIFO_Config,true);
	return true;
}

/*!
 ******************************************************************************
 *	Gets the number of used auxilliary CAN Message Buffers
 * \param[in]     ID     				CAN ID
 * \return        1 if success, 0 else
 ******************************************************************************
*/
int CAN_getNumberOfAuxMessageBuffers(uint8_t ID)
{
	return CAN_Descriptor[CAN_IF_IDENT].CAN_N_AUX_RX_filters;
}

/*!
 ******************************************************************************
 *	Changes the CAN Baudrate. The CAN interface will be disabled for this
 * change and then reenabled.
 * \param[in]     channel     CAN channel
 * \param[in]     baudrate    CAN baudrate
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_ChangeBaudrate(unsigned channel,unsigned baudrate)
{
   if (channel >= CAN_NR_IF)
      return false;
   CAN_Descriptor[channel].CAN_Baudrate = baudrate;
   return CAN_init(channel);
}

/*!
 ******************************************************************************
 *	Gets the CAN Baudrate.
 * change and then reenabled.
 * \param[in]     channel     CAN channel
 * \param[in]     baudrate    CAN baudrate
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_GetBaudrate(unsigned channel,unsigned *baudrate)
{
   if (channel >= CAN_NR_IF)
      return false;
   *baudrate = CAN_Descriptor[channel].CAN_Baudrate;
   return true;
}

/*!
 ******************************************************************************
 *	Changes the CAN ID. The CAN interface will be disabled for this
 * change and then reenabled.
 * \param[in]     channel     CAN channel
 * \param[in]     IDmask      CAN ID mask
 * \param[in]     type        CAN ID type (0 = standard, 1 = extended
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_ChangeID(unsigned channel,uint32_t ID,uint8_t type)
{
   if (channel >= CAN_NR_IF)
      return false;
   if (type)
      CAN_Descriptor[channel].CAN_HasExtendedID = 1;
   else
      CAN_Descriptor[channel].CAN_HasExtendedID = 0;
   CAN_Descriptor[channel].CAN_ID = ID;
   return CAN_init(channel);
}

/*!
 ******************************************************************************
 *	Gets the CAN ID.
 * change and then reenabled.
 * \param[in]     channel     CAN channel
 * \param[out]    ID      		CAN ID mask
 * \param[out]    type        CAN ID type (0 = standard, 1 = extended
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_GetID(unsigned channel,uint32_t *ID,uint8_t *type)
{
   if (channel >= CAN_NR_IF)
      return false;
	*type = CAN_Descriptor[channel].CAN_HasExtendedID;
	*ID = CAN_Descriptor[channel].CAN_ID;
   return true;
}

/*!
 ******************************************************************************
 *	Changes the CAN Address Mode. The CAN interface will be disabled for this
 * change and then reenabled.
 * \param[in]     channel     CAN channel
 * \param[in]     isExtended  if 0 the CAN Bus uses a standard ID, else an extended
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_ChangeAddressMode(unsigned channel,uint8_t isExtended)
{
   if (channel >= CAN_NR_IF)
      return false;
   if (isExtended)
      CAN_Descriptor[channel].CAN_HasExtendedID = 1;
   else
      CAN_Descriptor[channel].CAN_HasExtendedID = 0;
   return CAN_init(channel);
}

/*!
 ******************************************************************************
 *	Changes the CAN Parameters. The CAN interface will be disabled for this
 * change and then reenabled.
 * CAN Options:   Bit 0:   Extended ID if 1, Standard else
 *                Bit 1:   Use CAN Acceptance mask if 1
 * \param[in]     channel     CAN channel
 * \param[in]     baudrate    CAN baudrate
 * \param[in]     IDmask      CAN ID mask
 * \param[in]     ACCmask     CAN Acceptance Mask
 * \param[in]     options     CAN Options
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_ChangeParameters(unsigned channel,unsigned baudrate,uint32_t IDmask,
                         uint32_t ACCmask,uint8_t options)
{
   if (channel >= CAN_NR_IF)
      return false;
   CAN_Descriptor[channel].CAN_Baudrate = baudrate;
   CAN_Descriptor[channel].CAN_ID = IDmask;
   CAN_Descriptor[channel].CAN_ID_RX_mask = ACCmask;
   if ((options & (1 << CAN_OPTIONS_USE_EXT_ID_SHIFT)) != 0)
      CAN_Descriptor[channel].CAN_HasExtendedID = 1;
   else
      CAN_Descriptor[channel].CAN_HasExtendedID = 0;
   if ((options & (1 << CAN_OPTIONS_USE_MASK_SHIFT)) != 0)
      CAN_Descriptor[channel].CAN_Use_RX_Mask = 1;
   else
      CAN_Descriptor[channel].CAN_Use_RX_Mask = 0;
   return CAN_init(channel);
}

/*!
 ******************************************************************************
 *	Gets the CAN Parameters. The CAN interface will be disabled for this
 * change and then reenabled.
 * CAN Options:   Bit 0:   Extended ID if 1, Standard else
 *                Bit 1:   Use CAN Acceptance mask if 1
 * \param[in]     channel     CAN channel
 * \param[out]    baudrate    CAN baudrate
 * \param[out]    IDmask      CAN ID mask
 * \param[in]     ACCmask     CAN Acceptance Mask
 * \param[in]     options     CAN Options
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_GetParameters(unsigned channel,unsigned *baudrate,uint32_t *IDmask,
                      uint32_t *ACCmask,uint8_t *options)
{
   if (channel >= CAN_NR_IF)
      return false;
   *baudrate = CAN_Descriptor[channel].CAN_Baudrate;
   *IDmask = CAN_Descriptor[channel].CAN_ID;
   *ACCmask = CAN_Descriptor[channel].CAN_ID_RX_mask;
   if (CAN_Descriptor[channel].CAN_HasExtendedID)
      *options = 1 << CAN_OPTIONS_USE_EXT_ID_SHIFT;
   else
      *options = 0;
   if (CAN_Descriptor[channel].CAN_Use_RX_Mask)
      *options |= 1 << CAN_OPTIONS_USE_MASK_SHIFT;
   return true;
}

/*!
 ******************************************************************************
 *	Sends a CAN Message
 * \param[in]     channel     	CAN channel
 * \param[in]     address     	CAN ID
 * \param[in]     IDisExtended  	Set (true) if the CAN ID is an extended ID
 * \param[in]     payload     	CAN Payload
 * \param[in]     len         	CAN Payload length
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_SendMessage(unsigned channel,uint32_t address,bool IDisExtended,uint8_t *payload,int len)
{
flexcan_frame_t      txFrame;
int                  i;
uint64_t             timer;
CAN_Type             *CAN_IF;

   if ((CAN_IF = CAN_GetIfPtr(channel)) == NULL)
      return false;
#if TRACEALYZER != 0 && TRC_CAN != 0
	vTracePrintF(trcCAN,"Send CAN Message, Addr = %08X",address);
#endif
   FLEXCAN_SetTxMbConfig(CAN_IF,CAN_TX_MAILBOX_INDEX,true);
   if (CAN_Descriptor[channel].CAN_HasExtendedID && IDisExtended)
   {
      txFrame.id = FLEXCAN_ID_EXT(address);
      txFrame.format = kFLEXCAN_FrameFormatExtend;
   }
   else
   {
      txFrame.id = FLEXCAN_ID_STD(address);
      txFrame.format = kFLEXCAN_FrameFormatStandard;
   }
   txFrame.length = len;
	txFrame.type = kFLEXCAN_FrameTypeData;
   for (i = 0;i < len;i++)
   {
      switch (i)
      {
         case 0:
            txFrame.dataByte0 = payload[0];
            break;
         case 1:
            txFrame.dataByte1 = payload[1];
            break;
         case 2:
            txFrame.dataByte2 = payload[2];
            break;
         case 3:
            txFrame.dataByte3 = payload[3];
            break;
         case 4:
            txFrame.dataByte4 = payload[4];
            break;
         case 5:
            txFrame.dataByte5 = payload[5];
            break;
         case 6:
            txFrame.dataByte6 = payload[6];
            break;
         case 7:
            txFrame.dataByte7 = payload[7];
            break;
      }
   }
   FLEXCAN_WriteTxMb(CAN_IF,CAN_TX_MAILBOX_INDEX,&txFrame);
   timer = Now();
   while (!FLEXCAN_GetMbStatusFlags(CAN_IF,1 << CAN_TX_MAILBOX_INDEX))
   {
      if (DiffTime(timer) > CAN_TX_TIMEOUT)
      {
         // Abort sending the CAN message
         FLEXCAN_SetTxMbConfig(CAN_IF,CAN_TX_MAILBOX_INDEX,true);
         return false;
      }
      Sleep(2);
   }
	FLEXCAN_ClearMbStatusFlags(CAN_IF,1 << CAN_TX_MAILBOX_INDEX);
	Sleep(2);
//   FLEXCAN_ClearMbStatusFlags(CAN_IF,kFLEXCAN_RxFifoFrameAvlFlag);
   return true;
}

/*!
 ******************************************************************************
 *	Sends a CAN Remote Frame
 * \param[in]     channel     	CAN channel
 * \param[in]     address     	CAN ID
 * \param[in]     IDisExtended  	Set (true) if the CAN ID is an extended ID
 * \param[in]     payload     	CAN Payload
 * \param[in]     len         	CAN Payload length
 * \return        1 if success, 0 else
 ******************************************************************************
*/
bool CAN_RequestMessage(unsigned channel,uint32_t address,bool IDisExtended)
{
flexcan_frame_t      txFrame;
uint64_t             timer;
CAN_Type             *CAN_IF;

   if ((CAN_IF = CAN_GetIfPtr(channel)) == NULL)
      return false;
#if TRACEALYZER != 0 && TRC_CAN != 0
	vTracePrintF(trcCAN,"Request CAN Message, Addr = %08X",address);
#endif
   FLEXCAN_SetTxMbConfig(CAN_IF,CAN_TX_MAILBOX_INDEX,true);
   if (CAN_Descriptor[channel].CAN_HasExtendedID && IDisExtended)
   {
      txFrame.id = FLEXCAN_ID_EXT(address);
      txFrame.format = kFLEXCAN_FrameFormatExtend;
   }
   else
   {
      txFrame.id = FLEXCAN_ID_STD(address);
      txFrame.format = kFLEXCAN_FrameFormatStandard;
   }
   txFrame.length = 0;
	txFrame.type = kFLEXCAN_FrameTypeRemote;
   FLEXCAN_WriteTxMb(CAN_IF,CAN_TX_MAILBOX_INDEX,&txFrame);
   timer = Now();
   while (!FLEXCAN_GetMbStatusFlags(CAN_IF,1 << CAN_TX_MAILBOX_INDEX))
   {
      if (DiffTime(timer) > CAN_TX_TIMEOUT)
      {
         // Abort sending the CAN message
         FLEXCAN_SetTxMbConfig(CAN_IF,CAN_TX_MAILBOX_INDEX,true);
         return false;
      }
      Sleep(2);
   }
//   FLEXCAN_ClearMbStatusFlags(CAN_IF,kFLEXCAN_RxFifoFrameAvlFlag);
   return true;
}

static void CAN_MessageReceivedHandler(unsigned channel,CAN_Type *CAN_IF)
{
flexcan_frame_t   rxFrame;
CANmessage_t      *ptr;

   if (FLEXCAN_ReadRxFifo(CAN_IF,&rxFrame) != kStatus_Success)
      return;
#if TRACEALYZER != 0 && TRC_CAN != 0
	vTracePrint(trcCAN,"Request CAN Message RX Handler");
#endif
   ptr = &(CAN_RXbuffer[channel][CAN_Ptr_In_RX[channel]]);
   ptr->len = rxFrame.length;
   if (rxFrame.format == kFLEXCAN_FrameFormatExtend)
	{
		ptr->isExtID = true;
      ptr->ID = rxFrame.id & 0x1FFFFFFF;
	}
   else
	{
		ptr->isExtID = false;
      ptr->ID = (rxFrame.id >> 18) & 0x7FF;
	}
   if (!CAN_RX_buffer_full[channel])
   {
      for (int i = 0;i < rxFrame.length;i++)
      {
         switch (i)
         {
            case 0:
               ptr->payload[0] = rxFrame.dataByte0;
               break;
            case 1:
               ptr->payload[1] = rxFrame.dataByte1;
               break;
            case 2:
               ptr->payload[2] = rxFrame.dataByte2;
               break;
            case 3:
               ptr->payload[3] = rxFrame.dataByte3;
               break;
            case 4:
               ptr->payload[4] = rxFrame.dataByte4;
               break;
            case 5:
               ptr->payload[5] = rxFrame.dataByte5;
               break;
            case 6:
               ptr->payload[6] = rxFrame.dataByte6;
               break;
            case 7:
               ptr->payload[7] = rxFrame.dataByte7;
               break;
         }
      }
      CAN_Ptr_In_RX[channel]++;
      if (CAN_Ptr_In_RX[channel] >= CAN_RX_MESSAGE_BUFFER_DEPTH)
         CAN_Ptr_In_RX[channel] = 0;
      if (CAN_Ptr_In_RX[channel] == CAN_Ptr_Out_RX[channel])
         CAN_RX_buffer_full[channel] = 1;
      else
         CAN_RX_buffer_full[channel] = 0;
   }
}

void CAN_ErrorIRQhandler(unsigned channel)
{
CAN_Type    *CAN_IF;

   if ((CAN_IF = CAN_GetIfPtr(channel)) != NULL)
   {
		mCAN_Error = FLEXCAN_GetStatusFlags(CAN_IF);
		FLEXCAN_ClearStatusFlags(CAN_IF,CAN_ESR1_ERRINT_MASK);
	}
}

void CAN_MessageIRQhandler(unsigned channel)
{
uint32_t    StatusFlags;
CAN_Type    *CAN_IF;

   if ((CAN_IF = CAN_GetIfPtr(channel)) != NULL)
	{
		StatusFlags = FLEXCAN_GetMbStatusFlags(CAN_IF,0xFFFFFFFF);
//   	FLEXCAN_ClearMbStatusFlags(CAN_IF,0xFFFFFFFF);
		if ((StatusFlags & kFLEXCAN_RxFifoWarningFlag) != 0)
			CAN_Descriptor[channel].CAN_Error_Status |= eCANerr_RXfifo_Warning;
		if ((StatusFlags & kFLEXCAN_RxFifoOverflowFlag) != 0)
			CAN_Descriptor[channel].CAN_Error_Status |= eCANerr_RXfifo_Overflow;
		if ((StatusFlags & kFLEXCAN_RxFifoFrameAvlFlag) != 0)
			CAN_MessageReceivedHandler(channel,CAN_IF);
		FLEXCAN_ClearMbStatusFlags(CAN_IF,
			kFLEXCAN_RxFifoWarningFlag |
			kFLEXCAN_RxFifoOverflowFlag |
			kFLEXCAN_RxFifoFrameAvlFlag);
	}
}

uint32_t CAN_getErrorFlags(unsigned channel,uint8_t clear)
{
uint32_t    err_flags;

   if (channel >= CAN_NR_IF)
      return 0;
   err_flags = CAN_Descriptor[channel].CAN_Error_Status;
   if (clear != 0)
      CAN_Descriptor[channel].CAN_Error_Status = 0;
   return err_flags;
}

bool CAN_clearErrorFlags(unsigned channel)
{
   if (channel >= CAN_NR_IF)
      return false;
   CAN_Descriptor[channel].CAN_Error_Status = 0;
   return true;
}

bool CAN_isRxMessageAvailable(unsigned channel)
{
   if (channel >= CAN_NR_IF)
      return false;
   if (CAN_Ptr_In_RX[channel] == CAN_Ptr_Out_RX[channel] && !CAN_RX_buffer_full[channel])
      return false;
   else
      return true;
}

int CAN_NumberOfRxMessageAvailable(unsigned channel)
{
   if (channel >= CAN_NR_IF)
      return 0;
   int n = CAN_Ptr_In_RX[channel] - CAN_Ptr_Out_RX[channel];
   if (n < 0)
      n += CAN_RX_MESSAGE_BUFFER_DEPTH;
   return n;
}

void CAN_clearRxMessageBufferQueue(unsigned channel)
{
   CAN_Ptr_In_RX[channel] = 0;
   CAN_Ptr_Out_RX[channel] = 0;
   CAN_RX_buffer_full[channel]= 0;
}

bool CAN_getRxMessage(unsigned channel,uint32_t *address,bool *IDisExtended,uint8_t *payload,int *len)
{
#if TRACEALYZER != 0 && TRC_CAN != 0
	vTracePrint(trcCAN,"Get CAN RX Message");
#endif
   if (!CAN_isRxMessageAvailable(channel))
      return false;
#if TRACEALYZER != 0 && TRC_CAN != 0
	vTracePrint(trcCAN,"Get CAN RX Message -  Available");
#endif
   *address = CAN_RXbuffer[channel][CAN_Ptr_Out_RX[channel]].ID;
   *len = CAN_RXbuffer[channel][CAN_Ptr_Out_RX[channel]].len;
   for (int i = 0;i < CAN_RXbuffer[channel][CAN_Ptr_Out_RX[channel]].len;i++)
      payload[i] = CAN_RXbuffer[channel][CAN_Ptr_Out_RX[channel]].payload[i];
   CAN_RX_buffer_full[channel] = 0;
   (CAN_Ptr_Out_RX[channel])++;
   if (CAN_Ptr_Out_RX[channel] >= CAN_RX_MESSAGE_BUFFER_DEPTH)
      CAN_Ptr_Out_RX[channel] = 0;
   return true;
}

bool CAN_SetAcceptanceFilter(unsigned channel,uint32_t ACCmask,uint8_t flag)
{
CANdescriptor_t            *ptr;

   if (channel >= CAN_NR_IF)
      return false;
   ptr = &(CAN_Descriptor[channel]);
   ptr->CAN_ID_RX_mask = ACCmask;
   ptr->CAN_Use_RX_Mask = flag;
   if (ptr->CAN_Use_RX_Mask)
      ptr->CAN_ID_RX_mask = ACCmask;
   else
      ptr->CAN_ID_RX_mask = 0x3FFFFFFF;
   for (int i = 0;i < ptr->CAN_N_RX_filters;i++)
   {
      if (ptr->CAN_Use_RX_Mask)
         FLEXCAN_SetRxIndividualMask(ptr->CAN_IF,i,ACCmask);
      else
         FLEXCAN_SetRxIndividualMask(ptr->CAN_IF,i,0x3FFFFFFF);
   }
   return true;
}

bool CAN_GetAcceptanceFilter(unsigned channel,unsigned *ACCmask,uint8_t *flag)
{
CANdescriptor_t            *ptr;

   if (channel >= CAN_NR_IF)
      return false;
   ptr = &(CAN_Descriptor[channel]);
   *ACCmask = ptr->CAN_ID_RX_mask;
   *flag = ptr->CAN_Use_RX_Mask;
   return true;
}

/*!
 ******************************************************************************
 *	Enables or Disables CAN Interface Self Reception
 * \param[in]     ID  	CAN Interface ID
 * \return        Interface Pointer if success, NULL else
 ******************************************************************************
*/
bool CAN_EnableSelfReception(int ID,bool enable)
{
	CAN_Type *ptr = CAN_GetIfPtr(ID);
	if (ptr == NULL)
		return false;
	CAN_SetSelfReception(ptr,enable);
	return true;
}

