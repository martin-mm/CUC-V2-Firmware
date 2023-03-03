// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        CANNode.cpp
//! \brief       Encapsulate the functionality of a CAN node
//! \details     <UL><LI>Handle NMT request</LI>
//!                  <LI>Handle SDO requests to write a value into the object dictionary</LI><
//!                  <LI>Handle SDO requests to read  a value from the object dictionary</LI></UL>
//!              
//!              \par Each CAN accessible object has a different object index
//!              \par Each CAN accessible object must derive from ICANNodeDataProvider
//!              \par Each CAN accessible object must register itself to CANNode
//!              \par CAN data are	exchanged via instances of ProcessData
//!              \par When starting CANNode call the RegisterData method of each registered object to let them create associated ProcessData instances
//!              \par When a CAN message modifies a CAN data the OnNewDataAvailable method of the corresponding object is called
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "CANNode.h"
#include "ProcessData.h"
#include "board.h"

static CANNode 	*CAN_Node_Ptr = nullptr;
static bool			registered = false;

static const char	osCANTaskName[] = "CAN_NODE";

#if TRACEALYZER != 0 && TRC_CANNODE
static traceString 				trcCANNode;
#endif

// ----------------------------------------------------------------------------
// Constants
#define CAN_TASK_PERIOD	10

// ----------------------------------------------------------------------------
//! \brief Constructor
CANNode::CANNode(CANDriver &_driver, uint8_t _nCANId, osPriority_t _nPriority)
	: CUC_Task(_nPriority,osCANTaskName),
	  m_Driver(_driver)
{
	dbgprintf("CAN Node Constructor, CAN ID = %d, Priority = %d ...\n",_nCANId,(int)_nPriority);	
	m_nCANId = _nCANId;
	m_nNMTToggleBit = 0;
	m_nNMTState = ENMTState_Initialising;

	for (int i=0; i<CANNODE_MAX_OBJECTS; i++)
		m_aDataProviders[i] = NULL;

	m_ProcessDataMutex = xSemaphoreCreateMutex();
	nCanReceived = 0;
	nCanTransmitted = 0;
	if (!registered)
	{
		CAN_Node_Ptr = this;
		registered = true;
	}
#if TRACEALYZER != 0 && TRC_CANNODE != 0
	trcCANNode = xTraceRegisterString("CAN NODE");
#endif
	dbgprintf("... CAN Node Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Register a provider of CAN data along with its object index
bool CANNode::RegisterDataProvider(ICANNodeDataProvider *_pProvider, uint16_t _nObjIndex)
{
	if (_pProvider == NULL)
		return false;
	if (_nObjIndex == 0)
		return false;
	if (_nObjIndex >= CANNODE_MAX_OBJECTS)
		return false;
	if (NULL != m_aDataProviders[_nObjIndex])
		return false;
	m_aDataProviders[_nObjIndex] = _pProvider;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Used by data provider to declare CAN data
template<class T> bool CANNode::DeclareData(uint16_t _nObjIndex, uint8_t _nSubIndex, ProcessDataOut<T>* &_pProcessData)
{
	if (_nObjIndex >= CANNODE_MAX_OBJECTS || _nSubIndex >= CANNDOE_MAX_SUBINDEXES)
	{
		_pProcessData = NULL;
		return false;
	}
	_pProcessData = new ProcessDataOut<T>((uint8_t*)&(m_aData[_nObjIndex][_nSubIndex]), (uint8_t)sizeof(T), m_ProcessDataMutex);
	return (NULL != _pProcessData);
}

// ----------------------------------------------------------------------------
//! \brief Get the CAN ID
uint32_t CANNode::getCAN_ID(void)
{
	return m_nCANId;
}

// ----------------------------------------------------------------------------
//! \brief Get the NMT State
uint32_t CANNode::getNMT_State(void)
{
	return m_nNMTState;
}

// ----------------------------------------------------------------------------
//! \brief Get the number of registered Providers
int CANNode::getNumberOfProvider(void)
{
int	n = 0;
	
	for (int i = 0;i < CANNODE_MAX_OBJECTS;i++)
	{
		if (m_aDataProviders[i] != nullptr)
			n++;
	}
	return n;
}

// ----------------------------------------------------------------------------
//! \brief Checks if a Provider is registered
bool CANNode::checkProviderRegistered(unsigned id)
{
	if (id >= CANNODE_MAX_OBJECTS)
		return false;
	return m_aDataProviders[id] != nullptr;
}

// ----------------------------------------------------------------------------
//! \brief Gets a Provider's Content
bool CANNode::getProviderContent(unsigned id,unsigned subid,uint32_t &content,bool &valid)
{
	if (id >= CANNODE_MAX_OBJECTS)
		return false;
	if (subid >= CANNDOE_MAX_SUBINDEXES)
		return false;
	valid = m_aDataProviders[id] != nullptr;
	if (valid)
		content = m_aData[id][subid];
	else
		content = 0;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Task listening to the CAN bus
void CANNode::Main()
{
	dbgprintf("CAN Node Task has started.\n");	
//	if (m_Driver.SetCANid(m_nCANId))
//		dbgprintf("CAN-ID changed to %d (Standard Mode)\n",m_nCANId);
//	else
//		dbgprintf("Changing CAN-ID FAILED\n");
	if (m_Driver.AddCANid((unsigned)EMessageBase_NmtControl))
		dbgprintf("CAN-ID filter added, ID = 0x%X (Standard Mode)\n",(unsigned)EMessageBase_NmtControl);
	else
		dbgprintf("Adding CAN-ID FAILED\n");
	if (m_Driver.AddCANid((unsigned)EMessageBase_SyncAndEmergency))
		dbgprintf("CAN-ID filter added, ID = 0x%X (Standard Mode)\n",(unsigned)EMessageBase_SyncAndEmergency);
	else
		dbgprintf("Adding CAN-ID FAILED\n");
	if (m_Driver.AddCANid((unsigned)EMessageBase_RxSDO + m_nCANId))
		dbgprintf("CAN-ID filter added, ID = 0x%X (Standard Mode)\n",(unsigned)EMessageBase_RxSDO + m_nCANId);
	else
		dbgprintf("Adding CAN-ID FAILED\n");
	if (m_Driver.AddCANid((unsigned)EMessageBase_NmtMonitorng + m_nCANId))
		dbgprintf("CAN-ID filter added, ID = 0x%X (Standard Mode)\n",(unsigned)EMessageBase_NmtMonitorng + m_nCANId);
	else
		dbgprintf("Adding CAN-ID FAILED\n");
   dbgprintf("   Registering CAN Objects ...\n");
	// Register all CAN data	
	for (int nObjectIndex=0; nObjectIndex<CANNODE_MAX_OBJECTS; nObjectIndex++)
	{
		if (NULL != m_aDataProviders[nObjectIndex])
		{
			dbgprintf("      Data Provider Object, ID = %d, CAN ID = 0x%X\n",nObjectIndex,getCAN_ID());
			m_aDataProviders[nObjectIndex]->RegisterData(*this, nObjectIndex);
		}
	}
	
	nCanReceived = 0;
	nCanTransmitted = 0;

	// Start the CAN controller
	m_Driver.Start();

	dbgprintf("CAN Task - entering Main Loop.\n");	
	while (1)
	{
		CUC_Task::Wait(CAN_TASK_PERIOD);
		// Dispatch incomming messages
		CAN_msg msg;
		while (m_Driver.ReadMessage(msg))
		{
			// Dispatch message
			HandleMessage(msg);
		}
	}
}

// ----------------------------------------------------------------------------
//! \brief Handle an incomming message
bool CANNode::HandleMessage(CAN_msg &msg)
{	
	nCanReceived++;
	// Handle SYNC message
	if (msg.id == (unsigned)EMessageBase_SyncAndEmergency) 
	{
#ifdef DBGPRINTF_BOARDMGR
		dbgprintf("CAN-RX (Sync): ID = %04X, Cmd = %d, Object = %d, Subindex = %d, Data = %d\n",
					  (unsigned)EMessageBase_NmtControl + m_nCANId,0,0,0,0);
#endif
#if TRACEALYZER != 0 && TRC_CANNODE != 0
		vTracePrint(trcCANNode,"Sync Cmd");
#endif
		return OnSync();
	}

	// Handle NMT commands
	if (msg.id == (unsigned)EMessageBase_NmtControl)
	{
		ENMTCommand cmd = (ENMTCommand)msg.data[0];
		uint8_t nodeId = msg.data[1];	
		if (nodeId == 0 || nodeId == m_nCANId)
		{
#ifdef DBGPRINTF_BOARDMGR
			dbgprintf("CAN-RX (NMT Command): ID = %04X, Cmd = %d, Object = %d, Subindex = %d, Data = %d\n",
						  (unsigned)EMessageBase_NmtControl + m_nCANId,0,0,0,0);
#endif
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrintF(trcCANNode,"NMT Cmd: Cmd = %d",cmd);
#endif
			return OnNMTCommand(cmd);
		}
		return true;
	}

	// Handle NMT requests
	if (msg.id == (unsigned)EMessageBase_NmtMonitorng + m_nCANId)
	{
#ifdef DBGPRINTF_BOARDMGR
		dbgprintf("CAN-RX (NMT Request): ID = %04X, Cmd = %d, Object = %d, Subindex = %d, Data = %d\n",
					 (unsigned)EMessageBase_NmtMonitorng + m_nCANId,0,0,0,(uint32_t)m_nNMTToggleBit);
#endif
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrintF(trcCANNode,"NMT Request");
#endif
		return OnNMTRequest();
	}

	// Handle SDO download and upload requests
	if (msg.id == (unsigned)EMessageBase_RxSDO + m_nCANId)
	{
		uint8_t nCommand = msg.data[0];
		uint16_t nObject = msg.data[1] + (msg.data[2] << 8);
		uint8_t nSubIndex = msg.data[3];
		uint32_t nData = *(uint32_t*)&msg.data[4];

#ifdef DBGPRINTF_BOARDMGR
		dbgprintf("CAN-RX (RxSDO): ID = %04X, Cmd = %d, Object = %d, Subindex = %d, Data = %04X\n",
					 msg.id,nCommand,nObject,nSubIndex,nData);
#endif
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrintF(trcCANNode,"SDO Rx: Cmd=0x%02X, Obj=%d ,Sub=%d, Data=%d",nCommand,nObject,nSubIndex,nData);
#endif
		switch (nCommand)
		{
			case 0x23:	// Initiate 4 bytes download
				return OnDownloadRequest(nObject, nSubIndex, 4, nData);
			case 0x27:	// Initiate 3 bytes download
				return OnDownloadRequest(nObject, nSubIndex, 3, nData);
			case 0x2B:	// Initiate 2 bytes download
				return OnDownloadRequest(nObject, nSubIndex, 2, nData);
			case 0x2F:	// Initiate 1 bytes download
				return OnDownloadRequest(nObject, nSubIndex, 1, nData);
			case 0x40:  // Initiate upload
				return OnUploadRequest(nObject, nSubIndex);
			default:
				// Other types of download/upload not supported
				break;
		}
		return false;
	}

	return true;
}

// ----------------------------------------------------------------------------
//! \brief Called by HandleMessage when a SYNC message is received
bool CANNode::OnSync()
{
#if TRACEALYZER != 0 && TRC_CANNODE != 0
		vTracePrint(trcCANNode,"Sync");
#endif
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Called by HandleMessage when a NMT request is received
bool CANNode::OnNMTRequest()
{
	m_nNMTToggleBit = (0 == m_nNMTToggleBit) ? 0x80 : 0;
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("CAN-TX (NMT Request): ID = %04X, Cmd = %d, Object = %d, Subindex = %d, Data = %d\n",
				 (unsigned)EMessageBase_NmtMonitorng + m_nCANId,0,0,0,(uint32_t)m_nNMTToggleBit);
#endif
	nCanTransmitted++;
#if TRACEALYZER != 0 && TRC_CANNODE != 0
	vTracePrintF(trcCANNode,"NMT Request: %X",(int)(m_nNMTToggleBit + m_nNMTState));
#endif
	return m_Driver.SendMessage((unsigned)EMessageBase_NmtMonitorng + 
		m_nCANId, (uint8_t)(m_nNMTToggleBit + m_nNMTState));	
}

// ----------------------------------------------------------------------------
//! \brief Called by HandleMessage when a NMT command is received
bool CANNode::OnNMTCommand(ENMTCommand cmd)
{
	switch (cmd)
	{
		case ENMTCommand_Start:
			m_nNMTState = ENMTState_Operational;
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrint(trcCANNode,"NMT Cmd = Start, Response = Operational");
#endif
			return true;
		case ENMTCommand_Stop:
			m_nNMTState = ENMTState_Stopped;
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrint(trcCANNode,"NMT Cmd = Stop, Response = Stopped");
#endif
			return true;
		case ENMTCommand_EnterPreOperational:
			m_nNMTState = ENMTState_Preoperational;
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrint(trcCANNode,"NMT Cmd = Preoper., Response =Preoper.");
#endif
			return true;
		case ENMTCommand_Reset:
			m_nNMTState	= ENMTState_Preoperational;
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrint(trcCANNode,"NMT Cmd = Reset, Response = Preoper.");
#endif
			return true;
		case ENMTCommand_ResetCommunication:
			m_nNMTState	= ENMTState_Operational;
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrint(trcCANNode,"NMT Cmd = ResetComm., Response = Operational");
#endif
			return true;
		default:
#if TRACEALYZER != 0 && TRC_CANNODE != 0
			vTracePrint(trcCANNode,"NMT Cmd = Unknown (Error)");
#endif
			break;
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Called by HandleMessage when a SDO upload request is received
bool CANNode::OnUploadRequest(uint16_t _nObjIndex, uint8_t _nSubIndex)
{
	uint8_t nDataLen = 4;
	uint32_t data = 0;
	if (_nObjIndex < CANNODE_MAX_OBJECTS && _nSubIndex < CANNDOE_MAX_SUBINDEXES)
	{
		data = m_aData[_nObjIndex][_nSubIndex];
	}
	uint32_t cmd;
	uint8_t  h_cmd;
	cmd = h_cmd = (unsigned)ESdoCommand_ReadResponse4Bytes + ((4-nDataLen) << 2);
	cmd |= _nObjIndex << 8;
	cmd |= _nSubIndex << 24;
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("CAN-TX (Upload Request): ID = %04X, Cmd = %d, Object = %d, Subindex = %d, Data = %04X\n",
				 (unsigned)EMessageBase_TxSDO + m_nCANId,(uint32_t)h_cmd,
				 (uint32_t)_nObjIndex,(uint32_t)_nSubIndex,data);
#endif
	nCanTransmitted++;
#if TRACEALYZER != 0 && TRC_CANNODE != 0
	vTracePrintF(trcCANNode,"SDO Upl. Req.: Cmd=%X,ObjID=%X,SubID=%X,Data=%X",(unsigned)h_cmd,
		(unsigned)_nObjIndex,(unsigned)_nSubIndex,(unsigned)data);
#endif
	return m_Driver.SendMessage((unsigned)EMessageBase_TxSDO + m_nCANId, cmd, data);
}

// ----------------------------------------------------------------------------
//! \brief Called by HandleMessage when a SDO download request is received
bool CANNode::OnDownloadRequest(uint16_t _nObjIndex, uint8_t _nSubIndex, uint8_t _nDataLen, uint32_t _nData)
{
	if (_nObjIndex < CANNODE_MAX_OBJECTS && _nSubIndex < CANNDOE_MAX_SUBINDEXES)
	{
		m_aData[_nObjIndex][_nSubIndex] = _nData;
		ICANNodeDataProvider *pProvider = m_aDataProviders[_nObjIndex];
		if (NULL != pProvider) pProvider->OnNewDataAvailable(_nSubIndex);
	}
	uint32_t cmd;
	uint8_t  h_cmd;
	cmd = h_cmd = (unsigned)ESdoCommand_WriteResponse;
	cmd |= _nObjIndex << 8;
	cmd |= _nSubIndex << 24;
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("CAN-TX (Download Request): ID = %04X, Cmd = %d, Object = %d, Subindex = %d, Data = 0000\n",
				 (unsigned)EMessageBase_TxSDO + m_nCANId,(uint32_t)h_cmd,
				 (uint32_t)_nObjIndex,(uint32_t)_nSubIndex);
#endif
	nCanTransmitted++;
#if TRACEALYZER != 0 && TRC_CANNODE != 0
	vTracePrintF(trcCANNode,"SDO Dwl. Req.: Cmd=%X,ObjID=%X,SubID=%X",(unsigned)h_cmd,
		(unsigned)_nObjIndex);
#endif
	return m_Driver.SendMessage((unsigned)EMessageBase_TxSDO + m_nCANId, cmd);
}

// ----------------------------------------------------------------------------
//! \brief Gets the CAN Node TX- and RX counter
bool CANNode::GetCAN_Counters(int *TX_ctr,int *RX_ctr)
{
	*TX_ctr = nCanTransmitted;
	*RX_ctr = nCanReceived;
	return true;
}

// ----------------------------------------------------------------------------
// Instanciate all the templates we'll be using...
template bool CANNode::DeclareData(uint16_t _nObjIndex, uint8_t _nSubIndex, ProcessDataOut<uint8_t>*&);
template bool CANNode::DeclareData(uint16_t _nObjIndex, uint8_t _nSubIndex, ProcessDataOut<uint16_t>*&);
template bool CANNode::DeclareData(uint16_t _nObjIndex, uint8_t _nSubIndex, ProcessDataOut<uint32_t>*&);

// ----------------------------------------------------------------------------
//! \brief Get the CAN Node TX- and RX counter
extern "C" bool CAN_Node_GetCounters(int *TX_ctr,int *RX_ctr)
{
	if (CAN_Node_Ptr == nullptr)
		return false;
	CAN_Node_Ptr->GetCAN_Counters(TX_ctr,RX_ctr);
	return true;
}
