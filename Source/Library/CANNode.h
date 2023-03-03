// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        CANNode.h
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

#ifndef _CANNODE_H_
#define _CANNODE_H_

// ----------------------------------------------------------------------------
// Includes
#if defined(__cplusplus)
#include "cmsis_os2.h"
#include "CANDefs.h"
#include "CANDriver.h"
#include "base.h"
#include "Task_CMSIS2.h"

// ----------------------------------------------------------------------------
// Constants
#define CANNODE_MAX_OBJECTS 3
#define CANNDOE_MAX_SUBINDEXES 16

// ----------------------------------------------------------------------------
// Forward declarations
class CANNode;
template<class T> class ProcessDataOut;

// ----------------------------------------------------------------------------
//! \class      ICANNodeDataProvider
//! \brief      Interface that provider of CAN accessible data must support
class ICANNodeDataProvider
{
public:
    //! \cond
	virtual ~ICANNodeDataProvider() {}
    //! \endcond
	virtual void RegisterData(CANNode &_node, uint16_t _nObjIndex) = 0;
	virtual void OnNewDataAvailable(uint8_t _nSubIndex) {};
};

// ----------------------------------------------------------------------------
//! \struct     CANNodeDataProvider
//! \brief      Simple structure used internal by CANNode to keep track of CAN data providers.
struct CANNodeDataProvider
{
	uint16_t nObjectIndex;
	ICANNodeDataProvider *pProvider;
};

// ----------------------------------------------------------------------------
//! \class      CANNode
//! \brief      Encapsulate the functionality of a CAN node
class CANNode : public CUC_Task
{
public:
	CANNode(CANDriver &_driver, uint8_t _nCANId, osPriority_t _nPriority);
    //! \cond 
	virtual ~CANNode(void) {}
	HideDefaultMethods(CANNode);
    //! \endcond 

public:			
	virtual void Main();
	bool RegisterDataProvider(ICANNodeDataProvider *_pProvider, uint16_t _nObjIndex);
	template<class T> bool DeclareData(uint16_t _nObjIndex, uint8_t _nSubIndex, ProcessDataOut<T>* &_pProcessData);
	uint32_t getCAN_ID(void);
	uint32_t getNMT_State(void);
	int getNumberOfProvider(void);
	bool checkProviderRegistered(unsigned id);
	bool getProviderContent(unsigned id,unsigned subid,uint32_t &content,bool &valid);
	bool GetCAN_Counters(int *TX_ctr,int *RX_ctr);

private:
	int nCanReceived;
	int nCanTransmitted;

protected:
	virtual bool HandleMessage(CAN_msg &msg);
	virtual bool OnSync(void);
	virtual bool OnNMTRequest(void);
	virtual bool OnNMTCommand(ENMTCommand cmd);
	virtual bool OnUploadRequest(uint16_t _nObjIndex, uint8_t _nSubIndex);
	virtual bool OnDownloadRequest(uint16_t _nObjIndex, uint8_t _nSubIndex, uint8_t _nDataLen, uint32_t _nData);	

protected:
	CANDriver 					&m_Driver;
	uint8_t 						m_nCANId;
	uint8_t 						m_nNMTToggleBit;
	ENMTState 					m_nNMTState;
	ICANNodeDataProvider* 	m_aDataProviders[CANNODE_MAX_OBJECTS];

	SemaphoreHandle_t  		m_ProcessDataMutex;
	uint32_t 					m_aData[CANNODE_MAX_OBJECTS][CANNDOE_MAX_SUBINDEXES];
};

extern "C" bool CAN_Node_GetCounters(int *TX_ctr,int *RX_ctr);
#else
#include <stdint.h>
#include <stdbool.h>

extern bool CAN_Node_GetCounters(int *TX_ctr,int *RX_ctr);
#endif

#endif // _CANNODE_H_
