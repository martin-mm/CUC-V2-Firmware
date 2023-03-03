// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        Base.h
//! \brief       List of general definitions
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _BASE_H_
#define _BASE_H_

#include <stdint.h>

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

//#define nullptr	NULL
using namespace std;
//namespace std
//{
//	typedef decltype(nullptr) nullptr_t;
//}

// Definition of basic data types
// typedef char 						int8;
// typedef unsigned char 			uint8;
// typedef short int 				int16;
// typedef unsigned short int 	uint16;
// typedef int 						int32;
// typedef unsigned int 			uint32;

// Macro hiding all C++ default methods
#define HideDefaultMethods(clsName) \
private: \
clsName(); \
clsName(const clsName &right); \
clsName & operator=(const clsName &right); \
bool operator==(const clsName &right) const; \
bool operator!=(const clsName &right) const     // No semicolon here... long story but in short this is just to make doxygen happy

// Macro hiding all C++ default methods except the default constructor
#define HideCopyAssignCompMethods(clsName) \
private: \
clsName(const clsName &right); \
clsName & operator=(const clsName &right); \
bool operator==(const clsName &right) const; \
bool operator!=(const clsName &right) const     // No semicolon here... long story but in short this is just to make doxygen happy

// Basic exception mechanism working only when code does not use 
#define SIMPLE_TRY bool __bOk=false; for(;;) {
#define SIMPLE_CATCH __bOk=true; break; } if (!__bOk) 
#define THROW break;

#endif // _BASE_H_
