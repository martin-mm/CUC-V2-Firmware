#ifndef FRAM_h
#define FRAM_h

#include <stdint.h>
#include "board.h"

#define FRAM_I2C_Base				I2C0

#define FRAM_address             0xA0     //!< Slave address of FRAM
#define FRAM_MaxReadBufferLen    16       //!< Maximum Length of I2C Read Buffer

extern int InitializeFRAM(void);
extern int SetWriteProtect(void);
extern int ClrWriteProtect(void);
extern int WriteFRAM(uint16_t address,void *buf,int len);
extern int ReadFRAM(uint16_t address,void *buf,int len);

#endif		// FRAM_h
