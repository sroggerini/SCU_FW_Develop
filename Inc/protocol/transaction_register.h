#ifndef __TRANSACTION_REGISTER_H__
#define __TRANSACTION_REGISTER_H__

#include <stdint.h>
#include <time.h>
#include <string.h>

#define UID_MAX_SIZE 10
#define UID_TRANSACTION_FREE                    ((uint32_t)0x00000000)
#define UID_TRANSACTION_NET                     ((uint32_t)0x33333333)
#define UID_TRANSACTION_OCPP                    ((uint32_t)0x66666666)
#define UID_TRANSACTION_PERS_BY_CARD            ((uint32_t)0x88888888)
#define UID_TRANSACTION_PERS_BY_APP             ((uint32_t)0x99999999)
#define UID_TRANSACTION_DSO                     ((uint32_t)0xFFFFFFFF)

typedef __packed struct
{
  time_t    start;                // Unix Time
  time_t    stop;                 // Unix Time
  uint32_t  active_energy;        // Wh
  char   uid[ UID_MAX_SIZE ];  // string
} Transaction;

void TransactionRegister_init( void );

int32_t TransactionRegister_startTransaction( char *uid, uint32_t len );

int32_t TransactionRegister_stopTransaction( uint32_t activeEnergy_mWh );

uint32_t TransactionRegister_getCount( void );

int32_t TransactionRegister_getTransaction( Transaction *dst, uint32_t num );

int32_t TransactionRegister_clearAll( void );

#endif