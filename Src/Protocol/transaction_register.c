#include "transaction_register.h"
#include "ffconf.h"
#include "ExtFlash.h"
#include "rtcApi.h"

#define FLASH_TRANSACTION_ONLY_DATA_SIZE ( FLASH_TRANSACTION_SIZE - 4 )

typedef struct
{
  uint32_t count;
  uint8_t isTransactionRunning;
  time_t transactionStartTimestamp;
  char transactionStartUid[ UID_MAX_SIZE ];
} TransactionRegister;

typedef __packed struct
{
  uint8_t checksum;
  uint16_t count;
  Transaction transaction[ FLASH_TRANSACTION_ONLY_DATA_SIZE ];
} TransactionFlash;

static TransactionRegister transactionRegister;
#define reg ( &transactionRegister )

static const uint32_t maxNumTransaction = FLASH_TRANSACTION_ONLY_DATA_SIZE / sizeof( Transaction );
static uint32_t flashAddress;

static uint8_t computeChecksum( uint8_t *data, uint32_t size )
{
  uint8_t res;
  
  res = 0;
  for ( uint32_t i = 0; i < size; i++ )
  {
    res += data[ i ];
  }
  res = 0x100 - res;
  
  return res;
}

void TransactionRegister_init( void )
{
  TransactionFlash *flashArea;
  uint8_t check;
  
  flashAddress = getTransactionAddress();
  flashArea = ( TransactionFlash* )pvPortMalloc( FLASH_TRANSACTION_SIZE );
  if( flashArea != NULL )
  {
    if( FlashRead( flashAddress, (uint8_t*)flashArea, FLASH_TRANSACTION_SIZE ) == HAL_OK )
    {
      check = flashArea->checksum - computeChecksum( (uint8_t*)&flashArea->count, FLASH_TRANSACTION_ONLY_DATA_SIZE + 2 );
      if( check != FLASH_CHECKSUM_VALUE )
      {
        /* Clear all transaction flash area */
        flashArea->count = 0;
        memset( flashArea->transaction, 0xFF, maxNumTransaction * sizeof( Transaction ) );
        flashArea->checksum = computeChecksum( (uint8_t*)&flashArea->count, FLASH_TRANSACTION_ONLY_DATA_SIZE + 2 );
        FlashWrite( flashAddress, (uint8_t*)flashArea, FLASH_TRANSACTION_SIZE, FLASH_TRANSACTION_SIZE);
      }
      reg->count = flashArea->count;
      reg->isTransactionRunning = 0;
      reg->transactionStartTimestamp = 0;
      memset( reg->transactionStartUid, 0, UID_MAX_SIZE );
    }
  }
  vPortFree( flashArea );
}

int32_t TransactionRegister_startTransaction( char *uid, uint32_t len )
{
  int32_t res = -1;
  
  if( reg->isTransactionRunning == 0 )
  {
    memset( reg->transactionStartUid, 0, UID_MAX_SIZE );
    if( len > UID_MAX_SIZE )
      len = UID_MAX_SIZE;
    memcpy( &reg->transactionStartUid, uid, len );
    reg->transactionStartTimestamp = getCurrentUnixTime();
    reg->isTransactionRunning = 1;
    res = 0;
  }
  return res;
}

int32_t TransactionRegister_stopTransaction( uint32_t activeEnergy_Wh )
{
  TransactionFlash *flashArea;
  int32_t res = -1;
  uint8_t check;
  Transaction transaction;
  
  if( reg->isTransactionRunning == 1 )
  {
    transaction.start = reg->transactionStartTimestamp;
    memcpy( transaction.uid, reg->transactionStartUid, UID_MAX_SIZE );
    transaction.stop = getCurrentUnixTime();
    transaction.active_energy = activeEnergy_Wh;
    flashArea = ( TransactionFlash* )pvPortMalloc( FLASH_TRANSACTION_SIZE );
    if( flashArea != NULL )
    {
      if( FlashRead( flashAddress, (uint8_t*)flashArea, FLASH_TRANSACTION_SIZE ) == HAL_OK )
      {
        if( flashArea->count == maxNumTransaction )
        {
          memmove( &flashArea->transaction[ 0 ], &flashArea->transaction[ 1 ], ( ( maxNumTransaction - 1 ) * sizeof( Transaction ) ) );
          memcpy( &flashArea->transaction[ maxNumTransaction - 1 ], &transaction, sizeof( Transaction ) );
        }
        else
        {
          memcpy( &flashArea->transaction[ flashArea->count ], &transaction, sizeof( Transaction ) );
          flashArea->count++;
        }
        check = computeChecksum( (uint8_t*)&flashArea->count, FLASH_TRANSACTION_ONLY_DATA_SIZE + 2 );
        flashArea->checksum = check;
        res = FlashWrite( flashAddress, (uint8_t*)flashArea, FLASH_TRANSACTION_SIZE, FLASH_TRANSACTION_SIZE );
        reg->count = flashArea->count;
      }
    }
    vPortFree( flashArea );
  }
  reg->isTransactionRunning = 0;
  return res;
}

uint32_t TransactionRegister_getCount( void )
{
  return reg->count;
}

int32_t TransactionRegister_getTransaction( Transaction *dst, uint32_t num )
{
  int32_t res = -1;
  
  if( num >= reg->count )
  {
    if( dst != NULL )
    {
      if( FlashRead( flashAddress + 3, (uint8_t*)dst, ( num * sizeof( Transaction ) ) ) == HAL_OK )
      {
        res = 0;
      }
    }
  }
  
  return res;
}

int32_t TransactionRegister_clearAll( void )
{
  TransactionFlash *flashArea;
  int32_t res;
  
  flashArea = ( TransactionFlash* )pvPortMalloc( FLASH_TRANSACTION_SIZE );
  if( flashArea != NULL )
  {
    flashArea->count = 0;
    memset( flashArea->transaction, 0xFF, maxNumTransaction * sizeof( Transaction ) );
    flashArea->checksum = computeChecksum( (uint8_t*)&flashArea->count, FLASH_TRANSACTION_ONLY_DATA_SIZE + 2 );
    res = FlashWrite( flashAddress, (uint8_t*)flashArea, FLASH_TRANSACTION_SIZE, FLASH_TRANSACTION_SIZE);
    if( res == 0 )
    {
      reg->count = 0;
    }
  }
  vPortFree( flashArea );
  return res;
}