#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <stdint.h>
#include <stdbool.h>

#define VARIABLE_SIZE

#define MODBUS_PDU_MAX_BYTESIZE   256
#define MODBUS_PDU_ERROR_BYTESIZE 2

#define MODBUS_ERROR_BIT  0x80

#define MODBUS_OK                               ( 0 )
#define MODBUS_ERROR                            ( -1 )

#define MODBUS_EXCEPTION_NONE                   ( 0 )
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       ( 1 )
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS   ( 2 )
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE     ( 3 )
#define MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE  ( 4 )

#define MODBUS_FUNCTIONS_LIST             \
  ITEM( READ_HOLDING_REGISTERS,   0x03 )  \
  ITEM( WRITE_MULTIPLE_REGISTERS, 0x10 )

typedef enum
{
#define ITEM( id, code ) id = code,
  MODBUS_FUNCTIONS_LIST
#undef ITEM
  MODBUS_FUNCTION_UNKNOWN = 0xFF
} ModbusFunction;

__packed typedef struct
{
  uint8_t func;
  uint8_t data[ VARIABLE_SIZE ];
} ModbusPdu;

__packed typedef struct
{
  uint16_t start;
  uint16_t quantity;
} ReadHoldingRegsReq;

__packed typedef struct
{
  uint8_t byteCount;
  uint8_t value[ VARIABLE_SIZE ];
} ReadHoldingRegsRes;

__packed typedef struct
{
  uint16_t start;
  uint16_t quantity;
  uint8_t byteCount;
  uint8_t value[ VARIABLE_SIZE ];
} WriteMultipleRegsReq;

__packed typedef struct
{
  uint16_t start;
  uint16_t quantity;
} WriteMultipleRegsRes;

#define MODBUS_REGISTER_FLAG_R  ( 1U << 0 )
#define MODBUS_REGISTER_FLAG_W  ( 1U << 1 )
#define MODBUS_REGISTER_FLAG_RW ( MODBUS_REGISTER_FLAG_R | MODBUS_REGISTER_FLAG_W )

__packed typedef struct
{
  uint16_t address;
  uint16_t quantity;
  uint8_t  flags;
} ModbusRegister;

typedef struct
{
  uint16_t start;
  uint16_t end;
  const ModbusRegister *regs;
  int32_t regsCount;
  void *mem;
} ModbusTable;

typedef struct
{
  void      ( *init )( const ModbusTable * tables, uint32_t num );
  int32_t   ( *parse )( uint8_t *req, uint32_t len );
  bool      ( *exception )( void );
  uint8_t   ( *unit )( void );
  uint8_t   ( *function )( void );
  int32_t   ( *find )( void );
  int32_t   ( *execute )( void );
  int32_t   ( *response )( uint8_t *buff, uint32_t len );
} Modbus;

uint16_t Modbus_swapWord( uint16_t word );

extern Modbus modbus;

#endif