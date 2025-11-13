#include <string.h>
#include "modbus.h"

#define MODBUS_TCP_HEADER_BYTESIZE 7

__packed typedef struct
{
  uint16_t transaction;
  uint16_t protocol;
  uint16_t len;
  uint8_t unit;
  ModbusPdu pdu;
} ModbusTcpAdu;

typedef struct
{
  uint16_t transaction;
  uint16_t len;
  uint8_t unit;
  uint8_t func;
  uint16_t start;
  uint16_t quantity;
  uint8_t byteCount;
  uint8_t *values;
  uint8_t *regs;
  uint8_t exp;
} ModbusTcpParser;

typedef struct
{
  const ModbusTable *tables;
  uint32_t tablesCount;
  ModbusTcpParser parser;
} ModbusTcpCtl;

static void     ModbusTcp_init( const ModbusTable * tables, uint32_t num );
static int32_t  ModbusTcp_parse( uint8_t *req, uint32_t len );
static bool     ModbusTcp_exception( void );
static uint8_t  ModbusTcp_unit( void );
static uint8_t  ModbusTcp_function( void );
static int32_t  ModbusTcp_find( void );
static int32_t  ModbusTcp_execute( void );
static int32_t  ModbusTcp_response( uint8_t *buff, uint32_t len );

static ModbusTcpCtl modbusTcpCtl;
#define mdb ( &modbusTcpCtl )

static void ModbusTcp_init( const ModbusTable * tables, uint32_t num )
{
  memset( mdb, 0, sizeof( ModbusTcpCtl ) );
  mdb->tables = tables;
  mdb->tablesCount = num;
}

static void ModbusTcpParser_init( ModbusTcpParser * const parser )
{
  memset( parser, 0, sizeof( ModbusTcpParser ) );
}

static int32_t ModbusTcp_parse( uint8_t *req, uint32_t len )
{
  ModbusTcpAdu *reqAdu;
  ModbusTcpParser *parser;
  
  if( len < MODBUS_TCP_HEADER_BYTESIZE )
  {
    return MODBUS_ERROR;
  }
  
  parser = &mdb->parser;
  
  ModbusTcpParser_init( parser );
  
  reqAdu = ( ModbusTcpAdu* )req;
  
  if( reqAdu->protocol == 0 )
  {
#ifdef __LITTLE_ENDIAN__
    parser->transaction = Modbus_swapWord( reqAdu->transaction );
    parser->len = Modbus_swapWord( reqAdu->len );
#else
    parser->transaction = reqAdu->transaction;
    parser->len = reqAdu->len;
#endif
    parser->unit = reqAdu->unit;
    parser->func = reqAdu->pdu.func;
    switch( parser->func )
    {
    case READ_HOLDING_REGISTERS:
      {
        ReadHoldingRegsReq *rhrr = ( ReadHoldingRegsReq* )reqAdu->pdu.data;
#ifdef __LITTLE_ENDIAN__
        parser->start = Modbus_swapWord( rhrr->start );
        parser->quantity = Modbus_swapWord( rhrr->quantity );
#else
        parser->start = rhrr->start;
        parser->quantity = rhrr->quantity;
#endif
        parser->byteCount = parser->quantity * 2;
        if( ( parser->quantity >= 1 ) && ( parser->quantity <= 0x007D ) )
        {
          parser->exp = MODBUS_EXCEPTION_NONE;
        }
        else
        {
          parser->exp = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
      }
      break;
    case WRITE_MULTIPLE_REGISTERS:
      {
        WriteMultipleRegsReq *wmrr = ( WriteMultipleRegsReq* )reqAdu->pdu.data;
#ifdef __LITTLE_ENDIAN__
        parser->start = Modbus_swapWord( wmrr->start );
        parser->quantity = Modbus_swapWord( wmrr->quantity );
#else
        parser->start = wmrr->start;
        parser->quantity = wmrr->quantity;
#endif
        parser->byteCount = wmrr->byteCount;
        parser->values = wmrr->value;
        if( ( parser->quantity >= 1 ) && ( parser->quantity <= 0x007B ) && ( parser->byteCount == parser->quantity * 2 ) )
        {
          parser->exp = MODBUS_EXCEPTION_NONE;
        }
        else
        {
          parser->exp = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
      }
      break;
    default:
      parser->exp = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
      break;
    }
  }
  else
  {
    return MODBUS_ERROR;
  }
  
  return MODBUS_OK;
}

static bool ModbusTcp_exception( void )
{
  return ( ( mdb->parser.exp == MODBUS_EXCEPTION_NONE ) ? false : true );
}

static uint8_t ModbusTcp_unit( void )
{
  return mdb->parser.unit;
}

static uint8_t ModbusTcp_function( void )
{
  return mdb->parser.func;
}

static int32_t ModbusTcp_find( void )
{
  int32_t res, n, t;
  uint8_t flag;
  uint32_t offset;
  const ModbusTable *table;
  ModbusTcpParser *parser;

  res = MODBUS_ERROR;
  parser = &mdb->parser;
  
  /* Pick table */
  for( t = 0; t < mdb->tablesCount; t++ )
  {
    table = &mdb->tables[ t ];
    if( ( parser->start >= table->start ) && ( parser->start < table->end ) )
    {
      break;
    }
  }
  
  if( t != mdb->tablesCount )
  {
    /* Pick register */
    for( n = 0; n < table->regsCount; n++ )
    {
      if( table->regs[ n ].address == parser->start )
      {
        break;
      }
    }
    if( n != table->regsCount )
    {
      if( table->regs[ n ].address + parser->quantity <= table->end )
      {
        switch( parser->func )
        {
        case READ_HOLDING_REGISTERS: flag = MODBUS_REGISTER_FLAG_R; break;
        case WRITE_MULTIPLE_REGISTERS: flag = MODBUS_REGISTER_FLAG_W; break;
        default: flag = 0; break;
        }
        if( table->regs[ n ].flags & flag )
        {
          res = MODBUS_OK;
        }
      }
    }
  }
  
  if( res == MODBUS_OK )
  {
    if( table->mem != NULL )
    {
      offset = ( ( parser->start - table->start ) * 2 );
      parser->regs = ( ( ( uint8_t* )table->mem ) + offset );
    }
    else
    {
      res = MODBUS_ERROR;
    }
  }
  
  if( res == MODBUS_OK )
  {
    res = 0;
    for( int32_t i = 0; i < t; i++ )
    {
      res += mdb->tables[ i ].regsCount;
    }
    res += n;
  }
  else
  {
    parser->exp = MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE;
  }
  
  return res;
}

static int32_t ModbusTcp_execute( void )
{
  int32_t res;
  ModbusTcpParser *parser;
  
  res = MODBUS_ERROR;
  parser = &mdb->parser;
  switch( parser->func )
  {
  case READ_HOLDING_REGISTERS:
    if( parser->regs != NULL )
    {
      res = MODBUS_OK;
    }
    break;
  case WRITE_MULTIPLE_REGISTERS:
    if( parser->regs != NULL && parser->values != NULL )
    {
      memcpy( parser->regs, parser->values, parser->byteCount );
      res = MODBUS_OK;
    }
    break;
  }
  
  if( res == MODBUS_OK )
  {
    parser->exp = MODBUS_EXCEPTION_NONE;
  }
  else
  {
    parser->exp = MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE;
  }
  
  return res;
}

static int32_t ModbusTcp_response( uint8_t *buff, uint32_t len )
{
  ModbusTcpAdu *resAdu;
  int32_t respByteSize;
  ModbusTcpParser *parser;
  
  parser = &mdb->parser;
  
  respByteSize = 0;
  if( len >= MODBUS_TCP_HEADER_BYTESIZE )
  {
    resAdu = (ModbusTcpAdu*)buff;
#ifdef __LITTLE_ENDIAN__
    resAdu->transaction = Modbus_swapWord( parser->transaction );
#else
    resAdu->transaction = parser->transaction;
#endif
    resAdu->protocol = 0;
    resAdu->len = 0;
    resAdu->unit = parser->unit;
    respByteSize += MODBUS_TCP_HEADER_BYTESIZE;
  }
  else
  {
    return MODBUS_ERROR;
  }
  
  if( parser->exp == MODBUS_OK )
  {
    switch( parser->func )
    {
    case READ_HOLDING_REGISTERS:
      if( respByteSize + 2 + parser->byteCount > len )
      {
        return MODBUS_ERROR;
      }
      else
      {
        resAdu->pdu.func = READ_HOLDING_REGISTERS;
        ReadHoldingRegsRes *rhrr = ( ReadHoldingRegsRes* )resAdu->pdu.data;
        rhrr->byteCount = parser->byteCount;
        memcpy( rhrr->value, parser->regs, parser->byteCount );
        respByteSize += ( 2 + parser->byteCount );
      }
      break;
    case WRITE_MULTIPLE_REGISTERS:
      if( respByteSize + 5 > len )
      {
        return MODBUS_ERROR;
      }
      else
      {
        resAdu->pdu.func = WRITE_MULTIPLE_REGISTERS;
        WriteMultipleRegsRes *wmrr = ( WriteMultipleRegsRes* )resAdu->pdu.data;
#ifdef __LITTLE_ENDIAN__
        wmrr->start = Modbus_swapWord( parser->start );
        wmrr->quantity = Modbus_swapWord( parser->quantity );
#else
        wmrr->start = parser->start;
        wmrr->quantity = parser->quantity;
#endif
        respByteSize += 5;
      }
      break;
    }
  }
  else
  {
    if( len >= MODBUS_TCP_HEADER_BYTESIZE + MODBUS_PDU_ERROR_BYTESIZE )
    {
      resAdu->pdu.func = parser->func | MODBUS_ERROR_BIT;
      resAdu->pdu.data[ 0 ] = parser->exp;
      respByteSize += 2;
    }
    else
    {
      return MODBUS_ERROR;
    }
  }
  
#ifdef __LITTLE_ENDIAN__
      resAdu->len = Modbus_swapWord( respByteSize - MODBUS_TCP_HEADER_BYTESIZE + 1 );
#else
      resAdu->len = respByteSize - MODBUS_TCP_HEADER_BYTESIZE + 1;
#endif
  
  return respByteSize;
}

Modbus modbus = {
  ModbusTcp_init,
  ModbusTcp_parse,
  ModbusTcp_exception,
  ModbusTcp_unit,
  ModbusTcp_function,
  ModbusTcp_find,
  ModbusTcp_execute,
  ModbusTcp_response
};