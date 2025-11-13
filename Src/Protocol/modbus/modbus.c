#include "modbus.h"

uint16_t Modbus_swapWord( uint16_t word )
{
  return ( word >> 8 ) | ( word << 8 );
}