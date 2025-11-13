#include <string.h>
#include "stream.h"
#include "cmsis_os2.h"

void Stream_lock( Stream * const me )
{
  osMutexAcquire( me->mutex, osWaitForever );
}

void Stream_unlock( Stream * const me )
{
  osMutexRelease( me->mutex );
}

void Stream_init( Stream * const me, uint8_t *buff, uint32_t size )
{
  me->buff = buff;
  me->size = size;
  me->count = 0;
  me->mutex = osMutexNew( NULL );
}

void Stream_uninit( Stream * const me )
{
  me->buff = NULL;
  me->size = 0;
  me->count = 0;
  osMutexDelete( me->mutex );
  me->mutex = NULL;
}

int32_t Stream_write( Stream * const me, uint8_t * src, uint32_t size )
{
  int32_t i;
  
  Stream_lock( me );
  
  i = me->size - me->count;
  
  if( i > size )
    i = size;
  
  memcpy( me->buff + me->count, src, i );
  me->count += i;

  Stream_unlock( me );
  
  return i;
}

int32_t Stream_read( Stream * const me, uint8_t * dst, uint32_t size )
{
  int32_t i;
  
  Stream_lock( me );
  
  i = me->count;
  if( i > size )
    i = size;
  
  memcpy( dst, me->buff, i );
  memmove( me->buff, &me->buff[ i ], me->count - i );
  me->count -= i;
  memset( &me->buff[ me->count ], 0, me->size - me->count );
  
  Stream_unlock( me );
  
  return i;
}

uint8_t Stream_getByte( Stream * const me )
{
  uint8_t c;
  
  Stream_lock( me );
  
  c = me->buff[ 0 ];
  
  Stream_unlock( me );
  
  return c;
}

uint16_t Stream_getCount( Stream * const me )
{ 
  uint16_t cnt;
  
  Stream_lock( me );
  
  cnt = me->count;
  
  Stream_unlock( me );  
    
  return cnt;
}

uint16_t Stream_getAvailable( Stream * const me )
{
  uint16_t cnt;
  
  Stream_lock( me );
  
  cnt = ( me->size - me->count );
  
  Stream_unlock( me );
  
  return cnt;
}

uint16_t Stream_flush( Stream * const me, uint32_t qnt )
{
  Stream_lock( me );
  
  if( qnt > me->count )
    qnt = me->count;
  
  memmove( me->buff, &me->buff[ qnt ], me->count - qnt );
  me->count -= qnt;
  memset( &me->buff[ me->count ], 0, me->size - me->count );
  
  Stream_unlock( me );
  
  return qnt;
}

int32_t Stream_find( Stream * const me, const char *triggers, uint32_t num )
{
  int32_t res;
  uint8_t* find = NULL;
  
  Stream_lock( me );
  
  find = ( uint8_t* )strstr( ( char* )me->buff, triggers );
  if( find == NULL )
    res = -1;
  else
    res = find - me->buff;
  
  Stream_unlock( me );
  
  return res;
}

int32_t Stream_compare( Stream * const me, uint32_t offset, const char *string )
{
  int32_t res;
  
  Stream_lock( me );
  
  res = strncmp( ( char* )&me->buff[ offset ], string, strlen( string ) );
  
  Stream_unlock( me );
  
  return res;
}

int32_t Stream_copy( Stream * const dst, Stream * const src, uint32_t qnt )
{
  uint32_t i;
  
  Stream_lock( src );
  Stream_lock( dst );
  
  i = dst->size - dst->count;
  
  if( i > qnt )
    i = qnt;
  
  if( i > src->count )
    i = src->count;
  
  memcpy( &dst->buff[ dst->count ], &src->buff[ 0 ], i );
  dst->count += i;
  memmove( src->buff, &src->buff[ i ], src->count - i );
  src->count -= i;
  memset( &src->buff[ src->count ], 0, src->size - src->count );
  
  Stream_unlock( dst );
  Stream_unlock( src );
  
  return i;
}