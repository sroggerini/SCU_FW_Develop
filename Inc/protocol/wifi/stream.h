#ifndef __STREAM_H__
#define __STREAM_H__

#include <stdint.h>

typedef struct
{
  uint8_t *buff;
  uint32_t size;
  uint32_t count;
  void *mutex;
} Stream;

void Stream_init( Stream * const me, uint8_t *buff, uint32_t size );

void Stream_uninit( Stream * const me );

void Stream_lock( Stream * const me );

void Stream_unlock( Stream * const me );

int32_t Stream_write( Stream * const me, uint8_t * src, uint32_t size );

int32_t Stream_read( Stream * const me, uint8_t * dst, uint32_t size );

uint8_t Stream_getByte( Stream * const me );

uint16_t Stream_getCount( Stream * const me );

uint16_t Stream_getAvailable( Stream * const me );

uint16_t Stream_flush( Stream * const me, uint32_t qnt );

int32_t Stream_find( Stream * const me, const char *triggers, uint32_t num );

int32_t Stream_compare( Stream * const me, uint32_t offset, const char *string );

int32_t Stream_copy( Stream * const dst, Stream * const src, uint32_t qnt );

#endif