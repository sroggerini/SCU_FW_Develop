#include <string.h>
#include "usart_esp32.h"
#include "esp32.h"

typedef struct
{
  uint8_t rxBuffer[ MAX_USART_ESP32_RX_BUFFER_BYTESIZE ];
  uint8_t txBuffer[ MAX_USART_ESP32_TX_BUFFER_BYTESIZE ];
  volatile uint32_t write_idx;
  uint32_t read_idx;
} UsartEsp32Control;

static UsartEsp32Control usartEsp32Control;
#define pUsartCtr ( &usartEsp32Control )

void USART_ESP32_RxEventCallback( UART_HandleTypeDef *huart, uint16_t Size )
{
  if( huart == USART_HANDLER )
  {
    pUsartCtr->write_idx = ( Size & ( MAX_USART_ESP32_RX_BUFFER_BYTESIZE - 1 ) );
    At_notify( AT_NOTIFY_DATA_RX, NULL );
  }
}

void USART_ESP32_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if( huart == USART_HANDLER )
  {
    At_notify( AT_NOTIFY_TX_DONE, NULL );
  }
}

int32_t UsartEsp32_init( void )
{
  MX_USART3_UART_Init();
  pUsartCtr->write_idx = pUsartCtr->read_idx = 0;
  HAL_UARTEx_ReceiveToIdle_DMA( USART_HANDLER, pUsartCtr->rxBuffer, MAX_USART_ESP32_RX_BUFFER_BYTESIZE );
  return 0;
}

int32_t UsartEsp32_deinit( void )
{
  HAL_UART_DeInit( USART_HANDLER );
  return 0;
}

int32_t UsartEsp32_send( const void *data, uint32_t num )
{
  if( num > MAX_USART_ESP32_TX_BUFFER_BYTESIZE )
    num = MAX_USART_ESP32_TX_BUFFER_BYTESIZE;
  memcpy( pUsartCtr->txBuffer, data, num );
  if( HAL_UART_Transmit_DMA( USART_HANDLER, pUsartCtr->txBuffer, num ) == HAL_OK )
  {
    return num;
  }
  return -1;
}

int32_t UsartEsp32_receive( void *buff, uint32_t num )
{
  int32_t cnt;
  uint8_t *dst;
  
  dst = ( uint8_t* )buff;
  cnt = 0;
  while( cnt < num )
  {
    if( pUsartCtr->write_idx == pUsartCtr->read_idx )
    {
      break;
    }
    dst[ cnt ] = pUsartCtr->rxBuffer[ pUsartCtr->read_idx ];
    cnt++;
    pUsartCtr->read_idx = ( ( pUsartCtr->read_idx + 1 ) & ( MAX_USART_ESP32_RX_BUFFER_BYTESIZE - 1 ) );
  }
  return cnt;
}

Usart usart3 = {
  UsartEsp32_init,
  UsartEsp32_deinit,
  UsartEsp32_send,
  UsartEsp32_receive
};