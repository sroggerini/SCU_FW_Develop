#ifndef __USART_ESP32_H__
#define __USART_ESP32_H__

#include "usart.h"

#define USART_HANDLER ( &huart3 )

#define MAX_USART_ESP32_RX_BUFFER_BYTESIZE 4096 // It must be a power of 2
#define MAX_USART_ESP32_TX_BUFFER_BYTESIZE 512

void USART_ESP32_RxEventCallback( UART_HandleTypeDef *huart, uint16_t Size );
void USART_ESP32_TxCpltCallback( UART_HandleTypeDef *huart );

#endif