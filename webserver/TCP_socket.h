/**
  ******************************************************************************
  * File               : TCP_socket.h
  * Descrizione        : Contiene le funzioni, le define e i prototipi del socket IP
  ******************************************************************************
*/

// Define per evitare inclusioni ricorsive
#ifndef TCP_SOCKET_H_
#define TCP_SOCKET_H_

#define   HTTP_PORT           ((unsigned short)8063)
#define   EM_LOVATO_PORT      ((unsigned short)1001)

void tcp_socket_init          (void);
void tcp_socket_1_thread      (void const * pvParametersg);
void setHttpPort              (unsigned short numPort);
void freeHtmlMemory           (void);
#ifdef MODBUS_TCP_EM_ETH
void emLovato_socket_init     (void);
void tcpserver_init           (void); 
#endif
#endif /* TCP_SOCKET_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
