/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "TCP_socket.h"

#define TCP_THREAD_PRIO  (tskIDLE_PRIORITY + 4)

/*
***********************************SCAME**************************************
**                                                                          **
**                            External Variables                            **
**                                                                          **
******************************************************************************
*/

extern void K22_httpd_init(void);


/*
***********************************SCAME**************************************
**                                                                          **
**                            Global Variables                              **
**                                                                          **
****************************************************************************** 
*/ 



unsigned short EthernetTCP2Port;

/*
***********************************SCAME**************************************
**                                                                          **
**                            Function Definition                           **
**                                                                          **
******************************************************************************
*/


void tcp_socket_init(void)
{
  if (EthernetTCP2Port == HTTP_PORT)
  {
    K22_httpd_init();
  }
}

#ifdef MODBUS_TCP_EM_ETH
void emLovato_socket_init(void)
{
  tcpserver_init();
}
#endif

/**
*
* @brief        Set the HTTP port      
*
* @param [in]   uint16_t: http port number
*
* @retval       none 
*
***********************************************************************************************************************/
void setHttpPort(unsigned short numPort)
{
  EthernetTCP2Port = numPort;
}


/*************** END OF FILE ******************************************************************************************/

