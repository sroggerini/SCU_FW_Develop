#ifndef __CONFIGURATOR_H__
#define __CONFIGURATOR_H__

#include <stdint.h>
#include "lwip/tcp.h"

err_t configurator_init_config_reception (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

void configurator_deinit_config_reception (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

err_t configurator_get_version(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

err_t configurator_handle_config(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

//int32_t Configurator_handle(struct pbuf *p);

char * Configurator_getData(void);

uint32_t Configurator_getDataSize(void);

uint32_t Configurator_rebootRequire(void);

int32_t Configurator_saveWifiWebInfo(wifiWebInfo * const wifiWebConfig);


#endif