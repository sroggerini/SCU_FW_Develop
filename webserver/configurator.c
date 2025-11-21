#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "main.h"
#include "secure_area.h"
#include "eeprom.h"
#include "wrapper.h"
#include "rtcApi.h"
#include "httpserver-socket.h"
#include "configurator.h"

#define HTTP_RESPONSES_LIST                                     \
  ITEM(HTTP_SUCCESS,               "200 OK"                   ) \
  ITEM(HTTP_BAD_REQUEST,           "400 Bad Request"          ) \
  ITEM(HTTP_INTERNAL_SERVER_ERROR, "500 Internal Server Error")
  
typedef enum
{
#define ITEM(id, value) id,
  HTTP_RESPONSES_LIST
#undef ITEM
} HttpResponse;

static const char *httpResponses[] = {
#define ITEM(id, value) value,
  HTTP_RESPONSES_LIST
#undef ITEM
};

typedef struct
{
  char *name;
  char *value;
} parameter_t;

typedef struct
{
  uint32_t recv;
  char *body;
  uint32_t len;
  uint32_t count;
  tcp_recv_fn fn_backup;
  char boundary[71]; // Max boundary RFC specification len for multipart form
  SecureArea *backup;
} configurator_t;

static const char http_version[] = "HTTP/1.1 ";
static const char header[] = "\r\nServer: lwIP/1.3.1"
                             "\r\nContent-Type: text/plain; charset=utf-8"
                             "\r\nConnection: close";

static configurator_t configurator;

static int32_t configurator_getNumSocketFromTechCode(char *techCode)
{
  int32_t res;
  uint32_t len;

  res = 0;
  len = strlen(techCode);
  for(uint32_t i = len - 1; i > 0; i--)
  {
      if(techCode[i] == '.')
          break;
      else
          res += 1;
  }
        
  if((res == 0) || (res > 4))
    res = 1;
  
  return res;
}

err_t configurator_init_config_reception (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  char *ptr, *end;
  uint32_t content_len, header_len;
  
  ptr = (char*)p->payload;
  
  ptr = strstr(ptr, "boundary=");
  if (ptr == NULL)
    return ERR_ARG;
  
  ptr += 9;
  end = strstr(ptr, "\r\n");
  if (end == NULL)
    return ERR_ARG;
  
  memcpy(configurator.boundary, ptr, end - ptr);
  configurator.boundary[end - ptr] = '\0';
  
  ptr = (char*)p->payload;
  
  ptr = strstr(ptr, "Content-Length:");
  if (ptr == NULL)
    return ERR_ARG;
     
  ptr += 15;
  content_len = strtol(ptr, NULL, 10);
  
  configurator.body = (char*)malloc(content_len + 1); // Add '\0' terminating null character
  if (configurator.body == NULL)
    return ERR_MEM;
  
  configurator.body[content_len] = '\0';
  configurator.len = content_len;
  configurator.count = 0;
  
  ptr = strstr(ptr, "\r\n\r\n");
  if (ptr == NULL)
    return ERR_ARG;
  
  ptr += 4;
  header_len = ptr - (char*)p->payload;
  
  memcpy(configurator.body, ptr, p->len - header_len);
  configurator.count = p->len - header_len;
  configurator.body[configurator.count] = '\0';
  configurator.recv = p->len;
  
  configurator.fn_backup = tpcb->recv;
  tcp_recv(tpcb, configurator_handle_config);
  
  configurator.backup = (SecureArea*)malloc(sizeof(SecureArea));
  if (configurator.backup == NULL)
    return ERR_MEM;
  
  return ERR_OK;
}

void configurator_deinit_config_reception (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  if (configurator.body != NULL)
    free(configurator.body);
  if (configurator.backup != NULL)
    free(configurator.backup);
  configurator.len = 0;
  configurator.count = 0;
  configurator.recv = 0;
  tcp_recv(tpcb, configurator.fn_backup);
}

static err_t close_after_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  tcp_close(tpcb);
  return ERR_OK;
}

static void configurator_send_and_close(struct tcp_pcb *tpcb, struct pbuf *p)
{
  if (p != NULL)
  {
    pbuf_free(p);
  }
  
  tcp_sent(tpcb, close_after_sent);
}

static err_t configurator_write_response (struct tcp_pcb *tpcb, struct pbuf *p, HttpResponse resp, uint32_t len)
{
  err_t err;
  char content_len_str[32];

  err = tcp_write(tpcb, http_version, strlen(http_version), 0);
  if (err != ERR_OK) return err;
  err = tcp_write(tpcb, httpResponses[resp], strlen(httpResponses[resp]), 0);
  if (err != ERR_OK) return err;
  err = tcp_write(tpcb, header, strlen(header), 0);
  if (err != ERR_OK) return err;
  if (len != 0) {
    sprintf(content_len_str, "\r\nContent-Length: %u", len);
    err = tcp_write(tpcb, content_len_str, strlen(content_len_str), 0);
  }
  err = tcp_write(tpcb, "\r\n\r\n", 4, 0);
  if (err != ERR_OK) return err;
  
  return ERR_OK;
}

static char * configurator_get_parameter(char *buff, parameter_t * const param)
{
  char *ptr;
  
  ptr = buff;
  
  ptr = strstr(ptr, "name=\"");
  if (ptr == NULL)
    return ptr;
  
  ptr += 6;
  param->name = ptr;
  
  ptr = strchr(ptr, '"');
  if (ptr == NULL)
    return ptr;
  
  *ptr = '\0';
  ptr++;
  
  ptr = strstr(ptr, "\r\n\r\n");
  if (ptr == NULL)
    return ptr;
  
  ptr += 4;
  param->value = ptr;
  
  ptr = strstr(ptr, configurator.boundary);
  if (ptr == NULL)
    return ptr;
  
  ptr -= 2; // remove "\r\n"
  ptr -= 2; // remove two hyphen characters followed by the boundary
  *ptr = '\0';
  ptr += 4;
  
  return ptr;
}

HttpResponse configurator_process_config (void)
{
  int32_t res;
  char *buff;
  parameter_t param;
  uint8_t sn[4] = {0};
  uint32_t timestamp;
  char productCode[16];
  certificate_t header;
  
  buff = configurator.body;
  
  res = SecureArea_readAll(configurator.backup);
  if (res != 0)
    return HTTP_INTERNAL_SERVER_ERROR;
  
  memset(configurator.backup->remoteApSSID, 0, SECAREA_LOCAL_AP_SSID_BYTESIZE);
  memset(configurator.backup->remoteApPass, 0, SECAREA_LOCAL_AP_PASS_BYTESIZE);
  memset(configurator.backup->server_key, 0, SECAREA_SERVER_KEY_BYTESIZE);
  memset(configurator.backup->server_cert, 0, SECAREA_SERVER_CERT_BYTESIZE);
  memset(configurator.backup->ca_cert, 0, SECAREA_CA_CERT_BYTESIZE);
  
  while ((buff = configurator_get_parameter(buff, &param)) != NULL) {
    if (strcmp("scu_serial", param.name) == 0) {
      sn[0] |= (param.value[1] - '0');
      sn[0] |= (param.value[0] - '0') << 4;
      sn[1] |= (param.value[3] - '0');
      sn[1] |= (param.value[2] - '0') << 4;
      sn[2] |= (param.value[5] - '0');
      sn[2] |= (param.value[4] - '0') << 4;
      sn[3] |= (param.value[7] - '0');
      sn[3] |= (param.value[6] - '0') << 4;
      SCU_InfoStation_Set ((uint8_t *)&infoStation.serial, sn, 4);  /* ex SERNUM_BYTE0_EADD */
      res = setScuSerialNumberEeprom((char*)sn, param.value);
    } else if (strcmp("hw_code", param.name) == 0) {
      res = setScuHardwareVersionEeprom(param.value);
    } else if (strcmp("product_serial", param.name) == 0) {
      res = setProductSerialNumberEeprom(param.value, strlen(param.value), TRUE);
    } else if (strcmp("product_code", param.name) == 0) {
      strcpy(productCode, param.value);
      res = setStationProductCode(productCode, strlen(productCode));
    } else if (strcmp("tech_code", param.name) == 0) {
      res = setStationFakeProductCode(param.value, strlen(param.value));
      res = setNumSocketFromFakeCode(configurator_getNumSocketFromTechCode(param.value));
    } else if (strcmp("scu_pass", param.name) == 0) {
      SecureArea_generateSHA256((uint8_t*)param.value, strlen(param.value), configurator.backup->factoryScuPass);
      memcpy(configurator.backup->scuPass, configurator.backup->factoryScuPass, SECAREA_SCU_PASS_BYTESIZE);
    } else if (strcmp("wifi_ssid", param.name) == 0) {
      strncpy(configurator.backup->localApSSID, param.value, SECAREA_LOCAL_AP_PASS_LEN);
    } else if (strcmp("wifi_pass", param.name) == 0) {
      strncpy(configurator.backup->localApPass, param.value, SECAREA_LOCAL_AP_PASS_LEN);
    } else if (strcmp("activation_key", param.name) == 0) {
      strncpy(configurator.backup->activationKey, param.value, SECAREA_ACTIVATION_KEY_LEN);
    } else if (strcmp("timestamp", param.name) == 0) {
      timestamp = strtol(param.value, NULL, 10);
      BKP_SRAM_UnixTimestamp_Save(timestamp);
      setDateTimeFromUnixT(timestamp);
      UpdateGlobalDT();
      sn[0] = 1;
      // xx eeprom_array_set(RTC_VALID_EADD, &sn[0], 1);
      SCU_InfoStation_Set ((uint8_t *)&infoStation.rtcValid, &sn[0], 1);      /* ex RTC_VALID_EADD */
    } else if (strcmp("server_key", param.name) == 0) {
      header.magic = 0xF1F1;
      header.list_size = 1;
      header.id = 0;
      header.type = 3;
      header.content_len = strlen(param.value);
      header.len = header.content_len + 4;
      memcpy(configurator.backup->server_key, (uint8_t*)&header, sizeof(certificate_t));
      strncpy(configurator.backup->server_key + sizeof(certificate_t), param.value, SECAREA_SERVER_KEY_LEN - sizeof(certificate_t));
    } else if (strcmp("server_cert", param.name) == 0) {
      header.magic = 0xF1F1;
      header.list_size = 1;
      header.id = 0;
      header.type = 2;
      header.content_len = strlen(param.value);
      header.len = header.content_len + 4;
      memcpy(configurator.backup->server_cert, (uint8_t*)&header, sizeof(certificate_t));
      strncpy(configurator.backup->server_cert + sizeof(certificate_t), param.value, SECAREA_SERVER_CERT_LEN - sizeof(certificate_t));
    } else if (strcmp("ca_cert", param.name) == 0) {
      header.magic = 0xF1F1;
      header.list_size = 1;
      header.id = 0;
      header.type = 1;
      header.content_len = strlen(param.value);
      header.len = header.content_len + 4;
      memcpy(configurator.backup->ca_cert, (uint8_t*)&header, sizeof(certificate_t));
      strncpy(configurator.backup->ca_cert + sizeof(certificate_t), param.value, SECAREA_CA_CERT_LEN - sizeof(certificate_t));
    }
    
    if (res != 0)
        return HTTP_INTERNAL_SERVER_ERROR;
  }
  
  res = SecureArea_store(configurator.backup);
  if (res != 0)
    return HTTP_INTERNAL_SERVER_ERROR;
  
  getIdModelFromStringAndParConfig(productCode);
  
  return HTTP_SUCCESS;
}

err_t configurator_handle_config (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  HttpResponse resp;
  
  memcpy(configurator.body + configurator.count, p->payload, p->len);
  configurator.count += p->len;
  configurator.recv += p->len;
  configurator.body[configurator.count] = '\0';
  
  if (configurator.count == configurator.len) {
    tcp_recved(tpcb, configurator.recv);
    resp = configurator_process_config();
    configurator_write_response(tpcb, p, resp, 0);
    configurator_send_and_close(tpcb, p);
    configurator_deinit_config_reception(arg, tpcb, p, err);
  }
  else {
    tcp_recv(tpcb, configurator_handle_config);
  }
  return ERR_OK;
}

err_t configurator_get_version(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  uint32_t len;
  
  tcp_recved(tpcb, p->len);
  len = strlen(FW_VERSION);
  configurator_write_response(tpcb, p, HTTP_SUCCESS, 0);
  err = tcp_write(tpcb, FW_VERSION, len, 0);
  configurator_send_and_close(tpcb, p);
  return err;
}

/**
  * @brief  store in secure area the info about wifi and web server   
  *         
* @param [in]   wifiWebInfo *: pointer to struct with all info (scuPass, ssid, pass, activation key)
  * 
  * @retval none
  */
int32_t Configurator_saveWifiWebInfo(wifiWebInfo * const wifiWebConfig)
{
  SecureArea            backup;
  int32_t res;
 
  /* Secure area configuration */
  res = SecureArea_readAll(&backup);
  if(res == 0)
  {
    memset(backup.remoteApSSID, 0, SECAREA_LOCAL_AP_SSID_BYTESIZE);
    memset(backup.remoteApPass, 0, SECAREA_LOCAL_AP_PASS_BYTESIZE);
    
    strncpy(backup.activationKey, wifiWebConfig->key, SECAREA_ACTIVATION_KEY_LEN);
    backup.activationKey[SECAREA_ACTIVATION_KEY_LEN] = '\0';
    
    strncpy(backup.localApSSID, wifiWebConfig->ssid, SECAREA_LOCAL_AP_SSID_LEN);
    backup.localApSSID[SECAREA_LOCAL_AP_SSID_LEN] = '\0';
    
    strncpy(backup.localApPass, wifiWebConfig->pass, SECAREA_LOCAL_AP_PASS_LEN);
    backup.localApPass[SECAREA_LOCAL_AP_PASS_LEN] = '\0';
    
    SecureArea_generateSHA256((uint8_t*)wifiWebConfig->scuPass, strlen(wifiWebConfig->scuPass), backup.factoryScuPass);
    memcpy(backup.scuPass, backup.factoryScuPass, SECAREA_SCU_PASS_BYTESIZE);
    
    res = SecureArea_store(&backup);
  }
  return (res);
}