#ifndef __SECURE_AREA__
#define __SECURE_AREA__

#include <stdint.h>
#include "cmox_crypto.h"

#define SECAREA_LOCAL_AP_SSID_BYTESIZE  32 // worst case scenario: "ChargePointScame 500044886/001"
#define SECAREA_LOCAL_AP_PASS_BYTESIZE  16
#define SECAREA_REMOTE_AP_SSID_BYTESIZE 32
#define SECAREA_REMOTE_AP_PASS_BYTESIZE 64
#define SECAREA_ACTIVATION_KEY_BYTESIZE 6
#define SECAREA_USER_PIN_BYTESIZE       6
#define SECAREA_SCU_PASS_BYTESIZE       CMOX_SHA256_SIZE
#define SECAREA_SERVER_KEY_BYTESIZE     0x200
#define SECAREA_SERVER_CERT_BYTESIZE    0x600
#define SECAREA_CA_CERT_BYTESIZE        0x600

#define SECAREA_LOCAL_AP_SSID_LEN       (SECAREA_LOCAL_AP_SSID_BYTESIZE - 1)
#define SECAREA_LOCAL_AP_PASS_LEN       (SECAREA_LOCAL_AP_PASS_BYTESIZE - 1)
#define SECAREA_REMOTE_AP_SSID_LEN      (SECAREA_REMOTE_AP_SSID_BYTESIZE - 1)
#define SECAREA_REMOTE_AP_PASS_LEN      (SECAREA_REMOTE_AP_PASS_BYTESIZE - 1)
#define SECAREA_ACTIVATION_KEY_LEN      (SECAREA_ACTIVATION_KEY_BYTESIZE - 1)
#define SECAREA_USER_PIN_LEN            (SECAREA_USER_PIN_BYTESIZE - 1)
#define SECAREA_SERVER_KEY_LEN          (SECAREA_SERVER_KEY_BYTESIZE - 1)
#define SECAREA_SERVER_CERT_LEN         (SECAREA_SERVER_CERT_BYTESIZE - 1)
#define SECAREA_CA_CERT_LEN             (SECAREA_CA_CERT_BYTESIZE - 1)

/*
  This structure must be aligned to 32 byte compliant with internal flash.
*/
typedef __packed struct
{
  uint16_t magic;
  uint16_t list_size;
  uint32_t len;
  uint8_t type;
  uint8_t id;
  uint16_t content_len;
  uint8_t content[];
} certificate_t;

typedef __packed struct
{
  uint32_t magic;
  char localApSSID[SECAREA_LOCAL_AP_SSID_BYTESIZE];
  char localApPass[SECAREA_LOCAL_AP_PASS_BYTESIZE];
  char remoteApSSID[SECAREA_REMOTE_AP_SSID_BYTESIZE];
  char remoteApPass[SECAREA_REMOTE_AP_PASS_BYTESIZE];
  char activationKey[SECAREA_ACTIVATION_KEY_BYTESIZE];
  char userPin[SECAREA_USER_PIN_BYTESIZE];
  /* HASHED INFO */
  uint8_t factoryScuPass[SECAREA_SCU_PASS_BYTESIZE];
  uint8_t scuPass[SECAREA_SCU_PASS_BYTESIZE];
  /* TLS server certificates */
  char server_key[SECAREA_SERVER_KEY_BYTESIZE];
  char server_cert[SECAREA_SERVER_CERT_BYTESIZE];
  char ca_cert[SECAREA_CA_CERT_BYTESIZE];
} SecureArea;

void SecureArea_init(void);

int32_t SecureArea_verifyActivationKey(char *key);

int32_t SecureArea_verifyUserPin(char *pin);

int32_t SecureArea_verifyScuPass(char *pass);

int32_t SecureArea_readAll(SecureArea * const sa);

int32_t SecureArea_store(SecureArea * const sa);

int32_t SecureArea_getLocalApSSID(char *ssid, uint32_t len);

int32_t SecureArea_getLocalApPass(char *pass, uint32_t len);

int32_t SecureArea_getRemoteApSSID(char *ssid, uint32_t len);

int32_t SecureArea_getRemoteApPass(char *pass, uint32_t len);

int32_t SecureArea_getActivationKey(char *key, uint32_t len);

int32_t SecureArea_getUserPin(char *pin, uint32_t len);

int32_t SecureArea_storeUserPin(char *pin, uint32_t len);

int32_t SecureArea_storeLocalAp(char *ssid, char *pass);

int32_t SecureArea_storeRemoteAp(char *ssid, char *pass);

int32_t SecureArea_storeScuPass(char *pass);

int32_t SecureArea_resetRemoteAp(void);

int32_t SecureArea_restoreDefault(void);

int32_t SecureArea_clearAll(void);

int32_t SecureArea_storeWebPassword(char *stringPass, uint32_t len);

int32_t SecureArea_generateSHA256(uint8_t *msg, uint32_t len, uint8_t *hash);

certificate_t * SecureArea_getServerKey (void);

certificate_t * SecureArea_getServerCert (void);

certificate_t * SecureArea_getCaCert (void);

#endif