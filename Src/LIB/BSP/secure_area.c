#include "secure_area.h"
#include "wrapper.h"
#include "flash_if.h"

#define SECAREA_MAGIC_NUMBER            0x11AACD16

static void SecureArea_initLocalApSSID(SecureArea * const sa);
static void SecureArea_initLocalApPass(SecureArea * const sa);
static void SecureArea_initRemoteApSSID(SecureArea * const sa);
static void SecureArea_initRemoteApPass(SecureArea * const sa);
static void SecureArea_initActivationKey(SecureArea * const sa);
static void SecureArea_initUserPin(SecureArea * const sa);
static void SecureArea_initScuPass(SecureArea * const sa);

static SecureArea secureArea @ ".secure_area";
#define pSecArea (&secureArea)

#define SECURE_AREA_SIZE          ((uint32_t)0x00004000)
#define SECURE_AREA_ADDRESS_START ((uint32_t)&secureArea)
#define SECURE_AREA_ADDRESS_END   (SECURE_AREA_ADDRESS_START + SECURE_AREA_SIZE - 1)

extern infoStation_t infoStation;

void SecureArea_init(void)
{
  if(pSecArea->magic != SECAREA_MAGIC_NUMBER)
  {
    SecureArea *backup = (SecureArea*)malloc(sizeof(SecureArea));
    if(backup == NULL)
      return;
    SecureArea_initLocalApSSID(backup);
    SecureArea_initLocalApPass(backup);
    SecureArea_initRemoteApSSID(backup);
    SecureArea_initRemoteApPass(backup);
    SecureArea_initActivationKey(backup);
    SecureArea_initUserPin(backup);
    SecureArea_initScuPass(backup);
    memset(backup->server_key, 0, SECAREA_SERVER_KEY_BYTESIZE);
    memset(backup->server_cert, 0, SECAREA_SERVER_CERT_BYTESIZE);
    memset(backup->ca_cert, 0, SECAREA_CA_CERT_BYTESIZE);
    SecureArea_store(backup);
    free(backup);
  }
}

static void SecureArea_initLocalApSSID(SecureArea * const sa)
{
  strcpy(sa->localApSSID, "ChargePointScame ");
  strcpy(&sa->localApSSID[17], infoStation.productSn);
}

static void SecureArea_initLocalApPass(SecureArea * const sa)
{
  strcpy(sa->localApPass, "SCUwifi1963!");
}

static void SecureArea_initRemoteApSSID(SecureArea * const sa)
{
  strncpy(sa->remoteApSSID, infoStation.routerSsid, SECAREA_REMOTE_AP_SSID_LEN);
  sa->remoteApSSID[SECAREA_REMOTE_AP_SSID_LEN] = '\0';
}

static void SecureArea_initRemoteApPass(SecureArea * const sa)
{
  strncpy(sa->remoteApPass, infoStation.routerPass, SECAREA_REMOTE_AP_PASS_LEN);
  sa->remoteApPass[SECAREA_REMOTE_AP_PASS_LEN] = '\0';
}

static void SecureArea_initActivationKey(SecureArea * const sa)
{
  uint32_t i;
  uint8_t pi[] = {3, 1, 4, 1, 5, 9, 2, 6, 5};
  uint8_t sum[sizeof(pi)] = {0};
  
  for(i = 0; i < sizeof(pi); i++)
  {
    sum[i] = infoStation.productSn[i] - '0';
  }
  
  for(i = 0; i < sum[sizeof(pi) - 1]; i++)
  {
    uint8_t tmp = pi[sizeof(pi) - 1];
    for(uint32_t j = (sizeof(pi) - 1); j > 0; j--)
    {
      pi[j] = pi[j - 1];
    }
    pi[0] = tmp;
  }
  
  for(i = 0; i < sizeof(pi); i++)
  {
    uint8_t tmp = sum[i] + pi[i];
    sum[i] = (tmp < 10) ? tmp : ( tmp % 10 );
  }
  
  sa->activationKey[0] = sum[3] + '0';
  sa->activationKey[1] = sum[4] + '0';
  sa->activationKey[2] = sum[6] + '0';
  sa->activationKey[3] = sum[7] + '0';
  sa->activationKey[4] = sum[8] + '0';
  sa->activationKey[5] = '\0';
}

static void SecureArea_initUserPin(SecureArea * const sa)
{
  strncpy(sa->userPin, infoStation.userPin, SECAREA_USER_PIN_LEN);
  sa->userPin[SECAREA_USER_PIN_LEN] = '\0';
}

static void SecureArea_initScuPass(SecureArea * const sa)
{
  uint8_t initPass[] = "Scame01";
  
  SecureArea_generateSHA256(initPass, sizeof(initPass) - 1, sa->factoryScuPass);
  memcpy(sa->scuPass, sa->factoryScuPass, SECAREA_SCU_PASS_BYTESIZE);
}

int32_t SecureArea_generateSHA256(uint8_t *msg, uint32_t len, uint8_t *hash)
{
  cmox_hash_retval_t retval;
  size_t computed_size;
  
  cmox_init_arg_t init_target = {CMOX_INIT_TARGET_AUTO, NULL};
  
  /* Initialize cryptographic library */
  if (cmox_initialize(&init_target) != CMOX_INIT_SUCCESS)
  {
    return -1;
  }
  
  /* Compute directly the digest passing all the needed parameters */
  retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                             msg, len,
                             hash,
                             CMOX_SHA256_SIZE,
                             &computed_size);
  
  /* Verify API returned value */
  if (retval != CMOX_HASH_SUCCESS)
  {
    return -1;
  }
  
  /* Verify generated data size is the expected one */
  if (computed_size != CMOX_SHA256_SIZE)
  {
    return -1;
  }
  
  /* No more need of cryptographic services, finalize cryptographic library */
  if (cmox_finalize(NULL) != CMOX_INIT_SUCCESS)
  {
    return -1;
  }
  
  return 0;
}

int32_t SecureArea_readAll(SecureArea * const sa)
{
  int32_t res = 0;
  
  sa->magic = pSecArea->magic;
  res = SecureArea_getLocalApSSID(sa->localApSSID, SECAREA_LOCAL_AP_SSID_BYTESIZE);
  if(res == 0)
    res = SecureArea_getLocalApPass(sa->localApPass, SECAREA_LOCAL_AP_PASS_BYTESIZE);
  if(res == 0)
    res = SecureArea_getRemoteApSSID(sa->remoteApSSID, SECAREA_REMOTE_AP_SSID_BYTESIZE);
  if(res == 0)
    res = SecureArea_getRemoteApPass(sa->remoteApPass, SECAREA_REMOTE_AP_PASS_BYTESIZE);
  if(res == 0)
    res = SecureArea_getActivationKey(sa->activationKey, SECAREA_ACTIVATION_KEY_BYTESIZE);
  if(res == 0)
    res = SecureArea_getUserPin(sa->userPin, SECAREA_USER_PIN_BYTESIZE);
  if(res == 0)
  {
    memcpy(sa->factoryScuPass, pSecArea->factoryScuPass, SECAREA_SCU_PASS_BYTESIZE);
    memcpy(sa->scuPass, pSecArea->scuPass, SECAREA_SCU_PASS_BYTESIZE);
    memcpy(sa->server_key, pSecArea->server_key, SECAREA_SERVER_KEY_BYTESIZE);
    memcpy(sa->server_cert, pSecArea->server_cert, SECAREA_SERVER_CERT_BYTESIZE);
    memcpy(sa->ca_cert, pSecArea->ca_cert, SECAREA_CA_CERT_BYTESIZE);
  }
  
  return res;
}

int32_t SecureArea_getLocalApSSID(char *ssid, uint32_t len)
{
  if(len > SECAREA_LOCAL_AP_SSID_BYTESIZE)
    len = SECAREA_LOCAL_AP_SSID_BYTESIZE;
  
  strncpy(ssid, pSecArea->localApSSID, len);
  return 0;
}

int32_t SecureArea_getLocalApPass(char *pass, uint32_t len)
{
  if(len > SECAREA_LOCAL_AP_PASS_BYTESIZE)
    len = SECAREA_LOCAL_AP_PASS_BYTESIZE;
  
  strncpy(pass, pSecArea->localApPass, len);
  return 0;
}

int32_t SecureArea_getRemoteApSSID(char *ssid, uint32_t len)
{
  if(len > SECAREA_REMOTE_AP_SSID_BYTESIZE)
    len = SECAREA_REMOTE_AP_SSID_BYTESIZE;
  
  strncpy(ssid, pSecArea->remoteApSSID, len);
  return 0;
}

int32_t SecureArea_getRemoteApPass(char *pass, uint32_t len)
{
  if(len > SECAREA_REMOTE_AP_PASS_BYTESIZE)
    len = SECAREA_REMOTE_AP_PASS_BYTESIZE;
  
  strncpy(pass, pSecArea->remoteApPass, len);
  return 0;
}

int32_t SecureArea_getActivationKey(char *key, uint32_t len)
{
  if(len > SECAREA_ACTIVATION_KEY_BYTESIZE)
    len = SECAREA_ACTIVATION_KEY_BYTESIZE;
    
  strncpy(key, pSecArea->activationKey, len);
  return 0;
}

int32_t SecureArea_getUserPin(char *pin, uint32_t len)
{
  if(len > SECAREA_USER_PIN_BYTESIZE)
    len = SECAREA_USER_PIN_BYTESIZE;
    
  strncpy(pin, pSecArea->userPin, len);
  return 0;
}

/**
  * @brief  set the new password for account user in SCU web server   
  *         
  * @param  char*:    pointer to password string
  * @param  uint16_t: length of to password string
  * 
  * @retval uint8_t: 0 when successfull write, error code otherwise  
  */
int32_t SecureArea_storeWebPassword(char *stringPass, uint32_t len)
{
  int32_t     res;
  
  SecureArea *backup = (SecureArea*)malloc(sizeof(SecureArea));
  if(backup == NULL)
    return -1;
  
  res = SecureArea_readAll(backup);
  if(res == 0)
  {
    res = SecureArea_generateSHA256((uint8_t*)stringPass, strlen(stringPass), backup->scuPass);
    if(res == 0)
    {
        /* new password must be different from current password */
        res = SecureArea_store(backup);
    }
  }
  
  free(backup);
  
  return res;
}


int32_t SecureArea_storeUserPin(char *pin, uint32_t len)
{
  int32_t res;
  
  if(len > SECAREA_USER_PIN_LEN)
    return -1;
  
  SecureArea *backup = (SecureArea*)malloc(sizeof(SecureArea));
  if(backup == NULL)
    return -1;
  
  res = SecureArea_readAll(backup);
  if(res == 0)
  {
    if(strncmp(pin, backup->userPin, SECAREA_USER_PIN_LEN) != 0)
    {
      strncpy(backup->userPin, pin, SECAREA_USER_PIN_LEN);
      backup->userPin[SECAREA_USER_PIN_LEN] = '\0';
      res = SecureArea_store(backup);
    }
  }
  
  free(backup);
  
  return res;
}

int32_t SecureArea_storeLocalAp(char *ssid, char *pass)
{
  int32_t res;

  if(strlen(ssid) > SECAREA_LOCAL_AP_SSID_LEN)
    return -1;

  if(strlen(pass) > SECAREA_LOCAL_AP_PASS_LEN)
    return -1;
  
  SecureArea *backup = (SecureArea*)malloc(sizeof(SecureArea));
  if(backup == NULL)
    return -1;

  res = SecureArea_readAll(backup);
  if(res == 0)
  {
    if((strncmp(ssid, backup->localApSSID, SECAREA_LOCAL_AP_SSID_LEN) != 0) ||
       (strncmp(pass, backup->localApPass, SECAREA_LOCAL_AP_PASS_LEN) != 0))
    {
      strncpy(backup->localApSSID, ssid, SECAREA_LOCAL_AP_SSID_LEN);
      backup->localApSSID[SECAREA_LOCAL_AP_SSID_LEN] = '\0';
      strncpy(backup->localApPass, pass, SECAREA_LOCAL_AP_PASS_LEN);
      backup->localApPass[SECAREA_LOCAL_AP_PASS_LEN] = '\0';
      res = SecureArea_store(backup);
    }
  }
  
  free(backup);
    
  return res;
}

int32_t SecureArea_storeRemoteAp(char *ssid, char *pass)
{
  int32_t res;

  if(strlen(ssid) > SECAREA_REMOTE_AP_SSID_LEN)
    return -1;

  if(strlen(pass) > SECAREA_REMOTE_AP_PASS_LEN)
    return -1;
  
  SecureArea *backup = (SecureArea*)malloc(sizeof(SecureArea));
  if(backup == NULL)
    return -1;

  res = SecureArea_readAll(backup);
  if(res == 0)
  {
    if((strncmp(ssid, backup->remoteApSSID, SECAREA_REMOTE_AP_SSID_LEN) != 0) ||
       (strncmp(pass, backup->remoteApPass, SECAREA_REMOTE_AP_PASS_LEN) != 0))
    {
      strncpy(backup->remoteApSSID, ssid, SECAREA_REMOTE_AP_SSID_LEN);
      backup->remoteApSSID[SECAREA_REMOTE_AP_SSID_LEN] = '\0';
      strncpy(backup->remoteApPass, pass, SECAREA_REMOTE_AP_PASS_LEN);
      backup->remoteApPass[SECAREA_REMOTE_AP_PASS_LEN] = '\0';
      res = SecureArea_store(backup);
    }
  }
  
  free(backup);
  
  return res;
}

int32_t SecureArea_storeScuPass(char *pass)
{
  int32_t res;
  
  SecureArea *backup = (SecureArea*)malloc(sizeof(SecureArea));
  if(backup == NULL)
    return -1;
  
  res = SecureArea_readAll(backup);
  if(res == 0)
  {
    res = SecureArea_generateSHA256((uint8_t*)pass, strlen(pass), backup->scuPass);
    if(res == 0)
    {
      res = SecureArea_store(backup);
    }
  }
  
  free(backup);
  
  return res;
}

int32_t SecureArea_resetRemoteAp(void)
{
  int32_t res;
  
  SecureArea *backup = (SecureArea*)malloc(sizeof(SecureArea));
  if(backup == NULL)
    return -1;
  
  res = SecureArea_readAll(backup);
  if(res == 0)
  {
    if((strlen(backup->remoteApSSID) != 0) || (strlen(backup->remoteApPass) != 0))
    {
      memset(backup->remoteApSSID, 0, SECAREA_REMOTE_AP_SSID_BYTESIZE);
      memset(backup->remoteApPass, 0, SECAREA_REMOTE_AP_PASS_BYTESIZE);
      res = SecureArea_store(backup);
    }
  }
  
  free(backup);
  
  return res;
}

int32_t SecureArea_restoreDefault(void)
{
  int32_t res;
  
  SecureArea *backup = (SecureArea*)malloc(sizeof(SecureArea));
  if(backup == NULL)
    return -1;
  
  res = SecureArea_readAll(backup);
  if(res == 0)
  {
    memset(backup->userPin, 0, SECAREA_USER_PIN_BYTESIZE);
    memset(backup->remoteApSSID, 0, SECAREA_REMOTE_AP_SSID_BYTESIZE);
    memset(backup->remoteApPass, 0, SECAREA_REMOTE_AP_PASS_BYTESIZE);
    memcpy(backup->scuPass, backup->factoryScuPass, SECAREA_SCU_PASS_BYTESIZE);
    res = SecureArea_store(backup);
  }
  
  free(backup);
  
  return res;
}

int32_t SecureArea_clearAll(void)
{
  HAL_StatusTypeDef res;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SECTORError;
  
  HAL_FLASH_Unlock();
  
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = 2;
  EraseInitStruct.NbSectors = 1;
  
  res = HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
  
  HAL_FLASH_Lock();
  
  return res;
}

int32_t SecureArea_store(SecureArea * const sa)
{
  int32_t res;
  uint32_t i, address;
  
  sa->magic = SECAREA_MAGIC_NUMBER;
  
  res = SecureArea_clearAll();
  if( res == 0 ) {
    HAL_FLASH_Unlock();
    for(i = 0, address = SECURE_AREA_ADDRESS_START; 
        (i < (sizeof(SecureArea)/4)) && (address <= SECURE_AREA_ADDRESS_END);
        i++)
    {
      if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, *(((uint32_t*)sa) + i)) == HAL_OK)
      {
        address += 4;
      }
      else
      {
        res = -1;
        break;
      }
    }
    HAL_FLASH_Lock();
  }
  return res;
}

int32_t SecureArea_verifyActivationKey(char *key)
{
  return strncmp(key, pSecArea->activationKey, SECAREA_ACTIVATION_KEY_LEN);
}

int32_t SecureArea_verifyUserPin(char *pin)
{
  return strncmp(pin, pSecArea->userPin, SECAREA_USER_PIN_LEN);
}

int32_t SecureArea_verifyScuPass(char *pass)
{
  int32_t res;
  uint8_t sha[CMOX_SHA256_SIZE];
  
  res = SecureArea_generateSHA256((uint8_t*)pass, strlen(pass), sha);
  if(res == 0)
  {
    res = memcmp(sha, pSecArea->scuPass, CMOX_SHA256_SIZE);
  }
  
  return res;
}

certificate_t * SecureArea_getServerKey (void)
{
  return (certificate_t*)secureArea.server_key;
}

certificate_t * SecureArea_getServerCert (void)
{
  return (certificate_t*)secureArea.server_cert;
}

certificate_t * SecureArea_getCaCert (void)
{
  return (certificate_t*)secureArea.ca_cert;
}