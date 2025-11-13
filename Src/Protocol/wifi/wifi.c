#include "wifi.h"

const char *wifiEncStr[] = {
#define ITEM( enc, text ) text,
  LIST_WIFI_ENCRYPTION
#undef ITEM
};

const char * wifiEncryptionToStr( WifiEncryption enc )
{
  return wifiEncStr[ enc ];
}
    
int32_t Version_compare( Version *v1, Version *v2 )
{
  int32_t v_1, v_2;
  
  v_1 = ( v1->major << 24 ) | ( v1->minor << 16 ) | ( v1->patch << 8 ) | ( v1->hotfix );
  v_2 = ( v2->major << 24 ) | ( v2->minor << 16 ) | ( v2->patch << 8 ) | ( v2->hotfix );
  
  return (v_1 - v_2);
}