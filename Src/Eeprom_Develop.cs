/**
  * @brief  get stations general parameters   
  *         
  * @param  infoStation_t*: pointer where store data
  * 
  * @retval none
  */
void setGeneralStationParameters(uint8_t Type)
{
    unsigned char SerNum[4];
    infoStation_t* pInfoStation;
    uint32_t val;
    uint8_t tmp, keySN;

    // recupero i parametri generale della stazione / WB
    ReadFromEeprom(SCU_GENERAL_INFO_EE_ADDRES, (uint8_t*)pInfoStation, sizeof(infoStation_t));
    ReadFromEeprom(SN_KEY_EE_ADDRES, (uint8_t*)&keySN, 1);     // recupero key  SN 

    if ((pInfoStation->key != KEY_FOR_INFOSTATION_V0) && (pInfoStation->key != KEY_FOR_INFOSTATION_V1)
        && (pInfoStation->key != KEY_FOR_INFOSTATION_V2) && (pInfoStation->key != KEY_FOR_INFOSTATION_V3)
        && (pInfoStation->key != KEY_FOR_INFOSTATION_V5))
    {
        resetSpareMpuArea();

        /* Force DEFAULT value into the param */
        SRAM_Param_DEFAULT_Set(&pInfoStation->serial[0], 'F', BOARD_SN_LENGTH);
        /* Force DEFAULT value into the param */
        SRAM_Param_DEFAULT_Set(&pInfoStation->userPin[0], ' ', USER_PIN_LENGTH);
        /* Force DEFAULT value into the param */
        SRAM_Param_DEFAULT_Set(&pInfoStation->routerSsid[0], 0, MAX_ROUTER_SSID_LENGTH);
        /* Force DEFAULT value into the param */
        SRAM_Param_DEFAULT_Set(&pInfoStation->routerPass[0], 0, MAX_ROUTER_PASS_LENGTH);
        /* Force DEFAULT value into the param */
        SRAM_Param_DEFAULT_Set(&pInfoStation->productSn[0], '0', PRODUCT_SN_LENGTH);
        /* Force DEFAULT value into the param */
        SRAM_Param_DEFAULT_Set(&pInfoStation->productCode[0], 0, PRODUCT_CODE_LENGTH);
        /* Force DEFAULT value into the param */
        SRAM_Param_DEFAULT_Set(&pInfoStation->fakeProductCode[0], 0, FAKE_CODE_LENGTH);

        /*         destination       source */
        strcpy((char*)pInfoStation->name, (char*)"ChargePoint  \0");
        /* reset info on energy meter */
        pInfoStation->emTypeExt = pInfoStation->emTypeInt = UNKNOW;
        /* reset authorization */
        strcpy((char*)pInfoStation->auth.user, (char*)"");
        strcpy((char*)pInfoStation->auth.pass, (char*)"");
        pInfoStation->auth.auth_state = NO_AUTH;
        /* reset schedulation */
        for (int i = 0; i < MAX_SCHEDULATION_NUMBER; i++)
        {
            pInfoStation->scheds[i].days = 0;
            pInfoStation->scheds[i].id = 0;
            pInfoStation->scheds[i].start_hour = 0;
            pInfoStation->scheds[i].start_min = 0;
            pInfoStation->scheds[i].end_hour = 0;
            pInfoStation->scheds[i].end_min = 0;
            pInfoStation->scheds[i].power = 0;
            pInfoStation->scheds[i].enable = 0;
        }
        pInfoStation->socketActivatedFlag = 0;

        val = HAL_RTCEx_BKUPRead((RTC_HandleTypeDef*)getHandleRtc(), BACKUP_HW_INFO);
        if ((val & RESET_WIFI_ANT_MASK) == RESET_WIFI_ANT_VALID)
        {
            pInfoStation->antennaPresence = WIFI_ANTENNA_TEST_DONE;

            val &= (~RESET_WIFI_ANT_MASK);
            /* reset in RTC the flag for "test wifi antenna presence"  */
            HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_HW_INFO, val);
        }

        /* Adjust IRC16M settings */
        if ((val & IRC16M_OSC_MASK) == IRC16M_OSC_VALID_FLAG)
        {
            /* clear IRC16M flag --> --> use HXTAL */
            val &= ~IRC16M_OSC_ENABLE_FLAG;
            /* write setting in BKP registers */
            HAL_RTCEx_BKUPWrite((RTC_HandleTypeDef*)getHandleRtc(), (uint32_t)BACKUP_HW_INFO, val);
        }

        /* save new parameter in reserved EEPROM area */
        if (keySN == (uint8_t)0xA6)
        {
            if (ReadFromEeprom(PRD_SN_EE_ADDRES, (uint8_t*)pInfoStation->productSn, PRODUCT_SN_LENGTH) == osOK)    // recupero product SN 
            {
                if (pInfoStation->productSn[0] != (uint8_t)0xFF)
                {
                    pInfoStation->productSn[(PRODUCT_SN_LENGTH - 1)] = '\0';
                    tmp = WriteOnEeprom(PRD_SN_EE_ADDRES, (uint8_t*)pInfoStation->productSn, PRODUCT_SN_LENGTH);
                    if (tmp == osOK)
                    {
                        tPrintf("Reset Parametri InfoStation!!\n\r");
                        EVLOG_Message(EV_INFO, "Reset Parametri InfoStation!!");
                    }
                }
            }
        }

        /* set key and value for restore wifi module to factory parameters */
        pInfoStation->keyForRestoreModule = 0xA9;
        pInfoStation->restoreModule = (uint8_t)TRUE;

        pInfoStation->channelId = 1;

        /* Null key */
        pInfoStation->toRange1.keyValue = 0;

        /* default dummy value in the field    */
        pInfoStation->startTimeWebCollaudo = 0xFFFFFFFF;

        pInfoStation->key = KEY_FOR_INFOSTATION_V5;
        /*              destination                source                   len   */
        memCpyInfoSt((uint8_t*)&infoStation, (uint8_t*)pInfoStation, sizeof(infoStation_t));
    }

    switch (pInfoStation->key)
    {
        case KEY_FOR_INFOSTATION_V0:
        case KEY_FOR_INFOSTATION_V1:
            /* new FW, with new infostation structure, read for the first time a previous structure version */
            /* it is necessary to copy the product SN (100xxxxxx) from the old position in the new position  */
            memset((void*)pInfoStation->productSn, 0, sizeof(infoStation.productSn));
            /*              destination                             source                  len   */
            memcpy((void*)pInfoStation->productSn, (void*)pInfoStation->italyProductSn, sizeof(infoStation.italyProductSn));

        case KEY_FOR_INFOSTATION_V2:
            /* also put 0 in new field   */
            pInfoStation->toRange1.keyValue = pInfoStation->toRange1.timeRangeVal = 0;
            break;

        case KEY_FOR_INFOSTATION_V3:
            /* default dummy value in the field    */
            pInfoStation->startTimeWebCollaudo = DUMMY_INFO_VAL;
            break;

        default:
            break;
    }

    if (pInfoStation->key != KEY_FOR_INFOSTATION_VX)
        pInfoStation->key = KEY_FOR_INFOSTATION_VX;

    /*         destination       source */
    strcpy((char*)pInfoStation->firmware, (char*)getFwVer());
    strcat((char*)pInfoStation->firmware, (char*)" ");
    strcat((char*)pInfoStation->firmware, (char*)getScuHWverFromEeprom());
    /*                                             destination                             source           5 = len("Vw.ya" or "Vw.y\0")   */
    memcpy((void*)(((fwInfoVersion_u*)pInfoStation->firmware)->fwBootVer.bootVer), (void*)BOOT_ADDR_VER, (size_t)BOOT_VER_SIZE + 1);

    /* check product serial number in reserved area */
    if (keySN != (uint8_t)0xA6)
    {
        keySN = (uint8_t)0xA6;
        /* save on EEPROM SCU SN */
        tmp = WriteOnEeprom(SCU_SN_EE_ADDRES, (uint8_t*)SerNum, 4);
        /* save on EEPROM PRD SN */
        tmp |= WriteOnEeprom(PRD_SN_EE_ADDRES, (uint8_t*)pInfoStation->productSn, PRODUCT_SN_LENGTH);
        /* save on key  SN */
        tmp |= WriteOnEeprom(SN_KEY_EE_ADDRES, (uint8_t*)&keySN, 1);
        if (tmp == osOK)
        {
            tPrintf("Recovery SNs done!!\n\r");
            EVLOG_Message(EV_INFO, "Recovery SN done!!");
        }
        writeEEon = (uint8_t)TRUE;
    }

    /* Check if a different default value (oxFF) is on productSn, productCode, fakeProductCode 
       Starting from v4.3.x and 4.6.x, the default value for these parameters is ' ' and not 0xFF */
    SRAM_Check_DEFAULT_of_Code();

    /* 1) Initializes some modbus registers with socket/station parameters */
    initModbusRegisters();
}

/**
  * @brief  Config_Update  
  *         
  *   Update station config informations in both RAM and EEPROM
  *         
  * @param  Addr where to store data - Data to store
  * 
  * @retval none
  */

void Config_Update (uint8_t EEpromAddr, infoStation_t* pCfgData, uint8_t* pData)
{
    uint8_t mdbAddr, tmp;

    switch (EEpromAddr)
    {
        case RS485_ADD_EADD:                    /* AGGIUNGERE PARAMETRO IN INFOSTATION !!!*/
            /* Update RS485 address */
            scuAddr = *pData;
            /* Check RS485 address */
            if (scuAddr == (SCU_S_REPL_ADDR - 1))
            {
                /* jolly SCU has display number 99 but logic reference is 16 */
                scuAddr = RS485_ADD_MAX;
            }
            break;

        case SOCKET_TYPE_EADD:
            /* Update socket information  */
            pCfgData->wiring = (sck_wiring_e) *pData;
            break;

        case M3T_CURRENT_EADD:
            pCfgData->max_current = (int32_t) *pData * 1000;
            break;

        case M3S_CURRENT_EADD:
            pCfgData->max_currentSemp = (int32_t) *pData * 1000;
            break;

        case EVS_MODE_EADD:
            pCfgData->evs_mode = (evs_mode_en) *pData;
            break;

        case HIDDEN_MENU_ENB_EADD:
            pCfgData->pmModeEn = (*pData & HIDDEN_MENU_PMNG_ENB);
            break;

        case PMNG_UNBAL_EADD:
            pCfgData->pmUnbalEn = (power_management_unbal_en) *pData;
            break;

        case BATTERY_CONFIG_EADD:
            pCfgData->batteryBackup = (batteryBackup_e) *pData;
            break;

        case SERNUM_BYTE0_EADD:
            // point to Serial number
            ptr = (char*)pCfgData->serial;
            /*       destination       source     normLen  */
            memcpy((void*)ptr, (void*)"FFFFFFFFF", (size_t)8);
            if (*pData[3] != 0xFF)
            {
                ptr[1] = (*pData[0] & 0x0F) + '0';
                ptr[0] = ((*pData[0] >> 4) & 0x0F) + '0';
            }
            if (*pData[2] != 0xFF)
            {
                ptr[3] = (*pData[1] & 0x0F) + '0';
                ptr[2] = ((*pData[1] >> 4) & 0x0F) + '0';
            }
            if (*pData[1] != 0xFF)
            {
                ptr[5] = (*pData[2] & 0x0F) + '0';
                ptr[4] = ((*pData[2] >> 4) & 0x0F) + '0';
            }
            if (*pData[0] != 0xFF)
            {
                ptr[7] = (*pData[3] & 0x0F) + '0';
                ptr[6] = ((*pData[3] >> 4) & 0x0F) + '0';
            }
            break;

        default:
            break;  

    }

    /* set station V230 control from CONTROL_BYTE1_EADD bit  VBUS_CRL1  */
    eeprom_param_get(CONTROL_BYTE1_EADD, (uint8_t*)&tmp, 1);
    tmp = (((uint8_t)tmp & (uint8_t)VBUS_CRL1) == (uint8_t)VBUS_CRL1) ? ENABLED : DISABLED;
    pCfgData->v230MonFlag = (statusFlag_e)tmp;

    if (pCfgData->channelId < WIFI_AP_MIN_CHANNEL_ID || pCfgData->channelId > WIFI_AP_MAX_CHANNEL_ID)
        pCfgData->channelId = WIFI_AP_MIN_CHANNEL_ID;

    /* get wifi enable flag  from LCD_TYPE_EADD   */
    eeprom_param_get(LCD_TYPE_EADD, (uint8_t*)&tmp, 1);
    /* set the operating mode between WiFi and SBC */
    if ((tmp & SBC_WIFI_MASK) == SBC_WIFI_ON)
        setWifiSbcEnv(SBC_WIFI_ON);
    else
        setWifiSbcEnv(SBC_WIFI_OFF);

    /*         destination       source */
    strcpy((char*)pCfgData->firmware, (char*)getFwVer());
    strcat((char*)pCfgData->firmware, (char*)" ");
    strcat((char*)pCfgData->firmware, (char*)getScuHWverFromEeprom());
    /*                                             destination                             source           5 = len("Vw.ya" or "Vw.y\0")   */
    memcpy((void*)(((fwInfoVersion_u*)pCfgData->firmware)->fwBootVer.bootVer), (void*)BOOT_ADDR_VER, (size_t)BOOT_VER_SIZE + 1);

    /* write updated parameter on EEPROM  (eeprom_param_array & Infostation) */
    // WriteOnEeprom(EEpromAddr, Val, 1)
    /* Update full infostation structure in EEPROM */
    WriteOnEeprom(SCU_GENERAL_INFO_EE_ADDRES, (uint8_t*)&pCfgData, sizeof(pCfgData));
    tPrintf("Infostation data updated!\n\r");
    EVLOG_Message(EV_INFO, "Infostation data updated!");

    /* Check if a different default value (oxFF) is on productSn, productCode, fakeProductCode 
       Starting from v4.3.x and 4.6.x, the default value for these parameters is ' ' and not 0xFF */
    SRAM_Check_DEFAULT_of_Code();

    /* Reinit modbus registers according to the new settings */
    initModbusRegisters();

}