/*
 *  scame-devolo-greenphy - PLC Modem device
 *
 *  Copyright (C) 2010  Manuele Conti (manuele.conti@archimede-energia.com)
 *  Copyright (C) 2019  Bruno Zavettieri (bruno.zavettieri@archimede-energia.com)
 *  Copyright (C) 2019  Luca Valtellina (info@vlengineering.it)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * This code is made available on the understanding that it will not be
 * used in safety-critical situations without a full and competent review.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef _HOMEPLUGDEV_SPI_H
#define _HOMEPLUGDEV_SPI_H


#define HOMEPLUGDEV_THR_COMM_ERRORS     10U       /* Threshold communication error */

#define HOMEPLUGDEV_CRCSIZE		2
#define HOMEPLUGDEV_CMDSIZE		1
#define HOMEPLUGDEV_INFOSIZE		sizeof(homeplugdev_info)
#define HOMEPLUGDEV_INFOHOSTSIZE	sizeof(struct host_s)
#define HOMEPLUGDEV_INFODEVICESIZE	sizeof(struct device_s)
#define HOMEPLUGDEV_MACSIZE             6
#define HOMEPLUGDEV_BUFSIZE             (HOMEPLUGDEV_INFOSIZE + HOMEPLUGDEV_CRCSIZE)
#define HOMEPLUGDEV_STRING_LEN          24
#define EVCCID_STRING_LEN		8
#define EVSEID_STRING_LEN	 	8
#define SESSIONID_STRING_LEN	        8
#define SERV_NAME_STRING_LEN	        8

enum homeplugdev_cmd {
	RDMAC   = 0x01, // Read device macaddress
	TRINF   = 0x02, // Exchange information between EV and EVSE
	RDDAT   = 0x03, // Read data internal RTC
	WRDAT   = 0x04  // Write data internal RTC
};

enum evs_iso15118_data_req_val {
	ISO15118_CHARGING_STOP      = 0x00,   /*  0U */
	ISO15118_CHARGING_START     = 0x01,   /*  1U */
	ISO15118_CHARGING_PAUSE     = 0x02,   /*  2U */
	ISO15118_V2GSEQUENCEPAUSE   = 0x1C,   /* 28U */
	ISO15118_V2GSEQUENCEEND     = 0x1D    /* 29U */
};

typedef enum {
	ST_SEND_CMD = 0U,
	ST_SEND_DATA
} homeplugdev_state_t;

struct host_s {
    uint16_t presentCurrent;
    uint16_t presentVoltage;
    uint16_t maxCurrent;                        // To EV CharrgeParameterDiscoveryRes:  AC/DC_EVChargeParameter -> AC/DC_EVSEStatus
    uint16_t minCurrent;                        // To EV CharrgeParameterDiscoveryRes:  AC/DC_EVChargeParameter -> AC/DC_EVSEStatus
    uint16_t maxVoltage;                        // To EV CharrgeParameterDiscoveryRes:  AC/DC_EVChargeParameter -> AC/DC_EVSEStatus
    uint16_t minVoltage;
    uint16_t maxPower;                          // To EV is in the expression is in kW * 10
    uint8_t  freeService  :1;                   // To EV ServiceDiscoveryRes: FreeService (true/false 1 Bit is enought)
    uint8_t  stopEVSE     :1;
    uint8_t  emergency    :1;
    uint8_t  powerLimit   :1;
    uint8_t  stateCP      :4;
    uint8_t  cableCheckRes;
    uint8_t  sessionId[SESSIONID_STRING_LEN];   // To EV SessionSetupRes: SessionID
    uint8_t  evseId[EVSEID_STRING_LEN];         // To EV SessionSetupRes: EVSEID
    uint32_t evseTimeStamp;                     // To EV SessionSetupRes: EVSETimeStamp
    uint8_t  paymentOption;                     // To EV ServiceDiscoveryRes: PaymentOption (iso1paymentOptionType: Contract/ExternalPayment 2 Bits is enought)
    // NOTE: Because the service can be 1 ore more, evaluate the possibility to rotate the value inside triggered by the service ID
    uint8_t  serviceID;                         // To EV ServiceDiscoveryRes: ServiceId (can be 1 ore more)
    uint8_t  serviceName[SERV_NAME_STRING_LEN]; // To EV ServiceDiscoveryRes: ServiceName (Es SCAME DC, SCAME AC)
    uint8_t  pauseEVSE    :1;                   // To EV for advise that EVSE is in pause
    uint8_t  serviceCat   :3;                   // To EV ServiceDiscoveryRes: ServiceCategory (iso1serviceCategoryType: 3 Bits is enought)
    uint8_t  entransfMode :4;                   // To EV ServiceDiscoveryRes: EnergyTransferMode (iso1EnergyTransferModeType: 4 Bits is enought)
    //
    uint8_t  rdcStatus;                         // To EV CharrgeParameterDiscoveryRes: AC_EVSEStatus -> RDC (Is possible to use the cableCheckRes to reduce the variables usage)
 
    // NOTE: Dummy bytes for allign the structure to 32bits (usefull when using GPDMA)
    // The structure is 48 byte lenght no dummy needed
} __attribute__((packed));
 
struct device_s {
	uint8_t  fw_ver[HOMEPLUGDEV_STRING_LEN];    // For Application
	uint8_t  mac[HOMEPLUGDEV_MACSIZE];          // For Application
	uint16_t targetCurrent;                     // From EV PreChargeReq & CurrentDemandReq: EVTargetCurrent
	uint16_t targetVoltage;                     // From EV PreChargeReq & CurrentDemandReq: EVTargetVoltage
	uint16_t targetPower;                       // From EV PowerDeliveryreq: ChargingProfile
	uint8_t  pauseOpenV2G     :1;
	uint8_t  terminateOpenV2G :1;
	uint8_t  stateOpenV2G     :6;
	uint8_t  stateSlac        :4;
	uint8_t  stateSDP         :4;
	uint8_t  soc;
	uint8_t  errorCode;
	uint8_t  cableCheckReq;
	int32_t  remainBulkTime;
	int32_t  remainTimeToFull;                  // From EV CurrentDemandReq: RemainingTimeToFullSoc
	uint8_t  evccId[EVCCID_STRING_LEN];         // From EV SessionSetupReq: EVCCID
	uint32_t evDepartureTime;                   // From EV CharrgeParameterDiscoveryReq: AC/DC_EVChargeParameter -> DepartureTime (Optional) [when the vehicle intends to finish the charging process. Offset in seconds from the point in time of sending this message]
	uint16_t evMaxCurrentLimit;                 // From EV CharrgeParameterDiscoveryReq & CurrentDemandReq: AC/DC_EVChargeParameter -> EVMaximumCurrentLimit
	uint16_t evMinCurrent;                      // From EV CharrgeParameterDiscoveryReq: AC_EVChargeParameter -> EVMinCurrent
//	uint16_t evMaxPowerLimit;                   // From EV CharrgeParameterDiscoveryReq & CurrentDemandReq: DC_EVChargeParameter -> EVMaximumPowerLimit (Optional)
	uint16_t evMaxVoltageLimit;                 // From EV CharrgeParameterDiscoveryReq & CurrentDemandReq: AC/DC_EVChargeParameter -> EVMaximumVoltageLimit
	uint16_t evEnergyCapacity;                  // From EV CharrgeParameterDiscoveryReq: DC_EVChargeParameter -> EVEnergyCapacity (Optional)
	uint16_t evEnergyRequest;                   // From EV CharrgeParameterDiscoveryReq: AC/DC_EVChargeParameter -> EnergyRequest (To have the 16Bits is needed to convert in to kW * 10. Ex 65.5kW -> 650)
//	uint8_t  evFullSoc;                         // From EV CharrgeParameterDiscoveryReq: DC_EVChargeParameter -> FullSOC (Optional)
//	uint8_t  evBulkSoc;                         // From EV CharrgeParameterDiscoveryReq: DC_EVChargeParameter -> BulkSOC (Optional)
	uint8_t  evProtocol;
	uint8_t  bulkChargeComplete:4;              // From EV CurrentDemandReq: BulkChargingComplete (true/false)
	uint8_t  chargingComplete:4;                // From EV CurrentDemandReq: ChargingComplete (true/false)
 
    uint8_t serviceIdReq;                       // From EV ServiceDetailReq: ServiceID
    // NOTE: Dummy bytes for allign the structure to 32bits (can be usefull when using DMA)
    // The structure is 74 byte lenght, 2 dummy bytes are needed
    uint8_t dummy1;
    uint8_t dummy2;
} __attribute__((packed));

typedef struct homeplugdev_info_s {
    struct host_s host;
    struct device_s device;
} __attribute__((packed)) homeplugdev_info;

extern homeplugdev_info info;

void homeplugdev_init(void);
int8_t homeplugdev_readwrite_info(void);
int8_t homeplugdev_readwrite_MAC(void);
int8_t homeplugdev_error(void);
void homeplugdev_hw_reset(void);
int8_t homeplugdev_send_cmd_and_crc(enum homeplugdev_cmd cmd);
int8_t homeplugdev_txrx_data_check_crc(void *data, size_t len);

#endif
