/************************************************************
 * Include
 ************************************************************/
#include <stdint.h>
#include <main.h>
#include "blueMng.h"
#include "main.h"
#include "telnet.h"

/* Bluetooth includes */
#include "gatt_db.h"
#include "app_bluenrg_ms.h"
#include "sensor.h"
#include "bluenrg_def.h"
#include "hci.h"
#include "bluenrg_conf.h"
#include "scheduleMng.h"
#ifndef TRANSACTION_SIMULATION
#include "wrapper.h"
#include "hci_le.h"
#endif

/* Bluetooth task define */
#define BT_TASK_DELAY_MS 100
#define ACQ_DELAY 1000 / BT_TASK_DELAY_MS

#define RESP_POSITIVE 0xAA
#define RESP_NEGATIVE 0xFF
   
/* Definitions Schedulation Task  */
osThreadId_t schedTaskHandle;
const osThreadAttr_t schedTask_attributes = {
  .name = "SCHED_MNG_TASK",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = configMINIMAL_STACK_SIZE * 9
};

/* Bluetooth declarations */
osMessageQueueId_t xQueueBlueReq;
osSemaphoreId_t xBtStartSemaphore, xBtAckSemaphore;
BluetoothState btState;
uint8_t btBusy = 0;

/* SCU socket abstraction */
uint16_t appId = 0;

/* Declaration of extern functions */
extern tBleStatus Set_DeviceConnectable(socket_t * socket);

void init_schedules(sck_schedule_t *sched);
void init_blue_sck();

void blueMngTask(void * pvParameters)
{
  //bt_request_t req;
  BtTaskReq_t task_req;
  
  btState = INITIALIZE;
  //req.type = REQ_UNKNOWN;
  task_req.type = REQ_UNKNOWN;
  
  xQueueBlueReq = osMessageQueueNew( BT_QUEUE_MAX_LENGTH, sizeof(BtTaskReq_t), NULL );
  
  xBtStartSemaphore = osSemaphoreNew( 1, 1, NULL );
  osSemaphoreAcquire(xBtStartSemaphore, 0);
  
  xBtAckSemaphore = osSemaphoreNew( 1, 1, NULL );
  osSemaphoreAcquire(xBtAckSemaphore, 0);
  
  /* Initialize Socket, first update */
  init_socket(&socket);
  
  /* Initialize transactions register */
  init_transaction_register(&socket.reg, MAX_NUM_TRANSACTION_STORABLE); // Only for debug
  
  /* Initialize Schedule Manager Task */
  schedTaskHandle = osThreadNew(scheduleMngTask, NULL, &schedTask_attributes);
  
  for(;;)
  {
    switch(btState)
    {
    case INITIALIZE:
      if(osSemaphoreAcquire(xBtStartSemaphore, portMAX_DELAY) == osOK)
      {
        tPrintf("Bluetooth Start\n\r");
        MX_BlueNRG_MS_Init();
        btState = DISCONNECTED;
      }
      break;
    case DISCONNECTED:
      appId = 0;
      if(Set_DeviceConnectable(&socket) == BLE_STATUS_SUCCESS)
        btState = DISCOVERABLE;
      break;
    case DISCOVERABLE:
      if(osMessageQueueGet(xQueueBlueReq, &task_req, 0, 0) == osOK)
      {
        if(task_req.type == WRITE_STATE)
        {
          set_sck_state(task_req.state);
        }
        else if(task_req.type == UPDATE_ID)
        {
          int id = getStationId();
          set_sck_id(id);
        }
        uint8_t company_id[2] = {ID_SCAME};
        uint8_t resp_data[] = {
          7, 0xFF,                   // Manifacturer Section
          company_id[1],             // Manifacturer Id (Reserved Id Free)        
          company_id[0], 
          socket.id,                 // Socket id
          socket.wiring,             // Socket wiring
          socket.state.error,      // Socket state
          socket.state.value
        };
        int result = hci_le_set_scan_resp_data(sizeof(resp_data), resp_data);
      }
      // Wait a client connection request; polling HCI list read packets
      hci_user_evt_proc();
      osDelay(pdMS_TO_TICKS(BT_TASK_DELAY_MS));
      break;
    case IN_CONNECTION:
      // Update socket data
      init_blue_sck();
      // Turn ON characteristic notify descriptors
      turn_on_notify();
      // Reset bluetooth GATT values
      //char_clear_current_trans();
      char_update_current_trans(&socket.reg.transactions[socket.reg.num]);
      btState = CONNECTED;
      break;
    case CONNECTED:
      // Check Bluetooth request sent by the application
      if(osMessageQueueGet(xQueueBlueReq, &task_req, 0, 0) == osOK)
      {
        switch(task_req.type)
        {
          case READ_STATE:
            get_sck_state();
            break;
          case WRITE_STATE:
            set_sck_state(task_req.state);
            break;
          case UPDATE_MEASURES:
            char_update_measures(&task_req.measures);
            break;
          case READ_REGISTER:
            break;
          case SEND_REGISTER:
            send_register();
            break;
          case SEND_START_ACTIVE_TRANS:
            char_update_current_trans(&socket.reg.transactions[socket.reg.num]);
            char_update_trans_cp(START_SESSION, 0xAA, &socket.reg.transactions[socket.reg.num]);
            break;
          case SEND_STOP_ACTIVE_TRANS:
            char_clear_current_trans();
            char_update_trans_cp(STOP_SESSION, 0xAA, &socket.reg.transactions[socket.reg.num - 1]);
            break;
          case RETRY_SEND_LONG_CHAR:
            send_packets_queue(task_req.serv_handle, task_req.char_handle);
            break;
          case REQ_UNKNOWN:
          default:
            break;
        }
      }
      
      // Process Bluetooth HCI list read packets
      hci_user_evt_proc();
      osDelay(pdMS_TO_TICKS(BT_TASK_DELAY_MS));
      break;
    case WAIT_ACK:
      // Wait HCI event EVT_CMD_COMPLETE (ACK for hci_send_cmd())
        break;
    }
  }
}

uint8_t requestToBtTask(BtTaskReq_t *req)
{
  if(btBusy == 0)
  {
    osMessageQueuePut(xQueueBlueReq, req, 0, 0);
  }
  return btBusy;
}

void busyBtTask()
{
  btBusy = 1;
}

void releaseBtTask()
{
  btBusy = 0;
}

osMessageQueueId_t getBlueMngQueueHandle(void)
{
  return xQueueBlueReq;
}

void init_socket(socket_t *sck)
{
  strcpy(sck->name, getStationName());
  strcpy(sck->serial, getStationSerialNumber());
  strcpy(sck->firmware, (char*)getFwVer());
  sck->wiring = getStationSocketType();
  sck->mt_int = UNKNOW;
  sck->mt_ext = UNKNOW;
  sck->rfid = MIFARE_MINI_4B;
  sck->max_current = getStationMaxCurrentT();
  sck->max_power = 7000;
  sck->id = getStationId();
  sck->state.value = (sck_statevalue_e)EVSTATE_SOCKET_AVAILABLE;
  sck->state.error = ERROR_NONE;
  sck->modeFun = getStationModeWorking();
//  sck->modePwr = MODE_TRI_PH_PM_BAL;
  sck->modePwr = MODE_MONO_PH_NO_PM; 
  
  init_measures(&sck->measures);
  
  // Initialized socket authorization
  sck_auth_t *mem_auth = getAuthorization();
  sck->auth.auth_state = mem_auth->auth_state;
  strcpy(sck->auth.user, mem_auth->user);
  strcpy(sck->auth.pass, mem_auth->pass);
  
  // Initialized socket schedulations
  init_schedules(sck->scheds);
}

void init_measures(sck_measures_t *measures)
{
  measures->modePwr = socket.modePwr;
  measures->Etot = 0;
  measures->duration = 0;
  measures->Pist = 0;
  measures->Pest = 0;
  measures->currentL1 = 0;
  measures->currentL2 = 0;
  measures->currentL3 = 0;
  measures->Pest1 = 0;
  measures->Pest2 = 0;
  measures->Pest3 = 0;
}

void init_schedules(sck_schedule_t *sched)
{
  sck_schedule_t *memSched = getSchedulation();
  for(int i = 0; i < MAX_SCHEDS; i++)
  {
    sched[i].id = i;
    sched[i].days = memSched[i].days;
    sched[i].start_hour = memSched[i].start_hour;
    sched[i].start_min = memSched[i].start_min;
    sched[i].power = memSched[i].power;
    sched[i].end_hour = memSched[i].end_hour;
    sched[i].end_min = memSched[i].end_min;
    sched[i].enable = memSched[i].enable;
  }
}

void init_blue_sck()
{
  char_update_serial();
  char_update_firmware();
  char_update_emeters();
  char_update_rfid();
  char_update_max_current();
  char_update_wiring();
  char_update_max_power();
  char_update_id();
  char_update_state();
  char_update_fun_mode();
  char_update_pwr_mode();
}

/* Socket Getter Functions */

__weak char * get_sck_name()
{
  PRINTF("Socket read name -> %s\n", socket.name);
  return socket.name;
}

__weak char * get_sck_serial()
{
  PRINTF("Socket read serial -> %s\n", socket.serial);
  return socket.serial;
}

__weak char * get_sck_firmware()
{
  PRINTF("Socket read firmware -> %s\n", socket.firmware);
  return socket.firmware;
}

__weak uint8_t get_sck_wiring()
{
  PRINTF("Socket read wiring -> 0x%X\n", socket.wiring);
  return socket.wiring;
}

__weak uint16_t get_sck_emeters()
{
  socket.mt_int = getStationEmType(INTERNAL_EM);
  socket.mt_ext = getStationEmType(EXTERNAL_EM);
  PRINTF("Socket read energy meters -> 0x%02X\n", socket.mt_ext);
  return ((uint8_t)socket.mt_ext) << 8 | ((uint8_t)socket.mt_int);
}

__weak energy_meter_e get_sck_int_emeter()
{
  socket.mt_int = getStationEmTypeInt();
  PRINTF("Socket read internal energy meter -> 0x%X\n", socket.mt_int);
  return socket.mt_int;
}

__weak energy_meter_e get_sck_ext_emeter()
{
  socket.mt_ext = getStationEmTypeExt();
  PRINTF("Socket read external energy meter -> 0x%X\n", socket.mt_ext);
  return socket.mt_ext;
}

__weak uint8_t get_sck_rfid()
{
  PRINTF("Socket read rfid reader -> 0x%X\n", socket.rfid);
  return socket.rfid;
}

__weak int32_t get_sck_max_current()
{
  socket.max_current = getStationMaxCurrentT();
  PRINTF("Socket read max currentL1 -> %d mA\n", socket.max_current);
  
  return socket.max_current;
}

__weak int32_t get_sck_max_power()
{
  PRINTF("Socket read max power -> %d W\n", socket.max_power);
  return socket.max_power;
}

__weak modePwr_e get_sck_pwr_mode()
{
  PRINTF("Socket read pwrMode -> 0x%X\n", socket.modePwr);
  return socket.modePwr;
}

__weak uint8_t get_sck_id()
{
  PRINTF("Socket read id -> %d\n", socket.id);
  return socket.id;
}

__weak sck_state_t get_sck_state()
{
  PRINTF("Socket read state -> value = 0x%X error = 0x%X\n", socket.state.value, socket.state.error);
  return socket.state;
}

__weak modeFun_e get_sck_fun_mode()
{
  PRINTF("Socket read funMode -> 0x%X\n", socket.modeFun);
  return socket.modeFun;
}

__weak sck_measures_t * get_sck_measures()
{
  return &socket.measures;
}

__weak sck_auth_t * get_sck_auth()
{
  return &socket.auth;
}

/* Socket Setter Functions */

__weak void set_sck_name(char *name, uint8_t length)
{
  int i;
  for(i = 0; i < MAX_NAME_LENGTH; i++)
    socket.name[i] = 0;
  if(length <= MAX_NAME_LENGTH)
    memcpy(socket.name, name, length);
  
  // save in eeprom
  setStationName(name, length);
  
  PRINTF("Socket write name -> %s\n", socket.name);
}

__weak void set_sck_mode(modeFun_e mode)
{
  socket.modeFun = mode;
  PRINTF("Socket write mode -> 0x%X\n", socket.modeFun);
}

__weak void set_sck_max_current(int32_t max_current)
{
  socket.max_current = max_current;
  PRINTF("Socket write max current -> %d mA\n", socket.max_current);
}

__weak void set_sck_state(sck_state_t state)
{
  socket.state = state;
  PRINTF("Socket write state -> 0x%X\n", socket.state.value);
  char_update_state();
}
__weak void set_sck_id(int id)
{
  socket.id = id;
  PRINTF("Socket id changed -> %d\n", id);
}
__weak void set_sck_pwr_mode(modePwr_e currMode)
{
  socket.modePwr = currMode;
}

__weak void set_sck_wiring(uint8_t newWiring)
{
  socket.wiring = newWiring;
}

void set_sck_auth(char *user, char *pass)
{
  socket.auth.auth_state = AUTH_SETTED;
  strcpy(socket.auth.user, user);
  strcpy(socket.auth.pass, pass);
  // Save credential in eeprom
  saveAuthorization(user, pass);
}

uint8_t check_sck_auth(char *user, char *pass)
{
  uint8_t res = 0;
  if(socket.auth.auth_state == AUTH_SETTED)
  {
    if(strcmp(user, socket.auth.user) == 0 &&
       strcmp(pass, socket.auth.pass) == 0)
      res = 1;
  }
  return res;
}

void set_app_id(uint16_t id)
{
  appId = id;
}


/* Socket Register Functions */

__weak void init_transaction_register(sck_register_t *reg, unsigned int n)
{
#ifdef TRANSACTION_SIMULATION
  reg->num = n;
  reg->transactions = (transaction_t *)malloc(n * sizeof(transaction_t));
  populate_transaction_register(reg);
#else
  configASSERT(n <= MAX_NUM_TRANSACTION_STORABLE);
  populateTransactionRegister(reg, (socket_t*)&socket);
#endif
}

#ifdef TRANSACTION_SIMULATION
// This function is only for debug
void populate_transaction_register(sck_register_t *reg)
{
  struct tm tm_start, tm_stop;
  
  for(int i = 0; i < reg->num; i++)
  {
    tm_start.tm_mday = 15+i;
    tm_start.tm_mon = 6-1;
    tm_start.tm_year = 2021-1900;
    tm_start.tm_sec = 0;
    tm_start.tm_min = (34+i)%59;
    tm_start.tm_hour = (12+i)%23;
    reg->transactions[i].start = mktime(&tm_start);
    
    tm_stop.tm_mday = 16+i;
    tm_stop.tm_mon = 6-1;
    tm_stop.tm_year = 2021-1900;
    tm_stop.tm_sec = 0;
    tm_stop.tm_min = (35+i)%59;
    tm_stop.tm_hour = (14+i)%23;
    reg->transactions[i].stop = mktime(&tm_stop);
    
    reg->transactions[i].duration = difftime(reg->transactions[i].stop, reg->transactions[i].start);
    
    reg->transactions[i].active_energy = (float)rand()/(float)RAND_MAX * 100.0;
  }
}
#endif

__weak sck_register_t * get_sck_register()
{
//  PRINTF("Socket read register -> \n");
//  transaction_t * tr;
//  for(int i = 0; i < socket.reg.num; i++)
//  {
//    tr = socket.reg.transactions + i;
//    char s_start[20];
//    char s_stop[20];
//    strftime(s_start, 20, "%d/%m/%Y %R", gmtime((const time_t *)&tr->start));
//    strftime(s_stop, 20, "%d/%m/%Y %R", gmtime((const time_t *)&tr->stop));
//    PRINTF("\t%d | %s | %s | %.0lf | %.2f\n", i, s_start, s_stop, tr->duration, tr->active_energy);
//  }
  return &socket.reg;
}

/* Socket Process Requests */

//void sck_process_request(sck_cmd_t *command)
//{
//  switch(command->type)
//  {
//    case START_CHARGE:
//      PRINTF("Socket command -> charge START\n");
//      start_charge();
//      break;
//    case STOP_CHARGE:
//      PRINTF("Socket command -> charge STOP\n");
//      stop_charge();
//      break;
//    case ACTIVE_SCHEDULE:
//      PRINTF("Socket command -> schedule ACTIVE\n");
//      active_schedule();
//      break;
//    case DISABLE_SCHEDULE:
//      PRINTF("Socket command -> schedule DISABLE\n");
//      disable_schedule();
//      break;
//    case LOCK_SOCKET:
//      PRINTF("Socket command -> socket LOCK\n");
//      lock_socket();
//      break;
//    case UNLOCK_SOCKET:
//      PRINTF("Socket command -> socket UNLOCK\n");
//      unlock_socket();
//      break;
//    case CMD_UNKNOWN:
//    default:
//      PRINTF("Socket command -> Unknown command\n");
//      unknown_cmd();
//  }
//}

__weak void start_charge()
{
  /* SCU elaboration */
//  socket.state = IN_CHARGE;
//  PRINTF("Socket state changed -> IN CHARGE\n");
//  PRINTF("Socket state char update\n");
//  char_update_state();
}

__weak uint8_t stop_charge()
{
  EvsMngEvent_en evs_event;
  
  /* Send to EvsMng stop event */
  send_to_evs(EVS_PULS_STOP);
  
  /* Send to EvsMng RM signal update */
  if(sched_is_enable())
  {
    if(in_range())
    {
      evs_event = EVS_APP_RELEASE;
    }
    else
    {
      evs_event = EVS_APP_SUSPENDING;
    }
  }
  else
  {
    evs_event = EVS_APP_RELEASE;
  }
  send_to_evs(evs_event);
  
  return RESP_POSITIVE;
}

__weak void active_schedule()
{
  /* SCU elaboration */
  
}

__weak void disable_schedule()
{
  /* SCU elaboration */
  
}

__weak void lock_socket()
{
  /* SCU elaboration */
  
}

__weak void unlock_socket()
{
  /* SCU elaboration */
  
}

__weak void unknown_cmd()
{
  /* SCU elaboration */
  
}

/* Socket Schedulation Functions */
sck_schedule_t * get_schedulation()
{
  return socket.scheds;
}

void sched_received(uint8_t *data, uint8_t data_length)
{ 
  if(data_length != SCHED_BT_SIZE)
  {
    return;
  }
  
  uint8_t id = data[0];
  
  if(id > MAX_SCHEDS)
  {
    return;
  }
  
  socket.scheds[id].id = id;
  socket.scheds[id].days = data[1];
  socket.scheds[id].start_hour = data[2];
  socket.scheds[id].start_min = data[3];
  socket.scheds[id].end_hour = data[4];
  socket.scheds[id].end_min = data[5];
  
  int32_t power = 0;
  power += data[9];
  power += data[8] << 8;
  power += data[7] << 16;
  power += data[6] << 24;
  socket.scheds[id].power = power;
  
  socket.scheds[id].enable = data[10];
  
  // Save schedulation in eeprom
  saveSchedulation(socket.scheds);
  
  // Notify Schedule Task Management
  SchedTaskReq_t request;
  request.schedule = socket.scheds[id];
  send_to_scheduler(&request);
}