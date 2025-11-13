/**
* @file        scheduleMng.h
*
* @brief       Scheduler Charge Manager
*
* @author      Luca C
*
* @riskClass   C
*
* @moduleID
*
*
*
* @copyright
*       Copyright (C) 2016 SCAME S.p.A. All rights reserved.
*       This file is copyrighted and the property of Aesys S.p.A.. It contains confidential and proprietary
*       information. Any copies of this file (in whole or in part) made by any method must also include a copy of this
*       legend.
*       Developed by:  SCAME S.p.A.
***********************************************************************************************************************/

#ifndef _SCHEDMNG_H_
#define _SCHEDMNG_H_

#include <stdbool.h>

#define SCHED_TASK_DELAY 5000

#define NUM_SCHEDULER_DAYS            7
#define NUM_SCHEDULER_EVENTS_PER_DAY  2
#define MAX_SCHEDULATION_NUMBER       4

#define NUM_MAX_SCHEDULE_EVENTS   (NUM_SCHEDULER_DAYS * NUM_SCHEDULER_EVENTS_PER_DAY)
#define NUM_MAX_SCHEDULER_TICKS   (NUM_SCHEDULER_DAYS * 24 * 60)

#define SCHEDULATION_ENABLE_BIT 0x80

typedef __packed struct
{
  uint8_t id;
  uint8_t days;
  uint8_t start_hour;
  uint8_t start_min;
  uint8_t end_hour;
  uint8_t end_min;
  int32_t power;
  uint8_t enable;
} sck_schedule_t;

// Task request
typedef struct 
{
  sck_schedule_t schedules[MAX_SCHEDULATION_NUMBER];
} SchedTaskReq_t;
/** End Schedule task **/

void      scheduleMngTask             (void * pvParameters);

uint16_t  Scheduler_getScheduledPower (void);
void      Scheduler_scheduleCharge    (sck_schedule_t *schedulations);
void      send_to_scheduler           (SchedTaskReq_t *request);
bool      Scheduler_isDisabled        (void);

#endif