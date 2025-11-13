#include <stdlib.h>
#include <math.h>

#include "cmsis_os2.h"
#include "scheduleMng.h"
#include "telnet.h"
#include "rtcApi.h"
#include "PwmMng.h"
#include "EvsMng.h"
#include "wrapper.h"

typedef struct node_t node_t;
struct node_t
{
  node_t *next;
  node_t *back;
};

typedef struct list_t list_t;
struct list_t
{
    node_t *head;
    node_t *tail;
    uint32_t count;
};

void node_init(node_t * const me)
{
    me->next = NULL;
    me->back = NULL;
}

void list_init(list_t * const me)
{
    me->head = NULL;
    me->tail = NULL;
    me->count = 0;
}

bool list_is_empty(list_t * const me)
{
    return me->head == NULL && me->tail == NULL;
}

void circular_list_add_front(list_t * const me, node_t *node)
{
    if(me == NULL)
        return;
    if(me->head != NULL)
        me->head->back = node;
    else
        me->tail = node;
    node->next = me->head;
    node->back = me->tail;
    me->head = node;
    me->tail->next = node;
    me->count++;
}

void circular_list_add_back(list_t * const me, node_t *node)
{
    if(me == NULL)
        return;
    if(me->tail != NULL)
        me->tail->next = node;
    else
        me->head = node;
    node->back = me->tail;
    node->next = me->head;
    me->tail = node;
    me->head->back = node;
    me->count++;
}

bool circular_list_add_after(list_t * const me, node_t *nodeToSearch, node_t *nodeToAdd)
{
    if(me == NULL)
        return false;
    if(nodeToSearch == NULL)
        return false;
    if(nodeToAdd == NULL)
        return false;
    if(nodeToSearch == me->tail)
    {
        circular_list_add_back(me, nodeToAdd);
    }
    else
    {
        node_t *node = me->head;
        while(node != NULL)
        {
            if(node == nodeToSearch)
            {
                nodeToAdd->next = nodeToSearch->next;
                nodeToAdd->back = nodeToSearch->back;
                nodeToSearch->next->back = nodeToAdd;
                nodeToSearch->next = nodeToAdd;
                me->count++;
                return true;
            }
            node = node->next;
        }
        return false;
    }
    return true;
}

typedef enum
{
  SCHEDULER_UNKNOWN_EVENT,
  SCHEDULER_START_CHARGE_EVENT,
  SCHEDULER_STOP_CHARGE_EVENT
} SchedulerEventType;

typedef struct 
{
  SchedulerEventType type;
  uint32_t tick;
  int32_t maxPower_W;
} SchedulerEvent;

void SchedulerEvent_init(SchedulerEvent * const me,
                         SchedulerEventType type,
                         int32_t tick,
                         int32_t maxPower_W)
{
  me->type = type;
  me->tick = tick;
  me->maxPower_W = maxPower_W;
}

typedef struct SchedulerEventNode
{
  node_t node;
  SchedulerEvent event;
} SchedulerEventNode;

void SchedulerEventNode_init(SchedulerEventNode * const me,
                             SchedulerEvent *event)
{
  node_init(&me->node);
  SchedulerEvent_init(&me->event, event->type, event->tick, event->maxPower_W);
}

typedef struct SchedulerPlanning
{
  SchedulerEventNode eventsPool[NUM_MAX_SCHEDULE_EVENTS];
  list_t eventsSequence;
} SchedulerPlanning;

typedef struct
{
  SchedulerPlanning plannings[MAX_SCHEDULATION_NUMBER];
  uint32_t currentTick;
  SchedulerEvent *lastEvent;
  EvsMngEvent_en lastCommand;
} Scheduler;

bool Schedulation_endsNextDay(sck_schedule_t *schedulation);

void SchedulerEvent_initStart(SchedulerEvent * const me, uint8_t dayOfWeek, uint8_t hour, uint8_t min, int32_t maxPower_W);
void SchedulerEvent_initStop(SchedulerEvent * const me, uint8_t dayOfWeek, uint8_t hour, uint8_t min);

void SchedulerPlanning_init(SchedulerPlanning * const me);
void SchedulerPlanning_addSchedulation(SchedulerPlanning * const me,
                                       sck_schedule_t *schedulation);
void SchedulerPlanning_addEvent(SchedulerPlanning * const me,
                                SchedulerEvent *event);
SchedulerEvent * SchedulerPlanning_getEventBefore(SchedulerPlanning * const me,
                                                  uint32_t tick);

void Scheduler_init(Scheduler * const me);
void Scheduler_update(Scheduler * const me);
void Scheduler_processEvents(Scheduler * const me);
void Scheduler_enableCharge(Scheduler * const me);
void Scheduler_suspendCharge(Scheduler * const me);
void Scheduler_turnOff(Scheduler * const me);
void Scheduler_updateTime(Scheduler * const me);
SchedulerEvent * Scheduler_getEvent(Scheduler * const me);

uint32_t Scheduler_getTickFromWeekTime(uint8_t dayOfWeek, uint8_t hour, uint8_t min);
void Scheduler_updateEvents(Scheduler * const me,
                            sck_schedule_t *schedulations);
void Scheduler_cleanPlannings(Scheduler * const me);
void Scheduler_printEvent(Scheduler * const me, SchedulerEvent *event);
struct DataAndTime_t Scheduler_getDateTimeInUtc(void);

static Scheduler scheduler;
static Scheduler *schedulerHandler = &scheduler;
static uint16_t scheduledPower = 0;
static const char *weekDays[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

void scheduleMngTask(void * pvParameters)
{
  Scheduler_init(schedulerHandler);
  for(;;)
  {
    Scheduler_update(schedulerHandler);
    Scheduler_processEvents(schedulerHandler);
    osDelay(SCHED_TASK_DELAY);
  }
}

void Scheduler_init(Scheduler * const me)
{
  Scheduler_cleanPlannings(me);
  Scheduler_updateTime(me);
  me->lastCommand = EVS_EVENT_NULL;
}

void Scheduler_cleanPlannings(Scheduler * const me)
{
  for(int i = 0; i < MAX_SCHEDULATION_NUMBER; i++)
  {
    SchedulerPlanning_init(&me->plannings[i]);
  }
}

void SchedulerPlanning_init(SchedulerPlanning * const me)
{
  SchedulerEvent initEvent;
  SchedulerEventNode *eventNode;
  
  initEvent.type = SCHEDULER_UNKNOWN_EVENT;
  initEvent.tick = 0;
  initEvent.maxPower_W = 0;
  for(int i = 0; i < NUM_MAX_SCHEDULE_EVENTS; i++)
  {
    eventNode = &me->eventsPool[i];
    SchedulerEventNode_init(eventNode, &initEvent);
  }
  list_init(&me->eventsSequence);
}

void Scheduler_updateTime(Scheduler * const me)
{
  struct DataAndTime_t now;
  
  now = Scheduler_getDateTimeInUtc();
  me->currentTick = Scheduler_getTickFromWeekTime(now.DayWeek, now.Hour, now.Minute);
}

uint32_t Scheduler_getTickFromWeekTime(uint8_t dayOfWeek, uint8_t hour, uint8_t min)
{
  return min + (hour * 60) + (dayOfWeek * 24 * 60);
}

struct DataAndTime_t Scheduler_getDateTimeInUtc(void)
{
  struct DataAndTime_t dateAndTime;
  RTC_DateTypeDef rtcDate;
  RTC_TimeTypeDef rtcTime;

  HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
  
  dateAndTime.Second = rtcTime.Seconds;
  dateAndTime.Minute = rtcTime.Minutes;
  dateAndTime.Hour = rtcTime.Hours;
  if(rtcDate.WeekDay == 0)
  {
    dateAndTime.DayWeek = 6;
  }
  else
  {
    dateAndTime.DayWeek = (rtcDate.WeekDay - 1) % 7;
  }
  dateAndTime.Day = rtcDate.Date;
  dateAndTime.Month = rtcDate.Month;
  dateAndTime.Year = rtcDate.Year + 2000;
  dateAndTime.dstFlag = 0;
  
  return dateAndTime;
}

void send_to_scheduler(SchedTaskReq_t *request)
{
  Scheduler_updateEvents(schedulerHandler, request->schedules);
}

void Scheduler_updateEvents(Scheduler * const me,
                            sck_schedule_t *schedulations)
{
  sck_schedule_t *schedulation;
  
  Scheduler_cleanPlannings(me);
  for(int i = 0; i < MAX_SCHEDULATION_NUMBER; i++)
  {
    schedulation = &schedulations[i];
    if(schedulation->enable)
    {
      SchedulerPlanning_addSchedulation(&me->plannings[i], schedulation);
    }
  }
}

void SchedulerPlanning_addSchedulation(SchedulerPlanning * const me,
                                       sck_schedule_t *schedulation)
{
  SchedulerEvent startEvent, stopEvent;
  
  for(uint8_t day = 0; day < NUM_SCHEDULER_DAYS; day++)
  {
    if((schedulation->days & (1 << day)) != 0)
    {
      SchedulerEvent_initStart(&startEvent, day, schedulation->start_hour, schedulation->start_min, schedulation->power);
      if(Schedulation_endsNextDay(schedulation))
      {
        SchedulerEvent_initStop(&stopEvent, (day + 1) % NUM_SCHEDULER_DAYS, schedulation->end_hour, schedulation->end_min);
      }
      else
      {
        SchedulerEvent_initStop(&stopEvent, day, schedulation->end_hour, schedulation->end_min);
      }
      SchedulerPlanning_addEvent(me, &startEvent);
      SchedulerPlanning_addEvent(me, &stopEvent);
    }
  }
}

void SchedulerPlanning_addEvent(SchedulerPlanning * const me,
                                SchedulerEvent *event)
{
  SchedulerEventNode *freeNode, *headEvent;

  if( me->eventsSequence.count >= NUM_MAX_SCHEDULE_EVENTS )
  {
    // The sequence has reached the maximum number of elements.
    return;
  }
  freeNode = &me->eventsPool[ me->eventsSequence.count ];
  SchedulerEventNode_init( freeNode, event );
  if( me->eventsSequence.count != 0 )
  {
    headEvent = ( SchedulerEventNode* )me->eventsSequence.head;
    if( event->tick <= headEvent->event.tick )
    {
      circular_list_add_front( &me->eventsSequence, (node_t*)freeNode );
    }
    else
    {
      circular_list_add_back( &me->eventsSequence, (node_t*)freeNode );
    }
  }
  else
  {
    circular_list_add_back( &me->eventsSequence, (node_t*)freeNode );
  }
}

void SchedulerEvent_initStart(SchedulerEvent * const me, uint8_t dayOfWeek, uint8_t hour, uint8_t min, int32_t maxPower_W)
{
  uint32_t tick;
  
  tick = Scheduler_getTickFromWeekTime(dayOfWeek, hour, min);
  SchedulerEvent_init(me, SCHEDULER_START_CHARGE_EVENT, tick, maxPower_W);
}

bool Schedulation_endsNextDay(sck_schedule_t *schedulation)
{
  int minutesBeforeStart = (schedulation->start_hour * 60 + schedulation->start_min);
  int minutesBeforeStop = (schedulation->end_hour * 60 + schedulation->end_min);
  return (minutesBeforeStop - minutesBeforeStart) <= 0;
}

void SchedulerEvent_initStop(SchedulerEvent * const me, uint8_t dayOfWeek, uint8_t hour, uint8_t min)
{
  uint32_t tick;
  
  tick = Scheduler_getTickFromWeekTime(dayOfWeek, hour, min);
  SchedulerEvent_init(me, SCHEDULER_STOP_CHARGE_EVENT, tick, 0);
}

SchedulerEvent * SchedulerPlanning_getEventBefore(SchedulerPlanning * const me,
                                                  uint32_t tick)
{
  uint32_t nodeTick;
  node_t *node;
  
  if(me->eventsSequence.count == 0)
  {
    return NULL;
  }
  node = me->eventsSequence.head;
  for(int i = 0; i < me->eventsSequence.count; i++, node = node->next)
  {
    nodeTick = ((SchedulerEventNode*)node)->event.tick;
    if( tick < nodeTick )
      break;
  }
  return &((SchedulerEventNode*)node->back)->event;
}

void Scheduler_update(Scheduler * const me)
{
  SchedulerEvent *event;
  Scheduler_updateTime(me);
  event = Scheduler_getEvent(me);
  if(event != me->lastEvent)
  {
    Scheduler_printEvent(me, event);
  }
  me->lastEvent = event;
}

SchedulerEvent * Scheduler_getEvent(Scheduler * const me)
{
  SchedulerEvent *startEvent, *stopEvent;
  uint32_t diffTicks;
  
  startEvent = NULL;
  stopEvent = NULL;
  diffTicks = NUM_MAX_SCHEDULER_TICKS;
  for(int i = 0; i < MAX_SCHEDULATION_NUMBER; i++)
  {
    SchedulerEvent *event = SchedulerPlanning_getEventBefore(&me->plannings[i], me->currentTick);
    if(event != NULL)
    {
      if(event->type == SCHEDULER_START_CHARGE_EVENT)
      {
        startEvent = event;
      }
      else
      {
        uint32_t ticks = (me->currentTick - event->tick) % NUM_MAX_SCHEDULER_TICKS;
        if(ticks < diffTicks)
        {
          diffTicks = ticks;
          stopEvent = event;
        }
      }
    }
  }
  
  if(startEvent != NULL)
  {
    return startEvent;
  }
  
  if(stopEvent != NULL)
  {
    return stopEvent;
  }
  
  return NULL;
}

void Scheduler_printEvent(Scheduler * const me, SchedulerEvent *event)
{
  struct DataAndTime_t dataAndTime;
  const char *weekDay;
  
  if(event == NULL)
  {
    return;
  }
  
  dataAndTime = Scheduler_getDateTimeInUtc();
  weekDay = weekDays[dataAndTime.DayWeek];
  switch(event->type)
  {
  case SCHEDULER_START_CHARGE_EVENT:
    tPrintf("%s %02d:%02d (UTC) | Start schedule with max %d W\r\n", weekDay, dataAndTime.Hour, dataAndTime.Minute, event->maxPower_W);
    break;
  case SCHEDULER_STOP_CHARGE_EVENT:
    tPrintf("%s %02d:%02d (UTC) | Stop schedule\r\n", weekDay, dataAndTime.Hour, dataAndTime.Minute);
    break;
  default:
    break;
  }
}

void Scheduler_processEvents(Scheduler * const me)
{
  if(me->lastEvent != NULL)
  {
    if(me->lastEvent->type == SCHEDULER_START_CHARGE_EVENT)
    {
      Scheduler_enableCharge(me);
    }
    else
    {
      Scheduler_suspendCharge(me);
    }
  }
  else
  {
    Scheduler_turnOff(me);
  }
}

void Scheduler_enableCharge(Scheduler * const me)
{
  scheduledPower = me->lastEvent->maxPower_W / 100;
  pmng_app_schedule_mode_set(1);
  if(me->lastCommand != EVS_APP_RELEASE)
  {
    send_to_evs(EVS_APP_RELEASE);
  }
  me->lastCommand = EVS_APP_RELEASE;
}

void Scheduler_suspendCharge(Scheduler * const me)
{
  if(me->lastCommand != EVS_APP_SUSPENDING)
  {
    send_to_evs(EVS_APP_SUSPENDING);
  }
  me->lastCommand = EVS_APP_SUSPENDING;
}

void Scheduler_turnOff(Scheduler * const me)
{
  pmng_app_schedule_mode_set(0);
  if(me->lastCommand != EVS_EVENT_NULL)
  {
    send_to_evs(EVS_APP_RELEASE);
  }
  me->lastCommand = EVS_EVENT_NULL;
}

uint16_t Scheduler_getScheduledPower()
{
  return scheduledPower;
}

void Scheduler_scheduleCharge(sck_schedule_t *schedulations)
{
  SchedTaskReq_t request;
  
  request.schedules[0] = schedulations[0];
  request.schedules[1] = schedulations[1];
  request.schedules[2] = schedulations[2];
  request.schedules[3] = schedulations[3];
  send_to_scheduler(&request);
}

bool Scheduler_isDisabled(void)
{
  return schedulerHandler->lastEvent == NULL;
}