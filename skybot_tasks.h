#ifndef SKYBOT_TASKS_H_
#define SKYBOT_TASKS_H_

#include "FreeRTOS.h"
#include "queue.h"

xQueueHandle xMotorsQueue;
struct Speed
{
    float right;
    float left;
};

void init_tasks();
short get_distance();


#endif /* SKYBOT_TASKS_H_ */
