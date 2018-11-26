#ifndef SKYBOT_TASKS_H_
#define SKYBOT_TASKS_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

xQueueHandle xMotorsQueue;
struct Speed
{
    float right;
    float left;
};

xQueueHandle encoderQueue;
#define RIGHT_WHEEL         ((uint8_t)0)
#define LEFT_WHEEL          ((uint8_t)1)

void init_tasks();
short get_distance();


#endif /* SKYBOT_TASKS_H_ */
