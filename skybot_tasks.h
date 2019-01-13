#ifndef SKYBOT_TASKS_H_
#define SKYBOT_TASKS_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"


xTaskHandle xReactiveTask;


inline void setSpeed(float right, float left);
xQueueHandle motorsQueue;
struct Speed
{
    float right;
    float left;
};

xQueueHandle movementQueue;
struct MovementCommand
{
    uint8_t id;
    int16_t parameter;
};
// Command 0:
#define REGISTER_MOVEMENT   (0)
// Parameters
#define RIGHT_WHEEL         (0)
#define LEFT_WHEEL          (1)
// Command 1
#define MOVE                (1)
// Command 2
#define TURN                (2)

inline void sendEvent(uint8_t id);
inline void sendEventFromISR(uint8_t id, portBASE_TYPE higherPriorityTaskWoken);
xQueueHandle reactiveQueue;
struct Event
{
    uint8_t id;
};
#define UBOT_STOPPED        (0)
#define COLLISION_START     (1)
#define COLLISION_END       (2)
#define ENEMY_FOUND         (3)
#define ENEMY_LOST          (4)
#define POSITION_IN         (5)
#define POSITION_OUT        (6)

void init_tasks();
short get_distance();


#endif /* SKYBOT_TASKS_H_ */
