#ifndef SKYBOT_TASKS_H_
#define SKYBOT_TASKS_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"


xTaskHandle xReactiveTask;
xTaskHandle xProximityTask;


inline void setSpeed(float right, float left);

void move(int16_t distance);

void turn(int16_t angle);

void registerStep(int16_t wheel);
void registerStepFromISR(int16_t wheel, portBASE_TYPE * higherPriorityTaskWoken);
//wheel values
#define RIGHT_WHEEL         (0)
#define LEFT_WHEEL          (1)

void recordStep(uint8_t motion_type);
#define FORWARD_MOTION      (0)
#define BACKWARD_MOTION     (1)
#define RIGHT_MOTION        (2)
#define LEFT_MOTION         (3)

inline void sendEvent(uint8_t id);
inline void sendEventFromISR(uint8_t id, portBASE_TYPE * higherPriorityTaskWoken);
// id values

#define UBOT_STOPPED        (0)
#define COLLISION_START     (1)
#define COLLISION_END       (2)
#define ENEMY_FOUND         (3)
#define ENEMY_LOST          (4)
#define POSITION_IN         (5)
#define POSITION_OUT_LEFT   (6)
#define POSITION_OUT_RIGHT  (7)
#define POSITION_OUT_BOTH   (8)

short get_distance();

void init_tasks();


#endif /* SKYBOT_TASKS_H_ */
