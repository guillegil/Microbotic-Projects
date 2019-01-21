
#include "skybot_tasks.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
//#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"
#include "math.h"

#define RIGHT_SPEED_GET(x) MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED - x.right) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE)
#define LEFT_SPEED_GET(x) MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED + x.left) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE)

#define RIGHT_DUTY_SET(x) PWMPulseWidthSet(PWM1_BASE, PWM_OUT_RIGHT_MOTOR, (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (RIGHT_SPEED_GET(x))))
#define LEFT_DUTY_SET(x) PWMPulseWidthSet(PWM1_BASE, PWM_OUT_LEFT_MOTOR, (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (LEFT_SPEED_GET(x))))

#define LUT_SIZE    23

#define UNKNOWN_VALUE   FLT_MAX


extern void vUARTTask( void *pvParameters );


// Queues
xQueueHandle SensorsQueue;
xQueueHandle reactiveQueue;
xQueueHandle motorsQueue;
xQueueHandle motionQueue;
xQueueHandle mappingQueue;
xQueueHandle proximityQueue;
xQueueHandle proximityQueue;
xQueueHandle arbiterQueue;

// Messages structures
struct EventCommand
{
    uint8_t   id;
    uint16_t  turn;
    uint16_t  move;
};

struct MotionCommand
{
    uint8_t id;
    int16_t parameter;
};


#define REGISTER_STEP       (0)
#define MOVE                (1)
#define TURN                (2)

struct MappingCommand
{
    uint8_t id;
};
#define INTERSECTION_RIGHT  (4)
#define INTERSECTION_LEFT   (5)

struct Speed
{
    float right;
    float left;
};

struct Positions
{
    int16_t bot_x;
    int16_t bot_y;
    int16_t enemy_x;
    int16_t enemy_y;
    uint16_t azimuthal;
};


// Communication functions
inline void setSpeed(float right, float left)
{
    struct Speed speed = {right, left};
    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
}

inline void sendEvent(uint8_t id)
{
    struct EventCommand event = {id};
    xQueueSend(reactiveQueue, &event, portMAX_DELAY);
}

inline void sendEventFromISR(uint8_t id, portBASE_TYPE * higherPriorityTaskWoken)
{
    struct EventCommand event = {id};
    xQueueSendFromISR(reactiveQueue, &event, higherPriorityTaskWoken);
}

inline uint8_t getQuadrant(int16_t x, int16_t y)
{
    if(x >= 0)
    {
        if(y > 0)
            return 1;
        else
            return 4;
    }else
    {
        if(y > 0)
            return 2;
        else
            return 3;
    }
}

inline int16_t getAngleToCenter(uint8_t quadrant, uint16_t ath, int16_t x, int16_t y)
{
   uint16_t alpha = 0;

   if(x != 0)
       alpha = atan(y/x);
   else
       alpha = 0;

   switch(quadrant)
   {
       case 1:
           if(ath >= 0 && ath <= 180)           // alpha > 0
               return (270 - ath - alpha);
           else
           {
               if(alpha != 0)
                   return (180 - ath + alpha);

               return (270 - ath);
           }

           break;
       case 2:                                   // alpha < 0
           if(ath >= 0 && ath <= 180)
               return (-ath + alpha);

           return (360 - ath + alpha);

           break;
       case 3:                                   // alpha > 0
           if(ath >= 0 && ath <= 180)
               return (-ath + alpha);

           return (360- ath + alpha);

           break;
       case 4:                                  // alpha < 0
           if(ath >= 0 && ath <= 180)
           {
               if(alpha != 0)
                   return (180 - ath + alpha);
               else
                   return (-ath + 90);
           }

           if(alpha != 0)
               return (90 - ath - alpha);
           else
               return (-ath + 90);

           break;
   };






   return 0;
}

inline uint16_t getDistanceToCenter(float x, float y)
{
   return sqrt((x*x) + (y*y));
}


void move(int16_t distance)   // Distancia en mm
{
    struct MotionCommand command;
    command.id = MOVE;
    command.parameter = distance;
    xQueueSend(motionQueue, &command, portMAX_DELAY);
}

void turn(int16_t angle)        // Gira a la derecha si el ángulo es negativo.
{
    struct MotionCommand command;
    command.id = TURN;
    command.parameter = angle;
    xQueueSend(motionQueue, &command, portMAX_DELAY);
}

void registerStep(int16_t wheel)
{
    struct MotionCommand command;
    command.id = REGISTER_STEP;
    command.parameter = wheel;
    xQueueSend(motionQueue, &command, portMAX_DELAY);
}

void registerStepFromISR(int16_t wheel, portBASE_TYPE * higherPriorityTaskWoken)
{
    struct MotionCommand command;
    command.id = REGISTER_STEP;
    command.parameter = wheel;
    xQueueSendFromISR(motionQueue, &command, higherPriorityTaskWoken);
}

void sendMappingCommand(uint8_t id)
{
    struct MappingCommand command;
    command.id = id;
    xQueueSend(mappingQueue, &command, portMAX_DELAY);
}

void sendPositions(float bot_x, float bot_y, float bot_angle, float enemy_x, float enemy_y)
{
    struct Positions positions;
    positions.bot_x = bot_x;
    positions.bot_y = bot_y;
    positions.azimuthal = bot_angle;
    positions.enemy_x = enemy_x;
    positions.enemy_y = enemy_y;
    xQueueSend(arbiterQueue, &positions, portMAX_DELAY);
}

// Utility functions
void DodgeLeft()
{
    struct Speed speed;
    vTaskDelay(100); // .5s delay

    /****** Go Backward ******/
    speed.left = -0.5f;
    speed.right = -0.5f;

    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(1000); // 1s delay

    /****** Turn left ******/

    speed.left = 0.5f;
    speed.right = -0.5f;

    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(500);

    /****** Go Foreward ******/

    speed.left = 0.5f;
    speed.right = 0.5f;

    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(1500);

    /****** Turn right ******/

    speed.left = -0.5f;
    speed.right = 0.5f;

    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(300);

    /****** Go Foreward ******/

    speed.left = 1.0f;
    speed.right = 1.0f;

    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(1000);


}

short proximity_lut[LUT_SIZE] =
{
     3212, 3186, 2325, 1812, 1466,
     1225, 1030, 892,  791,  686,
     646,  560,  522,  490,  443,
     420,  387,  378,  344,  318,
     316,  305,  291
};




uint16_t calculte_distance(uint32_t data)
{
    short out, inc;
    unsigned int imin, imid, imax;

    imin = 0;
    imax = LUT_SIZE - 1;

    if(proximity_lut[imin] < data)
        return 20;
    if(proximity_lut[imax] > data)
        return LUT_SIZE * 20;

    while (imin + 1 < imax)
    {
        imid = (imin+imax)>>1;

        if (proximity_lut[imid] < data)
            imax = imid;
        else
            imin = imid;
    }

    inc = 20 * (proximity_lut[imin] - data) / (proximity_lut[imin] - proximity_lut[imax]);
    out = 20 * (1 + imin) + inc;

    return out;
}

short get_distance()
{
    short out;
    uint32_t data;

    ADCProcessorTrigger(ADC0_BASE, 1);          // Causes a processor trigger for a sample sequence
    ADCSequenceDataGet(ADC0_BASE, 1, &data);

    out = calculte_distance(data);

    return out;
}


// Task functions
static portTASK_FUNCTION(MotorsTask, pvParameters)
{
    struct Speed speed;
//    float right_duty_cycle;
//    float left_duty_cycle;

    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    while(1)
    {
        xQueueReceive(motorsQueue, &speed, portMAX_DELAY);

        if(speed.left > MAX_FORWARD_SPEED)
            speed.left = MAX_FORWARD_SPEED;

        if(speed.left < MAX_BACKWARD_SPEED)
            speed.left = MAX_BACKWARD_SPEED;

        if(speed.right > MAX_FORWARD_SPEED)
            speed.right = MAX_FORWARD_SPEED;

        if(speed.right < MAX_BACKWARD_SPEED)
            speed.right = MAX_BACKWARD_SPEED;

        RIGHT_DUTY_SET(speed);
        LEFT_DUTY_SET(speed);

        //right_duty_cycle = MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED - speed.right) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE);
        //left_duty_cycle = MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED - speed.left) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE);
//        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_RIGHT_MOTOR,
//                         (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * right_duty_cycle));
//        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_LEFT_MOTOR,
//                             (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * left_duty_cycle));
    }

    //vTaskDelete(NULL);
}

static portTASK_FUNCTION(ReactiveTask, pvParameters)
{
    struct EventCommand event;
    uint8_t state = TURN_STATE;
    uint16_t last_turn, last_move;

    turn(360);

    while(true)
    {
        xQueueReceive(reactiveQueue, &event, portMAX_DELAY);
        switch(event.id)
        {
            case UBOT_STOPPED:
                switch(state)
                {
                    case TURN_STATE:
                        turn(last_turn);
                        state = MOVE_STATE;
                        break;
                    case MOVE_STATE:
                        move(last_move);
                        state = TURN_STATE;
                        break;
                }
                break;
            case COLLISION_START:
                break;
            case COLLISION_END:
                break;
            case ENEMY_FOUND:
                move(event.move);
                break;
            case ENEMY_LOST:
                turn(event.turn);
                state = MOVE_STATE;

                break;
            case POSITION_OUT_LEFT:
                vTaskSuspend(xProximityTask);
                xQueueReset(reactiveQueue);

                if(state == TURN_STATE)
                    turn(-135);             // For now does not change

                state = MOVE_STATE;
                break;
            case POSITION_OUT_RIGHT:
                vTaskSuspend(xProximityTask);
                xQueueReset(reactiveQueue);

                if(state == TURN_STATE)
                    turn(135);              // For now does not change

                state = MOVE_STATE;
                break;
            case POSITION_IN:
                vTaskResume(xProximityTask);
                break;
        }

        last_turn = event.turn;
        last_move = event.move;

       SysCtlSleep();
    }
}


inline float middle_line_slope(float x1, float y1, float x2, float y2)
{
    return (x1 - x2) / (y2 - y1);
}


inline float middle_point(float x1, float x2)
{
    return x1 + (x2 - x1) / 2;
}


static portTASK_FUNCTION(MappingTask, pvParameters)
{
    float bot_x, bot_y, bot_angle;
    float enemy_x, enemy_y;
    float m_old, xm_old, ym_old;
    float x_old, y_old;
    bool enemy_found;
    uint8_t saved_points;
    struct MappingCommand command;

    bot_x = 0.0f;
    bot_y = 0.0f;
    bot_angle = 0.0f;
    enemy_x = UNKNOWN_VALUE;
    enemy_y = UNKNOWN_VALUE;
    x_old = UNKNOWN_VALUE;
    y_old = UNKNOWN_VALUE;
    m_old = UNKNOWN_VALUE;
    xm_old = UNKNOWN_VALUE;
    ym_old = UNKNOWN_VALUE;

    enemy_found = false;
    saved_points = 0;

    while(true)
    {
        struct Positions pos;

        xQueueReceive(mappingQueue, &command, portMAX_DELAY);

        switch(command.id)
        {
            case FORWARD_MOTION:
                bot_x += cosf(bot_angle) * STEP_DISTANCE;
                bot_y += sinf(bot_angle) * STEP_DISTANCE;
                break;
            case BACKWARD_MOTION:
                bot_x -= cosf(bot_angle) * STEP_DISTANCE;
                bot_y -= sinf(bot_angle) * STEP_DISTANCE;
                break;
            case LEFT_MOTION:
                bot_angle += STEP_ANGLE_RAD;
                break;
            case RIGHT_MOTION:
                bot_angle -= STEP_ANGLE_RAD;
                break;
        }

        if(command.id == INTERSECTION_RIGHT  ||  command.id == INTERSECTION_LEFT)
        {
            float x_new, y_new;

            if(command.id == INTERSECTION_RIGHT)
            {
                x_new = bot_x + FLOOR_SENSOR_SEPARATION * sinf(bot_angle);
                y_new = bot_y + FLOOR_SENSOR_SEPARATION * cosf(bot_angle);
            }
            else
            {
                x_new = bot_x - FLOOR_SENSOR_SEPARATION * sinf(bot_angle);
                y_new = bot_y - FLOOR_SENSOR_SEPARATION * cosf(bot_angle);
            }

            if(saved_points > 0)
            {
                float m_new = middle_line_slope(x_new, y_new, x_old, y_old);
                float xm_new = middle_point(x_new, x_old);
                float ym_new = middle_point(y_new, y_old);

                if(saved_points > 1)
                {
                    float center_x = (m_new * xm_new - m_old * xm_old + ym_old - ym_new) / (m_new - m_old);
                    float center_y = m_new * center_x + ym_new - m_new * xm_new;
                    bot_x += center_x;
                    bot_y += center_y;
                    if(enemy_found)
                    {
                        enemy_x += center_x;
                        enemy_y += center_y;
                    }
                }
                else
                {
                    saved_points = 2;
                }

                m_old = m_new;
                xm_old = xm_new;
                ym_old = ym_new;
            }
            else
            {
                saved_points = 1;
            }

            x_old = x_new;
            y_old = y_new;
        }

        sendPositions(bot_x, bot_y, bot_angle, enemy_x, enemy_y);
    }
}


static portTASK_FUNCTION(MotionTask, pvParameters)
{
    unsigned remain_right_increments, remain_left_increments;
    uint8_t motion;
    struct MotionCommand command;

    while(true)
    {
        xQueueReceive(motionQueue, &command, portMAX_DELAY);

        switch(command.id)
        {
            case MOVE:
                remain_right_increments = (unsigned) abs(command.parameter) / STEP_DISTANCE;
                remain_left_increments = remain_right_increments;
                if(command.parameter > 0)
                {
                    motion = FORWARD_MOTION;
                    setSpeed(MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
                }
                else
                {
                    motion = BACKWARD_MOTION;
                    setSpeed(MAX_BACKWARD_SPEED, MAX_BACKWARD_SPEED);
                }
                break;

            case TURN:
                remain_right_increments = (unsigned) abs(command.parameter) / STEP_ANGLE_DEG;
                remain_left_increments = remain_right_increments;
                if(command.parameter > 0)
                {
                    motion = LEFT_MOTION;
                    setSpeed(MAX_FORWARD_SPEED, MAX_BACKWARD_SPEED);
                }
                else
                {
                    motion  = RIGHT_MOTION;
                    setSpeed(MAX_BACKWARD_SPEED, MAX_FORWARD_SPEED);
                }
                break;

            case REGISTER_STEP:
                sendMappingCommand(motion);
                if(command.parameter == RIGHT_WHEEL)
                    remain_right_increments--;
                else if(command.parameter == LEFT_WHEEL)
                    remain_left_increments--;
                if(remain_right_increments == 0  || remain_left_increments == 0)
                {
                    setSpeed(0, 0);
                    sendEvent(UBOT_STOPPED);
                }
                break;
        }

    }
}

static portTASK_FUNCTION(proximityTask, pvParameters)
{
    uint32_t data;
    uint16_t distance;
    struct EventCommand event;
    uint8_t flag = 1;

    while(1)
    {
        xQueueReceive(proximityQueue, &data, portMAX_DELAY);
        distance = calculte_distance(data);

        if(flag == 1 && (distance > 20 && distance < LUT_SIZE*20))
        {
            event.id = ENEMY_FOUND;
            xQueueSend(reactiveQueue, &event, portMAX_DELAY);
            flag = 0;
        }
        else if(flag == 0 && (distance <= 20 || distance >= LUT_SIZE*20))
        {
            event.id = ENEMY_LOST;
            xQueueSend(reactiveQueue, &event, portMAX_DELAY);
            flag = 1;
        }
    }
}

static portTASK_FUNCTION(arbiterTask, pvParameters)
{
    struct Positions pos;
    struct EventCommand evn;



    memset(&pos, 0, sizeof(struct Positions));

    while(true)
    {
        xQueueReceive(arbiterQueue, &pos, portMAX_DELAY);
        evn.turn = getAngleToCenter(getQuadrant(pos.bot_x, pos.bot_y), pos.azimuthal, pos.bot_x, pos.bot_y);

    }
}



void init_tasks()
{
    motorsQueue = xQueueCreate(MOTORS_QUEUE_SIZE, sizeof(struct Speed));
    reactiveQueue = xQueueCreate(REACTIVE_QUEUE_SIZE, sizeof(struct EventCommand));
    motionQueue = xQueueCreate(MOTION_QUEUE_SIZE, sizeof(struct MotionCommand));
    proximityQueue = xQueueCreate(PROXIMITY_QUEUE_SIZE, sizeof(uint32_t));
    arbiterQueue = xQueueCreate(ARBITER_QUEUE_SIZE, sizeof(struct Positions));
    mappingQueue = xQueueCreate(MAPPING_QUEUE_SIZE, sizeof(struct MappingCommand));

    if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(MotorsTask, "MotorsTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(MotionTask, "MotionTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(ReactiveTask, "ReactiveTask", 256, NULL, tskIDLE_PRIORITY + 1, &xReactiveTask)) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(proximityTask, "proximityTask", 256, NULL, tskIDLE_PRIORITY + 1, &xProximityTask)) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(arbiterTask, "arbiterTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(MappingTask, "mappingTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }


}


