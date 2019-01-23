
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
    int16_t azimuthal;
};


// Communication functions
inline void setSpeed(float right, float left)
{
    struct Speed speed = {right, left};
    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
}

inline void sendEvent(uint8_t id)
{
    struct EventCommand event;
    event.id = id;
    event.move = 0;
    event.turn = 0;
    xQueueSend(reactiveQueue, &event, portMAX_DELAY);
}


inline void sendEventFromISR(uint8_t id, portBASE_TYPE * higherPriorityTaskWoken)
{
    struct EventCommand event;
    event.id = id;
    event.move = 0;
    event.turn = 0;
    xQueueSendFromISR(reactiveQueue, &event, higherPriorityTaskWoken);
}


inline int16_t getAngleToCenter(float x, float y)
{
   float angle;

   if(x == 0)
   {
       if(y > 0)
           angle = M_PI / 2.0f;
       else
           angle = -M_PI / 2.0f;
   }
   else if(x > 0)
   {
       angle = atanf(y/x) + M_PI;
   }
   else
   {
       angle = atanf(y/x);
   }

   return angle * 180.0f / M_PI;
}

inline uint16_t getDistanceToCenter(int16_t x, int16_t y)
{
   return sqrtf((x*x) + (y*y));
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

void sendMappingCommandFromISR(uint8_t id, portBASE_TYPE * higherPriorityTaskWoken)
{
    struct MappingCommand command;
    command.id = id;
    xQueueSendFromISR(mappingQueue, &command, higherPriorityTaskWoken);
}

void sendPositions(float bot_x, float bot_y, float bot_angle, float enemy_x, float enemy_y)
{
    struct Positions positions;
    positions.bot_x = bot_x;
    positions.bot_y = bot_y;
    positions.azimuthal = bot_angle;
    positions.enemy_x = enemy_x;
    positions.enemy_y = enemy_y;
    //xQueueSend(arbiterQueue, &positions, portMAX_DELAY);
}

// Utility functions


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
    uint8_t state;
    int16_t move_distance;
    int16_t turn_angle;
    int16_t wheel_out_angle;
    int16_t wheel_in_angle;

    move_distance = 150;
    turn_angle = 360;
    wheel_out_angle = 360;
    wheel_in_angle = 30;

    state = TURN_STATE;
    turn(360);

    while(true)
    {
        xQueueReceive(reactiveQueue, &event, portMAX_DELAY);
        switch(event.id)
        {
            case UBOT_STOPPED:
                switch(state)
                {
                    case MOVE_STATE:
                        state = TURN_STATE;
                        turn(turn_angle);
                        break;

                    case TURN_STATE:
                        state = MOVE_STATE;
                        move(move_distance);
                        break;
                }
                break;
            case ENEMY_FOUND:
                if(state != LEFT_WHEEL_OUT_STATE  && state != RIGHT_WHEEL_OUT_STATE)
                {
                    state = MOVE_STATE;
                    move(10000);
                }
                break;

            case ENEMY_LOST:
                if(state != LEFT_WHEEL_OUT_STATE  && state != RIGHT_WHEEL_OUT_STATE)
                {
                    state = TURN_STATE;
                    turn(turn_angle);
                }
                break;

            case POSITION_OUT_LEFT:
//                vTaskSuspend(xArbiterTask);
//                vTaskSuspend(xProximityTask);
                turn(-wheel_out_angle);
                state = LEFT_WHEEL_OUT_STATE;
                break;

            case POSITION_OUT_RIGHT:
                turn(wheel_out_angle);
                state = RIGHT_WHEEL_OUT_STATE;
                break;

            case POSITION_IN:
                if(state == LEFT_WHEEL_OUT_STATE)
                    turn(-wheel_in_angle);
                else
                    turn(wheel_in_angle);
                state = TURN_STATE;
                break;
            case UPDATE_VALUES:
//                move_distance = event.move;
//                turn_angle = event.turn;
                break;
        }

        UARTprintf("Event: %d\n", event.id);

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

inline float get_module(float x1, float y1, float x2, float y2)
{
    float x_dist;
    float y_dist;

    x_dist = x1 - x2;
    y_dist = y1 - y2;

    return sqrtf(x_dist * x_dist + y_dist * y_dist);
}


void get_center(float x1, float y1, float x2, float y2, float x3, float y3, float * center_x, float * center_y)
{
    float xm1, ym1, m1;
    float xm2, ym2, m2;

    xm1 = middle_point(x1, x2);
    ym1 = middle_point(y1, y2);
    m1 = middle_line_slope(x1, y1, x2, y2);

    xm2 = middle_point(x2, x3);
    ym2 = middle_point(y2, y3);
    m2 = middle_line_slope(x2, y2, x3, y3);

    *center_x = (m1 * xm1 - m2 * xm2 + ym2 - ym1) / (m1 - m2);
    *center_y = m1 * *center_x + ym1 - m1 * xm1;
}


static portTASK_FUNCTION(MappingTask, pvParameters)
{
    float bot_x, bot_y, bot_angle;
    float enemy_x, enemy_y;
    float x_old, y_old;
    float x_new, y_new;
    bool enemy_found;
    uint8_t saved_points;
    struct MappingCommand command;

    bot_x = 0.0f;
    bot_y = 0.0f;
    bot_angle = 0.0f;

    enemy_found = false;
    saved_points = 0;

    while(true)
    {
        struct Positions pos;

        xQueueReceive(mappingQueue, &command, portMAX_DELAY);

        switch(command.id)
        {
            case FORWARD_MOTION:
                bot_x += cosf(bot_angle) * STEP_DISTANCE;// / 2.0f;
                bot_y += sinf(bot_angle) * STEP_DISTANCE;// / 2.0f;
                break;
            case BACKWARD_MOTION:
                bot_x -= cosf(bot_angle) * STEP_DISTANCE;// / 2.0f;
                bot_y -= sinf(bot_angle) * STEP_DISTANCE;// / 2.0f;
                break;
            case LEFT_MOTION:
                bot_angle += STEP_ANGLE_RAD;// / 2.0f;
                if(bot_angle < 0)
                    bot_angle += 2.0f * M_PI;
                else if(bot_angle >= 2 * M_PI)
                    bot_angle -= 2.0f * M_PI;
                break;
            case RIGHT_MOTION:
                bot_angle -= STEP_ANGLE_RAD;// / 2.0f;
                break;
        }

        if(command.id == INTERSECTION_RIGHT  ||  command.id == INTERSECTION_LEFT)
        {
            float x_current, y_current;
            bool good_point;

//            if(command.id == INTERSECTION_RIGHT)
//            {
//                x_current = bot_x + FLOOR_SENSOR_SEPARATION * sinf(bot_angle);
//                y_current = bot_y + FLOOR_SENSOR_SEPARATION * cosf(bot_angle);
//            }
//            else
//            {
//                x_current = bot_x - FLOOR_SENSOR_SEPARATION * sinf(bot_angle);
//                y_current = bot_y - FLOOR_SENSOR_SEPARATION * cosf(bot_angle);
//            }

            x_current = bot_x;
            y_current = bot_y;

            good_point = get_module(x_current, y_current, x_new, y_new) > 50.0f &&  get_module(x_current, y_current, x_old, y_old) > 50.0f;

            if(saved_points == 2)
            {
                if(good_point)
                {
                    float center_x, center_y;
                    get_center(x_old, y_old, x_new, y_new, x_current, y_current, &center_x, &center_y);
                    bot_x -= center_x;
                    bot_y -= center_y;
                    if(enemy_found)
                    {
                        enemy_x -= center_x;
                        enemy_y -= center_y;
                    }

                    x_old = x_new;
                    y_old = y_new;
                    x_new = x_current;
                    y_new = y_current;
                }
            }
            else
            {
                saved_points++;

                x_old = x_new;
                y_old = y_new;
                x_new = x_current;
                y_new = y_current;
            }
        }

        if(saved_points == 2)
            sendPositions(bot_x, bot_y, bot_angle * 180.0f / M_PI, enemy_x, enemy_y);

        //UARTprintf("x: %d   y: %d  angle: %d\n",(int)bot_x,(int)bot_y,(int)bot_angle);
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

        if(flag == 1 && (distance > LOWER_PROXIMITY_THRESHOLD && distance < UPPER_PROXIMITY_THRESHOLD))
        {
            event.id = ENEMY_FOUND;
            event.move = 0;
            event.turn = 0;
            xQueueSend(reactiveQueue, &event, portMAX_DELAY);
            flag = 0;
        }
        else if(flag == 0 && (distance > OVER_RANGE_THRESHOLD))
        {
            event.id = ENEMY_LOST;
            event.move = 0;
            event.turn = 0;
            xQueueSend(reactiveQueue, &event, portMAX_DELAY);
            flag = 1;
        }
    }
}

static portTASK_FUNCTION(arbiterTask, pvParameters)
{
    struct Positions pos;
    struct EventCommand evn;

//    uint16_t last_enemy_x, last_enemy_y;
//    uint8_t last_event;

    memset(&pos, 0, sizeof(struct Positions));

    evn.id = UPDATE_VALUES;

    while(true)
    {
        xQueueReceive(arbiterQueue, &pos, portMAX_DELAY);

        if((pos.bot_x > 50 || pos.bot_x < -50) || (pos.bot_y > 50 || pos.bot_y < -50))
        {
            evn.move = getDistanceToCenter(pos.bot_x, pos.bot_y);
            evn.turn = getAngleToCenter(pos.bot_x, pos.bot_y) - pos.azimuthal;
        }
        else
        {
            evn.move = 0;
            evn.turn = 360;
        }


        // last_enemy_x = pos.enemy_x;
        // last_enemy_y = pos.enemy_y;//                vTaskSuspend(xProximityTask);

        xQueueSend(reactiveQueue, &evn, portMAX_DELAY);
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

//    if((xTaskCreate(proximityTask, "proximityTask", 256, NULL, tskIDLE_PRIORITY + 1, &xProximityTask)) != pdTRUE)
//    {
//        while(1);
//    }

    if((xTaskCreate(MappingTask, "mappingTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

//    if((xTaskCreate(arbiterTask, "arbiterTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
//    {
//        while(1);
//    }


}


