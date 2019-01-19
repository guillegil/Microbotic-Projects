
#include "skybot_tasks.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
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

#define RIGHT_SPEED_GET(x) MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED - x.right) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE)
#define LEFT_SPEED_GET(x) MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED + x.left) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE)

#define RIGHT_DUTY_SET(x) PWMPulseWidthSet(PWM1_BASE, PWM_OUT_RIGHT_MOTOR, (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (RIGHT_SPEED_GET(x))))
#define LEFT_DUTY_SET(x) PWMPulseWidthSet(PWM1_BASE, PWM_OUT_LEFT_MOTOR, (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (LEFT_SPEED_GET(x))))

#define LUT_SIZE    23


extern void vUARTTask( void *pvParameters );


// Queues
xQueueHandle SensorsQueue;
xQueueHandle reactiveQueue;
xQueueHandle motorsQueue;
xQueueHandle motionQueue;
xQueueHandle proximityQueue;



// Messages structures
struct Event
{
    uint8_t id;
};

struct MotionCommand
{
    uint8_t id;
    int16_t parameter;
};
#define REGISTER_STEP   (0)
#define MOVE                (1)
#define TURN                (2)

struct Speed
{
    float right;
    float left;
};

// Communication functions
inline void setSpeed(float right, float left)
{
    struct Speed speed = {right, left};
    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
}

inline void sendEvent(uint8_t id)
{
    struct Event event = {id};
    xQueueSend(reactiveQueue, &event, portMAX_DELAY);
}

inline void sendEventFromISR(uint8_t id, portBASE_TYPE * higherPriorityTaskWoken)
{
    struct Event event = {id};
    xQueueSendFromISR(reactiveQueue, &event, higherPriorityTaskWoken);
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
    struct Event event;
    uint8_t state = TURN_STATE;

    setSpeed(0, 0);

    while(true)
    {
        xQueueReceive(reactiveQueue, &event, portMAX_DELAY);

        switch(event.id)
        {
            case UBOT_STOPPED:
                switch(state)
                {
                    case TURN_STATE:
                        state = MOVE_STATE;
                        break;
                    case MOVE_STATE:
                        state = TURN_STATE;
                        break;
                }
                break;
            case COLLISION_START:
                break;
            case COLLISION_END:
                break;
            case ENEMY_FOUND:
                setSpeed(1.0f, 1.0f);
                break;
            case ENEMY_LOST:
                // ¿Busca?
                state = TURN_STATE;
                if(!state)
                    turn(180);
                else
                    move(200);
                break;
            case POSITION_OUT_LEFT:
                vTaskSuspend(xProximityTask);
                xQueueReset(reactiveQueue);
                state = TURN_STATE;
                if(!state)
                    turn(-135);
                else
                    move(150);

                break;
            case POSITION_OUT_RIGHT:
                vTaskSuspend(xProximityTask);
                xQueueReset(reactiveQueue);
                state = TURN_STATE;
                if(!state)
                    turn(135);
                else
                    move(150);

                break;
            case POSITION_IN:
                vTaskResume(xProximityTask);

                break;
        }

       SysCtlSleep();
    }
}


static portTASK_FUNCTION(MotionTask, pvParameters)
{
    uint32_t remain_right_increments, remain_left_increments;
    struct MotionCommand command;

    while(true)
    {
        xQueueReceive(motionQueue, &command, portMAX_DELAY);

        switch(command.id)
        {
            case MOVE:
                remain_right_increments = (abs(command.parameter) * ENCODER_STRIPES) / (unsigned)(WHEEL_DIAMETER * M_PI);
                remain_left_increments = (abs(command.parameter) * ENCODER_STRIPES) / (unsigned)(WHEEL_DIAMETER * M_PI);
                if(command.parameter > 0)
                    setSpeed(MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
                else
                    setSpeed(MAX_BACKWARD_SPEED, MAX_BACKWARD_SPEED);
                break;

            case TURN:
                remain_right_increments = (abs(command.parameter) * ENCODER_STRIPES * WHEELS_SEPARATION) / WHEEL_DIAMETER / 360;
                remain_left_increments = (abs(command.parameter) * ENCODER_STRIPES * WHEELS_SEPARATION) / WHEEL_DIAMETER / 360;
                if(command.parameter > 0)
                    setSpeed(MAX_FORWARD_SPEED, MAX_BACKWARD_SPEED);
                else
                    setSpeed(MAX_BACKWARD_SPEED, MAX_FORWARD_SPEED);
                break;

            case REGISTER_STEP:
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
    struct Event event;


    while(1)
    {
        xQueueReceive(proximityQueue, &data, portMAX_DELAY);
        distance = calculte_distance(data);

        if(distance > 20 && distance < LUT_SIZE*20)
            event.id = ENEMY_FOUND;
        else
            event.id = ENEMY_LOST;

        xQueueSend(reactiveQueue, &event, portMAX_DELAY);
    }
}



void init_tasks()
{
    motorsQueue = xQueueCreate(MOTORS_QUEUE_SIZE, sizeof(struct Speed));
    reactiveQueue = xQueueCreate(REACTIVE_QUEUE_SIZE, sizeof(uint8_t));
    motionQueue = xQueueCreate(MOTION_QUEUE_SIZE, sizeof(struct MotionCommand));
    proximityQueue = xQueueCreate(PROXIMITY_QUEUE_SIZE, sizeof(struct MotionCommand));

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


}


