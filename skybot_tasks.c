
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

xQueueHandle SensorsQueue;


inline void setSpeed(float right, float left)
{
    struct Speed speed = {right, left};
    xQueueSend(motorsQueue, &speed, portMAX_DELAY);
}

inline void sendEvent(uint8_t id)
{
    struct Event event = {id};
    xQueueSend(brainQueue, &event, portMAX_DELAY);
}

inline void sendEventFromISR(uint8_t id, portBASE_TYPE higherPriorityTaskWoken)
{
    struct Event event = {id};
    xQueueSendFromISR(brainQueue, &event, &higherPriorityTaskWoken);
}

void move(int16_t distance)
{
    struct MovementCommand command;
    command.id = MOVE;
    command.parameter = distance;
    xQueueSend(movementQueue, &command, portMAX_DELAY);
}

void turn(int16_t angle)
{
    struct MovementCommand command;
    command.id = TURN;
    command.parameter = angle;
    xQueueSend(movementQueue, &command, portMAX_DELAY);
}

void registerMovemente(int16_t wheel)
{
    struct MovementCommand command;
    command.id = REGISTER_MOVEMENT;
    command.parameter = wheel;
    xQueueSend(movementQueue, &command, portMAX_DELAY);
}


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


short get_distance()
{
    short out, inc;
    unsigned int imin, imid, imax;
    uint32_t data;

    ADCProcessorTrigger(ADC0_BASE, 1);          // Causes a processor trigger for a sample sequence
    ADCSequenceDataGet(ADC0_BASE, 1, &data);

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

static portTASK_FUNCTION(BrainTask, pvParameters)
{
    struct Event event;
    bool dodging, turning, long_side;

    dodging = false;
    turning = false;
    long_side = true;

    setSpeed(0, 0);

    move(180);

    while(true)
    {
        xQueueReceive(brainQueue, &event, portMAX_DELAY);

        switch(event.id)
        {
            case COLLISION_START:
                dodging = true;
                turning = false;
                move(-50);
                break;
            case COLLISION_END:
                break;
            case UBOT_STOPPED:
                if(dodging)
                {
                    if(!turning)
                    {
                        turn(-90);
                        turning = true;
                    }
                    else
                    {
                        dodging = false;
                        long_side = true;
                        move(180);
                    }
                }
                else
                {
                    if(turning)
                    {
                        turning = false;
                        if(long_side)
                        {
                            long_side = false;
                            move(120);
                        }
                        else
                        {
                            long_side = true;
                            move(180);
                        }
                    }
                    else
                    {
                        turning = true;
                        turn(90);
                    }
                }
                break;
        }

       SysCtlSleep();
    }
}


static portTASK_FUNCTION(MovementTask, pvParameters)
{
    uint32_t remain_right_increments, remain_left_increments;
    struct MovementCommand command;

    while(true)
    {
        xQueueReceive(movementQueue, &command, portMAX_DELAY);

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

            case REGISTER_MOVEMENT:
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

void init_tasks()
{
    motorsQueue = xQueueCreate(2, sizeof(struct Speed));
    brainQueue = xQueueCreate(1, sizeof(uint8_t));
    movementQueue = xQueueCreate(3, sizeof(struct MovementCommand));


    if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(MotorsTask, "MotorsTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(MovementTask, "MovementTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(BrainTask, "BrainTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

}


