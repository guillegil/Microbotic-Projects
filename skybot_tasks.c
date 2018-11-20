
#include <skybot_tasks.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
//#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "config.h"

#define RIGHT_SPEED_GET(x) MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED - x.right) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE)

#define LEFT_SPEED_GET(x) MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED - x.left) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE)

#define RIGHT_DUTY_SET(x) PWMPulseWidthSet(PWM1_BASE, PWM_OUT_RIGHT_MOTOR, (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (RIGHT_SPEED_GET(x))))

#define LEFT_DUTY_SET(x) PWMPulseWidthSet(PWM1_BASE, PWM_OUT_LEFT_MOTOR, (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (LEFT_SPEED_GET(x))))



xQueueHandle xMotorsQueue;
xQueueHandle SensorsQueue;
xQueueHandle whisker_queue;

struct Speed
{
    float right;
    float left;
};



static portTASK_FUNCTION(MotorsTask, pvParameters)
{
    struct Speed speed;
//    float right_duty_cycle;
//    float left_duty_cycle;

    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    while(1)
    {
        xQueueReceive(xMotorsQueue, &speed, portMAX_DELAY);
        // TODO: truncar velocidades superiores a la máxima

        RIGHT_DUTY_SET(speed);
        LEFT_DUTY_SET(speed);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

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
    struct Speed speed;
    speed.right = 0.0;
    speed.left = 0.0;

    uint8_t whisker_active  = 0;
    UARTprintf("Brain Task Start!\n\n");

    while(1)
    {
        xQueueReceive(whisker_queue, &whisker_active, 0);

        if(whisker_active)
        {
          memset(&speed, 0, sizeof(speed));
          xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);

          UARTprintf("Stoped\n");
        }else
        {
            UARTprintf("Let's go ahead!\n");
            // Go foreward ???
        }


       SysCtlSleep();
    }
}


//static portTASK_FUNCTION(SensorTask, pvParameters)
//{
//    struct Speed speed;
//
//    speed.right = 0.0;
//    speed.left = 0.0;
//    xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);
//
//    SysCtlSleep();
//}



void init_tasks()
{
    xMotorsQueue = xQueueCreate(2, sizeof(struct Speed));
    whisker_queue = xQueueCreate(1, sizeof(uint8_t));



    if((xTaskCreate(BrainTask, "BrainTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

//    if((xTaskCreate(SensorTask, "Sensor task", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
//    {
//        while(1);
//    }

    if((xTaskCreate(MotorsTask, "MotorsTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }


}
