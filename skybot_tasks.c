
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
#define LEFT_SPEED_GET(x) MAX_MOTORS_DUTYCYCLE - (MAX_FORWARD_SPEED + x.left) / (MAX_FORWARD_SPEED - MAX_BACKWARD_SPEED) * (MAX_MOTORS_DUTYCYCLE - MIN_MOTORS_DUTYCYCLE)

#define RIGHT_DUTY_SET(x) PWMPulseWidthSet(PWM1_BASE, PWM_OUT_RIGHT_MOTOR, (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (RIGHT_SPEED_GET(x))))
#define LEFT_DUTY_SET(x) PWMPulseWidthSet(PWM1_BASE, PWM_OUT_LEFT_MOTOR, (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (LEFT_SPEED_GET(x))))


xQueueHandle xMotorsQueue;
xQueueHandle SensorsQueue;
xQueueHandle whisker_queue;

float proximty_sensor_lut[]=
{
     3212.5, 3186.2, 2325.5, 1812.1, 1466.3,
     1225.4, 1030.2, 892,.2, 790.6, 686.1,
     646.5, 560.4, 521.7, 489.6, 443.4,
     419.7, 387.5, 377.7, 344.3, 318,
     315.7, 304.8, 291.5
};

//static float getDistante()
//{
//    uint32_t data_buff;
//    ADCSequenceDataGet(ADC0_BASE, 1, &data_buff);
//
//    uint8_t pos = 11;
//
//    if(data_buff > proximty_sensor_lut[pos])
//        pos = pos/2;
//    else
//        pos = pos + pos/2;
//
//    if(data_buff > proximty_sensor_lut[pos])
//        pos = pos/2;
//    else
//        pos = pos + pos/4;
//
//    if(data_buff > proximty_sensor_lut[pos])
//        pos = pos/2;
//    else
//        pos = pos + pos/8;
//}


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

        if(speed.left > MAX_FORWARD_SPEED)
            speed.left = MAX_FORWARD_SPEED;

        if(speed.left < MAX_BACKWARD_SPEED)
            speed.left = MAX_BACKWARD_SPEED;

        if(speed.right > MAX_FORWARD_SPEED)
            speed.left = MAX_FORWARD_SPEED;

        if(speed.right < MAX_BACKWARD_SPEED)
            speed.left = MAX_BACKWARD_SPEED;

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
    struct Speed speed;
    speed.right = 0.0;
    speed.left = 0.0;

    uint8_t whisker_active  = 0;
    UARTprintf("Brain Task Start!\n\n");

    //memset(&speed, 0, sizeof(struct Speed));

    //xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);

    while(1)
    {
        xQueueReceive(whisker_queue, &whisker_active, portMAX_DELAY);

        if(whisker_active)
        {
          memset(&speed, 0, sizeof(speed));
          xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);

          UARTprintf("Stoped\n");
        }else
        {


            xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);
            UARTprintf("Let's go ahead!\n");

        }

       xQueueReset(whisker_queue);
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

void DodgeLeft(struct Speed backward_speed, struct Speed foreward_speed)
{


    // Go Backward
    xQueueSend(xMotorsQueue, &backward_speed, portMAX_DELAY);
    vTaskDelay(1); // 1ms delay


    // Turn Left (right wheel back and left wheel foreward)
    // Go Foreward  with speed
}
