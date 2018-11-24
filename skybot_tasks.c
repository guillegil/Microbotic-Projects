
#include "skybot_tasks.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
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
xQueueHandle whisker_queue;


void DodgeLeft()
{
    struct Speed speed;
    vTaskDelay(100); // .5s delay

    /****** Go Backward ******/
    speed.left = -0.5f;
    speed.right = -0.5f;

    xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(1000); // 1s delay

    /****** Turn left ******/

    speed.left = 0.5f;
    speed.right = -0.5f;

    xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(500);

    /****** Go Foreward ******/

    speed.left = 0.5f;
    speed.right = 0.5f;

    xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(1500);

    /****** Turn right ******/

    speed.left = -0.5f;
    speed.right = 0.5f;

    xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);
    vTaskDelay(300);

    /****** Go Foreward ******/

    speed.left = 1.0f;
    speed.right = 1.0f;

    xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);
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
        xQueueReceive(xMotorsQueue, &speed, portMAX_DELAY);

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
    struct Speed speed;

    uint8_t whisker_active  = 0;
    UARTprintf("Brain Task Start!\n\n");     // Only for debug

    speed.left = 0.1;                        // Only for debug
    speed.right = 0.1;

    xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);

    while(1)
    {
        xQueueReceive(whisker_queue, &whisker_active, portMAX_DELAY);

        if(whisker_active)
        {
          memset(&speed, 0, sizeof(speed));
          xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);

          UARTprintf("Stoped\n");    // Only for debug
          DodgeLeft();               // Turn left.
        }else
        {
            //memset(&speed, 0, sizeof(speed));                   // Only for debug

            speed.left = 0.1;                        // Only for debug
            speed.right = 0.1;
            xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);


            UARTprintf("Let's go ahead!\n");                    // Only for debug
        }

       SysCtlSleep();
    }
}

void init_tasks()
{
    xMotorsQueue = xQueueCreate(2, sizeof(struct Speed));
    whisker_queue = xQueueCreate(1, sizeof(uint8_t));


    if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(BrainTask, "BrainTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }

    if((xTaskCreate(MotorsTask, "MotorsTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL)) != pdTRUE)
    {
        while(1);
    }
}


