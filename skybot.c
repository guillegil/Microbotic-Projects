
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (ROM_)
#include "driverlib/rom_map.h"       // TIVA: Funciones API incluidas en ROM de micro (ROM_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "utils/uartstdio.h"     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "queue.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/cpu_usage.h"

#include "skybot_tasks.h"
#include "config.h"


// Variables globales

extern xQueueHandle whisker_queue;
extern xQueueHandle proximityQueue;

/* An array to hold a count of the number of times each timer expires. */
uint32_t ui32ExpireCounters =  0 ;
int32_t i32Estado_led=0;

uint8_t poll = 0xF0;       // TODO: Change to an union with multiple flags

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************

#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}


//#if 0
void vApplicationIdleHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    	SysCtlSleep();
    	//SysCtlDeepSleep();
}
//#endif

//*****************************************************************************
//
// Configure the system clock and initialize all needed peripherals.
//
//*****************************************************************************
void ADCConfig(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
      while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER5))    // Wait for TIMER5 to be ready
      {
      }

      SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER5);

      TimerClockSourceSet(TIMER5_BASE,TIMER_CLOCK_SYSTEM);
      TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
      TimerLoadSet(TIMER5_BASE, TIMER_A, SysCtlClockGet() - 1);


       SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);         // Enable ADC0
       while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))    // Wait for ADC0 to be ready
       {
       }

       SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
       while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))    // Wait for GPIO_PE to be ready
       {
       }

       SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);

       GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3);

       ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

       ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);   // To trigger it from processor set to ADC_TRIGGER_PROCESSOR
       TimerControlTrigger(TIMER5_BASE, TIMER_A, true);

       ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0); // Interrupt + Last seq + channel 0 selected
       ADCSequenceEnable(ADC0_BASE, 1);

       ADCIntEnable(ADC0_BASE, 1);
       ADCIntClear(ADC0_BASE,1);
       IntPrioritySet(INT_ADC0SS1, configMAX_SYSCALL_INTERRUPT_PRIORITY);
       IntEnable(INT_ADC0SS1);

       TimerEnable(TIMER5_BASE, TIMER_A);
}

void OpticalSensorsConf(void)
{
    /*
    TODO: delete this block
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3))    // Wait for TIMER3 to be ready
    {
    }

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER3);

    TimerClockSourceSet(TIMER3_BASE,TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() - 1);

    TimerDisable(TIMER3_BASE, TIMER_A);
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_TIMER3A, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_TIMER4A);
    */

    SysCtlPeripheralEnable(OPTICAL_SENSORS_GPIO_PERIPH);
    while(!SysCtlPeripheralReady(OPTICAL_SENSORS_GPIO_PERIPH))    // Wait for GPIO_PF to be ready

    SysCtlPeripheralSleepEnable(OPTICAL_SENSORS_GPIO_PERIPH);

    GPIOPinTypeGPIOInput(OPTICAL_SENSORS_GPIO_BASE, ALL_OPTICAL_SENSOR_PINS);
    GPIOIntTypeSet(OPTICAL_SENSORS_GPIO_BASE, ALL_OPTICAL_SENSOR_PINS, GPIO_BOTH_EDGES) ;
    GPIOIntClear(OPTICAL_SENSORS_GPIO_BASE, ALL_OPTICAL_SENSOR_PINS);
    GPIOIntEnable(OPTICAL_SENSORS_GPIO_BASE, ALL_OPTICAL_SENSOR_PINS);
    IntPrioritySet(INT_GPIOB, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_GPIOB);
}


void WhiskerConf(void)
{
    /***** Setup for TIMER4_A *****/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4))    // Wait for TIMER4 to be ready
    {
    }

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER4);

    TimerClockSourceSet(TIMER4_BASE,TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER4_BASE, TIMER_A, (SysCtlClockGet()/10)/4 - 1);       // 25ms

    TimerDisable(TIMER4_BASE, TIMER_A);
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_TIMER4A, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_TIMER4A);

    /***** End TIMER4_A *****/

    /***** Setup for Whisker Input (PORTF and PIN 0, same as sw2 on tiva board) *****/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))    // Wait for GPIO_PF to be ready

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);

    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_INT_PIN_0, GPIO_FALLING_EDGE) ;
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
    IntPrioritySet(INT_GPIOF, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_GPIOF);

    /***** End Whisker *****/
}


void system_init()
{
    //
    // Set the clocking to run at 40 MHz from the PLL (200 Mhz divided by 5)
    // and enable clock gating
    //
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
            SYSCTL_OSC_MAIN);
    SysCtlPeripheralClockGating(true);
    CPUUsageInit(SysCtlClockGet(), configTICK_RATE_HZ/10, 3);
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //
    // se usa para mandar mensajes por el puerto serie
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, SysCtlClockGet());
    //
    // Configure PWM
    //
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);

    //
    // The PWM peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);

    //
    // PF2 and PF3 are needed for the PWM, so GPIOF must be enabled.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Configure the GPIO pin muxing to select PWM functions.
    //
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

    //
    // Configure the PWM function for these pins.
    //
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Configure the PWM1 to count up/down without synchronization.
    //
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to SERVOM_FREQUENCY (value in config.h).
    //
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, SysCtlClockGet()/16/MOTORS_FREQUENCY);

    //
    // Set PWM1 to a duty cycle of SERVOM_DUTYCYCLE (value in config.h).
    //
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,
                     (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (double)NEUTRAL_MOTORS_DUTYCYCLE));
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,
                         (uint32_t)((float)PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3) * (double)NEUTRAL_MOTORS_DUTYCYCLE));


    //
    // Enable the PWM1 Bit6 (PF2) and Bit7(PF3) output signal.
    //
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

}




//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int main(void)
{
    //
    // Initial configuration
    //
    system_init();

    ButtonsInit();

    //
    // Create tasks
    //
    WhiskerConf();
    OpticalSensorsConf();
    ADCConfig();

    init_tasks();
    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asa que no hace falta habilitarlas

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //

    while(1)
    {
    }
}

void ISR_ProximitySensor(void)                          // Remove after
{
   portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
   uint32_t data_buff;
   uint32_t data;

   ADCSequenceDataGet(ADC0_BASE, 1, &data_buff);
   ADCIntClear(ADC0_BASE,1);

   xQueueSendFromISR(proximityQueue, &data, &higherPriorityTaskWoken);
   portEND_SWITCHING_ISR(higherPriorityTaskWoken);

}

void ISR_WhiskerSensor(void)
{
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    poll = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
    TimerEnable(TIMER4_BASE,TIMER_A);                                   // Enable Timer4_A which will interrupt after 25ms

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_0);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}


void ISR_OpticalSensor(void)
{
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;


    if(GPIOIntStatus(OPTICAL_SENSORS_GPIO_BASE, FLOOR_SENSORS) == FLOOR_SENSORS)
    {
        if(GPIOPinRead(OPTICAL_SENSORS_GPIO_BASE, RIGHT_FLOOR_SENSOR_PIN))
        {
            sendEventFromISR(POSITION_OUT_RIGHT, &higherPriorityTaskWoken);
            GPIOIntClear(OPTICAL_SENSORS_GPIO_BASE, RIGHT_FLOOR_SENSOR_PIN);
        }else if(GPIOPinRead(OPTICAL_SENSORS_GPIO_BASE, LEFT_FLOOR_SENSOR_PIN))
        {
            sendEventFromISR(POSITION_OUT_LEFT, &higherPriorityTaskWoken);
            GPIOIntClear(OPTICAL_SENSORS_GPIO_BASE, LEFT_FLOOR_SENSOR_PIN);
        }else
        {
            sendEventFromISR(POSITION_IN, &higherPriorityTaskWoken);
            GPIOIntClear(OPTICAL_SENSORS_GPIO_BASE, FLOOR_SENSORS);
        }


//        if(GPIOPinRead(OPTICAL_SENSORS_GPIO_BASE, FLOOR_SENSOR_PIN) == 0)
//            sendEventFromISR(POSITION_IN, &higherPriorityTaskWoken);
//        else
//            sendEventFromISR(POSITION_OUT, &higherPriorityTaskWoken);


    }
    else
    {
        if(GPIOIntStatus(OPTICAL_SENSORS_GPIO_BASE, RIGHT_ENCODER_PIN) == RIGHT_ENCODER_PIN)
        {
            registerStepFromISR(RIGHT_WHEEL, &higherPriorityTaskWoken);
            GPIOIntClear(OPTICAL_SENSORS_GPIO_BASE, RIGHT_ENCODER_PIN);
        }
        else
        {
            registerStepFromISR(LEFT_WHEEL, &higherPriorityTaskWoken);
            GPIOIntClear(OPTICAL_SENSORS_GPIO_BASE, LEFT_ENCODER_PIN);
        }
    }
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}


void ISR_DebounceTimer(void)            // It Reads from Whisker button after 25ms in order to avoid multiple reads.
{
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if(!GPIOPinRead(GPIO_PORTF_BASE, GPIO_INT_PIN_0))                       // Send to BrainTask to stop (or change the direction) the ubot
    {
        sendEventFromISR(COLLISION_START, &higherPriorityTaskWoken);
        GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_INT_PIN_0, GPIO_RISING_EDGE) ;
    }
    else
    {
        sendEventFromISR(COLLISION_END, &higherPriorityTaskWoken);
        GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_INT_PIN_0, GPIO_FALLING_EDGE);
    }

    TimerDisable(TIMER4_BASE,TIMER_A);
    TimerIntClear(TIMER4_BASE, TIMER_A);
    TimerLoadSet(TIMER4_BASE, TIMER_A, (SysCtlClockGet()/10)/4 - 1);   // Reset the load into Timer4_A

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
