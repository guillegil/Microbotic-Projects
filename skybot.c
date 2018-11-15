
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

#include "skybot_tasks.h"
#include "config.h"


// Variables globales

/* An array to hold a count of the number of times each timer expires. */
uint32_t ui32ExpireCounters =  0 ;
int32_t i32Estado_led=0;
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
void system_init()
{
    //
    // Set the clocking to run at 40 MHz from the PLL (200 Mhz divided by 5)
    // and enable clock gating
    //
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
            SYSCTL_OSC_MAIN);
    SysCtlPeripheralClockGating(true);

    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //
    // se usa para mandar mensajes por el puerto serie
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
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

    //
    // Create tasks
    //
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
