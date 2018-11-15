//*****************************************************************************
//
// commands.c - FreeRTOS porting example on CCS4
//
// Este fichero contiene errores que seran explicados en clase
//
//*****************************************************************************


#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard TIVA includes */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

/* Other TIVA includes */
#include "utils/cpu_usage.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"

// Personal includes
#include "skybot_tasks.h"

// ==============================================================================
// The CPU usage in percent, in 16.16 fixed point format.
// ==============================================================================
uint32_t g_ui32CPUUsage;

// ==============================================================================
// Implementa el comando cpu que muestra el uso de CPU
// ==============================================================================
int  Cmd_cpu(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("ARM Cortex-M4F %u MHz - ",SysCtlClockGet() / 1000000);
    UARTprintf("%2u%% usage\n", (g_ui32CPUUsage+32768) >> 16);

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando free, que muestra cuanta memoria "heap" le queda al FreeRTOS
// ==============================================================================
int Cmd_free(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("%d free bytes\n", xPortGetFreeHeapSize());

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando task. S�lo es posible si la opci�n configUSE_TRACE_FACILITY de FreeRTOS est� habilitada
// ==============================================================================
#if ( configUSE_TRACE_FACILITY == 1 )

extern char *__stack;
int Cmd_tasks(int argc, char *argv[])
{
	char*	pcBuffer;
	uint8_t*	pi8Stack;
	portBASE_TYPE	x;
	
	pcBuffer = pvPortMalloc(1024);
	vTaskList(pcBuffer);
	UARTprintf("\t\t\t\tUnused\nTaskName\tStatus\tPri\tStack\tTask ID\n");
	UARTprintf("=======================================================\n");
	UARTprintf("%s", pcBuffer);
	
	// Calculate kernel stack usage
	x = 0;
	pi8Stack = (uint8_t *) &__stack;
	while (*pi8Stack++ == 0xA5)
	{
		x++;	//Esto s�lo funciona si hemos rellenado la pila del sistema con 0xA5 en el arranque
	}
	sprintf((char *) pcBuffer, "%%%us", configMAX_TASK_NAME_LEN);
	sprintf((char *) &pcBuffer[10], (const char *) pcBuffer, "kernel");
	UARTprintf("%s\t-\t*%u\t%u\t-\n", &pcBuffer[10], configKERNEL_INTERRUPT_PRIORITY, x/sizeof(portBASE_TYPE));
	vPortFree(pcBuffer);
	return 0;
}

#endif /* configUSE_TRACE_FACILITY */

#if configGENERATE_RUN_TIME_STATS
// ==============================================================================
// Implementa el comando "Stats"
// ==============================================================================
Cmd_stats(int argc, char *argv[])
{
	char*	pBuffer;

	pBuffer = pvPortMalloc(1024);
	if (pBuffer)
	{
		vTaskGetRunTimeStats(pBuffer); //Es un poco inseguro, pero por ahora nos vale...
		UARTprintf("TaskName\tCycles\t\tPercent\r\n");
		UARTprintf("===============================================\r\n");
		UARTprintf("%s", pBuffer);
		vPortFree(pBuffer);
	}
	return 0;
}
#endif

// ==============================================================================
// Implementa el comando help
// ==============================================================================
int Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    UARTprintf("Available commands \n");
    UARTprintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}

// ==============================================================================
// Implementa el comando help
// ==============================================================================
int Cmd_motor(int argc, char *argv[])
{
    struct Speed speed;

    if (argc != 3)
    {
        UARTprintf("use: motor <right_velocity> <left_velocity>\n");
    }
    else
    {
        speed.right = atof(argv[1]);
        speed.left = atof(argv[2]);
        xQueueSend(xMotorsQueue, &speed, portMAX_DELAY);

    }


    return(0);
}



// ==============================================================================
// Tabla con los comandos y su descripcion. Si quiero anadir alguno, debo hacerlo aqui
// ==============================================================================
tCmdLineEntry g_psCmdTable[] =
{
    { "help",     Cmd_help,      "     : List all commands" },
    { "?",        Cmd_help,      "        : Same as help" },
    { "motor",    Cmd_motor,     "       : Set right (r) or left (l) motor velocity"},
    { "cpu",      Cmd_cpu,       "      : Show CPU usage" },
    { "free",     Cmd_free,      "     : Show free memory" },
#if ( configUSE_TRACE_FACILITY == 1 )
	{ "tasks",    Cmd_tasks,     "    : Show tasks information" },
#endif
#if (configGENERATE_RUN_TIME_STATS)
	{ "stats",    Cmd_stats,      "    : Show tasks statistics" },
#endif
    { 0, 0, 0 }
};

// ==============================================================================
// Tarea UARTTask.  Espera la llegada de comandos por el puerto serie y los ejecuta al recibirlos...
// ==============================================================================
//extern xSemaphoreHandle g_xRxLineSemaphore;
void UARTStdioIntHandler(void);

void vUARTTask( void *pvParameters )
{
    char    pcCmdBuf[64];
    int32_t i32Status;
	
    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\n\n Welcome to SkyBot Project!\n");
	UARTprintf("\n\n FreeRTOS %s \n",tskKERNEL_VERSION_NUMBER);
	UARTprintf("\n Write ? to show the help info\n");
	UARTprintf("> ");    

	/* Loop forever */
	while (1)
	{

		/* Read data from the UART and process the command line */
		UARTgets(pcCmdBuf, sizeof(pcCmdBuf));
		if (strlen(pcCmdBuf) == 0)
		{
			UARTprintf("> ");
			continue;
		}

		//
		// Pass the line from the user to the command processor.  It will be
		// parsed and valid commands executed.
		//
		i32Status = CmdLineProcess(pcCmdBuf);

		//
		// Handle the case of bad command.
		//
		if(i32Status == CMDLINE_BAD_CMD)
		{
			UARTprintf("Bad command\n");	//No pongo acentos adrede
		}

		//
		// Handle the case of too many arguments.
		//
		else if(i32Status == CMDLINE_TOO_MANY_ARGS)
		{
			UARTprintf("Too many arguments\n");	//El maximo, CMDLINE_MAX_ARGS, esta definido en cmdline.c
		}

		//
		// Otherwise the command was executed.  Print the error code if one was
		// returned.
		//
		else if(i32Status != 0)
		{
			UARTprintf("The command returned the error %d\n",i32Status);
		}

		UARTprintf("> ");

	}
}
