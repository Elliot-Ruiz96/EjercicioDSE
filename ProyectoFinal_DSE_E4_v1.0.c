/*
Proyecto Integrador
Equipo:
Elliot Ruiz
Oswaldo Lara
Andres Muñoz
Eber Hernandez


Version Inicial para checar las principales actividades que se requiren 
*/
/*
 *  ======== heartBeatFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void heartBeatFxn(UArg arg0, UArg arg1)
{
	uint8_t uiDigit0 = 0;
	uint8_t uiDigit1 = 1;
	uint8_t uiDigit2 = 2;
	uint8_t uiDigit3 = 3;
	uint8_t uiDigit4 = 4;
	uint8_t uiDigit5 = 5;
	uint8_t uiDigit6 = 6;
	uint8_t uiDigit7 = 7;

Void tarea3(UArg arg0, UArg arg1)
{
    while (1) {
        Task_sleep((unsigned int)arg0);
        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0)
        {
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x01);
        }
        else
        {
        	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);
        }
    }
}
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>


/* Board Header file */
#include "Board.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"       // Conocer los registros de memoria
#include "inc/hw_types.h"        // Tipos de Hardware
#include "driverlib/debug.h"     // Para poder debuggear
#include "driverlib/sysctl.h"    // Manejo de timers
#include "driverlib/gpio.h"      // Manejo de entradas y salidas
#include "driverlib/adc.h"       // Incluir Definiciones del m�dulo ADC

#define TASKSTACKSIZE   512

Task_Struct taskADCStruct;
Char taskADCStack[TASKSTACKSIZE];



/* ADC variables  */
   uint32_t ui32ADC0Value[4];      //  Declaraci�n de la variable ui32ADC0Value que es un vector de 4 localidades de memoria
   uint8_t  banderaADC = 0;
   volatile uint32_t ui32Potenciometro;  //  Declaraci�n de la variable ui32Potenciometro para guardar 2 lecturas del potenciometro

   #define GPIO_PORTN_BASE 0x40064000 // Memory location in register map
/*
 *  ======== taskADCFxn ========
 *
 */
Void taskADCFxn(UArg arg0, UArg arg1)
{
    while (1)
    {
    			ADCIntClear(ADC0_BASE, 1);     //  Limpiar la interrupci�n del ADC

    	         ADCProcessorTrigger(ADC0_BASE, 1);     //  Inicio de conversi�n (TRIGGER) del ADC0

    	         while(!ADCIntStatus(ADC0_BASE, 1, false))  //  Rutina de espera por el fin de conversi�n del ADC0
    	         {
    	         }

    	         ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);   //  Traer los resusultados de la conversi�n (4 muestras) y guardar en el vector ui32ADC0Value

    	         ui32Potenciometro= (ui32ADC0Value[0] + ui32ADC0Value[1] + 1)/2;

    	         if( ui32Potenciometro > 1980)
    	         {
    	        	 //GPIO_toggle(Board_LED0);
    	        	 GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);
    	         }
    	         else
    	         {
    	        	 GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);
    	         }


    }
}


/*
 *  ======== main ========
 */
int main(void)
   {
    Task_Params taskParams;
    /* Call board init functions */
    Board_initGeneral();
    // Board_initEMAC();
    Board_initGPIO();

    /* ADC */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION),  //Enable Port N
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); // Configure as Output the PIN 0 (terminal) of PORT N
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);// Write a 0 in PN0, in other words, to turn off the LED1

    //
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

        										// Encontrar y seleccionar que perif�rico se va a utilizar
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //  Habilitar el perif�rico ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);// Habilitar el puerto E

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);   //  Configurar el secuenciador (m�dulo ADC0, Secuenciador SS1, Modo de disparo por sw, nivel de prioridad 0)

    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3), // Configurar PE3 Channel 0



    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_TS);      //  Configurar el paso del secuenciador (m�dulo ADC0, Secuenciador SS1, n�mero de paso = 0, fuente se�al anal. Sensor temperatura)
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_TS);      //  Configurar el paso del secuenciador (m�dulo ADC0, Secuenciador SS1, n�mero de paso = 1, fuente se�al anal. Sensor temperatura)
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_TS);      //  Configurar el paso del secuenciador (m�dulo ADC0, Secuenciador SS1, n�mero de paso = 2, fuente se�al anal. Sensor temperatura)
    ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END);  //  Configurar el paso del secuenciador (m�dulo ADC0, Secuenciador SS1, n�mero de paso = 0, fuente se�al anal. Sensor temperatura, �ltimo paso)

    ADCSequenceEnable(ADC0_BASE, 1);        //  Habilitar la conversi�n del ADC0

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &taskADCStack;
    Task_construct(&taskADCStruct, (Task_FuncPtr)taskADCFxn, &taskParams, NULL);




    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
