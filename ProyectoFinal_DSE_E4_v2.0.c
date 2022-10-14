/*
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
// #include <ti/drivers/EMAC.h>
#include <ti/drivers/GPIO.h>

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

/* Board Header file */
#include "Board.h"

#define OP_NOOP   0x00
#define OP_DIGIT0 0x01
#define OP_DIGIT1 0x02
#define OP_DIGIT2 0x03
#define OP_DIGIT3 0x04
#define OP_DIGIT4 0x05
#define OP_DIGIT5 0x06
#define OP_DIGIT6 0x07
#define OP_DIGIT7 0x08
#define OP_DECODEMODE  0x09
#define OP_INTENSITY   0x0A
#define OP_SCANLIMIT   0x0B
#define OP_SHUTDOWN    0x0C
#define OP_DISPLAYTEST 0x0F

void SpiSend(uint8_t, uint8_t );
void DisplayInit(void);
uint32_t ui32SysClkFreq;
uint8_t ui8Delay = 5;
uint8_t ui8Counter = 0;

UInt32 time_sleep;

#define TASKSTACKSIZE   512

Task_Struct task0Struct,task1Struct,task2Struct,task3Struct;                                                    //1. Asignar estructura
Char task0Stack[TASKSTACKSIZE],task1Stack[TASKSTACKSIZE],task2Stack[TASKSTACKSIZE],task3Stack[TASKSTACKSIZE];   //2. String (Stack)

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

	ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); //Habilitar puerto N
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3); //Configurar PN3 como salida CS = LOAD
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3,0x08);        //LOAD ON

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_3);    //  Configurar salidas PD1 = MOSI, PD3 = CLK

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);      /// Habilitacion del puerto F
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);    // Configurar PF0 como salida LED 4

	DisplayInit();

	while (1) {
		//Task_sleep((unsigned int)arg0);
		//GPIO_toggle(Board_LED0);

		SpiSend(OP_DIGIT0, uiDigit0+ui8Counter);
		SpiSend(OP_DIGIT1, uiDigit1+ui8Counter);
		SpiSend(OP_DIGIT2, uiDigit0);
		SpiSend(OP_DIGIT3, uiDigit0);
		SpiSend(OP_DIGIT4, uiDigit0);
		SpiSend(OP_DIGIT5, uiDigit0);
		SpiSend(OP_DIGIT6, uiDigit0);
		SpiSend(OP_DIGIT7, uiDigit0);

		/*
		// Turn on the LED
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 1); //blink LED
		SysCtlDelay(ui32SysClkFreq/6);//
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); //turn off LED
		SysCtlDelay(ui32SysClkFreq/6);//
		*/
		ui8Counter = ui8Counter + 1;
		Task_sleep(time_sleep);
	}
}

Void tarea1(UArg arg0, UArg arg1)
{
    while (1) {
        Task_sleep((unsigned int)arg0);
        GPIO_toggle(Board_LED1);
    }
}

Void tarea2(UArg arg0, UArg arg1)
{
	int cond = 0;
	while(1)
		{
		Task_sleep((unsigned int)arg0);
			if(cond == 0) {
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x10);
				cond = 1;
			}
			else{
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);
				cond = 0;
			}
		}
}

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

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 433;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)heartBeatFxn, &taskParams, NULL);

    // 3. Parametros de la tarea
    taskParams.arg0 = 769;
    taskParams.stackSize = TASKSTACKSIZE;
	taskParams.stack = &task1Stack;
	Task_construct(&task1Struct, (Task_FuncPtr)tarea1, &taskParams, NULL);

	taskParams.arg0 = 1259;
	taskParams.stackSize = TASKSTACKSIZE;
	taskParams.stack = &task2Stack;
	Task_construct(&task2Struct, (Task_FuncPtr)tarea2, &taskParams, NULL);

	taskParams.arg0 = 20;
	taskParams.stackSize = TASKSTACKSIZE;
	taskParams.stack = &task3Stack;
	Task_construct(&task3Struct, (Task_FuncPtr)tarea3, &taskParams, NULL);

	//Time sleep (uS) --> 10ms
	time_sleep = 100000 / Clock_tickPeriod;

     /* Turn on user LED */
    //GPIO_write(Board_LED0, Board_LED_ON);
    //GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); //Configure PN0 as Output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x01);

	//Button
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); //Enable PortJ
	GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_1 | GPIO_PIN_0, GPIO_DIR_MODE_IN);

    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

//Functions for Display
void SpiSend(uint8_t uiHiger, uint8_t uiLower )
{
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3,0x00); //LOAD OFF
      uint8_t i;
     //latch the data onto the display LOAD = 0

     SysCtlDelay(ui8Delay);
 for(i=8;i>0;i--)
     {
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,0x00);     // SPI CLK LOW
     SysCtlDelay(ui8Delay);
     if(uiHiger & 0x80)GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1,0x02);//MSB first // Charge Boolean Data in MOSI
            else GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
     uiHiger <<= 1;
     SysCtlDelay(ui8Delay);
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,0x08);     // SPI CLK HIGH
     SysCtlDelay(ui8Delay);

     }
 SysCtlDelay(ui8Delay);
  for(i=8;i>0;i--)
     {
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,0x00);     // SPI CLK LOW
     SysCtlDelay(ui8Delay);
     if(uiLower & 0x80)GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1,0x02);//LSB first // Charge Boolean Data in MOSI
            else GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
     uiLower <<= 1;
     SysCtlDelay(ui8Delay);
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,0x08);     // SPI CLK HIGH
     SysCtlDelay(ui8Delay);

     }
  GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3,0x08); //LOAD ON
}

void DisplayInit(void)
{
    SpiSend(OP_SHUTDOWN, 0x00); // OP_SHUTDOWN    0x0C
    SpiSend(OP_DISPLAYTEST, 0x00);
    SpiSend(OP_INTENSITY, 0x04);
    SysCtlDelay(10000000);
    SpiSend(OP_SCANLIMIT, 0x07);
    SpiSend(OP_DECODEMODE, 0xFF);
    SpiSend(OP_DISPLAYTEST, 0x01);
    SysCtlDelay(10000000);
    SpiSend(OP_DISPLAYTEST, 0x00);
    SpiSend(OP_SHUTDOWN, 0x01); // OP_SHUTDOWN    0x0C
}

