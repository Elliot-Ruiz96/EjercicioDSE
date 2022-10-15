/*
  @Company      :  Universidad Tecnol�gica de Quer�taro

  @Project Name :  PLectura de Temperatura con sensor BMP180

  @File Name    :  main.c

  @Author       : Aguilar Pereyra J. Felipe

  @Summary      : This is the generated main.c using TM4C123GXL MCU.

  @Description  : In this description of the source file, we will use the I2C communication
                  to obtain the data from the BMP180 pressure sensor
                  to send the measurements through the UART communication


  @Version: 1.0

    Generation Information
        Product Revision   :  TM4C129X
        Device             :  TM4C129ENCPDT

    The generated drivers are tested against the following:
        Languaje           :  C ANSI C 89 Mode
        Compiler           :  TI V18.12.3LTS
        CCS                :  CCS v9.2.0.00013

    (c) 2020 Aguilar Pereyra J. Felipe. You may use this software
    and any derivatives exclusively with Texas Instruments products.

    THIS SOFTWARE IS SUPPLIED BY Aguilar Pereyra J. Felipe "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
    NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION
    WITH TEXAS INSTRUMENTS PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    Aguilar Pereyra J. Felipe PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/
/*
 * References
 * https://components101.com/sites/default/files/component_datasheet/BMP180%20Sensor%20Datasheet.pdf
 * https://forum.digikey.com/t/i2c-communication-with-the-ti-tiva-tm4c123gxl/13451
 * file:///C:/Users/uteq/Dropbox/DDP/Tiva%20C/tm4c123gh6pm%20DataSheet.pdf
 * https://software-dl.ti.com/simplelink/esd/simplelink_msp432e4_sdk/2.30.00.14/docs/driverlib/msp432e4/html/i2c_8h.html#a515b348b82f9e571c7fb7c649c929dd7
 */
//Included Libraries

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"                             // Libraria uart
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#define SLAVE_ADDR 0x77       // BMP180: 0x77
#define IdSlave_I2C_ADDR      0xD0
#define AC1_ADDRESS 0xAA
#define AC5_ADDRESS 0xB2
#define AC6_ADDRESS 0xB4
#define MC_ADDRESS 0xBC
#define MD_ADDRESS 0xBE
#define UT_ADDRESS 0xF6

uint32_t ui32SysClkFreq;

    int16_t uiAC1;
    uint16_t uiAC5;
    uint16_t uiAC6;
    int16_t uiMC;
    int16_t uiMD;
    int32_t uiUT;
    int32_t Temperature;

//This code initializes I2C peripheral 2.

//Initialize I2C2 function

// Inicializar I2C


void InitI2C2(void)
{
    //enable I2C module 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2); //Habilitar Periferico de I2C2

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2); //Resetear I2C2 

    //enable GPIO peripheral that contains I2C 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Habilitar el puerto N , para poder utilizar las terminales

    // Configure the pin muxing for I2C2 functions on port N5 and N4.
    GPIOPinConfigure(GPIO_PN5_I2C2SCL); // Senal de Reloj
    GPIOPinConfigure(GPIO_PN4_I2C2SDA); // Senal de Datos

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5); //Indicar al microcontrolador que terminal 5 va ser de tipo Clock
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4); //Indicar al microcontrolador que terminal 4 va ser de tipo Dato

    // Enable and initialize the I2CN master module.  Use the system clock for
    // the I2C2 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C2_BASE, ui32SysClkFreq/20, false); //Configurar ciclo de trabajo que tendra el protocolo de I2C2
    //I2CMasterInitExpClk(I2C2_BASE, Frecuencia de Operacion(ui32SysClkFreq=120 MHz) /20 -> 6MHz, false); El parametro del false indica que el baud rate =100kps, true=400kbps
    SysCtlDelay(ui32SysClkFreq / 3); //Retardo para que la medicion se haga de manera estable
    //clear I2C FIFOs
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}

//I2C Send Function

//sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    uint8_t i;
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, false);

    //stores list of variable number of arguments
    va_list vargs;

    //specifies the va_list to "open" and the last fixed argument
    //so vargs knows where to start looking
    va_start(vargs, num_of_args);

    //put data to be sent into FIFO
    I2CMasterDataPut(I2C2_BASE, va_arg(vargs, uint32_t));

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(num_of_args == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C2_BASE));

        //"close" variable argument list
        va_end(vargs);
    }

    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C2_BASE));

        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(i = 1; i < (num_of_args - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C2_BASE, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C2_BASE));
        }

        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C2_BASE, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C2_BASE));

        //"close" variable args list
        va_end(vargs);
    }
}

// I2C Receive Function

//read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
    uint32_t uintI2CMasterError1=0;

    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C2_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));

    uintI2CMasterError1 = I2CMasterErr(I2C2_BASE);

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));

    //return data pulled from the specified register
    return I2CMasterDataGet(I2C2_BASE);

    uintI2CMasterError1 = I2CMasterErr(I2C2_BASE);
}

// I2C Receive 16 bits Function

//read specified register on slave device
uint32_t I2CReceive16(uint32_t slave_addr, uint8_t reg)
{
    uint16_t data1=0;
    uint32_t data2=0;
    uint32_t uintI2CMasterError1=0;

    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C2_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));
    uintI2CMasterError1 = I2CMasterErr(I2C2_BASE);

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));

    data1 = I2CMasterDataGet(I2C2_BASE);
    uintI2CMasterError1 = I2CMasterErr(I2C2_BASE);
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C2_BASE, reg+1);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));
    uintI2CMasterError1 = I2CMasterErr(I2C2_BASE);
    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));
    data2 = I2CMasterDataGet(I2C2_BASE);
    uintI2CMasterError1 = I2CMasterErr(I2C2_BASE);

   // data1 = I2CMasterDataGet(I2C3_BASE);
    data2 = (data1  << 8) | data2;
    //return data pulled from the specified register
    return data2;

}

//  BMP180: Register D0h, this value is fixed to 0x55
uint8_t readId(uint8_t Slave_I2C_ADDRESS, uint8_t IdSlave_I2C_ADDRESS)
    {

            uint8_t dataId = 0;

            GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_1, 2);
            //I2CSend(Slave_I2C_ADDRESS, 1,IdSlave_I2C_ADDRESS);
            /*I2CMasterSlaveAddrSet(I2C3_BASE, Slave_I2C_ADDRESS, false);             // Escribir Direccion de registro de sensor
            I2CMasterDataPut(I2C3_BASE, IdSlave_I2C_ADDRESS);                       // Registro de sensor meas
            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);           // Iniciar comunicacion con sensor
            while(I2CMasterBusy(I2C3_BASE)){}                                       // Espera de respuesta puerto I2C2
            */
            /*I2CMasterSlaveAddrSet(I2C3_BASE, Slave_I2C_ADDRESS, true);                 //Leer Direccion de sensor
            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);        // Recibir datos de I2C
            while(I2CMasterBusy(I2C3_BASE)){}                                       // Espera de respuesta puerto I2C2
            dataId = I2CMasterDataGet(I2C3_BASE);                                  // Dato recibido por I2C
            I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);       // Finalizar comunicacion de recibir con sensor break point
            while(I2CMasterBusy(I2C3_BASE)){}                                       // Espera de respuesta puerto I2C2
            */

            dataId =I2CReceive(Slave_I2C_ADDRESS, IdSlave_I2C_ADDRESS);
            GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_1, 0);
            return dataId;
        }

//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //
    // Initialize the UART for console I/O.
    //
    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE));
    //Configura el periferico UART 0 con una velocidad de 115200 Baudios, 8 bits de informacion, 1 bit de parada y sin bit paridad

}

//*****************************************************************************

void
ReadCalData(void)
{


    uiAC1 = I2CReceive16(SLAVE_ADDR, AC1_ADDRESS);
    uiAC5 = I2CReceive16(SLAVE_ADDR, AC5_ADDRESS);
    uiAC6 = I2CReceive16(SLAVE_ADDR, AC6_ADDRESS);
    uiMD = I2CReceive16(SLAVE_ADDR, MD_ADDRESS);
    uiMC = I2CReceive16(SLAVE_ADDR, MC_ADDRESS);
    uiUT = I2CReceive16(SLAVE_ADDR, UT_ADDRESS);

    /*
    UARTCharPut(UART0_BASE, 'A');                                              // Envio de datos  via UART
    UARTCharPut(UART0_BASE, 'C');
    UARTCharPut(UART0_BASE, '5');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'A');                                              // Envio de datos  via UART
    UARTCharPut(UART0_BASE, 'C');
    UARTCharPut(UART0_BASE, '6');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'M');                                              // Envio de datos  via UART
    UARTCharPut(UART0_BASE, 'C');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'M');
    UARTCharPut(UART0_BASE, 'D');
    UARTCharPut(UART0_BASE, ' ');
    */
}

void
ReadUT(void)
{
    I2CSend(SLAVE_ADDR, 2, 0xF4, 0x2E);
 //   I2CSend(SLAVE_ADDR, 1, 0x2E);
    SysCtlDelay(120000);       //4.5 ms
    uiUT= I2CReceive16(SLAVE_ADDR, UT_ADDRESS);

}

int32_t CalUT(void)
{
    int32_t result;
        int32_t x1 = ((uiUT-uiAC6)*uiAC5)>>15;
        int32_t x2 = (uiMC<<11)/(x1+uiMD);
        int32_t _b5 = x1+x2;
        result = ((_b5+8)>>4);
        return result;
}

uint8_t chipId = 0;


void main(void)
{

    // Set the clocking to run directly from the external crystal/oscillator.
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_480), 120000000);

    // Initialize the GPIO for the LED.
             //

             SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
             GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1); //Leds 
             GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x00); // Escribir ceros en terminales para apagar leds

    //initialize UART module 0
    ConfigureUART(); // Para pooder ocupar UART
    //initialize I2C module 2
    InitI2C2();

    // Print the welcome message to the terminal.
             //
         UARTCharPut(UART0_BASE, 'C');                                          // Envio de datos  via UART
         UARTCharPut(UART0_BASE, 'h');                                               // Envio de datos  via UART
         UARTCharPut(UART0_BASE, 'i');                                               // Envio de datos  via UART
         UARTCharPut(UART0_BASE, 'p');                                              // Envio de datos  via UART
         UARTCharPut(UART0_BASE, 'I');
         UARTCharPut(UART0_BASE, 'd');
         UARTCharPut(UART0_BASE, ' ');
         // Read the data from the BMP180 over I2C.  This command starts a
                               // temperature measurement.  Then polls until temperature is ready.
                               // Then automatically starts a pressure measurement and polls for that
                               // to complete. When both measurement are complete and in the local
                               // buffer then the application callback is called from the I2C
                               // interrupt context.  Polling is done on I2C interrupts allowing
                               // processor to continue doing other tasks as needed.
                               //
                               //    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x02);
                               chipId = readId(SLAVE_ADDR, IdSlave_I2C_ADDR);
                               // Print the welcome message to the terminal.
                                      //
                               UARTCharPut(UART0_BASE,chipId);
                               ReadCalData();
                               ReadUT();
                               Temperature=CalUT();

    while(1){
        //

        ReadUT();
        Temperature=CalUT();
/*        Temperatura[0] =  Temperature / 10;
                Temperatura[1] =  Temperature - (Temperatura[0] * 10);
                Temperatura[2] = (Temperature - (Temperatura[0] * 10) - Temperatura[1]) * 10;
                Temperatura[3] = (Temperature - (Temperatura[0] * 10) - Temperatura[1] - (Temperatura[2] / 10)) * 100;

        //UARTCharPut(UART0_BASE, 13);
UARTCharPut(UART0_BASE,'T' );                                                              // Envio de datos  via UART
         UARTCharPut(UART0_BASE, Temperatura[0] + 48);                                              // Envio de datos  via UART
         UARTCharPut(UART0_BASE, Temperatura[1] + 48);                                              // Envio de datos  via UART
         UARTCharPut(UART0_BASE, Temperatura[2] + 48);                                              // Envio de datos  via UART
         UARTCharPut(UART0_BASE, Temperatura[3] + 48);                                              // Envio de datos  via UART
         UARTCharPut(UART0_BASE,' ');
         //UARTCharPut(UART0_BASE, 13);
          * */

    };
}
