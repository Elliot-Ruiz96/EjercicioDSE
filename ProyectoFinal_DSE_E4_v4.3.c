/*
  @Company      : UTQ - Visteon

  @Project Name : Lectura de Temperatura con sensor BMP180

  @File Name    : main.c

  @Author       : Hernandez, Eber
                  Lara, Oswaldo
                  Munoz, Andres
                  Ruiz, Elliot

  @Summary      : This is the generated main.c using TM4C123GXL MCU.

  @Description  : I2C communication shall be used  to obtain the data from the 
                  BMP180 pressure sensor and send the measurements through the 
                  UART communication.


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

//----------------------------------------------------------------------------------------------------------------------
// Body Identification
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Included files to resolve specific definitions in this file
//----------------------------------------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------------------------------------
// Local macros
//----------------------------------------------------------------------------------------------------------------------
#define OP_NOOP             0x00
#define OP_DIGIT0           0x01
#define OP_DIGIT1           0x02
#define OP_DIGIT2           0x03
#define OP_DIGIT3           0x04
#define OP_DIGIT4           0x05
#define OP_DIGIT5           0x06
#define OP_DIGIT6           0x07
#define OP_DIGIT7           0x08
#define OP_DECODEMODE       0x09
#define OP_INTENSITY        0x0A
#define OP_SCANLIMIT        0x0B
#define OP_SHUTDOWN         0x0C
#define OP_DISPLAYTEST      0x0F

#define SLAVE_ADDR          0x77       // BMP180: 0x77
#define IdSlave_I2C_ADDR    0xD0
#define AC1_ADDRESS         0xAA
#define AC5_ADDRESS         0xB2
#define AC6_ADDRESS         0xB4
#define MC_ADDRESS          0xBC
#define MD_ADDRESS          0xBE
#define UT_ADDRESS          0xF6


//----------------------------------------------------------------------------------------------------------------------
// Local types
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Local data
//----------------------------------------------------------------------------------------------------------------------
uint32_t    ui32SysClkFreq;
uint32_t    globalcount = 0;
uint32_t    globalcount_max = 350;
uint32_t    flag_time_stamp_saved=0;
uint32_t    time_stamp=0;

uint16_t    uiAC5;
uint16_t    uiAC6;

uint8_t     ui8Delay = 5;
uint8_t     ui8Seconds = 0;
uint8_t     ui8Minutes = 0;
uint8_t     ui8Hours   = 9;
uint8_t     bt1 = 0;
uint8_t     bt2 = 0;
uint8_t     bt1_rebound = 0;
uint8_t     bt2_rebound = 0;
uint8_t     Display_State = 0;
uint8_t     Clock_State = 0;
uint8_t     counter_buffer = 0;
uint8_t     max_clock_counter [3] = {60,60,12};
uint8_t     counter_bt1_pressed = 0;
uint8_t     bt1_return_flag = 0;
uint8_t     bt1_time_stamp=0;
uint8_t     counter_bt1_pressed_flag=0;
uint8_t     ERROR_MSG_1 [16] = {'T','E','M','P','E','R','A','T','U','R','A',' ','A','L','T','A'};
uint8_t     ERROR_MSG_2 [16] = {'T','E','M','P','E','R','A','T','U','R','A',' ','B','A','J','A'};

int32_t     uiUT;
int32_t     Temperature;

int16_t     uiAC1;
int16_t     uiMC;
int16_t     uiMD;

//----------------------------------------------------------------------------------------------------------------------
// Exported data
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Constant exported data
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------
// Local function prototypes
//----------------------------------------------------------------------------------------------------------------------

/* Local function prototypes */

//======================================================================================================================
// LOCAL FUNCTIONS
//======================================================================================================================
//----------------------------------------------------------------------------------------------------------------------
/// @brief    This code initializes I2C function, module and peripheral 2.
///
/// @param    Void - Returns no value
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------
void InitI2C2(void)
{
    //Enable I2C module 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    //Reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

    //Enable GPIO peripheral that contains I2C 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //Configure the pin muxing for I2C2 functions on port N5 and N4
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    // Enable and initialize the I2CN master module.  Use the system clock for
    // the I2C2 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C2_BASE, ui32SysClkFreq/20, false);
    SysCtlDelay(ui32SysClkFreq / 3);

    //Clear I2C FIFOs
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}

//----------------------------------------------------------------------------------------------------------------------
/// @brief    I2C Send Function - this code sends an I2C command to the specified slave
///
/// @param    Void - Returns no value
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    uint8_t i;
    va_list vargs;  //Stores list of variable number of arguments

    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C2_BASE, slave_addr, false);

    //Specifies the va_list to "open" and search the last fixed argument
    va_start(vargs, num_of_args);

    //Put data to be sent into FIFO
    I2CMasterDataPut(I2C2_BASE, va_arg(vargs, uint32_t));

    //if there is only one argument, we only need to use the
    //single send I2C function
    if(1 == num_of_args)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C2_BASE));

        //"Close" variable argument list
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

//----------------------------------------------------------------------------------------------------------------------
/// @brief    I2C  Receive Function - this code reads specified register on slave device
///
/// @param    uint32_t slave_addr, uint8_t reg
///
/// @return   uint32_t uintI2CMasterError1
//----------------------------------------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------------------------------------
/// @brief    I2C Receive 16 bits Functionn - this code reads specified register on slave device
///
/// @param    uint32_t slave_addr, uint8_t reg
///
/// @return   uint32_t data2
//----------------------------------------------------------------------------------------------------------------------
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

//----------------------------------------------------------------------------------------------------------------------
/// @brief    BMP180: Register D0h, this value is fixed to 0x55
///
/// @param    uint8_t Slave_I2C_ADDRESS, IdSlave_I2C_ADDRESS
///
/// @return   uint8_t dataId
//----------------------------------------------------------------------------------------------------------------------
uint8_t readId(uint8_t Slave_I2C_ADDRESS, uint8_t IdSlave_I2C_ADDRESS)
{

    uint8_t dataId = 0;

    GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_1, 2);
    //I2CSend(Slave_I2C_ADDRESS, 1,IdSlave_I2C_ADDRESS);
    /*I2CMasterSlaveAddrSet(I2C3_BASE, Slave_I2C_ADDRESS, false);       // Escribir Direccion de registro de sensor
    I2CMasterDataPut(I2C3_BASE, IdSlave_I2C_ADDRESS);                   // Registro de sensor meas
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);       // Iniciar comunicacion con sensor
    while(I2CMasterBusy(I2C3_BASE))                                     // Espera de respuesta puerto I2C2
    */
    /*I2CMasterSlaveAddrSet(I2C3_BASE, Slave_I2C_ADDRESS, true);        //Leer Direccion de sensor
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);    // Recibir datos de I2C
    while(I2CMasterBusy(I2C3_BASE)){}                                   // Espera de respuesta puerto I2C2
    dataId = I2CMasterDataGet(I2C3_BASE);                               // Dato recibido por I2C
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);   // Finalizar comunicacion de recibir con sensor break point
    while(I2CMasterBusy(I2C3_BASE)){                                    // Espera de respuesta puerto I2C2
    */

    dataId =I2CReceive(Slave_I2C_ADDRESS, IdSlave_I2C_ADDRESS);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0);
    return dataId;
}

//----------------------------------------------------------------------------------------------------------------------
/// @brief    BMP180: Register D0h, this value is fixed to 0x55
///
/// @param    Void No value needed
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------
void ConfigureUART(void)
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
    //Configura el periferico UART 0 con una velocidad de 115200 Baudios, 8 bits de informacion, 1 bit de parada 
    // y sin bit de paridad
    //
    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_EVEN));

}

//----------------------------------------------------------------------------------------------------------------------
/// @brief    Receive data
///
/// @param    Void No value needed
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------
void ReadCalData(void)
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

//----------------------------------------------------------------------------------------------------------------------
/// @brief    Read data
///
/// @param    Void No value needed
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------
void ReadUT(void)
{
    I2CSend(SLAVE_ADDR, 2, 0xF4, 0x2E);
    //I2CSend(SLAVE_ADDR, 1, 0x2E);
    SysCtlDelay(120000);                            //4.5 ms
    uiUT = I2CReceive16(SLAVE_ADDR, UT_ADDRESS);

}

//----------------------------------------------------------------------------------------------------------------------
/// @brief    Read data
///
/// @param    Void No value needed
///
/// @return   int32_t result
//----------------------------------------------------------------------------------------------------------------------
int32_t CalUT(void)
{
    int32_t result;
    int32_t x1 = ((uiUT - uiAC6) * uiAC5) >> 15;
    int32_t x2 = (uiMC << 11 ) / (x1 + uiMD);
    int32_t _b5 = x1 + x2;
    result = ((_b5 + 8) >> 4);
    return result;
}

//----------------------------------------------------------------------------------------------------------------------
/// @brief    Main function
///
/// @param    Void No value needed
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------
void main(void)
{
    uint8_t uiDigit0 = 0;
    uint8_t uiDigit1 = 1;
    uint8_t uiDigit2 = 2;
    uint8_t uiDigit3 = 3;
    uint8_t uiDigit4 = 4;
    uint8_t uiDigit5 = 5;
    uint8_t uiDigit6 = 6;
    uint8_t uiDigit7 = 7;

    uint8_t seconds1 = 0;
    uint8_t seconds2 = 0;
    uint8_t minutes1 = 0;
    uint8_t minutes2 = 0;
    uint8_t hours1 = 0;
    uint8_t hours2 = 0;
    uint8_t buff1 = 0;
    uint8_t buff2 = 0;

    int8_t Temperature1=0;
    int8_t Temperature2=0;
    int8_t Temperature3=0;

    int8_t Counter = 0;

    int32_t Temperature_Prom [10] = {0};
    int32_t Temperature_Prom_fl = 0;

    int i = 0;
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);                    //Habilitar puerto N
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);             //Configurar PN3 como salida CS = LOAD
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3,0x08);                 //LOAD ON

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1|GPIO_PIN_3);  // Configurar salidas PD1 = MOSI, PD3 = CLK

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                    //Habilitacion del puerto F
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4);             //Configurar PF0 como salida LED 4

    //BUTTONS
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);                    // Enable Port J
    GPIODirModeSet(GPIO_PORTJ_BASE, GPIO_PIN_1|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_1|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    DisplayInit();

    // Set the clocking to run directly from the external crystal/oscillator.
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_480), 120000000);

    // Initialize the GPIO for the LED.

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x00);

    //initialize UART module 0
    ConfigureUART();
    //initialize I2C module 2
    InitI2C2();

    // Print the welcome message to the terminal - Envio de datos  via UART
             //
         /*UARTCharPut(UART0_BASE, 'C');                                          // Envio de datos  via UART
         UARTCharPut(UART0_BASE, 'h');                                               // Envio de datos  via UART
         UARTCharPut(UART0_BASE, 'i');                                               // Envio de datos  via UART
         UARTCharPut(UART0_BASE, 'p');                                              // Envio de datos  via UART
         UARTCharPut(UART0_BASE, 'I');
         UARTCharPut(UART0_BASE, 'd');
         UARTCharPut(UART0_BASE, ' ');*/
         // Read the data from the BMP180 over I2C.  This command starts a
                               // temperature measurement.  Then polls until temperature is ready.
                               // Then automatically starts a pressure measurement and polls for that
                               // to complete. When both measurement are complete and in the local
                               // buffer then the application callback is called from the I2C
                               // interrupt context.  Polling is done on I2C interrupts allowing
                               // processor to continue doing other tasks as needed.
                               //
                               //    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x02);
                               //chipId = readId(SLAVE_ADDR, IdSlave_I2C_ADDR);
                               // Print the welcome message to the terminal.
                                      //
                               //UARTCharPut(UART0_BASE,chipId);
    ReadCalData();
    ReadUT();

    Temperature = CalUT();

    while(1)
    {
        if(globalcount < globalcount_max)
        {
            globalcount ++;
        }
        else
        {
            globalcount = 0;
            ui8Seconds ++;

            UARTCharPut(UART0_BASE, 'T');
            UARTCharPut(UART0_BASE, 'e');
            UARTCharPut(UART0_BASE, 'm');
            UARTCharPut(UART0_BASE, 'p');
            UARTCharPut(UART0_BASE, ':');
            UARTCharPut(UART0_BASE, ' ');
            UARTCharPut(UART0_BASE, Temperature1+48);
            UARTCharPut(UART0_BASE, Temperature2+48);
            UARTCharPut(UART0_BASE, '.');
            UARTCharPut(UART0_BASE, Temperature3+48);
            UARTCharPut(UART0_BASE, ' ');
            UARTCharPut(UART0_BASE, 'C');
            UARTCharPut(UART0_BASE, '\n');
            UARTCharPut(UART0_BASE, 0x0D);

            if(250 < Temperature)
            {
                int i = 0;
                for(i = 0; i < 16; i++)
                {
                    UARTCharPut(UART0_BASE, ERROR_MSG_1[i]);
                }
                
                UARTCharPut(UART0_BASE, '\n');
                UARTCharPut(UART0_BASE, 0x0D);
            }

            if(220 > Temperature)
            {
                int i = 0;
                for(i = 0; i < 16; i++)
                {
                    UARTCharPut(UART0_BASE, ERROR_MSG_2[i]);
                }
                UARTCharPut(UART0_BASE, '\n');
                UARTCharPut(UART0_BASE, 0x0D);
            }

            if(60 == ui8Seconds)
            {
                ui8Minutes ++;
                ui8Seconds = 0;
                if(60 == ui8Minutes)
                {
                    ui8Hours ++;
                    ui8Minutes = 0;
                    if(12 == ui8Hours)
                    {
                        ui8Hours = 0;
                    }
                }
            }
        }

        ReadUT();
        Temperature = CalUT();

        if(10 > Counter)
        {
            Counter ++;
            Temperature_Prom [Counter] = Temperature;
        }
        else
        {
            Counter = 0;
            Temperature_Prom [9] = Temperature;
            for(i = 0; i <= Counter; i++)
            {
                Temperature_Prom_fl += Temperature_Prom[i];
            }
            Temperature_Prom_fl = Temperature_Prom_fl / (Counter + 1) ;
        }

        Temperature1 = Temperature_Prom_fl / 100;
        Temperature2 = (Temperature_Prom_fl % 100) / 10;
        Temperature3 = Temperature_Prom_fl % 10;


        /*Temperatura[0] =  Temperature / 10;
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
        */

        bt1 = 0;
        bt2 = 0;

        if(0 == GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))
        {
            bt1 = 1;
            if((0 == counter_bt1_pressed_flag) && (0 == Display_State))
            {
                bt1_time_stamp = ui8Seconds;
                counter_bt1_pressed_flag = 1;
            }

            //counter_bt1_pressed++;
        }
        else if((0 == bt1) && (1 == counter_bt1_pressed_flag) && (0 == Display_State))
        {
            if(ui8Seconds < bt1_time_stamp)
            {
                counter_bt1_pressed = (ui8Seconds + 60) - bt1_time_stamp;
            }
            else
            {
                counter_bt1_pressed = ui8Seconds - bt1_time_stamp;
            }
            counter_bt1_pressed_flag = 0;
        }
        if(0 == GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1))
        {
            bt2 = 1;
        }
        if((0 == bt1) && (0 < counter_bt1_pressed) && (10 > counter_bt1_pressed))
        {
            Display_State = 1;
            counter_bt1_pressed = 0;
        }
        else if((0 == bt1) && (10 <= counter_bt1_pressed) && (20 > counter_bt1_pressed))
        {
            Display_State = 2;
            counter_bt1_pressed = 0;
        }
        else
        {
            counter_bt1_pressed = 0;
        }

        /*if(bt1 != 0){
            if(bt1_rebound == 0){
                if(Display_State == 0){
                    Display_State = 1;
                }else if (Clock_State == 0) {
                    Display_State = 0;
                    Clock_State = 0;
                }else{
                    if(counter_buffer < max_clock_counter[Clock_State-1]){
                        counter_buffer ++;
                    }else{
                        counter_buffer = 1;
                    }

                }
                bt1_rebound = 1;
            }
        }else{
            bt1_rebound = 0;
        }*/

        if(1 == Display_State)
        {	
			seconds1 = ui8Seconds / 10;
            seconds2 = ui8Seconds % 10;

            minutes1 = ui8Minutes / 10;
            minutes2 = ui8Minutes % 10;

            hours1 = ui8Hours / 10;
            hours2 = ui8Hours % 10;

            buff1 = ':';
            buff2 = ':';

            SpiSend(OP_DIGIT0, seconds2);
            SpiSend(OP_DIGIT1, seconds1);
            SpiSend(OP_DIGIT2, ':');
            SpiSend(OP_DIGIT3, minutes2);
            SpiSend(OP_DIGIT4, minutes1);
            SpiSend(OP_DIGIT5, ':');
            SpiSend(OP_DIGIT6, hours2);
            SpiSend(OP_DIGIT7, hours1);

            if(0 == flag_time_stamp_saved)
            {
                time_stamp = ui8Seconds;
                flag_time_stamp_saved = 1;
            }

            if(3 <= (ui8Seconds - time_stamp))
            {
                Display_State = 0;
            }
        }

        else if(2 == Display_State)
        {
            /*if(Clock_State == 0 && bt1_rebound == 0 && bt2_rebound == 0 ){


                int i = 0;
                for(i = 0;i<=2000000;i++){
                    if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0 || GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){
                        break;
                    }
                }
                if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0 || GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){
                    ui8Seconds++;
                    if(ui8Seconds == 60){
                        ui8Minutes++;
                        ui8Seconds = 0;
                        if(ui8Minutes == 60){
                            ui8Hours++;
                            ui8Minutes = 0;
                        }
                    }
                }
            }*/

            if(0 != bt2)
            {
                if(0 != bt2_rebound)
                {
                    switch(Clock_State)
                    {
                        case 0:
                            Clock_State = 1;
                        break;
                        case 1:
                            Clock_State = 2;
                            if(0 != counter_buffer)
                            {
                                ui8Seconds = counter_buffer - 1;
                            }
                        break;
                        case 2:
                            Clock_State = 3;
                            if(0 != counter_buffer)
                            {
                                ui8Minutes = counter_buffer - 1;
                            }
                        break;
                        case 3:
                            Clock_State = 0;
                            if(0 != counter_buffer)
                            {
                                ui8Hours = counter_buffer - 1;
                            }
                        break;
                    }

                    counter_buffer = 0;
                    bt2_rebound = 1;
                }
            }
            else
            {
                bt2_rebound = 0;
            }

            seconds1 = ui8Seconds / 10;
            seconds2 = ui8Seconds % 10;

            minutes1 = ui8Minutes / 10;
            minutes2 = ui8Minutes % 10;

            hours1 = ui8Hours / 10;
            hours2 = ui8Hours % 10;

            buff1 = ':';
            buff2 = ':';

            if(0 != counter_buffer)
            {

                buff1 = ((counter_buffer - 1) / 10);
                buff2 = ((counter_buffer - 1) % 10);

            }
            else
            {
                buff1 = ':';
                buff2 = ':';
            }

            if(0 != bt1)
            {
                if(0 == bt1_rebound)
                {
                    if(counter_buffer < max_clock_counter[Clock_State - 1])
                    {
                        counter_buffer ++;
                    }
                    else
                    {
                        counter_buffer = 1;
                    }
                    bt1_rebound = 1;
                }
            }
            else
            {
                bt1_rebound = 0;
            }

            switch(Clock_State)
            {
            case 0:
                SpiSend(OP_DIGIT0, seconds2);
                SpiSend(OP_DIGIT1, seconds1);
                SpiSend(OP_DIGIT2, ':');
                SpiSend(OP_DIGIT3, minutes2);
                SpiSend(OP_DIGIT4, minutes1);
                SpiSend(OP_DIGIT5, ':');
                SpiSend(OP_DIGIT6, hours2);
                SpiSend(OP_DIGIT7, hours1);
            break;
            case 1:
                SpiSend(OP_DIGIT0, buff2);
                SpiSend(OP_DIGIT1, buff1);
                SpiSend(OP_DIGIT2, ':');
                SpiSend(OP_DIGIT3, minutes2);
                SpiSend(OP_DIGIT4, minutes1);
                SpiSend(OP_DIGIT5, ':');
                SpiSend(OP_DIGIT6, hours2);
                SpiSend(OP_DIGIT7, hours1);
            break;
            case 2:
                SpiSend(OP_DIGIT0, seconds2);
                SpiSend(OP_DIGIT1, seconds1);
                SpiSend(OP_DIGIT2, ':');
                SpiSend(OP_DIGIT3, buff2);
                SpiSend(OP_DIGIT4, buff1);
                SpiSend(OP_DIGIT5, ':');
                SpiSend(OP_DIGIT6, hours2);
                SpiSend(OP_DIGIT7, hours1);
            break;
            case 3:
                SpiSend(OP_DIGIT0, seconds2);
                SpiSend(OP_DIGIT1, seconds1);
                SpiSend(OP_DIGIT2, ':');
                SpiSend(OP_DIGIT3, minutes2);
                SpiSend(OP_DIGIT4, minutes1);
                SpiSend(OP_DIGIT5, ':');
                SpiSend(OP_DIGIT6, buff2);
                SpiSend(OP_DIGIT7, buff1);
            break;
            }

            if((0 != bt1) && (0 == Clock_State))
            {

                //Prender una bandera cuando ya quiere salir pero sigue presionando boton
                //Hasta que la deje de presionar entrara al "else if"
                bt1_return_flag = 1;
            }
            else if((0 == bt1) && (0 == Clock_State) && (1 == bt1_return_flag))
            {
                //Dejo de presionar boton y ya va salir de la funcion
                Display_State = 0;
                bt1_return_flag= 0;
            }

            else 
            {
                SpiSend(OP_DIGIT0, Temperature3);
                SpiSend(OP_DIGIT1, '_');
                SpiSend(OP_DIGIT2, Temperature2);
                SpiSend(OP_DIGIT3, Temperature1);
                SpiSend(OP_DIGIT4, '_');
                SpiSend(OP_DIGIT5, '_');
                SpiSend(OP_DIGIT6, '_');
                SpiSend(OP_DIGIT7, '_');

                flag_time_stamp_saved = 0;
                time_stamp = 0;
                bt1_return_flag = 0;

                //UARTCharPut(UART0_BASE,Temperature_Prom_fl);
            }

            if(0 == bt1)
            {
                counter_bt1_pressed = 0;
            }
    }
}

//----------------------------------------------------------------------------------------------------------------------
/// @brief    SpiSend function
///
/// @param    uint8_t uiHiger, uint8_t uiLower
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------
void SpiSend(uint8_t uiHiger, uint8_t uiLower)
{
    uint8_t i;
    
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0x00); //LOAD OFF
    
    //latch the data onto the display LOAD = 0

    SysCtlDelay(ui8Delay);
    for(i = 8; i > 0; i--)
    {
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,0x00);         // SPI CLK LOW
        SysCtlDelay(ui8Delay);
        if(uiHiger & 0x80)
        {
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1,0x02);     //MSB first - Charge Boolean Data in MOSI
        }
            
        else
        {
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
        }
            
    uiHiger <<= 1;
    SysCtlDelay(ui8Delay);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x08);        // SPI CLK HIGH
    SysCtlDelay(ui8Delay);

    }

    SysCtlDelay(ui8Delay);
    
    for(i = 8; i > 0; i--)
    {
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,0x00);     // SPI CLK LOW
        SysCtlDelay(ui8Delay);
        if (uiLower & 0x80)
        {
           GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1,0x02);  //LSB first // Charge Boolean Data in MOSI
        }
        else
        {
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00);
        } 
     uiLower <<= 1;
     SysCtlDelay(ui8Delay);
     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3,0x08);     // SPI CLK HIGH
     SysCtlDelay(ui8Delay);

     }
  GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3,0x08); //LOAD ON
}

//----------------------------------------------------------------------------------------------------------------------
/// @brief    Intialization for Display
///
/// @param    Void - Needs no value
///
/// @return   void
//----------------------------------------------------------------------------------------------------------------------
void DisplayInit(void)
{
    SpiSend(OP_SHUTDOWN, 0x00);     // OP_SHUTDOWN    0x0C
    SpiSend(OP_DISPLAYTEST, 0x00);
    SpiSend(OP_INTENSITY, 0x04);
    SysCtlDelay(10000000);
    SpiSend(OP_SCANLIMIT, 0x07);
    SpiSend(OP_DECODEMODE, 0xFF);
    SpiSend(OP_DISPLAYTEST, 0x01);
    SysCtlDelay(10000000);
    SpiSend(OP_DISPLAYTEST, 0x00);
    SpiSend(OP_SHUTDOWN, 0x01);     // OP_SHUTDOWN    0x0C
}

/**====================================================================================================================================
 **      Version         
 **====================================================================================================================================
 **     	1            
 **			2
 **			3            
 **====================================================================================================================================*/

/**====================================================================================================================================
** End of file
**==================================================================================================================================== */
