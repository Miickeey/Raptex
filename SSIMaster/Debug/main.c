
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "math.h"
#include "driverlib/ssi.h"
#include "driverlib/qei.h"


//
// Number of bytes to send and receive.
//
//*****************************************************************************
#define NUM_SSI_DATA            50
//*****************************************************************************
//
// Global variables used in interrupt handler and the main loop.
//
//*****************************************************************************
volatile unsigned long g_ulSSI2RXFF = 0, g_ulSSI2TXFF = 0;

uint32_t pui32DataTx[NUM_SSI_DATA] = {0};
uint32_t pui32DataRx[NUM_SSI_DATA] = {0};
uint32_t ulDataTx0[NUM_SSI_DATA] = {0};
uint32_t ulDataRx0[NUM_SSI_DATA] = {0};
uint32_t ui32DataRx1[NUM_SSI_DATA] = {0};

volatile uint32_t g_ui32ExpectedFIFOData = 11;
volatile uint32_t g_ui32Index = 0;


uint32_t TxFlag = 0;
uint32_t count = 0;
uint32_t len = 0;
uint32_t RxFlag = 0;

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
InitConsole(void)
{
       SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);


       GPIOPinConfigure(GPIO_PB0_U1RX);
       GPIOPinConfigure(GPIO_PB1_U1TX);


       GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);


       UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));


}
//*****************************************************************************
//
// Configure SSI2 in Master TI mode.  This example will send out 10 bytes of
// data, then wait for 3 bytes of data to come in.  This will all be done using
// the interrupt method.
//
//*****************************************************************************
void
InitSSI0(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                                                       // SSI0 pins are in PORTA
       GPIOPinConfigure(GPIO_PA2_SSI0CLK);                                                                // SPI Clock
       GPIOPinConfigure(GPIO_PA3_SSI0FSS);                                                                // Chip Select
       GPIOPinConfigure(GPIO_PA5_SSI0TX);                                                                 // MOSI
       GPIOPinConfigure(GPIO_PA4_SSI0RX);                                                                 // MISO
       GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_4);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
       while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0))                                                   // check if SSI module is Ready
       {
       }

           SSIConfigSetExpClk(SSI0_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,1000000,8);       // SPI clock polarity and phase,SPI Master mode, bit rate 1 MHz, 8 bit data
           SSIEnable(SSI0_BASE);                                                                               // SPI Base





    SSIIntEnable(SSI0_BASE,SSI_RXFF);

    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0]))
    {
    }
}

//*****************************************************************************
//
// Interrupt handler for SSI2 peripheral in slave mode.  It reads the interrupt
// status and if the interrupt is fired by a RX time out interrupt it reads the
// SSI2 RX FIFO and increments a counter to tell the main loop that RX timeout
// interrupt was fired.
//
//*****************************************************************************
void SSI2IntHandler(void)
{
    uint32_t ulStatus;

        //
        // Read interrupt status.
        //
        ulStatus = SSIIntStatus(SSI2_BASE, 1);

        //
        // Check the reason for the interrupt.
        //
        if((ulStatus & SSI_RXFF) == SSI_RXFF)
        {
            //
            // Clear the interrupt
            //
            SSIIntClear(SSI2_BASE, SSI_RXFF);

            //
            // Receive First byte from SSI2 Bus
            //
            SSIDataGetNonBlocking(SSI2_BASE, &pui32DataRx[g_ui32Index++]);


            //
            // Check first byte is valid or not
            //
            if(pui32DataRx[0] == g_ui32ExpectedFIFOData)
            {

                //
                // Receive the data using the "blocking" Get function. This function
                // will wait until there is data in the receive FIFO before returning.
                while(SSIDataGetNonBlocking(SSI2_BASE, &pui32DataRx[g_ui32Index++]));

                //
                // Wait until SSI2 is done receiving all the data in the receive FIFO.
                //
                while(SSIBusy(SSI2_BASE));

                //
                // Receive flag indicates the reception is completed
                //
                g_ulSSI2RXFF++;
                g_ui32Index--;

            }


            else
            {
                while(SSIDataGetNonBlocking(SSI2_BASE, &ulDataRx0[0]))
                {
                }

                g_ui32Index = 0;
            }
        }

        // Clear interrupts.
        //
        SSIIntClear(SSI2_BASE, ulStatus);

}

//*****************************************************************************
//
// This function is used to send multiple bytes of data over SSI2 bus
//
//*****************************************************************************
void
Send (uint32_t ulDataTx0[])
{
    uint32_t ui32Index;

    //
    // Transmit 10 bytes of  data over SSI2 Bus
    //
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        //
        // Load transmitted data into transmitted FIFO
        //
        pui32DataTx[ui32Index] = ulDataTx0[ui32Index];

        //
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        //
        SSIDataPutNonBlocking(SSI2_BASE, pui32DataTx[ui32Index]);

        UARTprintf("pui32DataTx[%d] = %d\n",ui32Index,pui32DataTx[ui32Index]);

    }

    //
    // Wait until SSI2 is done transferring all the data in the transmit FIFO.
    //
    while(SSIBusy(SSI2_BASE))
    {
    }

    UARTprintf("SPI Master Transmission is Done....\n\n");


}


//*****************************************************************************
//
// This function is used to receive the multiple bytes of data and prints on to
// the console screen
//
//*****************************************************************************
void
Ask (void)
{
    uint32_t count = 0;

    if(g_ulSSI2RXFF)
    {
        if (pui32DataRx[49] == 60)
        {
            for(count = 0; count < NUM_SSI_DATA; count++)
            {
               // ui32DataRx1[count]  = pui32DataRx[count];

                //
                // Display the data that SSI2 received.
                //
                UARTprintf("pui32DataRx[%d] = %d \n", count,pui32DataRx[count]);

            }

            UARTprintf("SPI Master Receiving is Done....\n\n");

            g_ui32Index = 0;
            g_ui32ExpectedFIFOData = 11;
        }
        g_ulSSI2RXFF = 0;
    }



}


//*****************************************************************************
//
// Main function entry starts here
//
//*****************************************************************************
int
main(void)
{


    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
            SYSCTL_XTAL_16MHZ);
    InitConsole();

    UARTprintf("SSI ->\n");
    UARTprintf("  Mode: TI\n");
    UARTprintf("  Data: 8-bit\n\n");

    for(count = 0; count < NUM_SSI_DATA; count++)
    {
        ulDataTx0[count] = count + 1;
    }

    InitSSI0();
    IntEnable(INT_SSI2);
    IntMasterEnable();

    while(1)
    {
        Send(ulDataTx0);
        SysCtlDelay(200000);
        Ask();
    }
}
