//File                : RapTEx Software
//Author              : Michael Hvarstad, Robin Danielsen, Gard Hope Bakke
//Company             : RapTex
//Created             : 01/07/2020
//Description         : System Software!!
//**************************************************************************
//                              INCLUDE                                   **
//**************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.c"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include "driverlib/gpio.c"
#include "driverlib/gpio.h"
//**************************************************************************
//                              FUNCTIONS                                 **
//**************************************************************************
void  UartConfig();
void  Spi0Config();
void  AdcConifg();
void  AsicConig();
void  uDmaConfig();
void  SODS();
void  TXD_interrupt();
void  AdcTempVoltage();
void  SysClkToAsic();
void  ledOut();
void  txd_o_SPI2();
void  heatingAsic();
void  UART_Interrupt();
void  Spi1SDConfig();
void  SpiSDTransfer(int (*bufferPtr));

//**************************************************************************
//                              Define                                    **
//**************************************************************************
#define MAX_BUFFER_SIZE         1000                                                           // Buffer size "1000 bytes"
#define GPIO_PA0_U0RX 0x00000001                                                               // Define UART
#define GPIO_PA1_U0TX 0x00000401                                                               // Define UART
#define pinRead5 GPIO_PIN_5
#define pinBaseC GPIO_PORTC_BASE
#define MIN_TEMP 22                                                                            // ASIC MIN temp 22 degrees celsius
#define MAX_TEMP 25                                                                            // ASIC MAX temp 25 degrees celsius
#define Button_PERIPH SYSCTL_PERIPH_GPIOC
#define inputSODS GPIO_PORTC_BASE
#define GPIO_PIN_4C GPIO_PIN_4
#define NUM_SSI_DATA_ASIC_CONFIG    92                                                         //Number of bytes send to ASIC config register "92 bytes"
//**************************************************************************
//                             Variables                                  **
//**************************************************************************
uint32_t  ui32Period;                                                                           // Used for, For loops periods
uint32_t  pui32ADC0Value[4];                                                                    // Variable for ADC multiplexer
uint8_t   tempAsic;                                                                             // Variable for Temp ASIC
uint8_t   tempSensor;                                                                           // Variable for Temp sensor
uint8_t   voltagMCU;                                                                            // Variable for Voltage MCU
uint8_t   voltagSensor;                                                                         // Variable for Voltage Sensor
uint32_t  ui32Index;                                                                            // Used for, For loops periods
uint32_t  ui32SysClock;                                                                         // do we need this?
uint32_t  currentSample = 0;                                                                    // Buffer counter
uint32_t  SD_buffer1[MAX_BUFFER_SIZE];                                                          // SD buffer 1 size 1000
uint32_t  SD_buffer2[MAX_BUFFER_SIZE];                                                          // SD buffer 2 size 1000
uint32_t  pui32dataFromAsic[1000];                                                              // SSI FIFO RX sends data to dataFromAsic before backbuffer
uint32_t  spiSdOutBuffer[MAX_BUFFER_SIZE];                                                      // UDMA sends data from activebuffer to spiSdOutBuffer before SSI FIFO TX to SD-card
uint32_t  *backbuffer;                                                                          // pointer to backbuffer
uint32_t  *activebuffer;                                                                        // pointer to activebuffer
uint32_t  pui32DataTx[NUM_SSI_DATA];                                                            // ASIC config array
uint64_t  numberOfWritenBytes = 0;                                                              // to keep count of number of bytes writen to SD-card. important if system restart
int       bufferPtr = 0;                                                                        // buffer pointer
int j = 0;                                                                                      // Used for if statement for UART
int i = 0;
int txdFlags = 0;                                                                                // SPI2 interrupt flags
//**************************************************************************
//                               MAIN                                     **
//**************************************************************************
int main(void)
    {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);     // Start system clock, and oscillator for SSI/UART
    SSIIntRegister(SSI2_BASE, TXD_interrupt);                                                   // SPI2 interrupt register
    UARTIntRegister(UART0_BASE, UART_Interrupt);                                                // UART0 interrupt register
    SysClkToAsic();                                                                             // SysClock ASIC and SPI2
//    char dataFrame[] = "1111101000110011011110110";                                           // For ground station test
    UartConfig();                                                                               // UART config
    uDmaConfig();                                                                               // DMA config
    Spi0Config();                                                                               // SPI for ASIC config
    txd_o_SPI2();                                                                               // SPI2 for TXD out, sensor data form ASIC
    Spi1SDConfig();                                                                             // SPI1 for SD-card
    heatingAsic();                                                                              // GPIO config heating elempent ASIC
    AdcConifg();                                                                                // ADC Config
    SysCtlDelay(3);                                                                             // System delay for system config
    SODS();                                                                                     // SODS
    AsicConig();                                                                                // ASIC config
    ledOut();                                                                                   // Status LED
    activebuffer = SD_buffer1;                                                                  // activeBuffer is SD_buffer 1
    backbuffer   = SD_buffer2;                                                                  // backBuffer is SD_buffer 2
    while(1)
    {
    if(txdFlags)                                                                                // if TXD flags is 1
    {
        backbuffer[currentSample++] = (uint32_t) &pui32dataFromAsic;                            // Backbuffer = dataFromAsic, Backbuffer is the buffer getting data
//        backbuffer[currentSample++] = (uint32_t)dataFrame;
        numberOfWritenBytes++;                                                                  // Byte counter
        if(currentSample >= 1000)                                                               // if currentSample is 1000 then change buffer
        {
            currentSample = 0;                                                                  // currentSample = 0
            if(bufferPtr == 0)                                                                  // if buffer pointer is 0
            {
                bufferPtr = 1;                                                                  // buffer pointer 1
                activebuffer = SD_buffer2;
                backbuffer = SD_buffer1;
                uDMAChannelEnable(UDMA_CHANNEL_SW);                                             // Enable UDMA
                uDMAChannelRequest(UDMA_CHANNEL_SW);                                            // Start data transfer
            }
            else                                                                                // if buffer pointer is 1
            {
                bufferPtr = 0;                                                                  // buffer pointer 0
                activebuffer = SD_buffer1;
                backbuffer = SD_buffer2;
                uDMAChannelEnable(UDMA_CHANNEL_SW);                                             // Enable UDMA
                uDMAChannelRequest(UDMA_CHANNEL_SW);                                            // Start data transfer
              }
            txdFlags = 0;                                                                       // TXD flags 0
          }

        j++;
        if(j == 100)                                                                            // Send UART REXUS service module every 100 loop
        {
            UARTCharPut(UART0_BASE, 'S');                                                       // Sends char S
            UARTprintf("'%d'", activebuffer[MAX_BUFFER_SIZE]);                                  // Sends buffer data. MAX_BUFFER_SIZE not correct
            AdcTempVoltage();                                                                   // Gets ADC values
            UARTCharPut(UART0_BASE, 'B');                                                       // Sends char B
            UARTprintf("'%d'", voltagMCU/2);                                                    // Sends Voltage MCU / 2 = 16. not 1.6
            UARTCharPut(UART0_BASE, 'C');                                                       // Sends char C
            UARTprintf("'%d'", voltagSensor/2);                                                 // Sends Voltage reverse bias / 2 = 16. not 1.6
            UARTCharPut(UART0_BASE, 'A');                                                       // Sends char A
            UARTprintf("'%d'", tempAsic);                                                       // Sends Temp ASIC.        PT100
            UARTCharPut(UART0_BASE, 'V');                                                       // Sends char V
            UARTprintf("'%d'", tempSensor/2/2);                                                 // Sends Temp sensor Card. PT1000
          j = 0;
        }
        }
    }

}
//*****************************************************************************************************************************************
//                                                     ASIC, UART, SD_CARD
//*****************************************************************************************************************************************
//**************************************************************************
//                     ASIC REGISTER CONFIG                               **
//**************************************************************************
void AsicConig()
{
//-------------------------------------------------------
//                      Channel 1                      --
//-------------------------------------------------------
    pui32DataTx[0] = 0b00000001;             // address1
    pui32DataTx[1] = 0b10011001;
    pui32DataTx[2] = 0b10001100;
    pui32DataTx[3] = 0b00000000;
//-------------------------------------------------------
//                      Channel 2                      --
//-------------------------------------------------------
    pui32DataTx[4] = 0b00000011;              // address2
    pui32DataTx[5] = 0b10011001;
    pui32DataTx[6] = 0b10001100;
    pui32DataTx[7] = 0b00000000;
//-------------------------------------------------------
//                      Channel 3                      --
//-------------------------------------------------------
    pui32DataTx[8] = 0b00000101;              // address3
    pui32DataTx[9] = 0b10011001;
    pui32DataTx[10] = 0b10001100;
    pui32DataTx[11] = 0b00000000;
//-------------------------------------------------------
//                      Channel 4                      --
//-------------------------------------------------------
    pui32DataTx[12] = 0b00000111;              // address4
    pui32DataTx[13] = 0b10011001;
    pui32DataTx[14] = 0b10001100;
    pui32DataTx[15] = 0b00000000;
//-------------------------------------------------------
//                      Channel 5                      --
//-------------------------------------------------------
    pui32DataTx[16] = 0b00001001;              // address5
    pui32DataTx[17] = 0b10011001;
    pui32DataTx[18] = 0b10001100;
    pui32DataTx[19] = 0b00000000;
//-------------------------------------------------------
//                      Channel 6                      --
//-------------------------------------------------------
    pui32DataTx[20] = 0b00001011;              // address6
    pui32DataTx[21] = 0b10011001;
    pui32DataTx[22] = 0b10001100;
    pui32DataTx[23] = 0b00000000;
//-------------------------------------------------------
//                      Channel 7                      --
//-------------------------------------------------------
    pui32DataTx[24] = 0b00001101;              // address7
    pui32DataTx[25] = 0b10011001;
    pui32DataTx[26] = 0b10001100;
    pui32DataTx[27] = 0b00000000;
//-------------------------------------------------------
//                      Channel 8                      --
//-------------------------------------------------------
    pui32DataTx[28] = 0b00001111;              // address8
    pui32DataTx[29] = 0b10011001;
    pui32DataTx[30] = 0b10001100;
    pui32DataTx[31] = 0b00000000;
//-------------------------------------------------------
//                      channel_config                 --
//-------------------------------------------------------
    pui32DataTx[32] = 0b00100011;              // address11
    pui32DataTx[33] = 0b10011001;
    pui32DataTx[34] = 0b10001100;
    pui32DataTx[35] = 0b00000000;
//-------------------------------------------------------
//                      channel_control                --
//-------------------------------------------------------
    pui32DataTx[35] = 0b00100101;              // address12
    pui32DataTx[36] = 0b10011001;
    pui32DataTx[37] = 0b10001100;
    pui32DataTx[38] = 0b00000000;
//-------------------------------------------------------
//                      adc_config                     --
//-------------------------------------------------------
    pui32DataTx[39] = 0b00100111;              // address13
    pui32DataTx[40] = 0b10011001;
    pui32DataTx[41] = 0b10001100;
    pui32DataTx[42] = 0b00000000;
//-------------------------------------------------------
//                      cal_dac                        --
//-------------------------------------------------------
    pui32DataTx[43] = 0b00101001;              // address14
    pui32DataTx[44] = 0b10011001;
    pui32DataTx[45] = 0b10001100;
    pui32DataTx[46] = 0b00000000;
//-------------------------------------------------------
//                      pd_modules                     --
//-------------------------------------------------------
    pui32DataTx[47] = 0b00101011;              // address15
    pui32DataTx[48] = 0b10011001;
    pui32DataTx[49] = 0b10001100;
    pui32DataTx[50] = 0b00000000;
//-------------------------------------------------------
//                      cal_ctrl                     --
//-------------------------------------------------------
    pui32DataTx[51] = 0b00101101;              // address16
    pui32DataTx[52] = 0b10011001;
    pui32DataTx[53] = 0b10001100;
    pui32DataTx[54] = 0b00000000;
//-------------------------------------------------------
//                      readout_fixes_list             --
//-------------------------------------------------------
    pui32DataTx[55] = 0b00101111;              // address17
    pui32DataTx[56] = 0b10011001;
    pui32DataTx[57] = 0b10001100;
    pui32DataTx[58] = 0b00000000;
//-------------------------------------------------------
//                      readout_mode                   --
//-------------------------------------------------------
    pui32DataTx[59] = 0b00110001;              // address18
    pui32DataTx[60] = 0b10011001;
    pui32DataTx[61] = 0b10001100;
    pui32DataTx[62] = 0b00000000;
//-------------------------------------------------------
//                      amux_ctrl                      --
//-------------------------------------------------------
    pui32DataTx[63] = 0b00110011;              // address19
    pui32DataTx[64] = 0b10011001;
    pui32DataTx[65] = 0b10001100;
    pui32DataTx[66] = 0b00000000;
//-------------------------------------------------------
//                      adc_clk_div_factor             --
//-------------------------------------------------------
    pui32DataTx[67] = 0b00110101;              // address1A
    pui32DataTx[68] = 0b10011001;
    pui32DataTx[69] = 0b10001100;
    pui32DataTx[70] = 0b00000000;
//-------------------------------------------------------
//                      sysclock_ctrl                  --
//-------------------------------------------------------
    pui32DataTx[71] = 0b00110111;              // address1B
    pui32DataTx[72] = 0b10011001;
    pui32DataTx[73] = 0b10001100;
    pui32DataTx[74] = 0b00000000;
//-------------------------------------------------------
//                      dcal_cmnd                      --
//-------------------------------------------------------
    pui32DataTx[75] = 0b00111001;              // address1C
    pui32DataTx[76] = 0b10011001;
    pui32DataTx[77] = 0b10001100;
    pui32DataTx[78] = 0b00000000;
//-------------------------------------------------------
//                      radout_cmnd                    --
//-------------------------------------------------------
    pui32DataTx[79] = 0b00111011;              // address1D
    pui32DataTx[80] = 0b10011001;
    pui32DataTx[81] = 0b10001100;
    pui32DataTx[82] = 0b00000000;
//-------------------------------------------------------
//                      trigger_latches                --
//-------------------------------------------------------
   pui32DataTx[83] = 0b00111101;              // address1E
   pui32DataTx[84] = 0b10011001;
   pui32DataTx[85] = 0b10001100;
   pui32DataTx[86] = 0b00000000;
//-------------------------------------------------------
//                      adc_out                        --
//-------------------------------------------------------
   pui32DataTx[87] = 0b00111111;              // address1F
   pui32DataTx[88] = 0b10011001;
   pui32DataTx[89] = 0b00000000;
   pui32DataTx[90] = 0b00000000;
//-------------------------------------------------------
//                      parity_err_reg                 --
//-------------------------------------------------------
   pui32DataTx[91] = 0b01000000;              // address20                                 // READ only, ERROR REGISTER...

    for (ui32Index = 0; ui32Index < NUM_SSI_DATA_ASIC_CONFIG; ui32Index++)
    {
        UARTprintf("%d ", pui32DataTx[ui32Index]);
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

}

//**************************************************************************
//                   TXD_O DATA FROM ASIC                                 **
//**************************************************************************
void TXD_interrupt(void)
{
    SSIIntClear(SSI2_BASE, SSI_RXFF);                                                       // clear the interrupt flags
    txdFlags = 1;
    for(ui32Index = 0; ui32Index < 8; ui32Index++)                                          // For loop until received all bytes
    {
        SSIDataGet(SSI2_BASE, &pui32dataFromAsic[ui32Index]);
    }
}
//**************************************************************************
//                   UART INTERRUPT                                       **
//**************************************************************************
void UART_Interrupt(void)                                                                    // if Ground station sends a command, UART RX interrupt will appear
{
    uint32_t ui32Status;
    char cThisChar;
    ui32Status = UARTIntStatus(UART0_BASE,true);
    UARTIntClear(UART0_BASE, ui32Status);
    while(UARTCharsAvail(UART0_BASE))
    {
    cThisChar = UARTCharGet(UART0_BASE);
    if(cThisChar == 'C')                                                                      // if the char form ground station is C then run config ASIC "ASIC RESTART!!!!!!!!!!
    {
    AsicConig();                                                                              // runs ASIC config
    }
    else if (cThisChar == 'P')                                                                // if error SD-Card
    {

    }
    }
}
//**************************************************************************
//                   SD CARD TRANSFER                                     **
//**************************************************************************
void SpiSDTransfer(int (*bufferPtr)) {
    UARTprintf("Sent:\n  ");
    for (ui32Index = 0; ui32Index < MAX_BUFFER_SIZE; ui32Index++) {
        if (bufferPtr != 0) {
            UARTprintf("Data sent to SD_CARD = '%8d' ", spiSdOutBuffer[ui32Index]);
            SSIDataPut(SSI1_BASE, spiSdOutBuffer[ui32Index]);
        }
        else {
            UARTprintf("Data sent to SD_CARD = '%8d' ", spiSdOutBuffer[ui32Index]);
            SSIDataPut(SSI1_BASE, spiSdOutBuffer[ui32Index]);
        }
    }
    while (SSIBusy(SSI1_BASE))
    {

    }
//    UARTprintf("\nReceived:\n  ");
}
//*****************************************************************************************************************************************
//                                                          CONFIG
//*****************************************************************************************************************************************
//**************************************************************************
//                     ADC Temp Volage                                    **
//**************************************************************************
void AdcTempVoltage()
{
    ADCIntClear(ADC0_BASE, 0);
    ADCProcessorTrigger(ADC0_BASE, 0);
    while(!ADCIntStatus(ADC0_BASE, 0, false))
    {
    }
    ADCSequenceDataGet(ADC0_BASE, 0, pui32ADC0Value);                                                             // ADC sample variable pui32ADC0Value 32 bit
    voltagMCU = (3300*pui32ADC0Value[0] /4096) / 100;                                                             // Voltage MCU
    voltagSensor = (3300*pui32ADC0Value[1] /4096) / 100;                                                          // Voltage Sensor
    tempAsic = (3300*pui32ADC0Value[2] /4096) / 100;                                                              // Temp ASIC
    tempSensor = (3300*pui32ADC0Value[3] /4096) / 100;                                                            // Temp Sensor card
}
//**************************************************************************
//                               ADC                                      **
//**************************************************************************
void AdcConifg()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                                                                    // Enable clock to ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                                                                   // Enable clock to GPIO E BASE
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);                                  // GPIO pins E0-E3
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);                                                         // ADC sample 0
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);                                                         // ADC sample 1
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2);                                                         // ADC sample 2
    ADCSequenceStepConfigure(ADC0_BASE,0, 3, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);                               // ADC sample 3
    ADCSequenceEnable(ADC0_BASE, 0);
}
//**************************************************************************
//                               DMA1                                     **
//**************************************************************************
void uDmaConfig()
{
    uint8_t pui8DMAControlTable[1024];                                                                                                  // UDMA address variable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA))
    {
    }
    uDMAEnable();                                                                                                                       // Enable UDMA
    uDMAControlBaseSet(&pui8DMAControlTable[0]);                                                                                        // UDMA address
    uDMAChannelControlSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT, UDMA_SIZE_32 | UDMA_SRC_INC_32 | UDMA_DST_INC_32 | UDMA_ARB_32);           // Software request, 32 bit
    uDMAChannelTransferSet(UDMA_CHANNEL_SW | UDMA_PRI_SELECT,UDMA_MODE_AUTO, activebuffer, spiSdOutBuffer, sizeof(spiSdOutBuffer));     // Sends data from activebuffer to spiSdOutBuffer
}
//**************************************************************************
//                               UART                                     **
//**************************************************************************
void UartConfig(void)
{
   SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
   GPIOPinConfigure(GPIO_PA0_U0RX);
   GPIOPinConfigure(GPIO_PA1_U0TX);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
   UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
   GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
   UARTStdioConfig(0, 9600, 16000000);                                                                 // 9600 bits per second, 16Mhz clock
   IntEnable(INT_UART0);                                                                               // Enable UART interrupt
   UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
   IntMasterEnable();                                                                                  // Enable processor interrupts
}
//**************************************************************************
//                              SPI0                                      **
//**************************************************************************
void Spi0Config()
{
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
   GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
   GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, 0x01);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                                                        // SSI0 pins are in PORTA
   GPIOPinConfigure(GPIO_PA2_SSI0CLK);                                                                 // SPI Clock
   GPIOPinConfigure(GPIO_PA3_SSI0FSS);                                                                 // Chip Select
   GPIOPinConfigure(GPIO_PA5_SSI0TX);                                                                  // MOSI
   GPIOPinConfigure(GPIO_PA4_SSI0RX);                                                                  // MISO
   GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0))
    {
    }
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(),SSI_FRF_MOTO_MODE_1,SSI_MODE_MASTER,4000000,8);       // SPI BASE,SPI clock polarity and phase,SPI Master mode, bit rate 4 MHz, 8 bit data
    SSIEnable(SSI0_BASE);
}
//**************************************************************************
//                          TXD_O SPI2                                    **
//**************************************************************************
void txd_o_SPI2()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, 0X00);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);                                                                 // PWM is Sysclk
    GPIOPinConfigure(GPIO_PB5_SSI2FSS);                                                                 // chip select, pin must be set to zero (ground) not ground form mcu 0 - 0.2!!!
    GPIOPinConfigure(GPIO_PB6_SSI2RX);                                                                  // both ASIC and MCU will operate in SLAVE mode
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2))
      {
      }
    SSIConfigSetExpClk(SSI2_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_SLAVE_OD, 1,8);          // Sets clock to 1, Sysclk controls the clock, SLAVE_OD: disable MOSI
    SSIEnable(SSI2_BASE);                                                                               // Enable SSI 2
    SSIIntEnable(SSI2_BASE, SSI_RXFF);                                                                  // Enable interrupt SSI RX FIFO
}
//**************************************************************************
//                          SD_Card SPI1                                  **
//**************************************************************************

void Spi1SDConfig()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                                                        // SSI1 pins are in PORTF
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);                                                                 // SPI Clock
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);                                                                 // Chip Select
    GPIOPinConfigure(GPIO_PF1_SSI1TX);                                                                  // MOSI
    GPIOPinConfigure(GPIO_PF0_SSI1RX);                                                                  // MISO
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_2);
    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 4000000, 8);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1))
    {
    }
    SSIEnable(SSI1_BASE);
    while (SSIDataGetNonBlocking(SSI0_BASE, &spiSdOutBuffer[ui32Index]))
    {
    }
    UARTprintf("Inserting trashbyte:\n  ");
    SSIDataPut(SSI1_BASE, 0xff);
}
//**************************************************************************
//                  SODE Signal form Rocket                               **
//**************************************************************************
void  SODS()
{
uint32_t value = 0;
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
SysCtlPeripheralEnable(GPIO_PORTC_BASE);
GPIOPinTypeGPIOInput(inputSODS, GPIO_PIN_4C);
GPIOPadConfigSet(inputSODS , GPIO_PIN_4C,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);                     // WPD pull-down. WPU Pull-up
value = GPIOPinRead(inputSODS, GPIO_PIN_4C);
if((value) != 0)                                                                                       // waiting for SODS, active 0
{
    for(;;)
        {
        AdcTempVoltage();                                                                              // ADC temp
        if(tempAsic < MIN_TEMP)                                                                        // if ASIC temp is lower than MIN temp
        {
         GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);                                        // heating element on
        }
        else if(MAX_TEMP < tempAsic)                                                                   // if ASIC temp is higher than MAX temp
        {
         GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);                                                 // heating element off
        }
        else{}                                                                                         // do nothing
    }
}
}
//**************************************************************************
//                    SysClkToAsic AND SPI2                               **
//**************************************************************************
void  SysClkToAsic()
{
     SysCtlPWMClockSet(SYSCTL_PWMDIV_2);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); 							                            // PWM PERIPHERAL ENABLE
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 							                            // GPIO FOR PWM1
     GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0); 							                            // GPIO PD0 FOR PWM1
     GPIOPinConfigure(GPIO_PD0_M1PWM0); 								                                // PD0 AS M1PWM0
     PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);                                          // SET PWM GENERATOR WITH MODEOF OPERATION AS COUNTING
     PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0,4); 								                            // SET PERIOD OF PWM GENERATOR
     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 2);                                                         // SET WIDTH OF PWM GENERATOR            current settings 4MHZ
     PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true); 					                                // ENABLE BIT0 OUTPUT
     PWMGenEnable(PWM1_BASE, PWM_GEN_0);   							                                    // ENABLE PWM_GEN_0 GENERATOR
}
//**************************************************************************
//                            STATUS LED                                  **
//**************************************************************************
void ledOut()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4);                                             // LED OUT PF4
}
//**************************************************************************
//                         HEATING ELEMENT ASIC                           **
//**************************************************************************
void heatingAsic()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);                                               // Heating element ASIC PD1
}
