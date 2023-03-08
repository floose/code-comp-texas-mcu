//###########################################################################
//
// FILE:   Example_2806xScia_Echoback.c
//
// TITLE:  SCI Echo Back Example
//
//!  \addtogroup f2806x_example_list
//!  <h1>SCI Echo Back(sci_echoback)</h1>
//!
//!  This test receives and echo-backs data through the SCI-A port.
//!
//!  The PC application 'hypterterminal' can be used to view the data
//!  from the SCI and to send information to the SCI.  Characters received
//!  by the SCI port are sent back to the host.
//!
//!  \b Running \b the \b Application
//!  -# Configure hyperterminal:
//!  Use the included hyperterminal configuration file SCI_96.ht.
//!  To load this configuration in hyperterminal
//!    -# Open hyperterminal
//!    -# Go to file->open
//!    -# Browse to the location of the project and
//!       select the SCI_96.ht file.
//!  -# Check the COM port.
//!  The configuration file is currently setup for COM1.
//!  If this is not correct, disconnect (Call->Disconnect)
//!  Open the File-Properties dialog and select the correct COM port.
//!  -# Connect hyperterminal Call->Call
//!  and then start the 2806x SCI echoback program execution.
//!  -# The program will print out a greeting and then ask you to
//!  enter a character which it will echo back to hyperterminal.
//!
//!  \note If you are unable to open the .ht file, you can create
//!  a new one with the following settings
//!  -  Find correct COM port
//!  -  Bits per second = 9600
//!  -  Date Bits = 8
//!  -  Parity = None
//!  -  Stop Bits = 1
//!  -  Hardware Control = None
//!
//!  \b Watch \b Variables \n
//!  - \b LoopCount, for the number of characters sent
//!  - ErrorCount
//!
//! \b External \b Connections \n
//!  Connect the SCI-A port to a PC via a transceiver and cable.
//!  - GPIO28 is SCI_A-RXD (Connect to Pin3, PC-TX, of serial DB9 cable)
//!  - GPIO29 is SCI_A-TXD (Connect to Pin2, PC-RX, of serial DB9 cable)
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:
// Copyright (C) 2009-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

/*############################## CODE TO DETECT AN ANALOG MANCHESTER SIGNAL
 * AUTHOR:FELIPE LOOSE, BASED ON TEXAS INSTRUMENT'S C2000 EXAMPLES
 * DESCRIPTION:
 * The basic idea is to use a timer interrupt to trigger the start of an ADC conversion.
 * To do this, you would configure the timer to generate an interrupt at a fixed frequency,
 * and then use the interrupt service routine (ISR) to trigger the start of an ADC conversion.

Here's a high-level overview of the steps involved:

    (1)Configure the timer module to generate an interrupt at a fixed frequency.
    For example, if you want to sample the analog signal every 1 microsecond,
    you could set the timer period to 1 microsecond.

    (2)In the timer interrupt service routine,
    start an ADC conversion by setting the SOC (Start of Conversion) bit in the ADC control register.

    (3)When the ADC completes the conversion, it will set the EOC (End of Conversion) bit in the ADC status register.
    You can check this bit in your main program loop to read the ADC result and start another conversion.
 *
 */

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
//#include <math.h>
//#include <stdbool.h>
//#include "manchester.h"
//
//
//My Defines
//
//##### Define BAUDRATE
//this guard defines the console application
//to communicate between the computer and the board with a baud-rate of 9600 bps
//Use teraterm to enable communication
//this baud rates indicates the rate between writing and receiving info from the PC
//via console
#define CONSOLE_APP
//#define CONSOLE_BAUDRATE_115200

//##### Define ECHO MODE
//enables echo mode (example of initial application)
//#define CONSOLE_ECHO_MODE

//#### DEFINE ADC CODE
//enables the codes for the ADC
#define ADC_CODE
//#### DEFINE TOGGLE GPO
//defines GPIOs to use them as flags of entrance at each part of the code
#define GPIO_TOGGLE

//#### DEFINE MANCHESTER PROCESSING
//defines to enable the function of processing manchester code
#define MANCHESTER_PROCESS

//#### DEFINE ADC CODE AND TIMER
#ifdef ADC_CODE
    #define ADC_RESOLUTION 4096 // 12-bit ADC resolution

    #define ADC_MID_VALUE 2048

    #define ADC_THRESHOLD_VALUE 2048

    #define ADC_FLOOR_VALUE 800

    //the next defines configure the sampling rate of the timer and adc
    /*
     * CPU_TIMER_MHZ dictates the clock of the system.Usual is 90 MHz
     * TIMER_PERIOD_US indicates the time between interruptions
     * SYMBOL_SIZE dictates the number of the buffer samples to be stored
     *
     */
    #define CPU_TIMER_MHZ 90 //clock of timer
    #define TIMER_PERIOD_US .5 //sampling rate period
    #define SYMBOL_SIZE 4 //defines the maximum samples per symbol
    #define ADC_BUFFER_SIZE SYMBOL_SIZE //defines the size of the adc buffer
    #define BYTE_WINDOW SYMBOL_SYZE*8
#endif


// Function Prototypes
//
#ifdef CONSOLE_APP
    void sciInit();
    void scia_fifo_init();
    void scia_xmit(int a);
    void scia_msg(char *msg);
    void write_console_init_msg();
#endif

#ifdef CONSOLE_ECHO_MODE
    void echo_mode_loop();
#endif

#ifdef ADC_CODE
    void adcInit();
    __interrupt void cpu_timer0_isr();
#endif


//
// Globals
//
#ifdef ADC_CODE
   // Uint16 adc_buffer[ADC_BUFFER_SIZE] = {0};
  //  volatile bool buffer_full = false;
    //volatile Uint8 adc_interrupt_flag = 0;
    volatile Uint16 adc_sample = 0;
    char* global_msg = "\0";
    char *aux_msg = "\0";
    Uint16 data_adc[ADC_BUFFER_SIZE] = {0};
#endif



//
// Main
//
void main(void)
{

    //clock_t start, end;
    //double cpu_time = 0;
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to its default state.
    //
    //InitGpio(); Skipped for this example

    //
    // For this example, only init the pins for the SCI-A port.
    // This function is found in the F2806x_Sci.c file.
    //
    InitSciaGpio();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();
    //

    //
    // Step 5. User specific code
    //

    //initialize LED to indicate Timer interrupt


    //initialize GPIO to toggle them at the timer interrupt
    #ifdef GPIO_TOGGLE
    EALLOW;
    //GPIO0 toggles at every ISR
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1; //configures GPIO0 as output
    GpioDataRegs.GPADAT.bit.GPIO0 = 0; //sets to zero
    //GPIO0 toggles at every manchester decode, when the data buffer is full
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1; //configures GPIO0 as output
    GpioDataRegs.GPADAT.bit.GPIO1 = 0; //sets to zero
    EDIS;
    #endif

    EnableInterrupts(); //enables interrupts

#ifdef CONSOLE_APP
    scia_fifo_init();      // Initialize the SCI FIFO
    sciInit();  // Initalize SCI
    write_console_init_msg();
#endif

/*
#ifdef MANCHESTER_PROCESS
    manchester_set_parameters(ADC_BUFFER_SIZE,SYMBOL_SIZE,ADC_THRESHOLD_VALUE);
#endif
*/
    //init cput timers and adc configs
#ifdef ADC_CODE
    adcInit();
    // Start Timer
    CpuTimer0Regs.TCR.bit.TSS = 0;
#endif

//unsigned char loopcount = 0; //formating the message
//unsigned char message[56] = {0}; //char display message

    //infinite loop

    for(;;)
    {

        #ifdef CONSOLE_ECHO_MODE //enables ECHO MODE
            void echo_mode_loop(void);
        #endif

        //#ifdef GPIO_TOGGLE
            //Toggles GPIO0 every interruption (used to check time between isr's)
          //  GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1; //toggle GPIO upon isr
        //#endif

        #ifdef MANCHESTER_PROCESS
            if(*global_msg != 0x00)
            {

               scia_msg(global_msg);
               aux_msg = "\r\n";
               scia_msg(aux_msg);
            }

        #endif
    }


}


//
// scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x0103,
// default, 1 STOP bit, no parity
//
#ifdef CONSOLE_APP
void sciInit()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function
    //

    //
    // 1 stop bit,  No loopback, No parity,8 char bits, async mode,
    // idle-line protocol
    //
    SciaRegs.SCICCR.all =0x0007;
    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
    SciaRegs.SCICTL1.all =0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    #ifdef CONSOLE_APP
        //
        // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
        //
        SciaRegs.SCIHBAUD    =0x0000;

        SciaRegs.SCILBAUD    =0x0017;

        SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    #endif


}

//
// scia_xmit - Transmit a character from the SCI
//
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {

    }
    SciaRegs.SCITXBUF=a;
}

//
// scia_msg -
//
void scia_msg(char * msg)
{
    Uint8 i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

//
// scia_fifo_init - Initalize the SCI FIFO
//
void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;
}

void write_console_init_msg()
{
    char *msg;
    msg = "\rHello World!\0";
    scia_msg(msg);
    msg = "\r\nConsole Application PLC-VLC\0";
    scia_msg(msg);


        msg = "\r\nConsole Baud Rate is 115200 bps\0";
        scia_msg(msg);


    #ifdef ADC_CODE
        msg = "\r\n ADC is Enabled \n\r\0";
        scia_msg(msg);
    #endif

    #ifdef CONSOLE_ECHO_MODE
    msg = "\r\nWARNING! Echo mode defined!\0";
    scia_msg(msg);
    msg = "\r\nYou will enter a character, and the DSP will echo it back! \n\0";
    scia_msg(msg);
    #endif

}

#endif

#ifdef CONSOLE_ECHO_MODE
void echo_mode_loop()
{
    Uint16 ReceivedChar;
    char *msg;

    msg = "\r\nEnter a character: \0";
    scia_msg(msg);

    //
    // Wait for inc character
    //
    while(SciaRegs.SCIFFRX.bit.RXFFST !=1)
    {
        //
        // wait for XRDY =1 for empty state
        //
    }

    //
    // Get character
    //
    ReceivedChar = SciaRegs.SCIRXBUF.all;

    //
    // Echo character back
    //
    msg = "  You sent: \0";
    scia_msg(msg);
    scia_xmit(ReceivedChar);

}
#endif

#ifdef ADC_CODE

    //configures timer and inits cpu interrupt
    void adcInit()
    {
        //reset adc do known levels
        InitAdc();
        AdcOffsetSelfCal();

        //Configuring the ADC
        EALLOW;
        AdcRegs.ADCCTL1.bit.ADCENABLE = 0; //disable ADC to make changes
        AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0; //adc triggers via software only
        //AdcRegs.ADCSOC0CTL.bit.CHSEL = 1; //adc triggers via cpu timer 0
        AdcRegs.ADCSOC0CTL.bit.CHSEL = 0;  //select adc channel ADCINA0
        AdcRegs.INTSEL1N2.bit.INT1SEL = 1; //Connect ADCINT1 to EOC1
        EDIS;

        //configure the timer to make conversion
        EALLOW;  // This is needed to write to EALLOW protected registers
        PieVectTable.TINT0 = &cpu_timer0_isr;
        EDIS;
        InitCpuTimers();
        ConfigCpuTimer(&CpuTimer0, CPU_TIMER_MHZ, TIMER_PERIOD_US);

        //
        // Use write-only instruction to set TSS bit = 0
        //
        CpuTimer0Regs.TCR.all = 0x4000;
        // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
        IER |= M_INT1;
        //
        // Enable TINT0 in the PIE: Group 1 interrupt 7
        //
        PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
        //
        // Enable global Interrupts and higher priority real-time debug events:
        //
        EINT;   // Enable Global interrupt INTM
        ERTM;   // Enable Global realtime interrupt DBGM
    }

        //interruption code of timer
    __interrupt void cpu_timer0_isr(void)
    {
        static unsigned short int last_sample = 0;
        static char decoded_byte = 0;
        static unsigned char bits_received = 0;
        static unsigned char dummy_counter = 0;
        static unsigned char loopcount = 0;

        //trick to avoid algorithm triggering after init. Needs change.
        if(dummy_counter < 2)
        {
            dummy_counter++;
        }

        /*
        //toggle pin to verify timing
        #ifdef GPIO_TOGGLE
            //Toggles GPIO0 every interruption (used to check time between isr's)
            GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
           // GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1;
        #endif
        */

        // Start ADC conversion
        AdcRegs.ADCSOCFRC1.bit.SOC0 = 1; //force SOC0 conversion
        //while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0){} //Wait for ADCINT1
        //adc_buffer[buffer_head] = AdcResult.ADCRESULT0; //get value from adc
        adc_sample = AdcResult.ADCRESULT0; //gets the result

        //this is a trick to check the value of the buffer size while processing the algorithm
        if(loopcount == ADC_BUFFER_SIZE)
        {
            loopcount = 0;
        }
        //end of trick

        if(adc_sample > ADC_THRESHOLD_VALUE && last_sample < ADC_THRESHOLD_VALUE && dummy_counter == 2)
        {
            decoded_byte = (decoded_byte << 1); // Append a 0 to the byte
            bits_received++;
            data_adc[loopcount] = adc_sample;
            loopcount++;
            //toggle pin to verify timing
            #ifdef GPIO_TOGGLE
                //Toggles GPIO0 every interruption (used to check time between isr's)
                GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
               // GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1;
            #endif
        }
        else if (adc_sample < ADC_THRESHOLD_VALUE && last_sample > ADC_THRESHOLD_VALUE & dummy_counter == 2)
        {
            decoded_byte = decoded_byte << 1; // left shift
            decoded_byte = decoded_byte | 0x01;
            bits_received++;
            data_adc[loopcount] = adc_sample;
            loopcount++;
            //toggle pin to verify timing
            #ifdef GPIO_TOGGLE
                //Toggles GPIO0 every interruption (used to check time between isr's)
                GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
               // GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1;
            #endif
        }


        if(bits_received == 8)
        {
            //*global_msg = decoded_byte;
            bits_received = 0;
            decoded_byte = 0;
        }

        last_sample = adc_sample;
        /*
        #ifdef GPIO_TOGGLE
             //Toggles GPIO0 every interruption (used to check time between isr's)
             GpioDataRegs.GPATOGGLE.bit.GPIO1 = 1;
        #endif
        */
        //adc_interrupt_flag = 1; //sets flag to convert it to manchester @ main

        AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Clear ADCINT1
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //acknowledge interrupt

    }
#endif



//
// End of File
//

