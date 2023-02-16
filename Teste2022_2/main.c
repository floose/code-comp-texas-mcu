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

//
//My Defines
//
//##### Define BAUDRATE
//uncoment the desired baudrate to define the registers
#define CONSOLE_BAUDRATE_9600
//#define CONSOLE_BAUDRATE_115200

//##### Define ECHO MODE
//enables echo mode (example of initial application)
#define CONSOLE_ECHO_MODE

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

//
// Function Prototypes
//
void sciInit(void);
void scia_fifo_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);
void write_console_init_msg(void);
void echo_mode_loop(void);

//
// Globals
//
Uint16 LoopCount;
Uint16 ErrorCount;

//
// Main
//
void main(void)
{
    Uint16 ReceivedChar;
    char *msg;

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
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    //
    //InitPeripherals(); // Not required for this example

    //
    // Step 5. User specific code
    //
    LoopCount = 0;
    ErrorCount = 0;

    scia_fifo_init();      // Initialize the SCI FIFO
    sciInit();  // Initalize SCI for echoback





    for(;;)
    {

        #ifdef CONSOLE_ECHO_MODE

        #endif
        LoopCount++;
    }
}

//
// scia_echoback_init - Test 1,SCIA  DLB, 8-bit word, baud rate 0x0103,
// default, 1 STOP bit, no parity
//
void
sciInit()
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

    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;

    //
    // 9600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK)
    //
    #ifdef CONSOLE_BAUDRATE_9600
    SciaRegs.SCIHBAUD    =0x0001;
    SciaRegs.SCILBAUD    =0x0024;
    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    #endif


}

//
// scia_xmit - Transmit a character from the SCI
//
void
scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {

    }
    SciaRegs.SCITXBUF=a;
}

//
// scia_msg -
//
void
scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

//
// scia_fifo_init - Initalize the SCI FIFO
//
void
scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;
}

void write_console_init_msg(void)
{
    char *msg;
    msg = "\rHello World!\0";
    scia_msg(msg);
    msg = "\r\nConsole Application PLC-VLC\0";
    scia_msg(msg);

    #ifdef CONSOLE_BAUDRATE_9600
    msg = "\r\nConsole Baud Rate is 9600 bps\0";
    scia_msg(msg);
    #endif

    #ifdef CONSOLE_ECHO_MODE
    msg = "\r\nWARNING! Echo mode defined!\0";
    scia_msg(msg);
    msg = "\r\nYou will enter a character, and the DSP will echo it back! \n\0";
    scia_msg(msg);
    #endif
    free(msg);
}

#ifdef CONSOLE_ECHO_MODE
void echo_mode_loop(void)
{
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
    free(msg);
}
#endif

//
// End of File
//
