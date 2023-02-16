//#include "F28x_Project.h" //chatgpt include
#include "DSP28x_Project.h"     // DSP28x Headerfile
//#include "device.h" //chatgpt include
//#include "sci.h"    //chatgpt include
#include <stdio.h>
#include "sci_io.h"

// Function prototypes
void initSCI(void);
void sendData(char *data);
void receiveData(char *data, Uint16 length);

void main(void)
{
    InitSysCtrl(); //Initialize System Control:
    InitSciaGpio(); //Only init the pins for the SCI-A port.
    DINT; //clear all interrupts and initialize PIE Vector
    InitPieCtrl(); //Initialize PIE control registers to their default state.
    //disable CPU INterrupts
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable(); //Initialize the PIE vector table with pointers to the shell Interrupt

    //Enter below the user specific code

    initSCI(); //initializes SCI

    char sendString[] = "Hello from the C2000 microcontroller\r\n";
    char receiveString[100];

    while (1)
    {
        // Send data to the computer
        sendData(sendString);

        // Receive data from the computer
        receiveData(receiveString, 100);

        // Echo the received data back to the computer
        sendData(receiveString);
    }
}

// Initialize the SCI module
void initSCI(void)
{
    // Initialize the SCI module
    SciaRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
                                     // No parity,8 char bits,
                                     // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
                                    // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    // Initialize the baud rate
    // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
    SciaRegs.SCIHBAUD = 0x0001;
    SciaRegs.SCILBAUD = 0x0017;
    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

    /*
    // 9600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK)
    SciaRegs.SCIHBAUD    =0x0001;
    SciaRegs.SCILBAUD    =0x0024;
    */

    SciaRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}

// Send data to the computer
void sendData(char *data)
{
    Uint32 i = 0;
    while (data[i] != '\0')
    {
        while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {};
        SciaRegs.SCITXBUF = data[i];
        i++;
    }
}

// Receive data from the computer
void receiveData(char *data, Uint16 length)
{
    Uint16 i = 0;
    while (i < length)
    {
        while (SciaRegs.SCIFFRX.bit.RXFFST == 0) {};
        data[i] = SciaRegs.SCIRXBUF.all;
        i++;
    }
    data[i] = '\0';
}
