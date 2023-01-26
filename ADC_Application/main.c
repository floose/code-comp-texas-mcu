#include <stdbool.h>
#include <stdio.h>
#include <file.h>

#include "DSP28x_Project.h"     // DSP28x Headerfile
#include "sci_io.h"

//functions
void scia_init();

void main(void)
{

    //variables to redirect stdio to scia unit
    volatile int status = 0;
    //uint16_t i;
    volatile FILE *fid;

    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    InitSysCtrl();
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to its default state.
    //
    // InitGpio(); Skipped for this example
    //
    // For this example, only init the pins for the SCI-A port.
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
    // Disable CPU interrupts and clear all CPU interrupt flags:
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
    //InitPieVectTable();

    //EALLOW;
    //PieVectTable.TINT0 = &cpu_timer0_isr;
    //EDIS;

    //
    // Initialize SCIA
    //
    scia_init();
    //InitCpuTimers();        // For this example, only initialize the Cpu Timers
    //ConfigCpuTimer(&CpuTimer0, 90, 1000);


    //
    // Redirect STDOUT to SCI
    //
    status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write,
                        SCI_lseek, SCI_unlink, SCI_rename);
    fid = fopen("scia","w");
    freopen("scia:", "w", stdout);
    setvbuf(stdout, NULL, _IONBF, 0);

    //
    // Use write-only instruction to set TSS bit = 1
    //
    //CpuTimer0Regs.TCR.all = 0x4010;

    //IER |= M_INT1;
    //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    //EINT;   // Enable Global interrupt INTM
    //ERTM;   // Enable Global realtime interrupt DBGM

    while(1)
    {
        //Toggle GPIOs 8-15
        //GpioDataRegs.GPADAT.all = 0x0000FF00;
        //toggle LED Pins of EVAL board
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
        DELAY_US(50000);
       // GpioDataRegs.GPADAT.all = 0x00000000;
        //toggle LED Pins of EVAL board
        GpioDataRegs.GPBTOGGLE.bit.GPIO39 = 1;
        DELAY_US(50000);
    }
}

//
// scia_init - SCIA  8-bit word, baud rate 0x000F, default, 1 STOP bit,
// no parity
//
void scia_init()
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

    SciaRegs.SCICTL2.bit.TXINTENA =1;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;

    //
    // 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
    //
    SciaRegs.SCIHBAUD    =0x0000;

    SciaRegs.SCILBAUD    =0x0017;

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

    return;
}
