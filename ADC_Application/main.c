#include "DSP28x_Project.h"

void main(void)
{
    //disable protection
    EALLOW;

    // Initialize GPIOs for the LEDs and turn them off
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO34 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO39 = 1;

    //make port B GPIOS outputs
    //GpioCtrlRegs.GPADIR.all = 0x0000FF00;

    //config LEDs to twiddle
    GpioDataRegs.GPBDAT.bit.GPIO34 = 0;
    GpioDataRegs.GPBDAT.bit.GPIO39 = 1;


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
