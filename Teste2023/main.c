#include "DSP28x_Project.h"

void main(void)
{
    //disable protection
    EALLOW;


    //make port B GPIOS outputs
    GpioCtrlRegs.GPADIR.all = 0x0000FF00;

    while(1)
    {
        //Toggle GPIOs 8-15
        GpioDataRegs.GPADAT.all = 0x0000FF00;
        DELAY_US(100);
        GpioDataRegs.GPADAT.all = 0x00000000;
        DELAY_US(100);
    }
}
