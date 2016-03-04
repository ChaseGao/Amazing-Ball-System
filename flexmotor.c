#include "flexmotor.h"

void motor_init(uint8_t chan)
{
    CLEARBIT(T2CONbits.TON);
    CLEARBIT(T2CONbits.TCS);
    CLEARBIT(T2CONbits.TGATE);
    TMR2 = 0x00;
    T2CONbits.TCKPS = 0b10;
    CLEARBIT(IFS0bits.T2IF);
    CLEARBIT(IEC0bits.T2IE);
    PR2 = 4000;

    if(chan==8)
    {
        CLEARBIT(TRISDbits.TRISD8);
        OC8R = 3700;
        OC8RS = 3700;
        OC8CON = 0x0006;
    SETBIT(T2CONbits.TON);
    }
    else if(chan==7)
    {
        CLEARBIT(TRISDbits.TRISD7);
        OC7R = 3820;
        OC7RS = 3820;
        OC7CON = 0x0006;
    SETBIT(T2CONbits.TON);
    }

}

void motor_set_duty(uint8_t chan, uint16_t duty_us)
{
    if(chan==8)
    {
        OC8RS = duty_us;
        //OC8CON = 0x0006;
    }

    else if(chan==7)
    {
        OC7RS = duty_us;
       // OC7CON = 0x0006;
    }

}