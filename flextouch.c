#include "flextouch.h"

void touch_init()
{
        //SETBIT(AD1PCFGHbits.PCFG20);   //Since PIN8 is shared with ADC1CH20, set bit to place it in digital mode
        SETBIT(TRISEbits.TRISE8);

        //Initialize
        CLEARBIT(AD1CON1bits.ADON);
        CLEARBIT(AD1PCFGLbits.PCFG15);
        CLEARBIT(AD1PCFGLbits.PCFG9);
        SETBIT(AD1CON1bits.AD12B);
        AD1CON1bits.FORM=0;
        AD1CON1bits.SSRC = 0x7;
        AD1CON2 = 0;
        CLEARBIT(AD1CON3bits.ADRC);
        AD1CON3bits.SAMC = 0x1F;
        AD1CON3bits.ADCS = 0x2;
        SETBIT(AD1CON1bits.ADON);
                //set three pins as output
        CLEARBIT(TRISEbits.TRISE1);
        CLEARBIT(TRISEbits.TRISE2);
        CLEARBIT(TRISEbits.TRISE3);


            SETBIT(LATEbits.LATE1);
            SETBIT(LATEbits.LATE2);
            SETBIT(LATEbits.LATE3);
}
void touch_select_dim(uint8_t dim)
{

            //setup read X
            if(dim==8)
            {
            CLEARBIT(LATEbits.LATE1);
            SETBIT(LATEbits.LATE2);
            SETBIT(LATEbits.LATE3);
            AD1CHS0bits.CH0SA = 0x00F;
            }
            else if(dim==9)
            {
             //setup read Y
            SETBIT(LATEbits.LATE1);
            CLEARBIT(LATEbits.LATE2);
            CLEARBIT(LATEbits.LATE3);
            AD1CHS0bits.CH0SA = 0x009;
            }
            else
            {}

            TMR1 = 0x00;


}
uint16_t touch_adc()
{
 SETBIT(AD1CON1bits.SAMP);
 while(!AD1CON1bits.DONE);
 CLEARBIT(AD1CON1bits.DONE);
 return ADC1BUF0;
}
