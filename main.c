#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <libpic30.h>

#include <stdlib.h>
#include "led.h"
#include "lcd.h"
#include "flexmotor.h"
#include "flextouch.h"
/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  

volatile int flag;

uint16_t y_servo;
volatile int32_t tempx,tempy,pre_error_x,integral_x,pre_error_y,integral_y;
float kp,kd,ki;
float u;
float val;
float dt = 0.05;
uint16_t duty_x,duty_y;
uint16_t x_pos, y_pos;
uint16_t x_set, y_set;
uint16_t x_tent, y_tent;
uint16_t x_max;
uint16_t x_min;
uint16_t y_max;
uint16_t y_min;
uint32_t i;
uint16_t get_sample()
{
 SETBIT(AD2CON1bits.SAMP);
 while(!AD2CON1bits.DONE);
 CLEARBIT(AD2CON1bits.DONE);
 return ADC2BUF0;
}



void main(){
	//Init LCD
	__C30_UART=1;	
	lcd_initialize();
	lcd_clear();
	lcd_locate(0,0);

        SETBIT(AD1PCFGHbits.PCFG20);   //Since PIN8 is shared with ADC1CH20, set bit to place it in digital mode
        SETBIT(TRISEbits.TRISE8);

        motor_init(7);
        __delay_ms(500);
        motor_init(8);
        __delay_ms(500);
         touch_init();

        //Control Signal timer
        CLEARBIT(T1CONbits.TON);   // Disable Timer
        CLEARBIT(T1CONbits.TCS);   //Select internal instruction cycle clock (12.8 MHz)
        CLEARBIT(T1CONbits.TGATE); //Disable Gated Timer mode
        TMR1 = 0x00;               //Clear Timer Register
        T1CONbits.TCKPS = 0b10;    //Set Prescaler (1:256)
        PR1 = 10000;                //Set Period
        IPC0bits.T1IP = 0x01;      //Set IPL
        CLEARBIT(IFS0bits.T1IF);   //Clear IF
        SETBIT(T1CONbits.TON);     //Enable Timer
        SETBIT(IEC0bits.T1IE);
        //Setup Joystick
        uint32_t x;
        uint8_t y;
        uint16_t vx1,vx2;
        uint16_t vy1,vy2;
        y = 0;
        x_max=0;
        x_min=1000;
        y_max=0;
        y_min=1000;
        //Initialize all relevannt bits
        CLEARBIT(AD2CON1bits.ADON);
        CLEARBIT(AD2PCFGLbits.PCFG4);
        CLEARBIT(AD2PCFGLbits.PCFG5);
        CLEARBIT(AD2CON1bits.AD12B);
        AD2CON1bits.FORM=0;
        AD2CON1bits.SSRC = 0x7;
        AD2CON2 = 0;
        CLEARBIT(AD2CON3bits.ADRC);
        AD2CON3bits.SAMC = 0x1F;
        AD2CON3bits.ADCS = 0x2;
        SETBIT(AD2CON1bits.ADON);

        //initiliaze all PID controller variables

        pre_error_x=0;
        integral_x = 0;
        duty_x = 0;
        pre_error_y=0;
        integral_y = 0;
        duty_y = 0;
        i=0;
        x_set = 1630;
        y_set = 1595;


        while(1)
        {

    	AD2CHS0bits.CH0SA = 0x004;         //select X axis of joystick for reading user inputs
        while(1)
        {
           for(x=0;x<10000;x++)            //give the ABS enough time to complete axis-reading selection
                {}
            vx1 = get_sample();            //read user inputs in X direction

        x_tent = (int)(((float)2560/(float)821)*((float)vx1-(float)201)+(float)280);  //use user inputs to get correspondoing coordinates for the ball
            if(i%2==0)                     //display relevant data
                {
                lcd_locate(0,0);
                lcd_printf("Cur_X:%d,Cur_Y:%d ",x_pos,y_pos);
         	lcd_locate(0,1);
                lcd_printf("Set_X:%d,Set_Y:%d ",x_set,y_set);
         	lcd_locate(0,2);
                lcd_printf("Joy_X:%d,Joy_Y:%d ",x_tent,y_tent);
                }
            if(PORTEbits.RE8==0)           //joystick button debouncing
            {
                for(x=0;x<20000;x++)
                {}
                if(PORTEbits.RE8==0)
                {
                    break;
                }
            }
        }
            for(x=0;x<100000;x++){}
        while(PORTEbits.RE8==0);


        AD2CHS0bits.CH0SA = 0x005;           //select Y axis of joystick for reading user inputs
        while(1){
            for(x=0;x<10000;x++)             //give the ABS enough time to complete axis-reading selection
			{}           
            vy1 = get_sample();              //read user inputs in X direction

        if(vy1>470)                          //use user inputs to get correspondoing coordinates for the ball 
        {
            y_tent = (int)(((float)905/(float)553)*((float)vy1-(float)470)+(float)1595);
        }
        else if(vy1<450)
        {
            y_tent = (int)(((float)930/(float)259)*((float)vy1-(float)191)+(float)425);
        }
        else
        {

           y_tent = 1595;
        }
            if(i%2==0)                       //display relevant data
                {
                lcd_locate(0,0);
                lcd_printf("Cur_X:%d,Cur_Y:%d ",x_pos,y_pos);
         	lcd_locate(0,1);
                lcd_printf("Set_X:%d,Set_Y:%d ",x_set,y_set);
         	lcd_locate(0,2);
                lcd_printf("Joy_X:%d,Joy_Y:%d ",x_tent,y_tent);
                }
            if(PORTEbits.RE8==0)          //joystick button debouncing
            {
                for(x=0;x<20000;x++)
                {}
                if(PORTEbits.RE8==0)
                {
                    break;
                }
            }
        }
            for(x=0;x<100000;x++){}
        while(PORTEbits.RE8==0);

        x_set = x_tent;
        y_set = y_tent;


        }

 

//debounce
}

void __attribute__ (( __interrupt__ )) _T1Interrupt(void)
{

    touch_select_dim(8);        //read the x-axis of the touchscreen to get ball's position
        __delay_ms(10);

    int x[5] = {0,0,0,0,0};     // used to store touchscreeen output 
    x[0] = touch_adc();
    x[1] = touch_adc();
    x[2] = touch_adc();
    x[3] = touch_adc();
    x[4] = touch_adc();
    tempx = x[2];               // select the third one in array to ensure reading correctness
    x_pos = x[2];
   int16_t error_x;             //PID controller calculations
    error_x = tempx - (int16_t)x_set;
    duty_x = 3700+(int16_t)((float)error_x*0.04+(float)(error_x-pre_error_x)*0.8 + (float)integral_x * 0.00001);
    motor_set_duty(8,duty_x);
    pre_error_x = error_x;
    integral_x += error_x;
 
    touch_select_dim(9);        //read the y-axis of the touchscreen to get ball's position
        __delay_ms(10);

    int y[5] = {0,0,0,0,0};     // used to store touchscreeen output 
    y[0] = touch_adc();
    y[1] = touch_adc();
    y[2] = touch_adc();
    y[3] = touch_adc();
    y[4] = touch_adc();
    tempy = y[2];               // select the third one in array to ensure reading correctness
    y_pos = y[2];
   int16_t error_y;             //PID controller calculations
    error_y = tempy - (int16_t)y_set;
    duty_y = 3740+(int16_t)((float)error_y*0.025+(float)(error_y-pre_error_y)*0.5 + (float)integral_y * 0.000001);
    motor_set_duty(7,duty_y);
    pre_error_y = error_y;
    integral_y += error_y;

    i++;

   CLEARBIT(IFS0bits.T1IF);
}