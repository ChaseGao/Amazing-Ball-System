/*
flexmotor.h and flexmotor.c are used to setup the PWM motor of the ABS
*/
#include <p33Fxxxx.h>

#include <lcd.h>

void motor_init(uint8_t chan);

void motor_set_duty(uint8_t chan, uint16_t duty_us);