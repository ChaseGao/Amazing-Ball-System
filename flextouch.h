/*
flextouch.h and flextouch.c are used to setup the touchscreen that reads the ball's position of the ABS
*/

#include <p33Fxxxx.h>

#include <lcd.h>

void touch_init();

void touch_select_dim(uint8_t dim);
