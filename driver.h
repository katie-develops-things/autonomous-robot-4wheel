/*****************************************************************************
* | File        :   MotorDriver.h
* | Author      :   Waveshare team
* | Function    :   Drive TB6612FNG
* | Info        :
*                TB6612FNG is a driver IC for DC motor with output transistor in
*                LD MOS structure with low ON-resistor. Two input signals, IN1
*                and IN2, can choose one of four modes such as CW, CCW, short
*                brake, and stop mode.
*----------------
* |	This version:   V1.0
* | Date        :   2018-09-04
* | Info        :   Basic version
*
******************************************************************************/
#include "DEV_Config.h"
#include "PCA9685.h"


// GPIO config for one motor
// *** Used same connections as WaveShare Team ***
#define PWMA        PCA_CHANNEL_0
#define AIN1        PCA_CHANNEL_1
#define AIN2        PCA_CHANNEL_2
#define PWMB        PCA_CHANNEL_5
#define BIN1        PCA_CHANNEL_3
#define BIN2        PCA_CHANNEL_4

// define variables
#define LEFTMOTOR      0
#define RIGHTMOTOR	    1

#define FORWARD		1
#define BACKWARD	0


void Motor_Init(void);
void motorRun(UBYTE motor, int dir, UWORD speed);
void slowDown(UBYTE motor, UWORD currspeed, UWORD endspeed, int dir);
void speedUp(UBYTE motor, UWORD currspeed, UWORD endspeed, int dir);
void motorStop(UBYTE motor);


