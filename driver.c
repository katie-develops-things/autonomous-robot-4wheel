/*****************************************************************************
* | File        :   MotorDriver.c
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
#include "driver.h"
#include "Debug.h"


// i2c init the motor at the found address
void Motor_Init(void)
{
    PCA9685_Init(0x40);
    PCA9685_SetPWMFreq(100);
}






// run the specified motor (only one motor for this assignment)
// either forward (1) or backward (0)
// with a given speed ( 0 - 100 )
void motorRun(UBYTE motor, int dir, UWORD speed)
{
    if(speed > 100)
        speed = 100;

    if(motor == LEFTMOTOR) {
        DEBUG("Motor A Speed = %d\r\n", speed);
        PCA9685_SetPwmDutyCycle(PWMA, speed);
        if(dir == FORWARD) {
            DEBUG("forward...\r\n");
            PCA9685_SetLevel(AIN1, 0);
            PCA9685_SetLevel(AIN2, 1);
        } else {
            DEBUG("backward...\r\n");
            PCA9685_SetLevel(AIN1, 1);
            PCA9685_SetLevel(AIN2, 0);
        }
    } else {
        DEBUG("Motor B Speed = %d\r\n", speed);
        PCA9685_SetPwmDutyCycle(PWMB, speed);
        if(dir == FORWARD) {
            DEBUG("forward...\r\n");
            PCA9685_SetLevel(BIN1, 0);
            PCA9685_SetLevel(BIN2, 1);
        } else {
            DEBUG("backward...\r\n");
            PCA9685_SetLevel(BIN1, 1);
            PCA9685_SetLevel(BIN2, 0);
        }
    }
}
	



// stop the motor 
void motorStop(UBYTE motor)
{
    if(motor == LEFTMOTOR) {
        PCA9685_SetPwmDutyCycle(PWMA, 0);
    } else {
        PCA9685_SetPwmDutyCycle(PWMB, 0);
    }
    
}


//~ void goStraight(){
	//~ motorRun(LEFT_MOTOR, FORWARD, 30);
	//~ motorRun(RIGHT_MOTOR, FORWARD, 30);
	//~ }
	
//~ void goRight(){
	//~ motorRun(LEFT_MOTOR, FORWARD, 30);
	//~ motorStop(RIGHT_MOTOR);
	//~ }

//~ void goLeft(){
	//~ motorRun(RIGHT_MOTOR, FORWARD, 30);
	//~ motorStop(LEFT_MOTOR);
	//~ }
