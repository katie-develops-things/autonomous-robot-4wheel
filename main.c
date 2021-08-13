/*****************************************************************************
* Authors: Team Mushroom
* Filename: main.c
* Description: Integrates components and implements logic for navigating a map.
******************************************************************************/

#include <stdio.h>      
#include <stdlib.h>     
#include <signal.h>
#include "DEV_Config.h"
#include "driver.h"
#include <pthread.h>
#include <wiringPi.h>
#include <unistd.h>
#include <fcntl.h>          
#include <string.h>
#include <sys/stat.h>
#include "lidar.h"
#include "lidar_data.h"

#define channel 0

#define RIGHTLINESENSOR	 17
#define MIDDLELINESENSOR 27
#define LEFTLINESENSOR 22

int onLine (int sensor){
	//printf("%d\n", digitalRead(sensor));
	return digitalRead(sensor);
	}

void  Handler(int signo)
{
    //System Exit
    printf("\r\nHandler:Motor Stop\r\n");
    motorStop(LEFTMOTOR);
    motorStop(RIGHTMOTOR);
    DEV_ModuleExit();

    exit(0);
}

/*
* Rotates the vehicle 90 degress left
*/
void Rotate90L() 
{
		motorRun(LEFTMOTOR, BACKWARD, 100);
		motorRun(RIGHTMOTOR, FORWARD, 100);
		
		delay(900);
		
		motorStop(LEFTMOTOR);
		motorStop(RIGHTMOTOR);
		
}



int main(void)
{
	
	printf("Start of main\n");
	
	struct lidar_data * lidar_data = malloc(sizeof(lidar_data));
	volatile float dist_cpy, theta_cpy, qual_cpy;
    // System and Motor Initialization
    if (DEV_ModuleInit())
    {
        exit(0);
	}
    //2.Motor Initialization
    Motor_Init();

	printf("done with init motor\n");
    // Exception handling:ctrl + c
    // *** From WaveShare Team ***
    // will run 'Handler' function when ctrl + c is entered in the terminal
    // 'Handler' will stop the motor and call DEV_ModuleExit to properly
    // and safely exit module
    signal(SIGINT, Handler);
    
    printf("signal done\n");
    
    wiringPiSetup();
	printf("Set up wiring pi\n");
    
    pinMode( RIGHTLINESENSOR, INPUT);
    pinMode( MIDDLELINESENSOR, INPUT);
    pinMode( LEFTLINESENSOR, INPUT);
    printf("pin mode set\n");
    // for motor control
    int minSpeed = 15;
    int maxSpeed = 100;
    
    int object_detected;

	printf("Setting up lidar thread\n");
	pthread_t lidar_reader_thread;
	pthread_create(&lidar_reader_thread, NULL, &read_pipe, (void *) lidar_data);
	printf("Lidar set\n");

	printf("about to enter loop\n");
    while(1) {

		// go straight
		if( (onLine(LEFTLINESENSOR) == 0) && (onLine(MIDDLELINESENSOR) == 1) && (onLine(RIGHTLINESENSOR) == 0) ) {
			
			//printf("Go forward.\n");
			
			theta_cpy = lidar_data->theta;
			dist_cpy = lidar_data->distance;
			qual_cpy = lidar_data->quality;
	
			
			if (dist_cpy < 304.0 && qual_cpy > 0.0 && 
				((theta_cpy > 350.0 && theta_cpy < 360.0) || // 20 degree FOV
				(theta_cpy > 0.0 && theta_cpy < 10.0))) {
					object_detected = 1;
					printf("Object found! theta: %lf %lf %lf\n", theta_cpy, dist_cpy, qual_cpy);

					while (object_detected) {

						motorStop(LEFTMOTOR);
						motorStop(RIGHTMOTOR);
						delay(2000);
						
						// check if object is still there
						if (lidar_data->distance > 305.0 && lidar_data->quality > 0 &&
							((lidar_data->theta > 350.0 && lidar_data->theta < 360.0) || // 20 degree FOV
							(lidar_data->theta > 0.0 && lidar_data->theta < 10.0))) {
								break;
						
						}
						// object has not moved after 2 seconds
						// navigate around it
						if (lidar_data->distance < 304 && lidar_data->quality > 0 && 
						(lidar_data->theta > 350.0 && lidar_data->theta < 359.99) ||
						(lidar_data->theta > 0.0 && lidar_data->theta < 10.0)) {
							object_detected = 1;
							
						} else {
							object_detected = 0;
						}
							
						// navigate around it
						
						//while (1) {
									
							// rotate until object is parallel to car
							// double check
							while (1) {
											
								if (lidar_data->distance < 354 && lidar_data->quality > 0 && 
									lidar_data->theta < 45 &&
									(lidar_data->theta > 0 && lidar_data->theta < 50)) {
											
									printf("Going parallel. lidar theta %lf\n", lidar_data->theta);
									Rotate90L();
									break;
								}
										
								// if (lidar_data->distance < 300 && lidar_data->quality > 0 && 
								// 	(lidar_data->theta > 45 && lidar_data->theta < 80)) {
																								
								// 	printf("At parallel. Stopping. lidar theta %lf\n", lidar_data->theta);
								// 	motorStop(LEFTMOTOR);
								// 	motorStop(RIGHTMOTOR);
								// 	break;		
								// }
											
							}

							// follow obstacle, remain within its vicinity
							// assumes obstacle is always to the right of car.
							// search for line, prepare right-angle left
							while (1) {
								if (lidar_data->distance > 295.0 && lidar_data->quality > 0.0 &&
									(lidar_data->theta > 0 && lidar_data->theta < 100)) { // leaving obstacle and obstacle is to the right
									printf("Correct right.\n");
									motorStop(RIGHTMOTOR);
									motorRun(LEFTMOTOR, FORWARD, 100);
									delay(350);
								}

								else if (lidar_data->distance < 270.0 && lidar_data->quality > 0.0 &&
									(lidar_data->theta > 0 && lidar_data->theta < 100)) { // too close to obstacle... correct to left
									printf("Correct left.\n");
									motorStop(LEFTMOTOR);
									motorRun(RIGHTMOTOR, FORWARD, 100);
									delay(350);
									
								}
								
								// car is on track to hit the obstacle
								else if (lidar_data->distance < 120.0 && lidar_data->quality > 0.0 &&
									(lidar_data->theta > 255.0 && lidar_data->theta < 360.0) ||
									(lidar_data->theta > 0.0 && lidar_data->theta < 5.0)) {
									printf("Car is about to crash!\n");
									motorRun(LEFTMOTOR, BACKWARD, 100);
									motorRun(RIGHTMOTOR, FORWARD, 100);
									delay(450);
								}
								
								else {
									printf("Perfect\n");
								}
								
								motorRun(LEFTMOTOR, FORWARD, 50);
								motorRun(RIGHTMOTOR, FORWARD, 52);

								// line search
								if( ((onLine(LEFTLINESENSOR) == 1) && (onLine(MIDDLELINESENSOR) == 1)) 
									|| ((onLine(RIGHTLINESENSOR) == 1) && (onLine(MIDDLELINESENSOR) == 1))  ) {
									printf("Line detected.\n");
									motorStop(LEFTMOTOR);
									motorStop(RIGHTMOTOR);
									break;
								}

							}					
						//}
						object_detected = 0;
					}
				}

			motorRun(LEFTMOTOR, FORWARD, 50);
			motorRun(RIGHTMOTOR, FORWARD, 52);
			
			}
		// turn right
		if( (onLine(LEFTLINESENSOR) == 1) && (onLine(MIDDLELINESENSOR) == 0) && (onLine(RIGHTLINESENSOR) == 0) ){
			
			//printf("Go right.\n");
			motorStop(LEFTMOTOR);
			motorRun(RIGHTMOTOR, FORWARD, 100);
			delay(150);
			}
		// turn left
		if( (onLine(LEFTLINESENSOR) == 0) && (onLine(MIDDLELINESENSOR) == 0) && (onLine(RIGHTLINESENSOR) == 1) ){
			
			//printf("Go left.\n");
			motorRun(LEFTMOTOR, FORWARD, 100);
			motorStop(RIGHTMOTOR);
			delay(150);
			
			}
		// stop
		if( (onLine(LEFTLINESENSOR) == 0) && (onLine(MIDDLELINESENSOR) == 0) && (onLine(RIGHTLINESENSOR) == 0) ){
			
//			printf("Stop.\n");
			motorStop(LEFTMOTOR);
			motorStop(RIGHTMOTOR);
			
			}
		// right angle turn to left
		if( (onLine(LEFTLINESENSOR) == 1) && (onLine(MIDDLELINESENSOR) == 1) && (onLine(RIGHTLINESENSOR) == 0)) {
		
			//while ( (onLine(LEFTLINESENSOR) != 0) && (onLine(MIDDLELINESENSOR) != 0) && (onLine(RIGHTLINESENSOR) != 0)){
			//	motorRun(LEFTMOTOR, FORWARD, 50);
			//	motorRun(RIGHTMOTOR, FORWARD, 50);
			//	delay(500);	
			//}	
			// need to stop motors first before switching directions
			// so they don't burn out
			//motorRun(LEFTMOTOR, FORWARD, 50);
			//motorRun(RIGHTMOTOR, FORWARD, 50);
			//delay(500);
			motorStop(LEFTMOTOR);
			motorStop(RIGHTMOTOR);
			while( (onLine(LEFTLINESENSOR) != 0) && (onLine(MIDDLELINESENSOR) == 1) && (onLine(RIGHTLINESENSOR) == 0)  ){			
				motorRun(LEFTMOTOR, BACKWARD, 100);
				motorRun(RIGHTMOTOR, FORWARD, 70);
				delay(100);
			}
		}
		// right angle turn to right
		if( (onLine(LEFTLINESENSOR) == 0) && (onLine(MIDDLELINESENSOR) == 1) && (onLine(RIGHTLINESENSOR) == 1) ){
			
			// need to stop motors first before switching directions
			// so they don't burn out
			motorStop(LEFTMOTOR);
			motorStop(RIGHTMOTOR);
			while( (onLine(LEFTLINESENSOR) == 0) && (onLine(MIDDLELINESENSOR) == 1) && (onLine(RIGHTLINESENSOR) != 0)){		
				motorRun(LEFTMOTOR, FORWARD, 100);
				motorRun(RIGHTMOTOR, BACKWARD, 100);
				
				
			}
			//delay(100);
			}
		while( (onLine(LEFTLINESENSOR) == 1) && (onLine(MIDDLELINESENSOR) == 1) && (onLine(RIGHTLINESENSOR) == 1) ){
			motorRun(LEFTMOTOR, FORWARD, 50);
			motorRun(RIGHTMOTOR, FORWARD, 50);
		}
			
	}
    // if somehow break infinite loop
    // stop motor and exit module

    DEV_ModuleExit();
    return 0;
}

