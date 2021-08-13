

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

#define PIPED_FILE_LOC "/tmp/lidar"

/*
* Threadable function for piping data
*/
void *read_pipe(void * arg) {

	char *ptr;
   	float ret, theta, distance, quality;

	// open piped file
	int fd = open(PIPED_FILE_LOC, O_RDONLY);
	char * buf = malloc(sizeof(char) * 100);

	// arg is lidar data
	struct lidar_data * lidar = (int *) arg;
	
	while (1) {
		// reading data in specific order
		// this order is implemented by modifying
		// SLAMTECH C++ library
		read(fd, buf, 100);
		theta = strtof(buf, &ptr);
		read(fd, buf, 100);
		distance = strtof(buf, &ptr);
		read(fd, buf, 100);
		quality = strtof(buf, &ptr);
		lidar->theta = theta;
		lidar->distance = distance;
		lidar->quality = quality;
	}
}
