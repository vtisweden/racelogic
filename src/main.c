/*
 * main.c
 *
 *  Created on: Jan 16, 2015
 *      Author: Bjorn Blissing
 */

#include "racelogic.h"
#include <stdlib.h>
#include <stdio.h>

int main()
{
	
	RaceLogicDevice* device = rl3_get_device("COM4");
	if (device == NULL) {
		fprintf(stderr, "ERROR: Unable to get device!\n");
		return EXIT_FAILURE;
	}

	if (rl3_open_device(device) == EXIT_SUCCESS) {
		// Initialize char arrays
		size_t time_str_len = rl3_get_time_str_len(device);
		size_t coordinate_str_len = rl3_get_coordinate_str_len(device);
		char* time_str = (char *) malloc(time_str_len);
		char* lat_str = (char *) malloc(coordinate_str_len);
		char* long_str = (char *) malloc(coordinate_str_len); 

		while (rl3_device_is_open(device) != 0) {
			// Read packets
			if (rl3_read_data(device) == EXIT_SUCCESS) {
				if (rl3_get_time_str(device, time_str, time_str_len) == EXIT_SUCCESS) {
					printf("Time: %s\n",time_str);
				}

				if (rl3_get_latitude_str(device, lat_str, coordinate_str_len) == EXIT_SUCCESS) {
					printf("Latitude: %s\n",lat_str);
				}
				if (rl3_get_longitude_str(device, long_str, coordinate_str_len) == EXIT_SUCCESS) {
					printf("Longitude: %s\n",long_str);
				}
			}
		}

		// Free memory used for char arrays
		free(time_str);
		free(lat_str);
		free(long_str);
	
		if (rl3_close_device(device) != EXIT_SUCCESS) {
			rl3_delete_device(device);
			return EXIT_FAILURE;
		}
	} else {
		rl3_delete_device(device);
		return EXIT_FAILURE;
	}

    rl3_delete_device(device);

    return EXIT_SUCCESS;
}