/*
 * main.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Bjorn Blissing
 */

#include "racelogic.h"
#include <iostream>
#include <iomanip>


int main()
{
    RaceLogicDevice* device = new RaceLogicDevice("COM4");
    if (device == 0) { 
		std::cerr << "ERROR: Unable to get device!" << std::endl;
		return 1;
	}
	if (device->open() == RaceLogicDevice::SUCCESS) {
		for (int i = 0; i < 1000; ++i) {
			// Read packets
			if (device->read() == RaceLogicDevice::SUCCESS) {
				std::cout << "Time: " << device->timeString() << std::endl;
				std::cout << "Satelites: " << (int) device->satelites() << std::endl;
				std::cout << "Latitude: " << device->latitudeString() << std::endl;
				std::cout << "Longitude: " << device->longitudeString() << std::endl;
			}
		}

		if (device->close() != RaceLogicDevice::SUCCESS) {
			delete device;
			return EXIT_FAILURE;
		}
	} else {
		delete device;
		return EXIT_FAILURE;
	}

    delete device;
    return EXIT_SUCCESS;
}