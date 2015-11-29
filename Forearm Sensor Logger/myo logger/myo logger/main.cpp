// Logger for the Master Thesis "Efficient Object Detection through Grasp Intention"
// Author: Marian Theiss


// Most Code developed by Thalmic Labs:
// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.


#define _USE_MATH_DEFINES
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <fstream>
#include <time.h>
#include <conio.h>

#include <myo/myo.hpp>

class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
	{
		//openFiles();
		actionscounter[0] = 0;
		actionscounter[1] = 0;
	}


	void openFiles() {
		if (filename.empty())
		{
			std::cout << "Name file: ";
			std::cin >> filename;
		}
		else
		{
			std::cout << "\n reconnected to Myo... starting new file\n";
		}
		time_t timestamp;
		struct tm  timeinfo;
		time(&timestamp);
		localtime_s(&timeinfo, &timestamp);
		if (filename.empty())
		{
			std::cout << "Name file: ";
			std::cin >> filename;
		}
		else
		{
			std::cout << "\n reconnected to Myo at time" <<std::to_string(timeinfo.tm_min) << ":" << std::to_string(timeinfo.tm_sec) <<" ... starting new file\n";
		}
		std::string datestring = std::to_string(timeinfo.tm_mon + 1) + "_" + std::to_string(timeinfo.tm_mday) + "-" + std::to_string(timeinfo.tm_hour) + "_" + std::to_string(timeinfo.tm_min) + "_" + std::to_string(timeinfo.tm_sec);
		// Open file for EMG log
		if (emgFile.is_open()) {
			emgFile.close();
		}
		std::ostringstream emgFileString;
		emgFileString << filename << "-emg-" << datestring << ".csv";
		emgFile.open(emgFileString.str(), std::ios::out);
		emgFile << "timestamp;emg1;emg2;emg3;emg4;emg5;emg6;emg7;emg8" << std::endl;

		// Open file for gyroscope log
		if (gyroFile.is_open()) {
			gyroFile.close();
		}
		std::ostringstream gyroFileString;
		gyroFileString << filename << "-gyro-" << datestring << ".csv";
		gyroFile.open(gyroFileString.str(), std::ios::out);
		gyroFile << "timestamp;x;y;z" << std::endl;

		// Open file for accelerometer log
		if (accelerometerFile.is_open()) {
			accelerometerFile.close();
		}
		std::ostringstream accelerometerFileString;
		accelerometerFileString <<  filename << "-accel-" << datestring << ".csv";
		accelerometerFile.open(accelerometerFileString.str(), std::ios::out);
		accelerometerFile << "timestamp;x;y;z" << std::endl;

		// Open file for orientatin log
		if (orientationFile.is_open()) {
			orientationFile.close();
		}
		std::ostringstream orientationFileString;
		orientationFileString << filename << "-orient-" << datestring << ".csv";
		orientationFile.open(orientationFileString.str(), std::ios::out);
		orientationFile << "timestamp;x;y;z;w" << std::endl;

		// Open file for actions log
		if (actionsFile.is_open()) {
			actionsFile.close();
		}
		std::ostringstream actionsFileString;
		actionsFileString << filename << "-actions-" << datestring << ".csv";
		actionsFile.open(actionsFileString.str(), std::ios::out);
		actionsFile << "timestamp;action1;action2" << std::endl;

		if (!(orientationFile.is_open() && accelerometerFile.is_open() && gyroFile.is_open() && emgFile.is_open() &&actionsFile.is_open()))
		{
			throw std::runtime_error("Unable to open one file!");
		}
	}

	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		throw std::runtime_error("Error: Lost the Myo");

	}

	// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
	{

		emgFile << timestamp;
		for (size_t i = 0; i < 8; i++) {
			emgFile << ';' << static_cast<int>(emg[i]);

		}
		emgFile << std::endl;
		emgTest = static_cast<int>(emg[0]);
	}

	// onOrientationData is called whenever new orientation data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onOrientationData(myo::Myo *myo, uint64_t timestamp, const myo::Quaternion< float > &rotation) {
		orientationFile << std::to_string(timestamp)
			<< ';' << rotation.x()
			<< ';' << rotation.y()
			<< ';' << rotation.z()
			<< ';' << rotation.w()
			<< std::endl;
	}

	// onAccelerometerData is called whenever new acceleromenter data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel) {

		printVector(accelerometerFile, timestamp, accel);

		//Checks for keyboard Data
		if (_kbhit())
		{
			input = _getch();
			switch (input)
			{
				case '1': ++actionscounter[0];
					std::cout << timestamp << " c1: " << actionscounter[0] << " c2: " << actionscounter[1] << std::endl;
					actionsFile << timestamp
						<< ';' << actionscounter[0]
						<< ';' << actionscounter[1]
						<< std::endl;
					break;
				case '2': ++actionscounter[1];
					std::cout << timestamp << " c1: " << actionscounter[0] << " c2: " << actionscounter[1] << " emgtest " << emgTest << std::endl;
					actionsFile << timestamp
						<< ';' << actionscounter[0]
						<< ';' << actionscounter[1]
						<< std::endl; 
					break;
			}
		}

	}

	// onGyroscopeData is called whenever new gyroscope data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) {
		printVector(gyroFile, timestamp, gyro);

	}

	void onConnect(myo::Myo *myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
		//Reneable streaming
		myo->vibrate(myo::Myo::vibrationLong);
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
		openFiles();
	}

	void onDisconnect(myo::Myo *myo, uint64_t timestamp) {
		//Reneable streaming
		myo->vibrate(myo::Myo::vibrationLong);
		std::cout << "disconnected at " << timestamp << std::endl;
	}

	// Helper to print out accelerometer and gyroscope vectors
	void printVector(std::ofstream &file, uint64_t timestamp, const myo::Vector3< float > &vector) {
		file << timestamp
			<< ';' << vector.x()
			<< ';' << vector.y()
			<< ';' << vector.z()
			<< std::endl;
	}

	
	// The files we are logging to
	std::string filename;
	std::ofstream emgFile;
	std::ofstream gyroFile;
	std::ofstream orientationFile;
	std::ofstream accelerometerFile;
	std::ofstream actionsFile;
	
	int emgTest;
	int actionscounter[2];

	char input;
};

int main(int argc, char** argv)
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {

		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.emg-data-sample");
		std::cout << "Press 1 to increase actions counter 1, Press 2 to increase actions counter 2" << std::endl;
		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo.
		std::cout << "Connected to a Myo armband! Logging to the file system. Check the folder this appliation lives in." << std::endl << std::endl;

		// Next we enable EMG streaming on the found Myo.
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;

		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);

		// Finally we enter our main loop.
		while (1) {
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 50 times a second, so we run for 1000/20 milliseconds.
			hub.run(1);
		}

		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}