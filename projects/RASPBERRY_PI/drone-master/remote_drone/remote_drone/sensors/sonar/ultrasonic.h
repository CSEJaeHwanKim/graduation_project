#ifndef SONIC_H
#define SONIC_H
/*
sensor : HC - SR04
frequency : 4kHz
angle : < 15 degree
Confidence Intervals : 2cm ~ 300cm

*/


#define INTERNAL 1
#define EXTERNAL 2
#define MEASUREMENT_MODE EXTERNAL

#if MEASUREMENT_MODE==INTERNAL
#define ECHO_PIN 24
#define TRIG_PIN 23
#endif

#define DEBUG_ON 1
#define DEBUG_OFF 2
#define SONAR_DEBUG DEBUG_ON

#if SONAR_DEBUG==DEBUG_ON
#include <fstream>
#endif


#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <mutex>
#include <iostream>
#include "../filters.h"
#include "../../singleton.hpp"


using namespace std;

class UltrasonicClass :public singleton<UltrasonicClass> {

private:
	float distance, velocity, acceleration;
#if SONAR_DEBUG==DEBUG_ON
	ofstream debug_output;
#endif
public:
	UltrasonicClass();
	~UltrasonicClass();
	float p_raw_value;
	float p_raw_filter;

	float getDistance();
	float getVelocity();
	float getAcceleration();


#if MEASUREMENT_MODE==INTERNAL
private:
	int trig, echo;
	bool is_measuring;

private:
	void starter();
	void measure_distance();

public:
	void start();
#elif MEASUREMENT_MODE==EXTERNAL
private:
	double prev_distance;
	unsigned int prev_timestamp;
	unsigned char vel_index;
	MAF *MAF_distance, *MAF_velocity;
public:
	void input_values(double raw_distance, unsigned int timestamp);

	void start();
#endif	
};


#endif        /* SONIC_H */