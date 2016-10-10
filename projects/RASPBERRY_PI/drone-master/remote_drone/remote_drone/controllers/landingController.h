#ifndef _LANDING_H_
#define _LANDING_H_
#include <thread>
#include <wiringPi.h>
#include "../sensors/sonar/ultrasonic.h"
#include "../singleton.hpp"
#include "pid.h"
#include "./droneController.h"


class LandingController :public singleton<LandingController> {
private:
	PID *_1st_pid, *_2nd_pid;
	DroneController *drone;
	UltrasonicClass *ultrasonic;
private:
	unsigned char setPoint; // cm max 250;
	bool is_working;

public:
	LandingController();
	void set_altitude(float set_point);
public:
	void start();
	void stop();
	void calculate();
};

#endif