#ifndef _ALTITUDECONTROLLER_H_
#define _ALTITUDECONTROLLER_H_

#include "../singleton.hpp"
#include "droneController.h"
#include "../sensors/sonar/ultrasonic.h"
#include "../sensors/gy-86/RPIGY86.h"
#include "pid.h"
#include "../sensors/filters.h"

#include <wiringPi.h>

#define ALT_BUF_SIZE 4

class AltitudeController : public singleton<AltitudeController> {
private:
	PID *_1st_pid, *_1st_pid_sonar, *_2nd_pid;
	DroneController *drone;
	UltrasonicClass *sonar;
	RPIGY86 *gy86;
	MAF * maf_landing;
	unsigned char landing_cnt;

private:
	unsigned char buf_index;
	float altBuffer[ALT_BUF_SIZE];
	unsigned int timeBuffer[ALT_BUF_SIZE];
	float baseAltitude;
	float sonar_distance;
	float setPoint; //m
	

private:
	void input_buffer(float altitude);

public:
	AltitudeController();
	void initialize();
	void set_altitude(float set_point);
public:
	void calculate();
};


#endif