#ifndef _POSITIONCONTROLLER_H_
#define _POSITIONCONTROLLER_H_

#include "../singleton.hpp"
#include "pid.h"
#include "droneController.h"
#include "../sensors/gy-86/RPIGY86.h"
#include "../sensors/gps/gps.h"
#include "./altitudeController.h"
#include <thread>
#include <cmath>
#define _USE_MATH_DEFINES

#define R_EARTH 6378137
#define CONVERT_RADIANS(X)  X * ( 3.141592 / 180 )

class PositionController : public singleton <PositionController>{
private:
	PID *pid_yaw, *pid_pitch;

	bool is_gps_measure_success;
	double cur_latitude, cur_longitude;
	double *latitudes, *longitudes;
	unsigned short cnt_waypoint, target_waypoint;
	unsigned short target_bearing;
	unsigned short cur_bearing;
	double remaining_distance;

	bool is_working;

	//airpress part
	float altitude;

	gps *gps_instance;
	RPIGY86 *gy86;
	DroneController *drone;
	AltitudeController *altController;
	
	

private:

	void initialize();

	void update_coordinate();
	void calc_target_bearing();
	void calc_target_distance();

	void update_heading();

public:
	PositionController();
	~PositionController();
	void input_coordinate(double *latitudes, double *longitudes, unsigned short cnt_coordinate);
	void start();
	void stop();

};

#endif