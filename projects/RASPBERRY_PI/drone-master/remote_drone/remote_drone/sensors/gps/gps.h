#ifndef _GPS_H_
#define _GPS_H_

#include "gps_serial.h"
#include "../../singleton.hpp"

class gps:public singleton<gps> {
private:
	double latitude;
	double longitude;
	double speed;
	double altitude;
	double course;
	

	bool is_measuring;
	gps_serial g_serial;
public:
	gps();
	// Initialize device
	void gps_init(void);
	// Activate device
	void gps_on(void);
	// Get the actual location
	void gps_update();
	// Turn off device (low-power consumption)
	void gps_off(void);
	void gps_convert_deg_to_dec(double *, char, double *, char);
	double gps_deg_dec(double);

	void check();
	
	double get_latitude();
	double get_longitude();
	double get_speed();
	double get_altitude();
	double get_course();
};







#endif
