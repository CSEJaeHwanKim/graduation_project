#include "gps.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <thread>

#include "gps_nmea.h"

gps::gps() {
	this->latitude = 0;
	this->longitude = 0;
	this->speed = 0;
	this->altitude = 0;
	this->course = 0;
	this->is_measuring = false;
}


void gps::gps_init(void) {
	this->g_serial.serial_init();
	this->g_serial.serial_config();

	//Write commands
}

void gps::gps_on(void) {
	if (!this->is_measuring) {
		gps_init();
		thread tt([&]() { gps_update(); });
		// tt.join();
		tt.detach();
	}
}

// Compute the GPS location using decimal scale
void gps::gps_update() {
	cout << "[gps] gps_update start\n";
	try
	{
		is_measuring = true;
		while (is_measuring) {
			uint8_t status = _EMPTY;
			while (status != _COMPLETED) {
				gpgga_t gpgga;
				gprmc_t gprmc;
				char buffer[256];

				g_serial.serial_readln(buffer, 256);
				switch (nmea_get_message_type(buffer)) {
				case NMEA_GPGGA:
					nmea_parse_gpgga(buffer, &gpgga);

					gps_convert_deg_to_dec(&(gpgga.latitude), gpgga.lat, &(gpgga.longitude), gpgga.lon);

					this->latitude = gpgga.latitude;
					this->longitude = gpgga.longitude;
					this->altitude = gpgga.altitude;

					status |= NMEA_GPGGA;
					break;
				case NMEA_GPRMC:
					nmea_parse_gprmc(buffer, &gprmc);

					this->speed = gprmc.speed;
					this->course = gprmc.course;

					status |= NMEA_GPRMC;
					break;
				}
			}
		}
	}
	catch (const std::exception&)
	{
		gps_off();
	}
	
}

void gps::gps_off(void) {
	//Write off
	this->g_serial.serial_close();
	is_measuring = false;
}

// Convert lat e lon to decimals (from deg)
void gps::gps_convert_deg_to_dec(double *latitude, char ns, double *longitude, char we)
{
	double lat = (ns == 'N') ? *latitude : -1 * (*latitude);
	double lon = (we == 'E') ? *longitude : -1 * (*longitude);

	*latitude = gps_deg_dec(lat);
	*longitude = gps_deg_dec(lon);
}

double gps::gps_deg_dec(double deg_point)
{
	double ddeg;
	double sec = modf(deg_point, &ddeg) * 60;
	int deg = (int)(ddeg / 100);
	int min = (int)(deg_point - (deg * 100));

	double absdlat = round(deg * 1000000.);
	double absmlat = round(min * 1000000.);
	double absslat = round(sec * 1000000.);

	return round(absdlat + (absmlat / 60) + (absslat / 3600)) / 1000000;
}

void gps::check() {
	if( !is_measuring ) {
		this->gps_on();
	}
}

double gps::get_latitude()
{
	check();
	return this->latitude;
}

double gps::get_longitude()
{
	check();
	return this->longitude;
}

double gps::get_speed()
{
	check();
	return this->speed;
}

double gps::get_altitude()
{
	check();
	return this->altitude;
}

double gps::get_course()
{
	check();
	return this->course;
}

