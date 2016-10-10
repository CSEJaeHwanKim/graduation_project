#include "positionController.h"

PositionController::PositionController()
{
	initialize();
}

PositionController::~PositionController()
{
}

void PositionController::initialize()
{
	drone = DroneController::getInstance();
	altController = AltitudeController::getInstance();
	gps_instance = gps::getInstance();
	gy86 = RPIGY86::getInstance();
	
	is_gps_measure_success = false;
	cur_latitude = cur_longitude = 0;
	cnt_waypoint = target_waypoint = 0;
	
	remaining_distance = 0;
	is_working = false;

	cout << "Position contorller initializing.." << endl;
	while (cur_latitude == 0 && false) {
		cur_latitude = gps_instance->get_latitude();
		cur_longitude = gps_instance->get_longitude();
	}
	cout << "coordinate measure success!!" << endl;
	
	
	cur_bearing = gy86->getHeading();
	target_bearing = cur_bearing;

}



void PositionController::update_coordinate()
{
	this->cur_latitude = CONVERT_RADIANS(gps_instance->get_latitude());
	this->cur_longitude = CONVERT_RADIANS(gps_instance->get_longitude());
	if (cur_latitude == 0) { is_gps_measure_success = false; }
	else { is_gps_measure_success = true; }
}

void PositionController::calc_target_bearing()
{
	double deltaLon = longitudes[target_waypoint] - cur_longitude;
	double y = sin(deltaLon)*cos(latitudes[target_waypoint]);
	double x = cos(cur_latitude)*sin(latitudes[target_waypoint]) - sin(cur_latitude)*cos(latitudes[target_waypoint])*cos(deltaLon);
	
	float raw_bearing = atan2(y, x) * 180 / M_PI;
	target_bearing = (int)(raw_bearing +360) % 360;
}

void PositionController::calc_target_distance()
{
	double deltaLat_half = (latitudes[target_waypoint] - cur_latitude)/2;
	double deltaLon_half = (longitudes[target_waypoint] - cur_latitude)/2;
	double a = sin(deltaLat_half)*sin(deltaLat_half) +
		sin(deltaLon_half) * sin(deltaLon_half) * cos(cur_latitude) * cos(latitudes[target_waypoint]);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	remaining_distance = R_EARTH * c;
}


void PositionController::update_heading()
{
	cur_bearing = gy86->getHeading();
}

void PositionController::input_coordinate(double *latitudes, double *longitudes, unsigned short cnt_coordinate)
{
	if (latitudes == nullptr) {
		delete latitudes;
		delete longitudes;
	}
	this->latitudes = new double[cnt_coordinate];
	this->longitudes = new double[cnt_coordinate];
	for (int i = 0; i < cnt_coordinate; i++) {
		this->latitudes[i] = CONVERT_RADIANS(latitudes[i]);
		this->longitudes[i] = CONVERT_RADIANS(longitudes[i]);
	}
	this->cnt_waypoint = cnt_coordinate;
	this->target_bearing = 0;
}

void PositionController::start() {
	
	is_working = true;
	while (is_working) {
		try
		{
			update_coordinate();
			update_heading();


			calc_target_bearing();
			calc_target_distance();
			altController->calculate();


			//maximum 20Hz
			delay(50);

		}
		catch (const std::exception& e)
		{
			continue;
		}
	}

}
void PositionController::stop() {
	is_working = false;
}