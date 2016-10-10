#include "landingController.h"

#include <iostream>

#define LANDING_DEBUG_MODE1



LandingController::LandingController()
{
	is_working = false;
	setPoint = 5;
	_1st_pid = new PID(0.02, 0.0002, 0.01, -0.4, 1, 0.05);	//
	_2nd_pid = new PID(0.1, 0, 0, -1, 1, 0.001);	// 
	drone = DroneController::getInstance();
	ultrasonic = UltrasonicClass::getInstance();
	ultrasonic->start();
}

void LandingController::set_altitude(float set_point)
{
	this->setPoint = set_point;
}

void LandingController::start() {
	if (!this->is_working) {
		thread tt([&]() { calculate(); });
		tt.detach();
	}
}
void LandingController::stop() {
	is_working = false;
}


void LandingController::calculate() {

	try
	{
		is_working = true;
		while (is_working) {
			float distance = ultrasonic->getDistance();
			float velocity = ultrasonic->getVelocity();
			if (distance + 10< setPoint) {
				setPoint -= 2;
			}

			if (distance < 10) {
				drone->order(commandType::stop);
				is_working = false;
				break;
			}

			double _1st_result = _1st_pid->calculate(this->setPoint, distance);
			//double output = _2nd_pid->calculate(_1st_result, velocity);
#ifdef LANDING_DEBUG_MODE
			cout << "distance : " << distance << " velocity : " << velocity << endl;
			cout << "_1st_result : " << _1st_result << " output : " << output << endl;
#endif
			drone->set_pwm_fluctuation((float)_1st_result,droneControlType::throttle);
			delay(100);
		}

	}
	catch (const std::exception&)
	{
		is_working = false;
	}
}