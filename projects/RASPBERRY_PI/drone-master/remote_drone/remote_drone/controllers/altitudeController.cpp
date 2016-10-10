#include "altitudeController.h"

#include <stdio.h>

AltitudeController::AltitudeController()
{
	initialize();
}

void AltitudeController::initialize()
{
	drone = DroneController::getInstance();
	gy86 = RPIGY86::getInstance();
	sonar = UltrasonicClass::getInstance();
	
	_1st_pid = new PID(10, 0.01, 0.01, -10, 30, 0.1);
	//_1st_pid_sonar = new PID(0.2, 0, 0, -15, 40, 0.1);
	_2nd_pid = new PID(0.15, 0, 0, -0.5, 1, 0.1);

	maf_landing = new MAF(0, 20);

	baseAltitude = gy86->getAltitude();

	for (int i = 0; i < ALT_BUF_SIZE; i++) {
		altBuffer[i] = baseAltitude;
		timeBuffer[i] = 0U;
	}
	buf_index = 0;
	cout << "base altitude initializing" << endl;
	for (int i = 0; i < ALT_BUF_SIZE*4; i++) {
		calculate();
		delay(100);
	}
	landing_cnt = 0;
	baseAltitude = altBuffer[ALT_BUF_SIZE-1];
	cout << "baseAltitude : " << baseAltitude << endl;
	cout << "AltitudeController initialize complete" << endl;
}

//output -> older altitude
void AltitudeController::input_buffer(float altitude)
{
	timeBuffer[buf_index] = millis();
	altBuffer[buf_index++] = altitude;
	if (buf_index >= ALT_BUF_SIZE) { buf_index = 0; }
}

void AltitudeController::set_altitude(float set_point)
{
	if (set_point < 0) return;
	this->setPoint = baseAltitude + set_point;
}

void AltitudeController::calculate()
{
	
	float altitude = gy86->getAltitude();
	maf_landing->step(altitude);
	input_buffer(altitude);
	float older_altitude = altBuffer[buf_index];
	
	double _1st_result = 0;
	if (drone->get_command() == commandType::landing) {
		if (altitude - 0.5 < setPoint) {
			landing_cnt++;
		}
		if (landing_cnt > 50) {
			drone->order(commandType::stop);
		}

		/*sonar_distance = sonar->getDistance();
		_1st_result = _1st_pid->calculate(5, sonar_distance);
		if (sonar_distance < 10) {
			drone->order(commandType::stop);
		}*/
		cout << "landing" << endl;
	}
	_1st_result = _1st_pid->calculate(setPoint, altitude);
	
	//result < 0  critical error 
	unsigned long travelTime = timeBuffer[(ALT_BUF_SIZE + buf_index - 1) % ALT_BUF_SIZE] - timeBuffer[buf_index];

	

	//printf("curT%d : %u preT%d : %u -- %u\n", (ALT_BUF_SIZE + buf_index - 1) % ALT_BUF_SIZE, timeBuffer[(ALT_BUF_SIZE + buf_index - 1) % ALT_BUF_SIZE], buf_index, timeBuffer[buf_index], millis() );
	float velocity = (altitude - older_altitude) * ( 100000 / travelTime );

	double _2nd_result = _2nd_pid->calculate(_1st_result, velocity);

	drone->set_pwm_fluctuation((float)_2nd_result,droneControlType::throttle);	
	//cout << "1st : " << _1st_result << " , _2nd : " << _2nd_result << ", throttle : " << drone->get_pwm(droneControlType::throttle)<<endl;
	//printf("altitude : %.1lf , older altitude : %.2lf, travel: %d, %.3lf, var :%.5lf\n", altitude,older_altitude,travelTime,velocity,maf_landing->get_variance());
	cout << "altitude : " << altitude << " , set_point : " << setPoint << endl;
	//double _1st_result = _1st_pid->calculate(this->setPoint, distance);

	//drone->set_pwm_fluctuation((float)_1st_result, droneControlType::throttle)
}
