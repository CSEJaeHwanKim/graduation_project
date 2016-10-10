#include "droneController.h"

using namespace std;


// default setting
DroneController::DroneController() {
	current_command = commandType::stop;
	
	PWM[droneControlType::throttle][dronePWM::current] = PWM[droneControlType::throttle][dronePWM::min] = 1000;
	PWM[droneControlType::throttle][dronePWM::neutral] = 1020;
	PWM[droneControlType::throttle][dronePWM::max] = 2000;


	PWM[droneControlType::pitch][dronePWM::current] = PWM[droneControlType::pitch][dronePWM::neutral] = 1500;
	PWM[droneControlType::pitch][dronePWM::min] = 1000;
	PWM[droneControlType::pitch][dronePWM::max] = 2000;

	PWM[droneControlType::roll][dronePWM::current] = PWM[droneControlType::roll][dronePWM::neutral] = 1500;
	PWM[droneControlType::roll][dronePWM::min] = 1000;
	PWM[droneControlType::roll][dronePWM::max] = 2000;

	PWM[droneControlType::yaw][dronePWM::current] = PWM[droneControlType::yaw][dronePWM::neutral] = 1500;
	PWM[droneControlType::yaw][dronePWM::min] = 1000;
	PWM[droneControlType::yaw][dronePWM::max] = 2000;

	pwm_percentage[droneControlType::throttle] = 0;
	pwm_percentage[droneControlType::pitch] = 50;
	pwm_percentage[droneControlType::roll] = 50;
	pwm_percentage[droneControlType::yaw] = 50;

	deadband[droneControlType::throttle][MIN] = 45;
	deadband[droneControlType::throttle][MAX] = 75;
	deadband[droneControlType::pitch][MIN] = 30;
	deadband[droneControlType::pitch][MAX] = 70;
	deadband[droneControlType::roll][MIN] = 30;
	deadband[droneControlType::roll][MAX] = 70;
	deadband[droneControlType::yaw][MIN] = 30;
	deadband[droneControlType::yaw][MAX] = 70;

	is_using_deadband = true;


	pwm_gap_calc();
	is_permit = false;
}

void DroneController::pwm_gap_calc()
{

	PWM[droneControlType::throttle][dronePWM::min2neu] = PWM[droneControlType::throttle][dronePWM::neutral] - PWM[droneControlType::throttle][dronePWM::min];
	PWM[droneControlType::throttle][dronePWM::neu2max] = PWM[droneControlType::throttle][dronePWM::max] - PWM[droneControlType::throttle][dronePWM::neutral];

	PWM[droneControlType::pitch][dronePWM::min2neu] = PWM[droneControlType::pitch][dronePWM::neutral] - PWM[droneControlType::pitch][dronePWM::min];
	PWM[droneControlType::pitch][dronePWM::neu2max] = PWM[droneControlType::pitch][dronePWM::max] - PWM[droneControlType::pitch][dronePWM::neutral];

	PWM[droneControlType::roll][dronePWM::min2neu] = PWM[droneControlType::roll][dronePWM::neutral] - PWM[droneControlType::roll][dronePWM::min];
	PWM[droneControlType::roll][dronePWM::neu2max] = PWM[droneControlType::roll][dronePWM::max] - PWM[droneControlType::roll][dronePWM::neutral];

	PWM[droneControlType::yaw][dronePWM::min2neu] = PWM[droneControlType::yaw][dronePWM::neutral] - PWM[droneControlType::yaw][dronePWM::min];
	PWM[droneControlType::yaw][dronePWM::neu2max] = PWM[droneControlType::yaw][dronePWM::max] - PWM[droneControlType::yaw][dronePWM::neutral];

}
void DroneController::set_pwm_percentage(float percent, droneControlType::Enum controlType)
{
	if (!is_permit) { return; }
	lock_guard<mutex> lock(mutex_pwm[controlType]);
	PWM[controlType][dronePWM::current] = map(percent, controlType);

}

void DroneController::set_pwm_fluctuation(float rate_of_change, droneControlType::Enum controlType)
{
	if (!is_permit) { return; }
	set_pwm_percentage(get_pwm_percentage(controlType) + rate_of_change, controlType);
}

void DroneController::set_deadband(char min, char max, droneControlType::Enum controlType)
{
	if (min > max || max > 100 || min < 0) {
		return;
	}
	deadband[controlType][MIN] = min;
	deadband[controlType][MAX] = max;
}

void DroneController::use_deadband(bool flag)
{
	is_using_deadband = flag;
}

float DroneController::get_pwm_percentage(droneControlType::Enum controlType)
{
	return this->pwm_percentage[controlType];
}

short DroneController::get_pwm(droneControlType::Enum controlType)
{
	return this->PWM[controlType][dronePWM::current];
}

string DroneController::get_pwm_string() {
	string temp = "#";
	temp += std::to_string(PWM[droneControlType::throttle][dronePWM::current]);
	temp += std::to_string(PWM[droneControlType::pitch][dronePWM::current]);
	temp += std::to_string(PWM[droneControlType::roll][dronePWM::current]);
	temp += std::to_string(PWM[droneControlType::yaw][dronePWM::current]);
	temp += "@";

	return temp;
}

double DroneController::constrain(double percent, droneControlType::Enum controlType)
{
	if (is_using_deadband) {
		if (percent > deadband[controlType][MAX]) return deadband[controlType][MAX];
		else if (percent < deadband[controlType][MIN]) return deadband[controlType][MIN];
	}
	if (percent > 100) return 100.0;
	else if (percent < 0) return 0.0;
	return percent;
}

short DroneController::map(float percent, droneControlType::Enum controlType)
{
	percent = constrain(percent, controlType);
	pwm_percentage[controlType] = percent;
	percent /= 100;
	switch (controlType) {
	case droneControlType::throttle:
		//cout << PWM[controlType][dronePWM::min] + PWM[controlType][dronePWM::neu2max] * percent << endl;
		return PWM[controlType][dronePWM::min] + PWM[controlType][dronePWM::neu2max] * percent;
		break;
	default:
		if (percent == 0.5) {
			return PWM[controlType][dronePWM::neutral];
		}
		else if (percent < 0.5) {
			return PWM[controlType][dronePWM::min] + PWM[controlType][dronePWM::min2neu] * percent;
		}
		else if (percent > 0.5) {
			percent -= 0.5;
			return PWM[controlType][dronePWM::neutral] + PWM[controlType][dronePWM::neu2max] * percent;
		}
	}

}

void DroneController::start_drone()
{
	PWM[droneControlType::throttle][dronePWM::current] = PWM[droneControlType::throttle][dronePWM::min];
	PWM[droneControlType::yaw][dronePWM::current] = PWM[droneControlType::yaw][dronePWM::neutral];
	delayMicroseconds(1000);
	PWM[droneControlType::throttle][dronePWM::current] = PWM[droneControlType::throttle][dronePWM::min];
	PWM[droneControlType::yaw][dronePWM::current] = PWM[droneControlType::yaw][dronePWM::max];
	delay(3000);
	PWM[droneControlType::throttle][dronePWM::current] = PWM[droneControlType::throttle][dronePWM::min];
	PWM[droneControlType::yaw][dronePWM::current] = PWM[droneControlType::yaw][dronePWM::neutral];
	is_permit = true;
}

void DroneController::stop_drone()
{
	is_permit = false;
	PWM[droneControlType::throttle][dronePWM::current] = PWM[droneControlType::throttle][dronePWM::min];
	PWM[droneControlType::pitch][dronePWM::current] = PWM[droneControlType::pitch][dronePWM::neutral];
	PWM[droneControlType::roll][dronePWM::current] = PWM[droneControlType::roll][dronePWM::neutral];
	PWM[droneControlType::yaw][dronePWM::current] = PWM[droneControlType::yaw][dronePWM::neutral];

	pwm_percentage[droneControlType::throttle] = 0;
	pwm_percentage[droneControlType::pitch] = 50;
	pwm_percentage[droneControlType::roll] = 50;
	pwm_percentage[droneControlType::yaw] = 50;
}

void  DroneController::order(commandType::Enum command)
{
	switch (command) {
	case commandType::stop:
		stop_drone();
		break;
	case commandType::start:
		start_drone();
		break;
	case commandType::pause:

		break;
	case commandType::move:

		break;
	case commandType::landing:

		break;

	}
	current_command = command;
}

commandType::Enum DroneController::get_command()
{
	return commandType::Enum(current_command);
}
