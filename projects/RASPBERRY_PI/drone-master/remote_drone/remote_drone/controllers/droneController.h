#ifndef DRONECONTROLLER_H
#define DRONECONTROLLER_H

#include <utility>
#include <mutex>
#include <iostream>
#include <wiringPi.h>
#include <string>
#include "../singleton.hpp"

#define MIN 0
#define MAX 1

namespace droneControlType {
	enum Enum {
		throttle, pitch, roll, yaw
	};
}
namespace dronePWM {
	enum Enum {
		current, neutral, min, max, min2neu, neu2max
	};
}

namespace commandType {
	enum Enum {
		stop, start, landing, move, pause
	};
}


class DroneController :public singleton<DroneController> {
private:
	short PWM[4][6];
	float pwm_percentage[4];
	bool is_permit,is_using_deadband;
	char deadband[4][2];
	mutex mutex_pwm[4];
	unsigned char current_command;
private:
	/*template<typename type>
	type constrain(type value, type min, type max);
	*/
	double constrain(double percent, droneControlType::Enum controlType);
	short map(float percent, droneControlType::Enum controlType);
	void pwm_gap_calc();

	void start_drone();
	void stop_drone();
public:
	DroneController();
	//back < 50(neutral) < front
	//left < 50(neutral) < right
	void set_pwm_percentage(float rate_of_change, droneControlType::Enum controlType);
	void set_pwm_fluctuation(float rate_of_change, droneControlType::Enum controlType);

	void set_deadband(char min, char max, droneControlType::Enum controlType);
	void use_deadband(bool flag);

	float get_pwm_percentage(droneControlType::Enum controlType);
	short get_pwm(droneControlType::Enum controlType);
	string get_pwm_string();

	

public:
	void order(commandType::Enum command);
	commandType::Enum get_command();

};




#endif
/*
template<typename type>
inline type DroneController::constrain(type value, type min, type max)
{
if (value > max) return max;
else if (value < min) return min;
else return value;
}
*/