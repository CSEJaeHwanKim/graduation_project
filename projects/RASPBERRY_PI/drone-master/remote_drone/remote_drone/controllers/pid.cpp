#include "pid.h"

PID::PID(double Kp, double Ki, double Kd, double min, double max, double dt) {

	this->_ori_Kp = Kp;
	this->_ori_Ki = Ki;
	this->_ori_Kd = Kd;

	this->_Kp = Kp;
	this->_Ki = Ki*dt;
	this->_Kd = Kd / dt;

	this->_min = min;
	this->_max = max;
	this->_dt = dt;
	this->_pre_error = 0;
	this->_integral = 0;
}

PID::PID(double Kp, double Ki, double Kd, double min, double max)
{

	this->_dt = 0.01; // default sampling time 0.01 s

	this->_ori_Kp = Kp;
	this->_ori_Ki = Ki;
	this->_ori_Kd = Kd;

	this->_Kp = Kp;
	this->_Ki = Ki* _dt;
	this->_Kd = Kd / _dt;

	this->_min = min;
	this->_max = max;
	this->_pre_error = 0;
	this->_integral = 0;

}

double PID::calculate(double setpoint, double current)
{

	double error = setpoint - current;

	double Pout = _Kp * error;
	double Iout = _Ki * error;
	double Dout = _Kd * (error - _pre_error);
	double output = Pout + Iout - Dout;

	if (output > _max) { output = _max; }
	else if (output < _min) { output = _min; }

	_pre_error = error;

	return output;
}

double PID::calculate(double setpoint, double current, double dt)
{
	double error = setpoint - current;

	double Pout = _ori_Kp * error;

	_integral += error * dt;
	double Iout = _ori_Ki * _integral;

	double derivative = (error - _pre_error) / dt;
	double Dout = _ori_Kd * derivative;

	double output = Pout + Iout + Dout;

	if (output > _max) { output = _max; }
	else if (output < _min) { output = _min; }

	_pre_error = error;

	return output;
}

void PID::setGain(double Kp, double Ki, double Kd)
{
	this->_ori_Kp = Kp;
	this->_ori_Ki = Ki;
	this->_ori_Kd = Kd;

	this->_Kp = Kp;
	this->_Ki = Ki*_dt;
	this->_Kd = Kd / _dt;

}

void PID::set_dt(double dt)
{
	this->_dt = dt;
	this->_Ki *= _dt;
	this->_Kd /= _dt;
}

void PID::init_param()
{
	this->_pre_error = 0;
	this->_integral = 0;
}


PID::~PID()
{
}
