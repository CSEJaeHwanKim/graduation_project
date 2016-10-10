#ifndef _PID_H_
#define _PID_H_

class PID
{
public:
	PID(double Kp, double Ki, double Kd, double min, double max, double dt );
	PID(double Kp, double Ki, double Kd, double min, double max);
	~PID();
	double calculate(double setpoint, double current);
	double calculate(double setpoint, double current,double dt);
	void setGain(double Kp, double Ki, double Kd);
	void set_dt(double dt);
	void init_param();

private:
	double _ori_Kp;
	double _ori_Ki;
	double _ori_Kd;
	double _Kp;
	double _Ki;
	double _Kd;
	double _dt;
	double _min;
	double _max;
	double _pre_error;
	double _integral;

};

#endif