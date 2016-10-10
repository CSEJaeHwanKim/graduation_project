#ifndef FILTERS_H
#define FILTERS_H





//START SIMPLE KALMAN FILTER 

/*

simeple kalman 1
reference site : http://www.magesblog.com/2014/12/measuring-temperature-with-my-arduino.html
simeple kalman 2
reference site : http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/

*/

#define SIMPLE_KALMAN_1
//#define SIMPLE_KALMAN_2
class simpleKfilter {
private:
	double q; //process noise covariance , varP
	double r; //measurement noise covariance, varM
	double x; //value
	double p; //estimation error covariance
	double k; //kalman gain
public:
	//simpleKfilter();
	simpleKfilter(float k = 1, float x = 5, float p = 5, float q = 0.11, float r = 0.8);
	float step(float data);

};

// ! START KALMAN FILTER 



// START MOVING AVERAGE FILTER
class MAF {
private :
	float average;
	unsigned short index;
	unsigned short buffer_size;
	double *buffer;
public:
	MAF(double init_vaule = 5 ,unsigned short buffer_size=50);
	~MAF();
	double step(double value);
	double nonsave_step(double value);
	void save_value(double value);
	void save_value(double value, double average);
	double get_variance();

	void set_buffer_size(unsigned short buffer_size);
	unsigned short get_buffer_size();
};

// ! MOVING AVERAGE FILTER


// START LOW PASS FILTER
/*
	reference site: http://pinkwink.kr/437

*/

class LPF {
private:
	float tau,sampling_time;	
	double pre_val;
public:
	LPF(float tau,float sampling_time);
	void set_smapling_time(float sampling_time);
	void set_tau(float tau);
	float get_tau();
	float get_sampling_time();
public:
	float step(float raw_value);
	float step(float raw_value, float sampling_t);
};



// ! HIGH PASS FILTER

// START LOW PASS FILTER
/*
reference site: http://pinkwink.kr/437

*/

class HPF {
private:
	float tau, sampling_time;
	double pre_val;
	double pre_raw_val;
public:
	HPF(float tau, float sampling_time);
	void set_smapling_time(float sampling_time);
	void set_tau(float tau);
	float get_tau();
	float get_sampling_time();
public:
	float step(float raw_value);
	float step(float raw_value, float sampling_t);
};



// ! HIGH PASS FILTER



#endif // !FILTERS_H