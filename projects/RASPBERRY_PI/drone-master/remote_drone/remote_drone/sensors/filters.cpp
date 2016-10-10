#include "filters.h"

#include <iostream>
using namespace std;

simpleKfilter::simpleKfilter(float k ,float x,float p ,float q, float r)
{
	this->k = k;	//칼만게인
	this->x = x;
	this->p = p; //상태 공분산
	this->q = q; //센서 노이즈 공분산
	this->r = r; //측정 공분산
}
#ifdef SIMPLE_KALMAN_1
float simpleKfilter::step(float data)
{
	p = p + q;
	k = p / (p + r);
	x = k * data + (1 - k) * x;
	p = (1 - k) * p;
	return x;
}
#endif 

#ifdef SIMPLE_KALMAN_2
float simpleKfilter::step(float data)
{

	p = p + q;

	k = p / (p + r);
	x = x + k * (data - x);
	p = (1 - k) * p;
	return x;
}
#endif 

// START MOVING AVERAGE FILTER
MAF::MAF(double init_vaule, unsigned short buffer_size)
{
	this->average = init_vaule;
	this->index = 0;
	this->buffer_size = buffer_size;
	this->buffer = new double[buffer_size];
	for (int i = 0; i < buffer_size; i++) { buffer[i] = init_vaule; }
}

MAF::~MAF()
{
	delete[] buffer;
}

double MAF::step(double value)
{
	average = average + (value - buffer[index]) / (double)buffer_size;
	buffer[index++] = value;
	if (index >= buffer_size) { index = 0; }

	return average;
}

double MAF::nonsave_step(double value)
{
	return (float)(average + (value - buffer[index]) / (double)buffer_size);
}

void MAF::save_value(double value)
{
	average = average + (value - buffer[index]) / (double)buffer_size;
	buffer[index++] = value;
	if (index >= buffer_size) { index = 0; }
}
void MAF::save_value(double value,double average)
{
	this->average = average;
	buffer[index++] = value;
	if (index >= buffer_size) { index = 0; }
}

double MAF::get_variance()
{
	double var = 0;
	for (int i = 0; i < buffer_size; i++) {
		var += buffer[i] - average;
	}
	
	return var;
}

void MAF::set_buffer_size(unsigned short buffer_size)
{
	if (this->buffer_size == buffer_size || buffer_size < 2) {
		return;
	}
	else if (this->buffer_size > buffer_size) {
		double *temp = new double[buffer_size];
		int gap = this->buffer_size - buffer_size;
		for (int i = 0; i < buffer_size; i++) {
			temp[i] = buffer[i + gap];
		}
		delete[] buffer;
		buffer = temp;
		if (index >= buffer_size) { index = 0; }
	}
	else { //this->buffer_size < buffer size
		double *temp = new double[buffer_size];
		for (int i = 0; i < this->buffer_size; i++) {
			temp[i] = buffer[i];
		}
		for (int i = this->buffer_size; i < buffer_size; i++) {
			temp[i] = buffer[this->buffer_size - 1];
		}
		delete[] buffer;
		buffer = temp;
	}
	this->buffer_size = buffer_size;
}

unsigned short MAF::get_buffer_size()
{
	return this->buffer_size;
}

// ! MOVING AVERAGE FILTER


// START LOW PASS FILTER
LPF::LPF(float tau, float sampling_time)
{
	this->tau = tau;
	this->sampling_time = sampling_time;
	this->pre_val = 0;
}

void LPF::set_smapling_time(float sampling_time)
{
	this->sampling_time = sampling_time;
}

void LPF::set_tau(float tau)
{
	this->tau = tau;
}

float LPF::get_tau()
{
	return tau;
}

float LPF::get_sampling_time()
{
	return sampling_time;
}

float LPF::step(float raw_value)
{
	pre_val = (tau * pre_val + sampling_time * raw_value) / (tau + sampling_time);
	return pre_val;
}
float LPF::step(float raw_value,float sampling_t)
{
	pre_val = (tau * pre_val + sampling_t * raw_value) / (tau + sampling_t);
	return pre_val;
}
// ! LOW PASS FILTER

// START HIGH PASS FILTER
HPF::HPF(float tau, float sampling_time)
{
	this->tau = tau;
	this->sampling_time = sampling_time;
	this->pre_val = 0;
	this->pre_raw_val = 0;
}

void HPF::set_smapling_time(float sampling_time)
{
	this->sampling_time = sampling_time;
}

void HPF::set_tau(float tau)
{
	this->tau = tau;
}

float HPF::get_tau()
{
	return tau;
}

float HPF::get_sampling_time()
{
	return sampling_time;
}

float HPF::step(float raw_value)
{
	pre_val= tau / (tau + sampling_time) * pre_val + tau / (tau + sampling_time) * (raw_value - pre_raw_val);
	pre_raw_val = raw_value;
	return pre_val;
}

float HPF::step(float raw_value, float sampling_t)
{
	pre_val = tau / (tau + sampling_t) * pre_val + tau / (tau + sampling_t) * (raw_value - pre_raw_val);
	pre_raw_val = raw_value;
	return pre_val;
}
// ! HIGH PASS FILTER