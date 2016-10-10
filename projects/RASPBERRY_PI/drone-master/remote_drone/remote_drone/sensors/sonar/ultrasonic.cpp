#include "ultrasonic.h"
//#include <Windows.h>



UltrasonicClass::UltrasonicClass() {
#if MEASUREMENT_MODE==INTERNAL
	this->echo = ECHO_PIN;	// 24
	this->trig = TRIG_PIN;	// 23
	this->is_measuring = false;

#elif MEASUREMENT_MODE==EXTERNAL
	cout << "UltrasonicClass()" << endl;
	this->MAF_distance = new MAF(5, 20);
	this->MAF_velocity = new MAF(0, 10);
	prev_distance = 5;
	vel_index = 0;
#endif




	this->distance = 5;
	this->acceleration = 0;
	this->velocity = 0;


#if SONAR_DEBUG==DEBUG_ON
	debug_output.open("ultrasonic.txt");
	debug_output << "law value\tkalman value" << endl;
	//debug_output << "law value\tkalman value\nvelocity\tvelocity kalman\tacc" << endl;
#endif
}

UltrasonicClass::~UltrasonicClass() {
#if SONAR_DEBUG==DEBUG_ON
	debug_output.close();
#endif

}

#if MEASUREMENT_MODE==INTERNAL

void UltrasonicClass::start() {
	wiringPiSetup();

	if (!this->is_measuring) {
		thread tt([&]() { starter(); });
		// tt.join();
		tt.detach();
	}

}

void UltrasonicClass::starter()
{
	char buffer_size = 10;
	float buffer[10] = { 0,1,2,3,4 };
	int index = 0;
	bool stop_flag = false;
	while (true) {
		if (!this->is_measuring) {
			thread tt([&]() { measure_distance(); });
			// tt.join();
			tt.detach();
			delay(1000);
		}
		buffer[index++] = this->distance;
		if (index >= buffer_size) {
			index = 0;
		}
		stop_flag = true;
		for (int i = 1; i < buffer_size; i++) {
			if (buffer[0] != buffer[i]) { stop_flag = false; break; }
		}
		if (stop_flag) {
			stop_flag = false;
			is_measuring = false;

		}

		delay(50);
	}
}

void UltrasonicClass::measure_distance() {

	this->is_measuring = true;


	int start_time, end_time, travel_time, prev_m_time = 0, time_gap;
	float prev_velocity = 0;
	float prev_distance = 0;
	float temp_distance, temp_velocity, temp_acceleration;
	float raw_value = 0;
	int acc_limit = 3;
	float maf_temp = 0;
	simpleKfilter distance_kfilter, variation_kfilter, tempKfilter,mix_kalman;
	LPF distanceLPF(0.1, 0.01),lpf_01_01(0.1,0.1),lpf_001_01(0.01,0.1),lpf_001_001(0.01,0.01),mix_lpf(0.1,0.01);
	MAF velocityMAF(0.0, 20U), distanceMAF(distance, 80U),mix_maf(distance,50U);

	int velocity_sampling_time = 100000; // 0.01 s
	int velocity_ref_distance = distance;
	int velocity_prevtime = 0;

	printf("measure_distance start\n");
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
	digitalWrite(trig, LOW);
	try
	{
		while (is_measuring) {
			digitalWrite(trig, HIGH);
			delayMicroseconds(100);
			digitalWrite(trig, LOW);
			while (digitalRead(echo) == 0);
			start_time = micros();
			while (digitalRead(echo) == 1);
			end_time = micros();
			travel_time = (end_time - start_time);
			//travel_time  = 

			//micro 70분 넘으면 0으로 초기화됨.
			//17400  = 300cm
			if (travel_time < 0 || travel_time > 17400) { continue; }
			raw_value = (travel_time / 58.);
			
			time_gap = end_time - prev_m_time;
			maf_temp = distanceMAF.nonsave_step(raw_value);
			//temp_distance = distance_kfilter.step(distanceLPF.step(maf_temp)); //distance
																			   //temp_distance = distanceMAF.nonsave_step(raw_value); //distance
			temp_velocity = velocityMAF.step((maf_temp - prev_distance) * (10000.0 / time_gap)); //velocity
			temp_acceleration = (temp_velocity - prev_velocity) * (10000.0 / time_gap); //acceleration
																						//cout << temp_distance <<", "<< (travel_time / 58.) << ", " << travel_time << endl;
			float temp_raw = maf_temp;
			p_raw_value = raw_value;
			p_raw_filter = prev_distance;
			if (temp_acceleration < acc_limit && temp_acceleration > -acc_limit)
			{
				p_raw_filter = raw_value;
				this->distance = maf_temp;
				distanceMAF.save_value(raw_value, maf_temp);
				//this->distance = distance_kfilter.step(distanceLPF.step((travel_time / 29. / 2.)));
				// 1000000 / travel_time <- cm/s , 10000 / travel_time <- m/s
				//this->velocity = temp_velocity;
				if (end_time - velocity_prevtime > velocity_sampling_time) {
					//cout << "prev distance : " << velocity_ref_distance << " cur distance : " << distance << " gap : " << (end_time - velocity_prevtime) << endl;
					this->velocity = (distance - velocity_ref_distance) / (100000.0 / (end_time - velocity_prevtime));
					velocity_prevtime = end_time;
					velocity_ref_distance = distance;
				}
				this->acceleration = (this->velocity - prev_velocity) * (10000.0 / travel_time);
				temp_raw = raw_value;
			}
#if SONAR_DEBUG==DEBUG_ON
			/*debug_output << raw_value << ",";
			debug_output << distance << ",";
			debug_output << distanceLPF.step(temp_raw) << ",";
			debug_output << lpf_01_01.step(temp_raw) << ",";
			debug_output << lpf_001_01.step(temp_raw) << ",";
			debug_output << lpf_001_001.step(temp_raw) << ",";
			debug_output << distance_kfilter.step(temp_raw) << ",";
			debug_output << mix_maf.step(mix_kalman.step(mix_lpf.step(temp_raw))) << endl;*/
#endif
			//printf("distance : %.2lf , velocity : %.2lfm/s , acc : %.2lfm/s \n", distance, velocity, acceleration);
			//printf("%.2lf %.2lf\n", this->distance, prev_distance);
			if (temp_acceleration < acc_limit && temp_acceleration > -acc_limit)
			{
				prev_velocity = temp_velocity;
				prev_distance = this->distance;
				prev_m_time = start_time;
			}
			delayMicroseconds(1000);
		}
	}
	catch (const std::exception&)
	{
		this->is_measuring = false;
	}


	this->is_measuring = false;
}




#elif MEASUREMENT_MODE==EXTERNAL
void UltrasonicClass::input_values(double raw_distance, unsigned int timestamp) {
	this->distance = MAF_distance->step(raw_distance);
	if (vel_index++ < 2) {
		this->velocity = MAF_velocity->step((distance - prev_distance) / (10000.0 / (timestamp - prev_timestamp)));
		prev_distance = distance;
		prev_timestamp = timestamp;
		vel_index = 0;
	}
#if SONAR_DEBUG==DEBUG_ON
	//debug_output << raw_distance << ",";
	//	debug_output << velocity << ",";

	debug_output << this->distance << endl;
	//debug_output << (this->distance - prev_distance) * (10000.0 / travel_time) << "\t";
	//debug_output << this->velocity << "\t";
	//debug_output << this->acceleration << endl;
#endif
}

void UltrasonicClass::start() {}

#endif

float UltrasonicClass::getDistance() {
	return this->distance;
}

float UltrasonicClass::getVelocity()
{
	return this->velocity;
}

float UltrasonicClass::getAcceleration()
{
	return this->acceleration;
}

