#ifndef RPIGY86_H_
#define RPIGY86_H_

#include <stdint.h>


#define DEFAULT_SEAPRESSURE 1013.5
#include "../filters.h"
#include "../../singleton.hpp"

class MPU6050;
class HMC5883L;
class MS5611;
using namespace std;
class RPIGY86 : public singleton<RPIGY86>{

public:
    RPIGY86();
    ~RPIGY86();
	float getHeading();
	float getAltitude();
	float getRateClimb();

	float getPressure();
	float getTemperature();

private:


    /**
     * initialize MPU6050, HMC5883L and MS6511
     */
    void initialize();

    void getMotion6();
    void getMotion9();
    void setAccelXOffset(int32_t offset);
    void setAccelYOffset(int32_t offset);
    void setAccelZOffset(int32_t offset);
    void setGryoXOffset(int32_t offset);
    void setGryoYOffset(int32_t offset);
    void setGryoZOffset(int32_t offset);
    void setMagXOffset(int32_t offset);
    void setMagYOffset(int32_t offset);
    void setMagZOffset(int32_t offset);
    void setGryoRangeScale(int32_t scale);
    void getGryoRangeScale();
    void setAccelRangeScale(int32_t scale);
    void getAccelRangeScale();
    void calibrateMPU6050();
    void measure(int* m_ax, int* m_ay, int* m_az, int* m_gx, int* m_gy, int* m_gz);
    void getHeadingXYZ();
   // void getHeading();
    void setMagGain(int32_t gain);
    void getMagGain();

	unsigned int timer_climb;
	double seaLevelPressure, pressure, temperature, altitude, prev_altitude, rate_climb;
	MAF *maf_pressure, *maf_temperature, *maf_altitude;
	simpleKfilter *skalman_altitude;
	
	MPU6050* mpu6050;
    HMC5883L* hmc5883l;
    MS5611* ms5611;
};

#endif /* RPIGY86_H_ */
