#include <stdio.h>
//#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <wiringPi.h>

#include "RPIGY86.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"

#include <iostream>
using namespace std;

#define usleep(x) delayMicroseconds(x)

#define FUNCTION_TEMPLATE_CLASS "RPiGY86"


//not able to directly set these offset into HMC5883L
//therefore, keep a local offsets here.
int32_t gMagXOffset = 0, gMagYOffset = 0, gMagZOffset = 0;

float gMagGainTable[] = { 0.73f, 0.92f, 1.22f, 1.52f, 2.27f, 2.56f, 3.03f, 4.35f };

//GYRO_FS_250 : 0,  //  131 LSB/°/s
//GYRO_FS_500 : 1,  //  65.5 LSB/°/s
//GYRO_FS_1000: 2,  //  32.8 LSB/°/s
//GYRO_FS_2000: 3 };//  16.4 LSB/°/s
float gGryoScaleTable[] = { 131, 65.5f, 32.8f, 16.4f };

//FS_2   : 0,  //  16384 LSB/g
//FS_4   : 1,  //   8192 LSB/g
//FS_8   : 2,  //   4096 LSB/g
//FS_16  : 3 } //   2048 LSB/g
int gAccelScaleTable[] = { 16384, 8192, 4096, 2048 };




RPIGY86::RPIGY86()
    : mpu6050(nullptr), hmc5883l(nullptr), ms5611(nullptr)
{
    initialize();
}

void RPIGY86::initialize()
{
    mpu6050 = new MPU6050();
    hmc5883l = new HMC5883L();
    ms5611 = new MS5611();
	mpu6050->initialize();
	mpu6050->setI2CBypassEnabled(true);
	hmc5883l->initialize();
	ms5611->begin();

	seaLevelPressure = DEFAULT_SEAPRESSURE;

	float tem_press = ms5611->readPressure()*0.01;
	while(tem_press == 0){ tem_press=ms5611->readPressure()*0.01; }
	float tem_temperature = ms5611->readTemperature();
	float temp_alt = (((pow((seaLevelPressure / tem_press), 0.1902949f) - 1.0) * (tem_temperature + 273.15)) / 0.0065f
		);

	
	maf_altitude = new MAF(temp_alt, 5);
	maf_pressure = new MAF(tem_press, 5);

	maf_temperature = new MAF(tem_temperature, 5);
	skalman_altitude = new simpleKfilter(1, 0, 0.1, 0.05, 0.05);

   
}

RPIGY86::~RPIGY86()
{
    delete mpu6050;
    delete hmc5883l;
    delete ms5611;
}

void RPIGY86::getMotion6()
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu6050->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

void RPIGY86::getMotion9()
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    mpu6050->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    hmc5883l->getHeading(&mx, &my, &mz);
}

void
RPIGY86::setGryoXOffset(int32_t offset)
{
    mpu6050->setXGyroOffset(offset);
}

void
RPIGY86::setGryoYOffset(int32_t offset)
{
    mpu6050->setYGyroOffset(offset);
}

void
RPIGY86::setGryoZOffset(int32_t offset)
{
    mpu6050->setZGyroOffset(offset);
}

void
RPIGY86::setAccelXOffset(int32_t offset)
{
    mpu6050->setXAccelOffset(offset);
}

void
RPIGY86::setAccelYOffset(int32_t offset)
{
    mpu6050->setYAccelOffset(offset);
}

void
RPIGY86::setAccelZOffset(int32_t offset)
{
    mpu6050->setZAccelOffset(offset);
}

void
RPIGY86::setGryoRangeScale(int32_t scale)
{
    mpu6050->setFullScaleGyroRange(scale);
}

void
RPIGY86::getGryoRangeScale()
{
    uint8_t scale = mpu6050->getFullScaleGyroRange();
}

void
RPIGY86::setAccelRangeScale(int32_t scale)
{
    mpu6050->setFullScaleAccelRange(scale);
}

void
RPIGY86::getAccelRangeScale()
{
    uint8_t scale = mpu6050->getFullScaleAccelRange();
}

void
RPIGY86::setMagXOffset(int32_t offset)
{
    gMagXOffset = offset;
}

void
RPIGY86::setMagYOffset(int32_t offset)
{
    gMagYOffset = offset;
}

void
RPIGY86::setMagZOffset(int32_t offset)
{
    gMagZOffset = offset;
}

void
RPIGY86::setMagGain(int32_t gain)
{
    hmc5883l->setGain(gain);
}

void
RPIGY86::getMagGain()
{
    uint8_t gain = hmc5883l->getGain();
}

void RPIGY86::measure(int* m_ax, int* m_ay, int* m_az, int* m_gx, int* m_gy,
        int* m_gz)
{
    int i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0,
            buff_gz = 0;
    int16_t ax=0, ay=0, az=0, gx=0, gy=0,gz=0;
    static int buffersize = 1000;

    while (i < (buffersize + 101)) {
        // read raw accel/gyro measurements from device
        mpu6050->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
            buff_ax += ax;
            buff_ay += ay;
            buff_az += az;
            buff_gx += gx;
            buff_gy += gy;
            buff_gz += gz;
        }
        i++;
        usleep(2); //Needed so we don't get repeated measures
    }

    *m_ax = buff_ax / buffersize;
    *m_ay = buff_ay / buffersize;
    *m_az = buff_az / buffersize;
    *m_gx = buff_gx / buffersize;
    *m_gy = buff_gy / buffersize;
    *m_gz = buff_gz / buffersize;
    printf("mean:%d, %d, %d, %d, %d, %d\n", *m_ax, *m_ay, *m_az, *m_gx,
            *m_gy, *m_gz);
}

void
RPIGY86::calibrateMPU6050()
{
    static int acel_deadzone=8;
    static int giro_deadzone=1;
    int ax_offset=0, ay_offset=0, az_offset=0,
         gx_offset=0, gy_offset=0, gz_offset=0;
    int mean_ax=0, mean_ay=0, mean_az=0, mean_gx=0, mean_gy=0, mean_gz=0;
    mpu6050->setXAccelOffset(ax_offset);
    mpu6050->setYAccelOffset(ay_offset);
    mpu6050->setZAccelOffset(az_offset);
    mpu6050->setXGyroOffset(gx_offset);
    mpu6050->setYGyroOffset(gy_offset);
    mpu6050->setZGyroOffset(gz_offset);
    usleep(1000);

    measure(&mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
    usleep(1000);
    ax_offset=-mean_ax/8;
    ay_offset=-mean_ay/8;
    az_offset=(16384-mean_az)/8;
    gx_offset=-mean_gx/4;
    gy_offset=-mean_gy/4;
    gz_offset=-mean_gz/4;

    while (1){
        int ready=0;
        mpu6050->setXAccelOffset(ax_offset);
        mpu6050->setYAccelOffset(ay_offset);
        mpu6050->setZAccelOffset(az_offset);
        mpu6050->setXGyroOffset(gx_offset);
        mpu6050->setYGyroOffset(gy_offset);
        mpu6050->setZGyroOffset(gz_offset);

        measure(&mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);

        if (std::abs(mean_ax)<=acel_deadzone) ready++;
        else ax_offset=ax_offset-mean_ax/acel_deadzone;

        if (std::abs(mean_ay)<=acel_deadzone) ready++;
        else ay_offset=ay_offset-mean_ay/acel_deadzone;

        if (std::abs(16384-mean_az)<=acel_deadzone) ready++;
        else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

        if (std::abs(mean_gx)<=giro_deadzone) ready++;
        else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

        if (std::abs(mean_gy)<=giro_deadzone) ready++;
        else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

        if (std::abs(mean_gz)<=giro_deadzone) ready++;
        else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

        if (ready==6) break;
        printf("ready:%d, %d, %d, %d, %d, %d, %d\n", ready, ax_offset, ay_offset,
           az_offset, gx_offset, gy_offset, gz_offset);
    }
}

void
RPIGY86::getHeadingXYZ()
{
    int16_t mx, my, mz;

    hmc5883l->getHeading(&mx, &my, &mz);
}

float RPIGY86::getHeading()
{
    int16_t mx, my, mz;

	//http://www.magnetic-declination.com/
    //todo: get this value from somewhere
	//Latitude: 36° 47' 14.3" N
	//Longitude: 127° 12' 18.7" E
	//Magnetic declination : -8° 8' 
    static float declination = -0.1373;
    uint8_t gain = hmc5883l->getGain();
    float scale = gMagGainTable[gain];
    hmc5883l->getHeading(&mx, &my, &mz);

    float myf = (my - gMagYOffset) * scale;
    float mxf = (mx - gMagXOffset) * scale;
    float heading = atan2(myf, mxf);
    if(heading < 0)
      heading += 2 * M_PI;
    heading = heading * 180/M_PI + declination;
    if ( heading < 0 ) {
        heading += 360;
    }
	return heading;
}


float RPIGY86::getAltitude() {

	getPressure();
	getTemperature();
	altitude = maf_altitude->step(skalman_altitude->step(
		((pow((seaLevelPressure / pressure), 0.1902949f) - 1.0) * (temperature + 273.15)) / 0.0065f
	));
	//unsigned int time_gap = millis() - timer_climb;
	//if(time_gap >0 ){
	//	rate_climb = (altitude - prev_altitude) / (1000 / time_gap);
	//	timer_climb = millis();
	//	prev_altitude = altitude;
	//}
	
	return (float)altitude;;
}
float RPIGY86::getPressure() {
	pressure = maf_pressure->step(ms5611->readPressure()*0.01);
	return (float)pressure;
}

float RPIGY86::getTemperature() {

	temperature = maf_temperature->step(ms5611->readTemperature());
	return (float)temperature;
}	

//float RPIGY86::getRateClimb() {
//	return (float)rate_climb;
//}