#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include "I2Cdev.h"
#include "MS5611.h"

#define DEFAULT_DEV "/dev/i2c-1"

MS5611::MS5611() : ct(0), uosr(0), TEMP2(0), OFF2(0), SENS2(0)
{
    i2cdev = new I2Cdev(DEFAULT_DEV);
    devAddr = MS5611_ADDRESS;
}
MS5611::MS5611(uint8_t add)
 : ct(0), uosr(0), TEMP2(0), OFF2(0), SENS2(0)
{
    i2cdev = new I2Cdev(DEFAULT_DEV);
    devAddr = add;
}

MS5611::~MS5611()
{
    delete i2cdev;
}

bool MS5611::begin(ms5611_osr_t osr) {
    //reset();
    //setOversampling(osr);
    readPROM();
    return true;
}

// Set oversampling value
void MS5611::setOversampling(ms5611_osr_t osr) {
    switch (osr) {
    case MS5611_ULTRA_LOW_POWER:
        ct = 1;
        break;
    case MS5611_LOW_POWER:
        ct = 2;
        break;
    case MS5611_STANDARD:
        ct = 3;
        break;
    case MS5611_HIGH_RES:
        ct = 5;
        break;
    case MS5611_ULTRA_HIGH_RES:
        ct = 10;
        break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611::getOversampling(void) {
    return (ms5611_osr_t) uosr;
}

void MS5611::reset(void) {
    i2cdev->writeByte(devAddr, MS5611_CMD_RESET);
    usleep(50000);
}

uint16_t MS5611::read16(uint8_t devAddr, uint8_t cmd) {
    uint8_t buff[2];
    i2cdev->readBlock(devAddr, cmd, 2, buff);
    uint16_t rev = (((uint16_t) buff[0]) << 8) | (uint16_t) buff[1];
    return rev;
}

uint32_t MS5611::read24(uint8_t devAddr, uint8_t cmd) {
    uint8_t buff[3];
    i2cdev->readBlock(devAddr, cmd, 3, buff);
    uint32_t rev = (((uint32_t) buff[0]) << 16) | (((uint32_t) buff[1]) << 8)
            | (uint32_t) buff[2];
    return rev;
}

void MS5611::readPROM(void) {
    for (uint8_t offset = 0; offset < 6; offset++) {
        fc[offset] = read16(devAddr, MS5611_CMD_READ_PROM + (offset * 2));
        usleep(50000);
    }
}

uint32_t MS5611::readRawTemperature(void) {
    i2cdev->writeByte(devAddr, 0x58);
    //i2cdev->writeByte(devAddr, MS5611_CMD_CONV_D2 + uosr);
    usleep(30000);
    return read24(devAddr, MS5611_CMD_ADC_READ);
}

uint32_t MS5611::readRawPressure(void) {
    i2cdev->writeByte(devAddr, 0x48);
    //i2cdev->writeByte(devAddr, MS5611_CMD_CONV_D1 + uosr);
    usleep(30000);
    return read24(devAddr, MS5611_CMD_ADC_READ);
}

int32_t MS5611::readPressure(bool compensation) {
    uint32_t D1 = readRawPressure();

    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t) fc[4] * 256;

    int64_t OFF = (int64_t) fc[1] * 65536 + (int64_t) fc[3] * dT / 128;
    int64_t SENS = (int64_t) fc[0] * 32768 + (int64_t) fc[2] * dT / 256;

    if (compensation) {
        int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

        OFF2 = 0;
        SENS2 = 0;

        if (TEMP < 2000) {
            OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
            SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
        }

        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
        }

        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double MS5611::readTemperature(bool compensation) {
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t) fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation) {
        if (TEMP < 2000) {
            TEMP2 = (dT * dT) / (2 << 30);
        }
    }

    TEMP = TEMP - TEMP2;

    return ((double) TEMP / 100);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::getAltitude(double pressure, double seaLevelPressure) {
    return (44330.0f
            * (1.0f
                    - pow((double) pressure / (double) seaLevelPressure,
                            0.1902949f)));
}

double MS5611::getAltitude(double temperature, double pressure, double seaLevelPressure) {
	return ((pow((seaLevelPressure / pressure), 0.1902949f) - 1.0) * (temperature + 273.15)) / 0.0065f;
}

// Calculate sea level from Pressure given on specific altitude
double MS5611::getSeaLevel(double pressure, double altitude) {
    return ((double) pressure
            / pow(1.0f - ((double) altitude / 44330.0f), 5.255f));
}

