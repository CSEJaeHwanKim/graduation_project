#ifndef MS5611_h
#define MS5611_h

#include "I2Cdev.h"

#define MS5611_ADDRESS                (0x77)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

typedef enum {
    MS5611_ULTRA_HIGH_RES = 0x08,
    MS5611_HIGH_RES = 0x06,
    MS5611_STANDARD = 0x04,
    MS5611_LOW_POWER = 0x02,
    MS5611_ULTRA_LOW_POWER = 0x00
} ms5611_osr_t;

class MS5611 {
public:
    MS5611();
    MS5611(uint8_t add);
    ~MS5611();

    bool begin(ms5611_osr_t osr = MS5611_HIGH_RES);
    uint32_t readRawTemperature(void);
    uint32_t readRawPressure(void);
    double readTemperature(bool compensation = false);
    int32_t readPressure(bool compensation = false);
    double getAltitude(double pressure, double seaLevelPressure = 1013.25);
	double getAltitude(double temperature, double pressure, double seaLevelPressure = 1013.25);
    double getSeaLevel(double pressure, double altitude);
    void setOversampling(ms5611_osr_t osr);
    ms5611_osr_t getOversampling(void);

private:
    uint16_t read16(uint8_t devAddr, uint8_t cmd);
    uint32_t read24(uint8_t devAddr, uint8_t cmd);

    I2Cdev* i2cdev;
    uint16_t fc[6];
    uint8_t ct;
    uint8_t uosr;
    int32_t TEMP2;
    int64_t OFF2, SENS2;

    void reset(void);
    void readPROM(void);

    uint8_t devAddr;
};

#endif
