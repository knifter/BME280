/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include "BME280.h"

#define REG_DIG_T1              0x88
#define REG_DIG_T2              0x8A
#define REG_DIG_T3              0x8C
#define REG_DIG_P1              0x8E
#define REG_DIG_P2              0x90
#define REG_DIG_P3              0x92
#define REG_DIG_P4              0x94
#define REG_DIG_P5              0x96
#define REG_DIG_P6              0x98
#define REG_DIG_P7              0x9A
#define REG_DIG_P8              0x9C
#define REG_DIG_P9              0x9E
#define REG_DIG_H1              0xA1
#define REG_DIG_H2              0xE1
#define REG_DIG_H3              0xE3
#define REG_DIG_H4              0xE4
#define REG_DIG_H5              0xE5
#define REG_DIG_H6              0xE7
#define REG_CHIPID              0xD0
    #define CHIPID_BMP          0x58
    #define CHIPID_BME          0x60
// #define REG_VERSION             0xD1
#define REG_SOFTRESET           0xE0
    #define SOFTRESET_RESET     0xB6
#define REG_CONTROLHUMID        0xF2
#define REG_STATUS              0XF3
    #define STATUS_UPDATE       0x01
    #define STATUS_MEASURING    0x08
#define REG_CONTROL             0xF4
#define REG_CONFIG              0xF5
#define REG_PRESSUREDATA        0xF7
#define REG_TEMPDATA            0xFA
#define REG_HUMIDDATA           0xFD

/**************************************************************************/
/*!
    @brief  Initialise sensor with given parameters / settings
*/
bool BMP280::begin()
{
    if(!TwoWireDevice::begin())
    {
#ifdef DEBUG
		Serial.printf("BMP280 Error: wire.begin() failed.\n");
#endif
        return false;
    };

    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    reset();
    if(_last_error)
    {
#ifdef DEBUG
		Serial.printf("BMP280 Error: reset() failed.\n");
#endif
        return false;
    };

    // check if sensor, i.e. the chip ID is correct
    uint8_t chipid = readManufacturerId();
    if(chipid != CHIPID_BMP && chipid != CHIPID_BME)
    {
#ifdef DEBUG
		Serial.printf("BMP280 Error: invalid manuf. id = %x != %x or %x\n", readManufacturerId(), CHIPID_BMP, CHIPID_BME);
#endif
        return false;
    };

    // if chip is still reading calibration, delay
    while (isReadingCalibration())
        delay(50);

    // read trimming parameters, see DS 4.2.2
    readCoefficients();

    return true;
};

uint8_t BMP280::readManufacturerId()
{
    return readreg8(REG_CHIPID);
};

void BMP280::reset()
{
    writereg8(REG_SOFTRESET, SOFTRESET_RESET);
    return;
};

/**************************************************************************/
/*!
    @brief  setup sensor with given parameters / settings

    This is simply a overload to the normal begin()-function, so SPI users
    don't get confused about the library requiring an address.
*/
/**************************************************************************/
void BMP280::setSampling(
        sensor_mode       mode,
        sensor_sampling   tempSampling,
        sensor_sampling   pressSampling,
        sensor_filter     filter,
        standby_duration  duration)
{
    _configReg.filter = filter;
    _configReg.t_sb   = duration;
    writereg8(REG_CONFIG, _configReg.get());

    _measReg.mode     = mode;
    _measReg.osrs_t   = tempSampling;
    _measReg.osrs_p   = pressSampling;
    writereg8(REG_CONTROL, _measReg.get());
};

void BME280::setSampling(
        BMP280::sensor_mode       mode,
        BMP280::sensor_sampling   tempSampling,
        BMP280::sensor_sampling   pressSampling,
        BMP280::sensor_sampling   humSampling,
        BMP280::sensor_filter     filter,
        BMP280::standby_duration  duration)
{
    _humReg.osrs_h    = humSampling;
    writereg8(REG_CONTROLHUMID, _humReg.get());
    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    BMP280::setSampling(mode, tempSampling, pressSampling, filter, duration);
};

/**************************************************************************/
/*!
    @brief  Take a new measurement (only possible in forced mode)
*/
/**************************************************************************/
void BMP280::takeForcedMeasurement()
{
    // If we are in forced mode, the BME sensor goes back to sleep after each
    // measurement and we need to set it to forced mode once at this point, so
    // it will take the next measurement and then return to sleep again.
    // In normal mode simply does new measurements periodically.
    if (_measReg.mode == MODE_FORCED) 
    {
        // set to forced mode, i.e. "take next measurement"
        writereg8(REG_CONTROL, _measReg.get());

        // wait until measurement has been completed, otherwise we would read
        // the values from the last measurement
        while (readreg8(REG_STATUS) & STATUS_MEASURING)
		    delay(1);
    };
};


/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void BMP280::readCoefficients(void)
{
    _calibt.T1 = readreg16_LM(REG_DIG_T1);
    _calibt.T2 = readreg16_LM(REG_DIG_T2);
    _calibt.T3 = readreg16_LM(REG_DIG_T3);

    _calibp.P1 = readreg16_LM(REG_DIG_P1);
    _calibp.P2 = readreg16_LM(REG_DIG_P2);
    _calibp.P3 = readreg16_LM(REG_DIG_P3);
    _calibp.P4 = readreg16_LM(REG_DIG_P4);
    _calibp.P5 = readreg16_LM(REG_DIG_P5);
    _calibp.P6 = readreg16_LM(REG_DIG_P6);
    _calibp.P7 = readreg16_LM(REG_DIG_P7);
    _calibp.P8 = readreg16_LM(REG_DIG_P8);
    _calibp.P9 = readreg16_LM(REG_DIG_P9);
};

void BME280::readCoefficients(void)
{
    BMP280::readCoefficients();

    _calibh.H1 = readreg8(REG_DIG_H1);
    _calibh.H2 = readreg16_LM(REG_DIG_H2);
    _calibh.H3 = readreg8(REG_DIG_H3);
    _calibh.H4 = (readreg8(REG_DIG_H4) << 4) | (readreg8(REG_DIG_H4+1) & 0xF);
    _calibh.H5 = (readreg8(REG_DIG_H5+1) << 4) | (readreg8(REG_DIG_H5) >> 4);
    _calibh.H6 = readreg8(REG_DIG_H6);
};

/**************************************************************************/
/*!
    @brief return true if chip is busy reading cal data
*/
/**************************************************************************/
bool BMP280::isReadingCalibration(void)
{
  return (readreg8(REG_STATUS) & STATUS_UPDATE);
}

/**************************************************************************/
/*!
    @brief  Returns the temperature from the sensor
*/
/**************************************************************************/
//Datasheet code:
// 	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// 	// t_fine carries fine temperature as global value
//
// 	//get the reading (adc_T);
//     uint8_t buffer[3];
// 	readRegisterRegion(buffer, BME280_TEMPERATURE_MSB_REG, 3);
//     int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
//
// 	//By datasheet, calibrate
// 	int64_t var1, var2;
//
// 	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
// 	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
// 	((int32_t)calibration.dig_T3)) >> 14;
// 	t_fine = var1 + var2;
// 	float output = (t_fine * 5 + 128) >> 8;
//
// 	output = output / 100;
//
// 	return output;
// }

float BMP280::readTemperature(void)
{
    int32_t adc_T = readreg24_ML(REG_TEMPDATA);
    
    if (adc_T == 0x800000) // value in case temp measurement was disabled
        return NAN;
    adc_T >>= 4;

    int32_t var1, var2;
    var1 = ((((adc_T>>3) - ((int32_t)_calibt.T1 <<1))) *
            ((int32_t)_calibt.T2)) >> 11;

    var2 = (((((adc_T>>4) - ((int32_t)_calibt.T1)) *
              ((adc_T>>4) - ((int32_t)_calibt.T1))) >> 12) *
            ((int32_t)_calibt.T3)) >> 14;

    _t_fine = var1 + var2;

    float T = (_t_fine * 5 + 128) >> 8;
    return T/100;
};

/**************************************************************************/
/*!
    @brief  Returns the temperature from the sensor
*/
/**************************************************************************/
float BMP280::readPressure(void) 
{
    //TODO: Combine functions? check _t_fine set?
    readTemperature(); // must be done first to get t_fine

    int32_t adc_P = readreg24_ML(REG_PRESSUREDATA);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;
    adc_P >>= 4;

    // TODO: Do this (partially) in advance?
    int64_t var1, var2;
    var1 = ((int64_t)_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_calibp.P6;
    var2 = var2 + ((var1*(int64_t)_calibp.P5)<<17);
    var2 = var2 + (((int64_t)_calibp.P4)<<35);
    var1 = ((var1 * var1 * (int64_t)_calibp.P3)>>8) +
           ((var1 * (int64_t)_calibp.P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_calibp.P1)>>33;

    if (var1 == 0) {
        return NAN; // avoid exception caused by division by zero
    }
    int64_t p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)_calibp.P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)_calibp.P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_calibp.P7)<<4);
    return (float)p/256;
};

/**************************************************************************/
/*!
    @brief  Returns the humidity from the sensor
*/
/**************************************************************************/
float BME280::readHumidity(void)
{
    readTemperature(); // must be done first to get t_fine

    int32_t adc_H = readreg16_ML(REG_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;

    int32_t var = (_t_fine - ((int32_t)76800));

    var = (((((adc_H << 14) - (((int32_t)_calibh.H4) << 20) -
                    (((int32_t)_calibh.H5) * var)) + ((int32_t)16384)) >> 15) *
                 (((((((var * ((int32_t)_calibh.H6)) >> 10) *
                      (((var * ((int32_t)_calibh.H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)_calibh.H2) + 8192) >> 14));

    var = (var - (((((var >> 15) * (var >> 15)) >> 7) *
                               ((int32_t)_calibh.H1)) >> 4));

    var = (var < 0) ? 0 : var;
    var = (var > 419430400) ? 419430400 : var;
    float h = (var>>12);
    return  h / 1024.0;
};
