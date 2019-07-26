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
    #define CHIPID_IS           0x60
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

uint16_t from_LE(uint16_t BE)
{
    return (BE >> 8) | (BE << 8);
};

int16_t from_SLE(uint8_t UBE)
{
    return (int16_t) from_LE(UBE);
};

/**************************************************************************/
/*!
    @brief  Initialise sensor with given parameters / settings
*/
bool BME280::begin()
{
    TwoWireDevice::begin();

    // check if sensor, i.e. the chip ID is correct
    if(readManufacturerId() != CHIPID_IS)
        return false;

    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    reset();

    // wait for chip to wake up.
    delay(300); //TODO: needed?

    // if chip is still reading calibration, delay
    while (isReadingCalibration())
        delay(50);

    // read trimming parameters, see DS 4.2.2
    readCoefficients();

    // use defaults
    setSampling();

    delay(300); // TODO: needed?

    return true;
}

uint8_t BME280::readManufacturerId()
{
    return readreg8(REG_CHIPID);
}

void BME280::reset()
{
    writereg8(REG_SOFTRESET, SOFTRESET_RESET);
    return;
}

/**************************************************************************/
/*!
    @brief  setup sensor with given parameters / settings

    This is simply a overload to the normal begin()-function, so SPI users
    don't get confused about the library requiring an address.
*/
/**************************************************************************/
void BME280::setSampling(sensor_mode       mode,
		 sensor_sampling   tempSampling,
		 sensor_sampling   pressSampling,
		 sensor_sampling   humSampling,
		 sensor_filter     filter,
		 standby_duration  duration)
{
    _configReg.filter = filter;
    _configReg.t_sb   = duration;
    writereg8(REG_CONFIG, _configReg.get());

    _humReg.osrs_h    = humSampling;
    writereg8(REG_CONTROLHUMID, _humReg.get());

    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    _measReg.mode     = mode;
    _measReg.osrs_t   = tempSampling;
    _measReg.osrs_p   = pressSampling;
    writereg8(REG_CONTROL, _measReg.get());
}

/**************************************************************************/
/*!
    @brief  Take a new measurement (only possible in forced mode)
*/
/**************************************************************************/
void BME280::takeForcedMeasurement()
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
    }
}


/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void BME280::readCoefficients(void)
{
    _bme280_calib.dig_T1 = from_LE(readreg16(REG_DIG_T1));
    _bme280_calib.dig_T2 = from_SLE(readreg16(REG_DIG_T2));
    _bme280_calib.dig_T3 = from_SLE(readreg16(REG_DIG_T3));
    // _bme280_calib.dig_T1 = readreg16(REG_DIG_T1);
    // _bme280_calib.dig_T2 = readS16(REG_DIG_T2);
    // _bme280_calib.dig_T3 = readS16(REG_DIG_T3);

    _bme280_calib.dig_P1 = from_LE(readreg16(REG_DIG_P1));
    _bme280_calib.dig_P2 = from_SLE(readreg16(REG_DIG_P2));
    _bme280_calib.dig_P3 = from_SLE(readreg16(REG_DIG_P3));
    _bme280_calib.dig_P4 = from_SLE(readreg16(REG_DIG_P4));
    _bme280_calib.dig_P5 = from_SLE(readreg16(REG_DIG_P5));
    _bme280_calib.dig_P6 = from_SLE(readreg16(REG_DIG_P6));
    _bme280_calib.dig_P7 = from_SLE(readreg16(REG_DIG_P7));
    _bme280_calib.dig_P8 = from_SLE(readreg16(REG_DIG_P8));
    _bme280_calib.dig_P9 = from_SLE(readreg16(REG_DIG_P9));

    _bme280_calib.dig_H1 = readreg8(REG_DIG_H1);
    _bme280_calib.dig_H2 = from_SLE(readreg16(REG_DIG_H2));
    _bme280_calib.dig_H3 = readreg8(REG_DIG_H3);
    _bme280_calib.dig_H4 = (readreg8(REG_DIG_H4) << 4) | (readreg8(REG_DIG_H4+1) & 0xF);
    _bme280_calib.dig_H5 = (readreg8(REG_DIG_H5+1) << 4) | (readreg8(REG_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t)readreg8(REG_DIG_H6);
}

/**************************************************************************/
/*!
    @brief return true if chip is busy reading cal data
*/
/**************************************************************************/
bool BME280::isReadingCalibration(void)
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

float BME280::readTemperature(void)
{
    int32_t var1, var2;

    int32_t adc_T = readreg24(REG_TEMPDATA);
    if (adc_T == 0x800000) // value in case temp measurement was disabled
        return NAN;
    adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)_bme280_calib.dig_T1 <<1))) *
            ((int32_t)_bme280_calib.dig_T2)) >> 11;

    var2 = (((((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1)) *
              ((adc_T>>4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
            ((int32_t)_bme280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T/100;
}


/**************************************************************************/
/*!
    @brief  Returns the temperature from the sensor
*/
/**************************************************************************/
float BME280::readPressure(void) {
    int64_t var1, var2, p;

    readTemperature(); // must be done first to get t_fine

    int32_t adc_P = readreg24(REG_PRESSUREDATA);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)_bme280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)_bme280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3)>>8) +
           ((var1 * (int64_t)_bme280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bme280_calib.dig_P1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)_bme280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7)<<4);
    return (float)p/256;
}


/**************************************************************************/
/*!
    @brief  Returns the humidity from the sensor
*/
/**************************************************************************/
float BME280::readHumidity(void)
{
    readTemperature(); // must be done first to get t_fine

    int32_t adc_H = readreg16(REG_HUMIDDATA);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;

    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                    (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)_bme280_calib.dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h = (v_x1_u32r>>12);
    return  h / 1024.0;
}


/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float BME280::readAltitude(float seaLevel)
{
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    float atmospheric = readPressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!  Driver55
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa)
	with Temperature Compensation.
	(see https://keisan.casio.com/exec/system/1224585971)
	NOTE:
	If the altitude is more than 11km high above sea level,
	the hypsometric formula cannot be applied because the temperature
	lapse rate varies considerably with altitude.
     @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
	@param  temperature   temperature in Celsius
*/
/**************************************************************************/
float BME280::readAltitudeTC(float seaLevel, float atmospheric, float temperature)
{
    // Equation taken from https://keisan.casio.com/exec/system/1224585971
    return ( (pow(seaLevel / atmospheric, 0.1902225) - 1.0) * (temperature + 273.15) ) / 0.0065;
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).
    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float BME280::seaLevelForAltitude(float altitude, float atmospheric)
{
    // Equation taken from BMP180 datasheet (page 17):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude. See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}
