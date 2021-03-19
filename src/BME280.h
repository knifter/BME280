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
#ifndef __BME280_H
#define __BME280_H

#include <Arduino.h>
#include <TwoWireDevice.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
// #define DEBUG
#define BMP280_ADDRESS_DEFAULT              (0x76)
#define BME280_ADDRESS_DEFAULT              (0x76)
/*=========================================================================*/

class BMP280: public TwoWireDevice 
{
    public:
        BMP280(TwoWire &wire, const uint8_t addr = BME280_ADDRESS_DEFAULT) : TwoWireDevice(wire, addr) {};
        BMP280(const uint8_t addr = BME280_ADDRESS_DEFAULT) : TwoWireDevice(addr) {};

        bool begin();
        uint8_t readManufacturerId();
        void reset();
        uint8_t getLastError();

        enum sensor_sampling {
            SAMPLING_NONE = 0b000,
            SAMPLING_X1   = 0b001,
            SAMPLING_X2   = 0b010,
            SAMPLING_X4   = 0b011,
            SAMPLING_X8   = 0b100,
            SAMPLING_X16  = 0b101
        };

        enum sensor_mode {
            MODE_SLEEP  = 0b00,
            MODE_FORCED = 0b01,
            MODE_NORMAL = 0b11
        };

        enum sensor_filter {
            FILTER_OFF = 0b000,
            FILTER_X2  = 0b001,
            FILTER_X4  = 0b010,
            FILTER_X8  = 0b011,
            FILTER_X16 = 0b100
        };

        // standby durations in ms
        enum standby_duration {
            STANDBY_MS_0_5  = 0b000,
            STANDBY_MS_10   = 0b110,
            STANDBY_MS_20   = 0b111,
            STANDBY_MS_62_5 = 0b001,
            STANDBY_MS_125  = 0b010,
            STANDBY_MS_250  = 0b011,
            STANDBY_MS_500  = 0b100,
            STANDBY_MS_1000 = 0b101
        };

        void setSampling(
            sensor_mode mode = MODE_NORMAL,
            sensor_sampling tempSampling = SAMPLING_X16,
            sensor_sampling pressSampling = SAMPLING_X16,
            sensor_filter filter = FILTER_OFF,
            standby_duration duration = STANDBY_MS_0_5
            );

        void takeForcedMeasurement();
        float readTemperature(void);
        float readPressure(void);

        float readAltitude(float seaLevel);
        float seaLevelForAltitude(float altitude, float pressure);
        float readAltitudeTC(float seaLevel, float atmospheric, float temperature);

    protected:
        // Calibration coefficients
        void readCoefficients(void);
        bool isReadingCalibration(void);
        struct
        {
            uint16_t T1;
            int16_t  T2;
            int16_t  T3;
        } _calibt;

        struct
        {
            uint16_t P1;
            int16_t  P2;
            int16_t  P3;
            int16_t  P4;
            int16_t  P5;
            int16_t  P6;
            int16_t  P7;
            int16_t  P8;
            int16_t  P9;
        } _calibp;

        // The config register
        typedef struct {
            // inactive duration (standby time) in normal mode
            // 000 = 0.5 ms
            // 001 = 62.5 ms
            // 010 = 125 ms
            // 011 = 250 ms
            // 100 = 500 ms
            // 101 = 1000 ms
            // 110 = 10 ms
            // 111 = 20 ms
            unsigned int t_sb : 3;

            // filter settings
            // 000 = filter off
            // 001 = 2x filter
            // 010 = 4x filter
            // 011 = 8x filter
            // 100 and above = 16x filter
            unsigned int filter : 3;

            // unused - don't set
            unsigned int none : 1;
            unsigned int spi3w_en : 1;

            unsigned int get() {
                return (t_sb << 5) | (filter << 2) | spi3w_en;
            }
        } config_t;
        config_t _configReg;

        // The ctrl_meas register
        typedef struct {
            // temperature oversampling
            // 000 = skipped
            // 001 = x1
            // 010 = x2
            // 011 = x4
            // 100 = x8
            // 101 and above = x16
            unsigned int osrs_t : 3;

            // pressure oversampling
            // 000 = skipped
            // 001 = x1
            // 010 = x2
            // 011 = x4
            // 100 = x8
            // 101 and above = x16
            unsigned int osrs_p : 3;

            // device mode
            // 00       = sleep
            // 01 or 10 = forced
            // 11       = normal
            unsigned int mode : 2;

            unsigned int get() {
                return (osrs_t << 5) | (osrs_p << 2) | mode;
            }
        } ctrl_meas_t;
        ctrl_meas_t _measReg;

        int32_t _t_fine;

    private:
        BMP280(const BMP280&);
        BMP280& operator=(const BMP280&);
};

class BME280 : public BMP280
{
    public:
        BME280(TwoWire &wire, const uint8_t addr = BME280_ADDRESS_DEFAULT) : BMP280(wire, addr) {};
        BME280(const uint8_t addr = BME280_ADDRESS_DEFAULT) : BMP280(addr) {};

        void setSampling(BMP280::sensor_mode mode = MODE_NORMAL,
              BMP280::sensor_sampling tempSampling = SAMPLING_X16,
              BMP280::sensor_sampling pressSampling = SAMPLING_X16,
              BMP280::sensor_sampling humSampling = SAMPLING_X16,
              BMP280::sensor_filter filter = FILTER_OFF,
              BMP280::standby_duration duration = STANDBY_MS_0_5
              );

        float readHumidity(void);

    private:
        // Calibration co-efficients
        void readCoefficients(void);        
        struct
        {
            uint8_t  H1;
            int16_t  H2;
            uint8_t  H3;
            int16_t  H4;
            int16_t  H5;
            int8_t   H6;
        } _calibh;
        
        // The ctrl_hum register
        typedef struct {
            // unused - don't set
            unsigned int none : 5;

            // pressure oversampling
            // 000 = skipped
            // 001 = x1
            // 010 = x2
            // 011 = x4
            // 100 = x8
            // 101 and above = x16
            unsigned int osrs_h : 3;

            unsigned int get() {
                return (osrs_h);
            }
        } ctrl_hum_t;
        ctrl_hum_t _humReg;    
};

#endif // __BME280_H
