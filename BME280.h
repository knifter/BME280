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
#define BME280_ADDRESS_DEFAULT              (0x76)
/*=========================================================================*/

class BME280: public TwoWireDevice {
    public:
        BME280(TwoWire &wire, const uint8_t addr = BME280_ADDRESS_DEFAULT) : TwoWireDevice(wire, addr) {};
        BME280(const uint8_t addr = BME280_ADDRESS_DEFAULT) : TwoWireDevice(addr) {};

        bool begin();
        uint8_t readManufacturerId();
        void reset();
        uint8_t getLastError();

        // bool begin(const uint8_t addr = BME280_ADDRESS_DEFAULT) { return TwoWireDevice::begin(addr) && init(); };
        // bool begin(TwoWire *wire, const uint8_t addr = BME280_ADDRESS_DEFAULT) { return TwoWireDevice::begin(wire, addr) && init(); };

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

        void setSampling(sensor_mode mode  = MODE_NORMAL,
              sensor_sampling tempSampling  = SAMPLING_X16,
              sensor_sampling pressSampling = SAMPLING_X16,
              sensor_sampling humSampling   = SAMPLING_X16,
              sensor_filter filter          = FILTER_OFF,
              standby_duration duration     = STANDBY_MS_0_5
              );

        void takeForcedMeasurement();
        float readTemperature(void);
        float readPressure(void);
        float readHumidity(void);

        float readAltitude(float seaLevel);
        float seaLevelForAltitude(float altitude, float pressure);
        float readAltitudeTC(float seaLevel, float atmospheric, float temperature);

    protected:
        bool init();

        void readCoefficients(void);
        bool isReadingCalibration(void);

        uint16_t  readreg16_LE(uint8_t reg);    // little endian
        int16_t   readregS16_LE(uint8_t reg);   // little endian

        /*=========================================================================
        CALIBRATION DATA
        -----------------------------------------------------------------------*/
        typedef struct
        {
            uint16_t dig_T1;
            int16_t  dig_T2;
            int16_t  dig_T3;

            uint16_t dig_P1;
            int16_t  dig_P2;
            int16_t  dig_P3;
            int16_t  dig_P4;
            int16_t  dig_P5;
            int16_t  dig_P6;
            int16_t  dig_P7;
            int16_t  dig_P8;
            int16_t  dig_P9;

            uint8_t  dig_H1;
            int16_t  dig_H2;
            uint8_t  dig_H3;
            int16_t  dig_H4;
            int16_t  dig_H5;
            int8_t   dig_H6;
        } calib_data_t;
        /*=========================================================================*/

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

        int32_t   t_fine;
        calib_data_t _bme280_calib;

    private:
        BME280(const BME280&);
        BME280& operator=(const BME280&);
};

#endif // __BME280_H
