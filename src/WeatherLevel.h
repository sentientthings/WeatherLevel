/*
  WeatherLevel.h - Weather and Level adapter library
  Copyright (c) 2019 Sentient Things, Inc.  All right reserved.
  Based on work by Rob Purser, Mathworks, Inc.

  Version 0.1.2
  Bug fixes
  Version 0.1.1
  Changes Include:
  1. Using DS18 library for DS18B20
  Version 0.1.0 is not backwards compatible with the previous versions!
  Changes include:
  1. The addition of the Maxbotix class that works with one or two Maxbotix sensors
  2. The removal of ThingSpeak related functions to separate sensor readings from cloud storage
  See complete diff for details.
*/



// ensure this library description is only included once
#ifndef WeatherLevel_h
#define WeatherLevel_h

// include core Particle library
#include "Particle.h"

// include description files for other libraries used (if any)
#include "DS18.h"
#include <Adafruit_AM2315.h>
#include <SparkFun_MPL3115A2.h>
#include <RunningMedianST.h>
#include "IoTNode.h"
#include <Adafruit_TSL2591.h>
#include <Adafruit_Sensor.h>

// Updated for Oct 2018 v2 Weather and Level board
#define ONE_WIRE_BUS N_D4

#define MAX_SEL GIOF

// library interface description
class Weather
{
  // user-accessible "public" interface
  public:
    // Weather() : oneWire(ONE_WIRE_BUS), ds18b20(&oneWire), airTempKMedian(30), relativeHumidtyMedian(30)
    Weather() : airTempKMedian(30), relativeHumidtyMedian(30)
    {
      pinMode(AnemometerPin, INPUT_PULLUP);
      attachInterrupt(AnemometerPin, &Weather::handleAnemometerEvent, this, FALLING);

      pinMode(RainPin, INPUT_PULLUP);
      attachInterrupt(RainPin, &Weather::handleRainEvent, this, FALLING);
    }
    void handleAnemometerEvent() {
      // Activated by the magnet in the anemometer (2 ticks per rotation)
       unsigned int timeAnemometerEvent = millis(); // grab current time

      //If there's never been an event before (first time through), then just capture it
      if(lastAnemoneterEvent != 0) {
          // Calculate time since last event
          unsigned int period = timeAnemometerEvent - lastAnemoneterEvent;
          // ignore switch-bounce glitches less than 10mS after initial edge (which implies a max windspeed of 149mph)
          if(period < 10) {
            return;
          }
          if(period < GustPeriod) {
              // If the period is the shortest (and therefore fastest windspeed) seen, capture it
              GustPeriod = period;
          }
          AnemoneterPeriodTotal += period;
          AnemoneterPeriodReadingCount++;
      }

      lastAnemoneterEvent = timeAnemometerEvent; // set up for next event
    }

    void handleRainEvent() {
      // Count rain gauge bucket tips as they occur
      // Activated by the magnet and reed switch in the rain gauge, attached to input D2
      unsigned int timeRainEvent = millis(); // grab current time

      // ignore switch-bounce glitches less than 10mS after initial edge
      if(timeRainEvent - lastRainEvent < 10) {
        return;
      }
      rainEventCount++; //Increase this minute's amount of rain
      lastRainEvent = timeRainEvent; // set up for next event
    }

    // float getWaterTempC(void);
    // int16_t getWaterTempRAW(void);
    void begin(void);
    float readPressure();

    float getAndResetAnemometerMPH(float * gustMPH);
    float getAndResetRainInches();

    void captureWindVane();
    void captureTempHumidityPressure();
    void captureAirTempHumid();
    void captureWaterTemp();
    void capturePressure();
    void captureLightLux();
    void captureBatteryVoltage();

    float getAndResetWindVaneDegrees();
    float getAndResetTempF();
    float getAndResetHumidityRH();
    float getAndResetPressurePascals();
    float getAndResetWaterTempF();
    uint16_t getAndResetLightLux();
    uint16_t getAndResetBatteryMV();

    uint16_t getAirTempKMedian();
    uint16_t getRHMedian();
    
    String dsType();

    void printDebugInfo();

  // library-accessible "private" interface
  private:
    // OneWire oneWire;
    // DS18B20 ds18b20;
    Adafruit_AM2315 am2315;
    MPL3115A2 barom;
    RunningMedianInt32 airTempKMedian;
    RunningMedianInt32 relativeHumidtyMedian;
    Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
    String minimiseNumericString(String ss, int n);


    // Updated for Oct 2018 v2 Weather and Level board
    int RainPin = N_D2;
    volatile unsigned int rainEventCount;
    unsigned int lastRainEvent;
    float RainScaleInches = 0.011; // Each pulse is .011 inches of rain

    // Updated for Oct 2018 v2 Weather and Level board
    const int AnemometerPin = N_D1;
    float AnemometerScaleMPH = 1.492; // Windspeed if we got a pulse every second (i.e. 1Hz)
    volatile unsigned int AnemoneterPeriodTotal = 0;
    volatile unsigned int AnemoneterPeriodReadingCount = 0;
    volatile unsigned int GustPeriod = UINT_MAX;
    unsigned int lastAnemoneterEvent = 0;

    //int WindVanePin = A0;
    int WindVanePin = N_A2;
    float windVaneCosTotal = 0.0;
    float windVaneSinTotal = 0.0;
    unsigned int windVaneReadingCount = 0;

    float humidityRHTotal = 0.0;
    unsigned int humidityRHReadingCount = 0;
    float tempFTotal = 0.0;
    unsigned int tempFReadingCount = 0;
    float pressurePascalsTotal = 0.0;
    unsigned int pressurePascalsReadingCount = 0;
    float waterTempFTotal = 0.0;
    unsigned int waterTempFReadingCount = 0;

    float lookupRadiansFromRaw(unsigned int analogRaw);
    //Light
    uint16_t lightLux = 0;
    unsigned int lightLuxTotal = 0;
    unsigned int lightLuxCount = 0;
    //Battery Voltage
    float batVoltage = 0;
    float batVoltageTotal = 0;
    unsigned int batVoltageCount = 0;
};

// library interface description
class Maxbotix
{
  // user-accessible "public" interface
  public:
    /**
     * @brief Construct a new Maxbotix object
     * 
     * Note that the IoTNode library uses Fram and the library also
     * keeps track of memory allocation of arrays in the Fram object.
     * Passing this Fram object through to the library allows the
     * memory mapping of the arrays to work correctly.
     * 
     * @param node typically passed through from the ino
     * @param size is the number readings for the median
     */
    Maxbotix(IoTNode& node, const uint16_t size);

    /**
     * @brief Run during the main program setup to configure the Fram
     * 
     * @return int 
     */
    int setup();

    /**
     * @brief Run in the loop to check for new readings from
     * the Maxbotix sensors (one or two)
     * 
     */
    void readMaxbotixCharacter();

    /**
     * @brief The range reading of the Maxbotix sensor
     * connected to the Range 1 port on the Sentient
     * Things Weather and Level adapter
     * 
     * @return int the range in mm
     */
    int range1Median();

    /**
     * @brief The range reading of the Maxbotix sensor
     * connected to the Range 2 port on the Sentient
     * Things Weather and Level adapter
     * 
     * @return int the range in mm
     */
    int range2Median();

    /**
     * @brief toggle between sensor 1 and sensor 2
     * 
     */
    void toggle();

    /**
     * @brief Switch to sensor 1
     * 
     */
    void sensor1On();

    /**
     * @brief Switch to sensor 2
     * 
     */
    void sensor2On();

    /**
     * @brief Check if sensor 1 is on
     * 
     * @return true 
     * @return false 
     */
    bool isSensor1On();

    /**
     * @brief Check if sensor 2 is on
     * 
     * @return true 
     * @return false 
     */
    bool isSensor2On();

    /**
     * @brief Check availability of reading
     * from sensor 1 
     * 
     * @return true 
     * @return false 
     */
    bool isSensor1Available();

    /**
     * @brief Check availability of reading
     * from sensor 2 
     * 
     * @return true 
     * @return false 
     */    
    bool isSensor2Available();

    /**
     * @brief Run to calibrate the difference
     * in range reading between sensor 1 and sensor 2.
     * Run this when the sensors are mounted and pointing at the water
     * or snow.  The result is saved in Fram.
     * isDualSensorCalibrated() will return true when done.
     * 
     * @return int 
     */
    int calibrateDualSensorOffSet();
    
    /**
     * @brief Returns the range from sensor 1
     * after verifying that the difference range from sensor 2
     * is calib.dualSensorOffset -+30mm.
     * This is used to verify good readings during snow storm
     * events or cases where insects or other object affect
     * sensor readings.  Requires both sensors to provide
     * valid readings.
     * 
     * @return int The range from sensor 1 in mm
     */
    int range1Dual();

    /**
     * @brief Empties the Serial1 buffer
     * 
     */
    void emptySerial1Chars();

    /**
     * @brief Clears the dual sensor
     * calibration from Fram
     * 
     */
    void clearDualSensorCalibration();

    /**
     * @brief Checks for dual sensor calibration
     * 
     * @return true 
     * @return false 
     */
    bool isDualSensorCalibrated();

    /**
     * @brief Sensor 1 model number
     * MBXXXX
     * The value is XXXX
     * 
     */
    int sensor1ModelNum = 0;

    /**
     * @brief Sensor 2 model number
     * MBXXXX
     * The value is XXXX
     * 
     */    
    int sensor2ModelNum = 0;
    bool dualSensor;

    typedef struct
    {
      int dualSensorOffset;
      bool calibrated;
    }calib_t;

    calib_t calib;

  // library-accessible "private" interface
  private:
    IoTNode _node;
    uint32_t _size;
    RunningMedianInt32 max1MedianRunning;
    RunningMedianInt32 max2MedianRunning;

    framArray framCalib;

    int sensor1TimeOutCount = 0;
    int sensor2TimeOutCount = 0;
    int value;
    uint8_t serial1BufferIndex = 0;
    bool readingRange = false;
    char serial1Buf[6];
    uint32_t rangeBegin;
    bool maxSelect;
    
    uint32_t readingCount;
    uint32_t sensor1RangeMin;
    uint32_t sensor1RangeMax;
    uint32_t sensor1Timeout;
    uint32_t sensor1Scale;
    bool isSensor1TempComp;
    uint32_t sensor2RangeMin;
    uint32_t sensor2RangeMax;
    uint32_t sensor2Timeout;
    uint32_t sensor2Scale;
    bool isSensor2TempComp;
    bool sensor1Available=false;
    bool sensor2Available=false;
    uint32_t maxReadTime = 0;
};

#endif
