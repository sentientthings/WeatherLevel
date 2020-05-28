/*
  WeatherLevel.h - Weather and Level adapter library
  Copyright (c) 2019 Sentient Things, Inc.  All right reserved.
*/



// include this library's description file
#include "WeatherLevel.h"

#include <math.h>

DS18 dstemp(ONE_WIRE_BUS);

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

/*Weather::Weather()
{
  // initialize this instance's variables


  // do whatever is required to initialize the library

}
*/
// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries
void Weather::begin(void)
{
  AnemoneterPeriodTotal = 0;
  AnemoneterPeriodReadingCount = 0;
  GustPeriod = UINT_MAX;  //  The shortest period (and therefore fastest gust) observed
  lastAnemoneterEvent = 0;

  barom.begin();
  barom.setModeBarometer();
  barom.setOversampleRate(7);
  barom.enableEventFlags();

  am2315.begin();

//   ds18b20.begin();
//   ds18b20.setResolution(11);
//   ds18b20.requestTemperatures();
  
  if (tsl.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
    
  /* Display some basic information on this sensor */
  sensor_t sensor;
  tsl.getSensor(&sensor);
  delay(500);
  
  /* Configure the sensor */
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)


//   tsl2591Gain_t gain = tsl.getGain();
}

float  Weather::getAndResetAnemometerMPH(float * gustMPH)
{
    if(AnemoneterPeriodReadingCount == 0)
    {
        *gustMPH = 0.0;
        return 0;
    }
    // Nonintuitive math:  We've collected the sum of the observed periods between pulses, and the number of observations.
    // Now, we calculate the average period (sum / number of readings), take the inverse and muliple by 1000 to give frequency, and then mulitply by our scale to get MPH.
    // The math below is transformed to maximize accuracy by doing all muliplications BEFORE dividing.
    float result = AnemometerScaleMPH * 1000.0 * float(AnemoneterPeriodReadingCount) / float(AnemoneterPeriodTotal);
    AnemoneterPeriodTotal = 0;
    AnemoneterPeriodReadingCount = 0;
    *gustMPH = AnemometerScaleMPH  * 1000.0 / float(GustPeriod);
    GustPeriod = UINT_MAX;
    return result;
}

void Weather::captureBatteryVoltage()
{
  unsigned int rawVoltage = 0;
  Wire.requestFrom(0x4D, 2);
  if (Wire.available() == 2)
  {
    rawVoltage = (Wire.read() << 8) | (Wire.read());
    batVoltageTotal += (float)(rawVoltage)/4096.0*13.64; // 3.3*(4.7+1.5)/1.5
    batVoltageCount ++;
  }  
}

uint16_t Weather::getAndResetBatteryMV()
{
 uint16_t result = (uint16_t) 1000*(batVoltageTotal/batVoltageCount);
 batVoltageTotal = 0;
 batVoltageCount = 0;
 return result;
}

void Weather::captureLightLux()
{
  sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  // Serial.print(F("[ ")); Serial.print(event.timestamp); Serial.print(F(" ms ] "));
  if ((event.light == 0) |
      (event.light > 4294966000.0) | 
      (event.light <-4294966000.0))
  {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    Serial.println(F("Invalid data (adjust gain or timing)"));
  }
  else
  {
    lightLuxTotal += (unsigned int)event.light;
    lightLuxCount ++;
    // Serial.println(lightLuxTotal);
    // Serial.println(lightLuxCount);    
    // Serial.print(event.light); Serial.println(F(" lux"));
  }

}

uint16_t Weather::getAndResetLightLux()
{
  if (lightLuxTotal)
  {
    uint16_t result = (uint16_t)(lightLuxTotal/lightLuxCount);
    lightLuxTotal=0;
    lightLuxCount=0;
    return result;
  }
  else
  {
    lightLuxTotal=0;
    lightLuxCount=0;
    return 0;
  } 
}

float Weather::getAndResetRainInches()
{
    float result = RainScaleInches * float(rainEventCount);
    rainEventCount = 0;
    return result;
}
/// Wind Vane
void Weather::captureWindVane() {
    // Read the wind vane, and update the running average of the two components of the vector
    unsigned int windVaneRaw = analogRead(WindVanePin);
    //Serial.println(windVaneRaw);
    float windVaneRadians = lookupRadiansFromRaw(windVaneRaw);
    //Serial.println(windVaneRadians);
    if(windVaneRadians > 0 && windVaneRadians < 6.14159)
    {
        windVaneCosTotal += cos(windVaneRadians);
        windVaneSinTotal += sin(windVaneRadians);
        windVaneReadingCount++;
    }
    return;
}

float Weather::getAndResetWindVaneDegrees()
{
    if(windVaneReadingCount == 0) {
        return 0;
    }
    float avgCos = windVaneCosTotal/float(windVaneReadingCount);
    float avgSin = windVaneSinTotal/float(windVaneReadingCount);
    float result = atan(avgSin/avgCos) * 180.0 / 3.14159;
    windVaneCosTotal = 0.0;
    windVaneSinTotal = 0.0;
    windVaneReadingCount = 0;
    // atan can only tell where the angle is within 180 degrees.  Need to look at cos to tell which half of circle we're in
    if(avgCos < 0) result += 180.0;
    // atan will return negative angles in the NW quadrant -- push those into positive space.
    if(result < 0) result += 360.0;

   return result;
}

float Weather::lookupRadiansFromRaw(unsigned int analogRaw)
{
    //Serial.println(analogRaw);
    // The mechanism for reading the weathervane isn't arbitrary, but effectively, we just need to look up which of the 16 positions we're in.
    if(analogRaw >= 2200 && analogRaw < 2400) return (3.14);//South
    if(analogRaw >= 2100 && analogRaw < 2200) return (3.53);//SSW
    if(analogRaw >= 3200 && analogRaw < 3299) return (3.93);//SW
    if(analogRaw >= 3100 && analogRaw < 3200) return (4.32);//WSW
    if(analogRaw >= 3890 && analogRaw < 3999) return (4.71);//West
    if(analogRaw >= 3700 && analogRaw < 3780) return (5.11);//WNW
    if(analogRaw >= 3780 && analogRaw < 3890) return (5.50);//NW
    if(analogRaw >= 3400 && analogRaw < 3500) return (5.89);//NNW
    if(analogRaw >= 3570 && analogRaw < 3700) return (0.00);//North
    if(analogRaw >= 2600 && analogRaw < 2700) return (0.39);//NNE
    if(analogRaw >= 2750 && analogRaw < 2850) return (0.79);//NE
    if(analogRaw >= 1510 && analogRaw < 1580) return (1.18);//ENE
    if(analogRaw >= 1580 && analogRaw < 1650) return (1.57);//East
    if(analogRaw >= 1470 && analogRaw < 1510) return (1.96);//ESE
    if(analogRaw >= 1900 && analogRaw < 2000) return (2.36);//SE
    if(analogRaw >= 1700 && analogRaw < 1750) return (2.74);//SSE
    if(analogRaw > 4000) return(-1); // Open circuit?  Probably means the sensor is not connected
   // Particle.publish("error", String::format("Got %d from Windvane.",analogRaw), 60 , PRIVATE);
    return -1;
}

/// end Wind vane

void Weather::captureTempHumidityPressure() {
  // Read the humidity and pressure sensors, and update the running average
  // The running (mean) average is maintained by keeping a running sum of the observations,
  // and a count of the number of observations

  // Measure Relative Humidity and temperature from the AM2315
  float humidityRH, tempC, tempF;
  bool validTH = am2315.readTemperatureAndHumidity(tempC, humidityRH);

  uint16_t tempKx10 = uint16_t(tempC*10)+2732;
  airTempKMedian.add(tempKx10);

  relativeHumidtyMedian.add(humidityRH);

if (validTH){
    //If the result is reasonable, add it to the running mean
    if(humidityRH > 0 && humidityRH < 105) // It's theoretically possible to get supersaturation humidity levels over 100%
    {
        // Add the observation to the running sum, and increment the number of observations
        humidityRHTotal += humidityRH;
        humidityRHReadingCount++;
    }


    tempF = (tempC * 9.0) / 5.0 + 32.0;
    //If the result is reasonable, add it to the running mean
    if(tempF > -50 && tempF < 150)
    {
        // Add the observation to the running sum, and increment the number of observations
        tempFTotal += tempF;
        tempFReadingCount++;
    }
  }
  //Measure Pressure from the MPL3115A2
  float pressurePascals = barom.readPressure();

  //If the result is reasonable, add it to the running mean
  // What's reasonable? http://findanswers.noaa.gov/noaa.answers/consumer/kbdetail.asp?kbid=544
  if(pressurePascals > 80000 && pressurePascals < 110000)
  {
      // Add the observation to the running sum, and increment the number of observations
      pressurePascalsTotal += pressurePascals;
      pressurePascalsReadingCount++;
  }
}

void Weather::captureAirTempHumid() {
  // Read the humidity and pressure sensors, and update the running average
  // The running (mean) average is maintained by keeping a running sum of the observations,
  // and a count of the number of observations

  // Measure Relative Humidity and temperature from the AM2315
  float humidityRH, tempC, tempF;
  bool validTH = am2315.readTemperatureAndHumidity(tempC, humidityRH);

  uint16_t tempKx10 = uint16_t(tempC*10.0+2732.0);
  airTempKMedian.add(tempKx10);

  relativeHumidtyMedian.add(humidityRH);

  if (validTH){
      //If the result is reasonable, add it to the running mean
      if(humidityRH > 0 && humidityRH < 105) // It's theoretically possible to get supersaturation humidity levels over 100%
      {
          // Add the observation to the running sum, and increment the number of observations
          humidityRHTotal += humidityRH;
          humidityRHReadingCount++;
      }


      tempF = (tempC * 9.0) / 5.0 + 32.0;
      //If the result is reasonable, add it to the running mean
      if(tempF > -50 && tempF < 150)
      {
          // Add the observation to the running sum, and increment the number of observations
          tempFTotal += tempF;
          tempFReadingCount++;
      }
    }
}

void Weather::captureWaterTemp() {
//  Measure water temperature from the DS18B20

    if (dstemp.read())
    {
        float waterTempF = dstemp.fahrenheit();
        waterTempFTotal += waterTempF;
        waterTempFReadingCount++;
        // Serial.println(waterTempF);
        // printDebugInfo();
    }
    else
    {
        // Serial.println("Unable to read water temp");
        // printDebugInfo();
    }
    

}

void Weather::capturePressure() {
  //Measure Pressure from the MPL3115A2
  float pressurePascals = barom.readPressure();

  //If the result is reasonable, add it to the running mean
  // What's reasonable? http://findanswers.noaa.gov/noaa.answers/consumer/kbdetail.asp?kbid=544
  if(pressurePascals > 80000 && pressurePascals < 110000)
  {
      // Add the observation to the running sum, and increment the number of observations
      pressurePascalsTotal += pressurePascals;
      pressurePascalsReadingCount++;
  }
}

float Weather::getAndResetTempF()
{
    if(tempFReadingCount == 0) {
        return 0;
    }
    float result = tempFTotal/float(tempFReadingCount);
    tempFTotal = 0.0;
    tempFReadingCount = 0;
    return result;
}

float Weather::getAndResetHumidityRH()
{
    if(humidityRHReadingCount == 0) {
        return 0;
    }
    float result = humidityRHTotal/float(humidityRHReadingCount);
    humidityRHTotal = 0.0;
    humidityRHReadingCount = 0;
    return result;
}


float Weather::getAndResetPressurePascals()
{
    if(pressurePascalsReadingCount == 0) {
        return 0;
    }
    float result = pressurePascalsTotal/float(pressurePascalsReadingCount);
    pressurePascalsTotal = 0.0;
    pressurePascalsReadingCount = 0;
    return result;
}

float Weather::getAndResetWaterTempF()
{
    if(waterTempFReadingCount == 0) {
        return 0;
    }
    float result = waterTempFTotal/float(waterTempFReadingCount);
    waterTempFTotal = 0.0;
    waterTempFReadingCount = 0;
    return result;
}

uint16_t Weather::getAirTempKMedian()
{
  uint16_t airKMedian = airTempKMedian.getMedian();
  return airKMedian;
}

uint16_t Weather::getRHMedian()
{
  uint16_t RHMedian = relativeHumidtyMedian.getMedian();
  return RHMedian;
}


float Weather::readPressure()
{
  return barom.readPressure();
}

String Weather::dsType()
{
      // Print the sensor type
  const char *type;
  switch(dstemp.type()) {
    case WIRE_DS1820: type = "DS1820"; break;
    case WIRE_DS18B20: type = "DS18B20"; break;
    case WIRE_DS1822: type = "DS1822"; break;
    case WIRE_DS2438: type = "DS2438"; break;
    default: type = "UNKNOWN"; break;
  }
  return type;
}


void Weather::printDebugInfo() {
  // If there's an electrical error on the 1-Wire bus you'll get a CRC error
  // Just ignore the temperature measurement and try again
  if (dstemp.crcError()) {
    Serial.print("CRC Error ");
  }

  // Print the sensor type
  const char *type;
  switch(dstemp.type()) {
    case WIRE_DS1820: type = "DS1820"; break;
    case WIRE_DS18B20: type = "DS18B20"; break;
    case WIRE_DS1822: type = "DS1822"; break;
    case WIRE_DS2438: type = "DS2438"; break;
    default: type = "UNKNOWN"; break;
  }
  Serial.print(type);

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  dstemp.addr(addr);
  Serial.printf(
    " ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]
  );

  // Print the raw sensor data
  uint8_t data[9];
  dstemp.data(data);
  Serial.printf(
    " data=%02X%02X%02X%02X%02X%02X%02X%02X%02X",
    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]
  );
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

//https://stackoverflow.com/questions/277772/avoid-trailing-zeroes-in-printf
String Weather::minimiseNumericString(String ss, int n) {
    int str_len = ss.length() + 1;
    char s[str_len];
    ss.toCharArray(s, str_len);

//Serial.println(s);
    char *p;
    int count;

    p = strchr (s,'.');         // Find decimal point, if any.
    if (p != NULL) {
        count = n;              // Adjust for more or less decimals.
        while (count >= 0) {    // Maximum decimals allowed.
             count--;
             if (*p == '\0')    // If there's less than desired.
                 break;
             p++;               // Next character.
        }

        *p-- = '\0';            // Truncate string.
        while (*p == '0')       // Remove trailing zeros.
            *p-- = '\0';

        if (*p == '.') {        // If all decimals were zeros, remove ".".
            *p = '\0';
        }
    }
    return String(s);
}

/// Maxbotix

// include description files for other libraries used (if any)


// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

Maxbotix::Maxbotix(IoTNode& node, const uint16_t size): 
            _node(node), _size(size), max1MedianRunning(size), max2MedianRunning(size),
            framCalib(node.makeFramArray(1, sizeof(calib)))
{
  // initialize this instance's variables
    Serial1.begin(9600);
    Serial1.blockOnOverrun(false);
    pinMode(MAX_SEL, OUTPUT);
    
    maxSelect = LOW;
    readingCount = 0;
    // Select maxbotix 1 on startup by default
    digitalWrite(MAX_SEL, maxSelect);
}

int Maxbotix::setup()
{
    Serial1.begin(9600);
    framCalib.read(0, (uint8_t*)&calib);
    _node.powerOFF(EXT5V);
    delay(1000);
    sensor1On();
    _node.powerON(EXT3V3);
    _node.powerON(EXT5V);
    char target[] = "MB";
    char *ptr = target;
    bool findModel = Serial1.find(ptr);
    if (!findModel) return 0;

    int device1 = Serial1.parseInt();
    if (!(device1>=1000 && device1<=10000))
    {
       return 0; 
    }
    else
    {
        sensor1ModelNum = device1;
    }
    bool knownDevice = false;
        // Set up sensor1's parameters
    if (device1 == 7138 || device1 == 7139)
        {
        //20,350,6.7,No,cm
        sensor1Scale = 10;
        sensor1RangeMin = 20*sensor1Scale;
        sensor1RangeMax = 350*sensor1Scale;
        sensor1Timeout = 1000+1000*1/6.7;       
        isSensor1TempComp = false;
        knownDevice = true;
        
        }
    else if (device1 == 7052)
    {
        //20,765,6.7,No,cm        
        sensor1Scale = 10;
        sensor1RangeMin = 20*sensor1Scale;
        sensor1RangeMax = 765*sensor1Scale;
        sensor1Timeout = 1000+1000*1/6.7;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7060 || device1 == 7062 || device1 == 7070 || device1 == 7072 || device1 == 7092)
    {
        //20,765,10,No,cm
        sensor1Scale = 10;
        sensor1RangeMin = 20*sensor1Scale;
        sensor1RangeMax = 765*sensor1Scale;
        sensor1Timeout = 1000+1000*1/10;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7066 || device1 == 7076)
    {
        //20,1068,10,No,cm
        sensor1Scale = 10;
        sensor1RangeMin = 20*sensor1Scale;
        sensor1RangeMax = 1068*sensor1Scale;
        sensor1Timeout = 1000+1000*1/10;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7051 || device1 == 7053)
    {
        //25,1041,5.1,No,cm
        sensor1Scale = 10;
        sensor1RangeMin = 25*sensor1Scale;
        sensor1RangeMax = 1041*sensor1Scale;
        sensor1Timeout = 1000+1000*1/5.1;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7560 || device1 == 7569 || device1 == 7580 || device1 == 7589)
    {
        //300,5000,0.6,Yes,mm
        sensor1Scale = 1;
        sensor1RangeMin = 300;
        sensor1RangeMax = 5000;
        sensor1Timeout = 1000+1000*1/0.6;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7369 || device1 == 7389)
    {
        //300,5000,6.7,Yes,mm
        sensor1Scale = 1;
        sensor1RangeMin = 300;
        sensor1RangeMax = 5000;
        sensor1Timeout = 1000+1000*1/6.7;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7360)
    {
        //300,5000,7.5,No,mm
        sensor1Scale = 1;
        sensor1RangeMin = 300;
        sensor1RangeMax = 5000;
        sensor1Timeout = 1000+1000*1/6.7;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7380)
    {
        //300,5000,7.5,Yes,mm
        sensor1Scale = 1;
        sensor1RangeMin = 300;
        sensor1RangeMax = 5000;
        sensor1Timeout = 1000+1000*1/7.5;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7368)
    {
        //300,9999,5.2,Yes,mm
        sensor1Scale = 1;
        sensor1RangeMin = 300;
        sensor1RangeMax = 9999;
        sensor1Timeout = 1000+1000*1/5.2;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7375 || device1 == 7395)
    {
        //500,1525,6.33,Yes,mm
        sensor1Scale = 1;
        sensor1RangeMin = 500;
        sensor1RangeMax = 1525;
        sensor1Timeout = 1000+1000*1/6.33;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }  
    else if (device1 == 7534 || device1 == 7544 || device1 == 7554 || device1 == 7564 || device1 == 7574 || device1 == 7584)
    {
        //300,5000,0.6,Yes,mm
        sensor1Scale = 1;
        sensor1RangeMin = 500;
        sensor1RangeMax = 5000;
        sensor1Timeout = 1000+1000*1/0.6;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7334 || device1 == 7344 || device1 == 7354 || device1 == 7364 || device1 == 7374 || device1 == 7384)
    {
        //300,5000,6,Yes,mm
        sensor1Scale = 1;
        sensor1RangeMin = 500;
        sensor1RangeMax = 5000;
        sensor1Timeout = 1000+1000*1/6;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7563 || device1 == 7566 || device1 == 7568 || device1 == 7583 || device1 == 7586 || device1 == 7588)
    {
        //300,9999,0.6,Yes,mm
       sensor1Scale = 1;
        sensor1RangeMin = 500;
        sensor1RangeMax = 9999;
        sensor1Timeout = 1000+1000*1/0.6;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    else if (device1 == 7363 || device1 == 7366 || device1 == 7383 || device1 == 7386 || device1 == 7388)
    {
        //300,9999,6,Yes,mm
        sensor1Scale = 1;
        sensor1RangeMin = 300;
        sensor1RangeMax = 9999;
        sensor1Timeout = 1000+1000*1/6;       
        isSensor1TempComp = false;
        knownDevice = true;
        
    }
    if (!knownDevice) return 0;

    // Serial.println(deviceModel);

    _node.powerOFF(EXT5V);
    delay(1000);
    sensor2On();
    _node.powerON(EXT5V);
    Serial1.find(ptr);
    int device2 = Serial1.parseInt();
    if (!(device2>=1000 && device2<=10000))
    {
       sensor2ModelNum = -1;
       dualSensor = false;
       sensor1On();
       return 1; 
    }
    else
    {
        sensor2ModelNum = device2;
    }

    knownDevice = false;
        // Set up sensor2's parameters
    if (device2 == 7138 || device2 == 7139)
        {
        //20,350,6.7,No,cm
        sensor2Scale = 10;
        sensor2RangeMin = 20*sensor2Scale;
        sensor2RangeMax = 350*sensor2Scale;
        sensor2Timeout = 1000+1000*1/6.7;       
        isSensor2TempComp = false;
        knownDevice = true;
        
        }
    else if (device2 == 7052)
    {
        //20,765,6.7,No,cm        
        sensor2Scale = 10;
        sensor2RangeMin = 20*sensor2Scale;
        sensor2RangeMax = 765*sensor2Scale;
        sensor2Timeout = 1000+1000*1/6.7;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7060 || device2 == 7062 || device2 == 7070 || device2 == 7072 || device2 == 7092)
    {
        //20,765,10,No,cm
        sensor2Scale = 10;
        sensor2RangeMin = 20*sensor2Scale;
        sensor2RangeMax = 765*sensor2Scale;
        sensor2Timeout = 1000+1000*1/10;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7066 || device2 == 7076)
    {
        //20,1068,10,No,cm
        sensor2Scale = 10;
        sensor2RangeMin = 20*sensor2Scale;
        sensor2RangeMax = 1068*sensor2Scale;
        sensor2Timeout = 1000+1000*1/10;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7051 || device2 == 7053)
    {
        //25,1041,5.1,No,cm
        sensor2Scale = 10;
        sensor2RangeMin = 25*sensor2Scale;
        sensor2RangeMax = 1041*sensor2Scale;
        sensor2Timeout = 1000+1000*1/5.1;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7560 || device2 == 7569 || device2 == 7580 || device2 == 7589)
    {
        //300,5000,0.6,Yes,mm
        sensor2Scale = 1;
        sensor2RangeMin = 300;
        sensor2RangeMax = 5000;
        sensor2Timeout = 1000+1000*1/0.6;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7369 || device2 == 7389)
    {
        //300,5000,6.7,Yes,mm
        sensor2Scale = 1;
        sensor2RangeMin = 300;
        sensor2RangeMax = 5000;
        sensor2Timeout = 1000+1000*1/6.7;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7360)
    {
        //300,5000,7.5,No,mm
        sensor2Scale = 1;
        sensor2RangeMin = 300;
        sensor2RangeMax = 5000;
        sensor2Timeout = 1000+1000*1/6.7;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7380)
    {
        //300,5000,7.5,Yes,mm
        sensor2Scale = 1;
        sensor2RangeMin = 300;
        sensor2RangeMax = 5000;
        sensor2Timeout = 1000+1000*1/7.5;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7368)
    {
        //300,9999,5.2,Yes,mm
        sensor2Scale = 1;
        sensor2RangeMin = 300;
        sensor2RangeMax = 9999;
        sensor2Timeout = 1000+1000*1/5.2;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7375 || device2 == 7395)
    {
        //500,1525,6.33,Yes,mm
        sensor2Scale = 1;
        sensor2RangeMin = 500;
        sensor2RangeMax = 1525;
        sensor2Timeout = 1000+1000*1/6.33;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }  
    else if (device2 == 7534 || device2 == 7544 || device2 == 7554 || device2 == 7564 || device2 == 7574 || device2 == 7584)
    {
        //300,5000,0.6,Yes,mm
        sensor2Scale = 1;
        sensor2RangeMin = 500;
        sensor2RangeMax = 5000;
        sensor2Timeout = 1000+1000*1/0.6;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7334 || device2 == 7344 || device2 == 7354 || device2 == 7364 || device2 == 7374 || device2 == 7384)
    {
        //300,5000,6,Yes,mm
        sensor2Scale = 1;
        sensor2RangeMin = 500;
        sensor2RangeMax = 5000;
        sensor2Timeout = 1000+1000*1/6;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7563 || device2 == 7566 || device2 == 7568 || device2 == 7583 || device2 == 7586 || device2 == 7588)
    {
        //300,9999,0.6,Yes,mm
       sensor2Scale = 1;
        sensor2RangeMin = 500;
        sensor2RangeMax = 9999;
        sensor2Timeout = 1000+1000*1/0.6;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }
    else if (device2 == 7363 || device2 == 7366 || device2 == 7383 || device2 == 7386 || device2 == 7388)
    {
        //300,9999,6,Yes,mm
        sensor2Scale = 1;
        sensor2RangeMin = 300;
        sensor2RangeMax = 9999;
        sensor2Timeout = 1000+1000*1/6;       
        isSensor2TempComp = false;
        knownDevice = true;
        
    }

    if (!knownDevice)
    {
        dualSensor = false;
        sensor1On();
        sensor2ModelNum = -1;
        return 1;
    }
    else
    {
        dualSensor = true;
        sensor2ModelNum = device2;
        return 2;
    }
    
}


void Maxbotix::readMaxbotixCharacter()
{
    uint32_t millisnow = millis();

    // if (millisnow-rangeBegin>max(sensor1Timeout,sensor2Timeout))
    if (millisnow-rangeBegin>2000)
    {
        // readings have timed out so reset
        // Serial.println(millisnow);
        // Serial.println(rangeBegin);
        // Serial.println(max(sensor1Timeout,sensor2Timeout));
        readingRange = false;
        serial1BufferIndex = 0;
        memset(serial1Buf, 0x00, sizeof serial1Buf);
        if (isSensor1On())
        {
            sensor1TimeOutCount++;
            // if (sensor1TimeOutCount>0)
            // {
            //     Serial.print("sensor1TimeOutCount=");
            //     Serial.println(sensor1TimeOutCount);
            // }
        }
        else
        {
            sensor2TimeOutCount++;
            // if (sensor2TimeOutCount>0)
            // {
            //     Serial.print("sensor2TimeOutCount=");
            //     Serial.println(sensor2TimeOutCount);
            // }
        }
        
        if (dualSensor)
        {
            toggle(); 
        }
        
        emptySerial1Chars();
        rangeBegin = millis();       
        return;
    }

    if (Serial1.available())
    {
        char c = Serial1.read();
        // Serial.print(c);

        // Check for start of range reading RxxxCR or RxxxxCR
        // Can be mm, cm, or inches.
        if (c == 'R')
        {
            // Serial.println();
            readingRange = true;
            // rangeBegin = millis();
            serial1BufferIndex = 0;
            memset(serial1Buf, 0x00, sizeof serial1Buf);
            return;
        }

        // True if R has been received ..
        if (readingRange)
        {
            // To test valid range digits from 0 to 9
            uint8_t cnum = c -'0';
            // True if this is the end of the range message CR
            if (c=='\r')
            {
                // Serial.println();
                // and the number of digits is 3 or 4
                if (serial1BufferIndex==3 || serial1BufferIndex==4)
                {
                    int rangeInt = atoi(&serial1Buf[0]);
                    // Serial.println(rangeInt);
                    if (isSensor1On())
                    {
                        // Serial.printlnf("1: %d",millis());
                        maxReadTime = max(maxReadTime,millis()-rangeBegin);
                        rangeInt = rangeInt*sensor1Scale;
                        max1MedianRunning.add((int32_t)rangeInt);
                    }
                    else
                    {
                        // Serial.printlnf("2: %d",millis());
                        maxReadTime = max(maxReadTime,millis()-rangeBegin);
                        rangeInt = rangeInt*sensor2Scale;
                        max2MedianRunning.add((int32_t)rangeInt);
                    }

                    // Need to consider timeouts too
                    readingCount++;
                    // _size is the size of the runnningmedian array
                    if (readingCount==_size)
                    {
                        readingCount=0;
                        if (isSensor1On())
                        {
                            // Serial.print("Sensor 1 median: ");
                            // Serial.print(range1Median());
                            // Serial.print(" Max read time: ");
                            // Serial.println(maxReadTime);
                            sensor1Available = true;
                            sensor1TimeOutCount = 0;
                        }
                        else
                        {
                            // Serial.print("Sensor 2 median: ");
                            // Serial.print(range2Median());
                            // Serial.print(" Max read time: ");
                            // Serial.println(maxReadTime);
                            sensor2Available = true;
                            sensor2TimeOutCount = 0;
                        }    
                        maxReadTime=0;
                        if (dualSensor) toggle();
                        emptySerial1Chars();
                    }

                    rangeBegin = millis();
                    readingRange = false;
                    serial1BufferIndex = 0;
                    memset(serial1Buf, 0x00, sizeof serial1Buf);
                    return;
                }
                // if not then there is an error
                else
                {
                    // rangeBegin = millis();
                    readingRange = false;
                    serial1BufferIndex = 0;
                    memset(serial1Buf, 0x00, sizeof serial1Buf);
                    return;
                }
            }
            // if the received char is 0 to 9 add it to the buffer
            else if (cnum >= 0 && cnum <=9)
            {
                serial1Buf[serial1BufferIndex] = c;
                serial1BufferIndex++;
                return;
            }
            // if neither then there is an error
            else
            {
                // rangeBegin = millis();
                readingRange = false;
                serial1BufferIndex = 0;
                memset(serial1Buf, 0x00, sizeof serial1Buf);
                return;                
            }
        
        }
        else
        {
            return;
        }
    }
    else
    {
        return;
    }

}

void Maxbotix::emptySerial1Chars()
{
    int numChars = Serial1.available();
    // Serial.print("Char avail: ");
    // Serial.println(numChars);
    // Serial.println();
    for (int ii=0; ii < numChars; ii++ )
    {    
        Serial1.read();
        // char c = Serial1.read();
        // if (c=='\r')
        // {
        //     Serial.println();
        // }
        // else
        // {
        //     Serial.print(c);
        // }
        
        
    }
    // while(Serial1.read() >= 0);
}

int Maxbotix::range1Median()
{
    sensor1Available = false;
    return max1MedianRunning.getMedian();
}

int Maxbotix::range2Median()
{
    sensor2Available = false;
    return max2MedianRunning.getMedian();
}

void Maxbotix::toggle()
{
    maxSelect = !maxSelect;
    digitalWrite(MAX_SEL, maxSelect);
}

void Maxbotix::sensor1On()
{
    digitalWrite(MAX_SEL, LOW);
}

void Maxbotix::sensor2On()
{
    digitalWrite(MAX_SEL, HIGH);
}


bool Maxbotix::isSensor1On()
{
    if (maxSelect)
    {
        return false;
    }
    else
    {
        return true;
    }  
}

bool Maxbotix::isSensor2On()
{
    if (maxSelect)
    {
        return true;
    }
    else
    {
        return false;
    } 
}

bool Maxbotix::isSensor1Available()
{
    return sensor1Available;
}

bool Maxbotix::isSensor2Available()
{
    return sensor2Available;
}

void Maxbotix::clearDualSensorCalibration()
{
    calib.dualSensorOffset = 0;
    calib.calibrated = false;
    framCalib.write(0, (uint8_t*)&calib);
}

bool Maxbotix::isDualSensorCalibrated()
{
    return calib.calibrated;
}

int Maxbotix::calibrateDualSensorOffSet()
{    
    if (!dualSensor) return -1;
    
    emptySerial1Chars();

    while (!isSensor2Available()&&sensor2TimeOutCount<4)
    {
        readMaxbotixCharacter();
    }

    if (sensor1TimeOutCount>3)
    {
        // Serial.println("Error - sensor 2 timed out.");
        return -1;        
    }
    // Serial.print("Cal range2 is: ");
    int range2 = range2Median();
    // Serial.println(range2);  
    while (!isSensor1Available()&&sensor1TimeOutCount<4)
    {
        readMaxbotixCharacter();
    }

    if (sensor1TimeOutCount>3)
    {
        // Serial.println("Error - sensor 1 timed out.");
        return -1;
    }
    // Serial.print("Cal range1 is: ");
    int range1 = range1Median();
    // Serial.println(range1);

    // Can add checks for max min and being too close to the max and min
    // for a valid delta
    int delta = range1-range2;

    calib.dualSensorOffset = delta;

    calib.calibrated = true;

    framCalib.write(0, (uint8_t*)&calib);
  
    return delta;
}

int Maxbotix::range1Dual()
{
    framCalib.read(0, (uint8_t*)&calib);
    if (abs(range1Median()-(range2Median()+calib.dualSensorOffset))<30)
    {
        return range1Median();
    }
    else
    {
        return -1;
    }
    
}

bool Maxbotix::isValid()
{
    if (dualSensor)
    {
        if (sensor1TimeOutCount>_size/2 || sensor2TimeOutCount>_size/2)
        {
            return false;
        }
        else
        {
            return true;
        }        
    }
    else
    {
        if (sensor1TimeOutCount>_size/2)
        {
            return false;
        }
        else
        {
            return true;
        }
    }    
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library


