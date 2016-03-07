#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFunLSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TMP006.h"
#include "SparkFunHTU21D.h"
#include <SFE_BMP180.h>
#include <SparkFunTSL2561.h>

//Because the M0 core is stupid...
#define Serial SerialUSB

//pin used by the feather for SS on the SPI bus
#define cardSelect 4

//creating file object to write into the SD card
File logfile;


//////////// Use the LSM9DS1 class to create an object. 9 dof sensor///////////////////////
LSM9DS1 imu;

////////////////// TMP006 object creation, temp sensor//////////////////////
Adafruit_TMP006 thermopile(0x44); //default address is 0x40
// addr1 pin must be connected to vcc addr0 to gnd
//Adafruit_TMP006 tmp006(0x41);  // start with a diferent i2c address!

///////////////////Create an instance of the humidity sensor/////////////////////
HTU21D humidity; //i2c address 0x40

// //////////////////SFE_BMP180 object, pressure sensor////////////////////
SFE_BMP180 pressure;
double baseline; // baseline pressure


/////////////// Create an SFE_TSL2561 object, light sensor//////////////
SFE_TSL2561 light;
boolean gain;     // Gain setting, 0 = X1, 1 = X16;
unsigned int ms;  // Integration ("shutter") time in milliseconds

/////////////////////////////////////////////////////////////////////////
// IMU I2C Setup //
///////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.


////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 2000 // xxx ms between prints

void setup() {

//Starting the Serial Port
Serial.begin(9600);
//built in feather LED pin
 pinMode(8, OUTPUT);

////////////////////////////staring SD card /////////////////////////////////
 pinMode(13, OUTPUT); //don't really know it was in the example?

 // see if the card is present and can be initialized:
 if (!SD.begin(cardSelect)) {
   Serial.println("Card init. failed!");
 }
 char filename[15];
 strcpy(filename, "SENSOR00.TXT");
 for (uint8_t i = 0; i < 100; i++) {
   filename[6] = '0' + i/10;
   filename[7] = '0' + i%10;
   // create if does not exist, do not open existing, write, sync after write
   if (! SD.exists(filename)) {
     break;
   }
 }

 logfile = SD.open(filename, FILE_WRITE);
 if( ! logfile ) {
   Serial.print("Couldnt create ");
   Serial.println(filename);
 }
 Serial.print("Writing to ");
 Serial.println(filename);
 pinMode(13, OUTPUT);                  // twice for some reason, still dont know what pin 13 is
 Serial.println("SD Card Ready!");

//Starting all the sensors

////////////////////// starting the 9 dof sensor ///////////////////
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!initLSM9DS1())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1) //freeze
      ;
  }
  //////////////////////// starting thermopile sensor //////////////////////////////

  // you can also use tmp006.begin(TMP006_CFG_1SAMPLE) or 2SAMPLE/4SAMPLE/8SAMPLE to have
  // lower precision, higher rate sampling. default is TMP006_CFG_16SAMPLE which takes
  // 4 seconds per reading (16 samples)
  if (!thermopile.begin()) {
    Serial.println("No temp006 found");
    while (1); //freeze
  }

  //////////////////////////////// start humidity sensor //////////////////////////////
  humidity.begin();

/////////////////// start pressure sensor /////////////////////////////////////
   if (!pressure.begin()) {
    Serial.println("No BMP180 found");
    while (1); //freeze
  }
  // Get the baseline pressure:
  baseline = getPressure();

////////////////////////// start light sensor ///////////////////////////////////
  // Initialize the SFE_TSL2561 library

  // You can pass nothing to light.begin() for the default I2C address (0x39),
  // or use one of the following presets if you have changed
  // the ADDR jumper on the board:

  // TSL2561_ADDR_0 address with '0' shorted on board (0x29)
  // TSL2561_ADDR   default address (0x39)
  // TSL2561_ADDR_1 address with '1' shorted on board (0x49)

  // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor

  light.begin();
  // Get factory ID from sensor:
  // (Just for fun, you don't need to do this to operate the sensor)

  unsigned char ID;
if (light.getID(ID))
  {
    Serial.print("Got factory ID: 0X");
    Serial.print(ID,HEX);
    Serial.println(", should be 0X5X");
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else
  {
    byte error = light.getError();
    printError(error);
  }

  // The light sensor has a default integration time of 402ms,
  // and a default gain of low (1X).

  // If you would like to change either of these, you can
  // do so using the setTiming() command.

  // If gain = false (0), device is set to low gain (1X)
  // If gain = high (1), device is set to high gain (16X)

  gain = 0;

  // If time = 0, integration will be 13.7ms
  // If time = 1, integration will be 101ms
  // If time = 2, integration will be 402ms
  // If time = 3, use manual start / stop to perform your own integration

  unsigned char time = 2;

  // setTiming() will set the third parameter (ms) to the
  // requested integration time in ms (this will be useful later):

  Serial.println("Set timing...");
  light.setTiming(gain,time,ms);

  // To start taking measurements, power up the sensor:

  Serial.println("Powerup...");
  light.setPowerUp();

  // The sensor will now gather light during the integration time.
  // After the specified time, you can retrieve the result from the sensor.
  // Once a measurement occurs, another integration period will start.

}

void loop() {
  
  // make an empty string for assembling the data to log:
  String dataString = "";
  
///////////dumping IMU data ////////////////////////////////////
if (imu.accelAvailable())
  {
    imu.readAccel();
  }

  // imu.gyroAvailable() returns 1 if new gyroscope
  // data is ready to be read. 0 otherwise.
  if (imu.gyroAvailable())
  {
    imu.readGyro();
  }

  // imu.magAvailable() returns 1 if new magnetometer
  // data is ready to be read. 0 otherwise.
  if (imu.magAvailable())
  {
    imu.readMag();
  }

  // imu.tempAvailable() returns 1 if new temperature sensor
  // data is ready to be read. 0 otherwise.
  if (imu.tempAvailable())
  {
    imu.readTemp();
  }

  Serial.print("A: ");
  Serial.print(imu.calcAccel(imu.ax));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az));
  Serial.println(" g \t| ");
  Serial.print("G: ");
  Serial.print(imu.calcGyro(imu.gx));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz));
  Serial.println(" dps \t| ");
  Serial.print("M: ");
  Serial.print(imu.calcMag(imu.mx));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz));
  Serial.println(" Gs \t| ");
  Serial.print("T: ");
  Serial.print(imu.temperature);
  Serial.print(" \t\t\t| ");
  Serial.println();
  
  //adding data to the string for writing to the SD card
  dataString += String(imu.calcAccel(imu.az),5);
  dataString += ",";
  
  
////////////////////dumping thermopile data //////////////////////////////////////////////
// Grab temperature measurements and print them.
  float objt = thermopile.readObjTempC();
  Serial.print("Object Temperature: "); Serial.print(objt); Serial.println("*C");
  float diet = thermopile.readDieTempC();
  Serial.print("Die Temperature: "); Serial.print(diet); Serial.println("*C");
  
    //adding data to the string for writing to the SD card
  dataString += String(objt,5);
  dataString += ",";

 //////////////////////////////////////dumping humidity data///////////////////////////////////
 float humd = humidity.readHumidity();
 float temp = humidity.readTemperature();

  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");
  Serial.println();
  
    //adding data to the string for writing to the SD card
  dataString += String(humd,5);
  dataString += ",";

 //if humd or temp = 999 then there is a CRC error
 //if humd or temp = 998 if the sensor is not detected

 //////////////////////////////////////dumping pressure data ///////////////////////////////////
 double a,P;

  // Get a new pressure reading:

  P = getPressure();

  // Show the relative altitude difference between
  // the new reading and the baseline reading:

  a = pressure.altitude(P,baseline);

  Serial.print("relative altitude: ");
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a,1);
  Serial.print(" meters, ");
  Serial.print("Pressure: ");
  Serial.print(P);
  Serial.println("mbars");
  
  //adding data to the string for writing to the SD card
  dataString += String(a,5);
  dataString += ",";
  
  //////////////////////////////dumping light data ////////////////////////////////////////////////
  // Wait between measurements before retrieving the result
  // (You can also configure the sensor to issue an interrupt
  // when measurements are complete)

  // This sketch uses the TSL2561's built-in integration timer.
  // You can also perform your own manual integration timing
  // by setting "time" to 3 (manual) in setTiming(),
  // then performing a manualStart() and a manualStop() as in the below
  // commented statements:

  // ms = 1000;
  // light.manualStart();
  //delay(ms);
  // light.manualStop();

  // Once integration is complete, we'll retrieve the data.

  // There are two light sensors on the device, one for visible light
  // and one for infrared. Both sensors are needed for lux calculations.

  // Retrieve the data from the device:

  unsigned int data0, data1;
  double lux;    // Resulting lux value
  boolean good;  // True if neither sensor is saturated

  if (light.getData(data0,data1))
  {
    // getData() returned true, communication was successful

    Serial.print("EO?: ");
    Serial.print(data0);
    Serial.print(" IR?: ");
    Serial.print(data1);

    // To calculate lux, pass all your settings and readings
    // to the getLux() function.

    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
    // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor

    // Perform lux calculation:

    good = light.getLux(gain,ms,data0,data1,lux);

    // Print out the results:

    Serial.print(" lux: ");
    Serial.print(lux);
    
  //adding data to the string for writing to the SD card
  dataString += lux;
  dataString += " , ";
  
    if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
  }
  else
  {
    // getData() returned false because of an I2C error, inform the user.

    byte error = light.getError();
    printError(error);
  }

///////////////////////Writing things to the sd card////////////////////////////////////////
digitalWrite(8, HIGH); //LED on for the writing to the card
logfile.println(dataString);
logfile.flush();       //Force write everything to the SD card
digitalWrite(8, LOW);  //LED off after the write is finished
 
// wait around for the next go
delay(PRINT_SPEED);
}//end of loop//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////// LSM9DS1 sensor settings/////////////////////////////////////////////
void setupDevice()
{
  // [commInterface] determines whether we'll use I2C or SPI
  // to communicate with the LSM9DS1.
  // Use either IMU_MODE_I2C or IMU_MODE_SPI
  imu.settings.device.commInterface = IMU_MODE_I2C;
  // [mAddress] sets the I2C address or SPI CS pin of the
  // LSM9DS1's magnetometer.
  imu.settings.device.mAddress = 0x1E; // Use I2C addres 0x1E
  // [agAddress] sets the I2C address or SPI CS pin of the
  // LSM9DS1's accelerometer/gyroscope.
  imu.settings.device.agAddress = 0x6B; // I2C address 0x6B
}

void setupGyro()
{
  // [enabled] turns the gyro on or off.
  imu.settings.gyro.enabled = true;  // Enable the gyro
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 245; // Set scale to +/-245dps
  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 3; // 59.5Hz ODR
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;
  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  // [flipX], [flipY], and [flipZ] are booleans that can
  // automatically switch the positive/negative orientation
  // of the three gyro axes.
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
}

void setupAccel()
{
  // [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true; // Enable accelerometer
  // [enableX], [enableY], and [enableZ] can turn on or off
  // select axes of the acclerometer.
  imu.settings.accel.enableX = true; // Enable X
  imu.settings.accel.enableY = true; // Enable Y
  imu.settings.accel.enableZ = true; // Enable Z
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 8; // Set accel scale to +/-8g.
  // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 1; // Set accel to 10Hz.
  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Accel cutoff freqeuncy can be any value between -1 - 3.
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = 0; // BW = 408Hz
  // [highResEnable] enables or disables high resolution
  // mode for the acclerometer.
  imu.settings.accel.highResEnable = false; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;
}

void setupMag()
{
  // [enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true; // Enable magnetometer
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 12; // Set mag scale to +/-12 Gs
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 5; // Set OD rate to 20Hz
  // [tempCompensationEnable] enables or disables
  // temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;
  // [XYPerformance] sets the x and y-axis performance of the
  // magnetometer to either:
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
  // [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
  // [lowPowerEnable] enables or disables low power mode in
  // the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode
}

void setupTemperature()
{
  // [enabled] turns the temperature sensor on or off.
  imu.settings.temp.enabled = true;
}

uint16_t initLSM9DS1()
{
  setupDevice(); // Setup general device parameters
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters
  setupMag(); // Set up magnetometer parameters
  setupTemperature(); // Set up temp sensor parameter

  return imu.begin();
}

/////////////////////////////////////////// pressure sensor functions ///////////////////////////////////
double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}




/////////////////////////////////////// i2c stuff/tools //////////////////////////////////////////////////
void printError(byte error)
  // If there's an I2C error, this function will
  // print out an explanation.
{
  Serial.print("I2C error: ");
  Serial.print(error,DEC);
  Serial.print(", ");

  switch(error)
  {
    case 0:
      Serial.println("success");
      break;
    case 1:
      Serial.println("data too long for transmit buffer");
      break;
    case 2:
      Serial.println("received NACK on address (disconnected?)");
      break;
    case 3:
      Serial.println("received NACK on data");
      break;
    case 4:
      Serial.println("other error");
      break;
    default:
      Serial.println("unknown error");
  }
}
