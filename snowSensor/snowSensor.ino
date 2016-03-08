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

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t count = 0;  // used to control display output rate
uint32_t delt_t = 0; // used to control display output rate
float pitch, yaw, roll, heading;
float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float temperature; //temp of the imu sensor

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.


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

/////////////////////////writing first line of the CSV file
writeLog("roll(deg),pitch(deg),yaw(deg),temp,object temp, die temp,humidity, temp, Pressure, altitude (m),EO,IR,Lux");

}

void loop() {

  // make an empty string for assembling the data to log:
  String dataString = "";

  ///////////dumping IMU data ////////////////////////////////////
  if (imu.accelAvailable())
    {
      imu.readAccel();
      ax=imu.calcAccel(imu.ax)-imu.aBias[0]; //convert to G and remove bias
      ay=imu.calcAccel(imu.ay)-imu.aBias[1];
      az=imu.calcAccel(imu.az)-imu.aBias[2];
    }

    // imu.gyroAvailable() returns 1 if new gyroscope
    // data is ready to be read. 0 otherwise.
    if (imu.gyroAvailable())
    {
      imu.readGyro();
      gx=imu.calcGyro(imu.gx)-imu.gBias[0]; //convert to deg/s and remove bias
      gy=imu.calcGyro(imu.gy)-imu.gBias[1];
      gz=imu.calcGyro(imu.gz)-imu.gBias[2];
    }

    // imu.magAvailable() returns 1 if new magnetometer
    // data is ready to be read. 0 otherwise.
    if (imu.magAvailable())
    {
      imu.readMag();
      mx=imu.calcMag(imu.mx); //convert to gauss
      my=imu.calcMag(imu.my);
      mz=imu.calcMag(imu.mz);
    }

  // imu.tempAvailable() returns 1 if new temperature sensor
  // data is ready to be read. 0 otherwise.
  if (imu.tempAvailable())
  {
    imu.readTemp();
    temperature=imu.temperature;
  }

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
   MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
 //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);

   roll = getRoll(); //getting the Euler angles from the quaternion
   pitch = getPitch();
   yaw = getYaw();

   roll  *= 180.0f / PI;
   pitch *= 180.0f / PI;
   yaw   *= 180.0f / PI;
   yaw   += DECLINATION; // compensate for the Declination

  //adding data to the string for writing to the SD card
  dataString += String(int(1000*roll));
  dataString += ",";
  dataString += String(int(1000*pitch));
  dataString += ",";
  dataString += String(int(1000*yaw));
  dataString += ",";
  dataString += String(int(temperature));
  dataString += ",";

////////////////////dumping thermopile data //////////////////////////////////////////////
// Grab temperature measurements and print them.
  float objt = thermopile.readObjTempC(); //object temp
  float diet = thermopile.readDieTempC(); //sensor internal temp

    //adding data to the string for writing to the SD card
  dataString += String(int(1000*objt));
  dataString += ",";
  dataString += String(int(1000*diet));
  dataString += ",";

 //////////////////////////////////////dumping humidity data///////////////////////////////////
 float humd = humidity.readHumidity();
 float temp = humidity.readTemperature();

  //adding data to the string for writing to the SD card
  dataString += String(int(1000*humd));
  dataString += ",";
  dataString += String(int(1000*temp));
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

  //adding data to the string for writing to the SD card
  dataString += String(int(1000*P));
  dataString += ",";
  dataString += String(int(1000*a));
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

  unsigned int data0, data1; //IR and EO intensity (or something)
  double lux;    // Resulting lux value
  boolean good;  // True if neither sensor is saturated
  // Retrieve the data from the device:
  if (light.getData(data0,data1))
  {
    // getData() returned true, communication was successful

    // To calculate lux, pass all your settings and readings
    // to the getLux() function.

    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
    // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor

    // Perform lux calculation:

    good = light.getLux(gain,ms,data0,data1,lux);

    //adding data to the string for writing to the SD card
    dataString += String(int(data0*1000));
    dataString += " , ";
    dataString += String(int(data1*1000));
    dataString += " , ";
    dataString += String(int(lux*1000));
    dataString += " , ";

    }
    else
    {
      // getData() returned false because of an I2C error, inform the user.

      byte error = light.getError();
      printError(error);
    }
/////////////////////printingToSerial///////////////////////////////////////////////////////////////////////////
// Serial print and/or display at 0.5 s rate independent of data rates
delt_t = millis() - count;
if (delt_t > 500) { // update once per half-second independent of read rate
  ///////////////////AHRS RESULTS

    Serial.print("ax = "); Serial.print((int)1000*ax);
    Serial.print(" ay = "); Serial.print((int)1000*ay);
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2);
    Serial.print(" gy = "); Serial.print( gy, 2);
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)1000*mx);
    Serial.print(" my = "); Serial.print( (int)1000*my);
    Serial.print(" mz = "); Serial.print( (int)1000*mz); Serial.println(" mG");

    Serial.print("temperature = "); Serial.println(temperature, 2);

    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]);
    Serial.print(" qy = "); Serial.print(q[2]);
    Serial.print(" qz = "); Serial.println(q[3]);

    Serial.print("filter rate = "); Serial.println(1.0f/deltat, 1);
  ///////////////////temp

  Serial.print("Object Temperature: "); Serial.print(objt); Serial.println("*C");
  Serial.print("Die Temperature: "); Serial.print(diet); Serial.println("*C");

  //////////////////humidity

  Serial.print(" Temperature:");
  Serial.print(temp, 1);
  Serial.print("C");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);
  Serial.print("%");
  Serial.println();

  ///////////////////pressure

  Serial.print("relative altitude: ");
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a,1);
  Serial.print(" meters, ");
  Serial.print("Pressure: ");
  Serial.print(P);
  Serial.println("mbars");

  /////////////////////light

  Serial.print("EO?: ");
  Serial.print(data0);
  Serial.print(" IR?: ");
  Serial.print(data1);

  Serial.print(" lux: ");
  Serial.print(lux);
  if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");



count = millis(); //recording when the last print was made
}

writeLog(dataString); //Writing alllllllllllll the data to the sd card

}//end of loop//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void writeLog(String dataz) //writes to the log file and flashes the LED on while doing it
{
digitalWrite(8, HIGH); //LED on for the writing to the card
logfile.println(dataz);
logfile.flush();       //Force write everything to the SD card
digitalWrite(8, LOW);  //LED off after the write is finished
}

//////////////////////////////////// LSM9DS1 sensor settings/////////////////////////////////////////////
uint16_t initLSM9DS1()
{
  setupDevice(); // Setup general device parameters
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters
  setupMag(); // Set up magnetometer parameters
  setupTemperature(); // Set up temp sensor parameter

  return imu.begin();
}

void setupDevice()
{
  // [commInterface] determines whether we'll use I2C or SPI
  // to communicate with the LSM9DS1.
  // Use either IMU_MODE_I2C or IMU_MODE_SPI
  imu.settings.device.commInterface = IMU_MODE_I2C;

  // SDO_XM and SDO_G are both pulled high, so our addresses are:

  // [mAddress] sets the I2C address or SPI CS pin of the
  // LSM9DS1's magnetometer.
  imu.settings.device.mAddress = 0x1E; // Would be 0x1C if SDO_M is LOW
  // [agAddress] sets the I2C address or SPI CS pin of the
  // LSM9DS1's accelerometer/gyroscope.
  imu.settings.device.agAddress = 0x6B; // Would be 0x6A if SDO_AG is LOW
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
////////////////////////////////////////////////////////AHRS algorithms///////////////////////////////////////////////////////////////
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }

        // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
           void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
       {
           float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
           float norm;
           float hx, hy, bx, bz;
           float vx, vy, vz, wx, wy, wz;
           float ex, ey, ez;
           float pa, pb, pc;

           // Auxiliary variables to avoid repeated arithmetic
           float q1q1 = q1 * q1;
           float q1q2 = q1 * q2;
           float q1q3 = q1 * q3;
           float q1q4 = q1 * q4;
           float q2q2 = q2 * q2;
           float q2q3 = q2 * q3;
           float q2q4 = q2 * q4;
           float q3q3 = q3 * q3;
           float q3q4 = q3 * q4;
           float q4q4 = q4 * q4;

           // Normalise accelerometer measurement
           norm = sqrt(ax * ax + ay * ay + az * az);
           if (norm == 0.0f) return; // handle NaN
           norm = 1.0f / norm;        // use reciprocal for division
           ax *= norm;
           ay *= norm;
           az *= norm;

           // Normalise magnetometer measurement
           norm = sqrt(mx * mx + my * my + mz * mz);
           if (norm == 0.0f) return; // handle NaN
           norm = 1.0f / norm;        // use reciprocal for division
           mx *= norm;
           my *= norm;
           mz *= norm;

           // Reference direction of Earth's magnetic field
           hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
           hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
           bx = sqrt((hx * hx) + (hy * hy));
           bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

           // Estimated direction of gravity and magnetic field
           vx = 2.0f * (q2q4 - q1q3);
           vy = 2.0f * (q1q2 + q3q4);
           vz = q1q1 - q2q2 - q3q3 + q4q4;
           wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
           wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
           wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

           // Error is cross product between estimated direction and measured direction of gravity
           ex = (ay * vz - az * vy) + (my * wz - mz * wy);
           ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
           ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
           if (Ki > 0.0f)
           {
               eInt[0] += ex;      // accumulate integral error
               eInt[1] += ey;
               eInt[2] += ez;
           }
           else
           {
               eInt[0] = 0.0f;     // prevent integral wind up
               eInt[1] = 0.0f;
               eInt[2] = 0.0f;
           }

           // Apply feedback terms
           gx = gx + Kp * ex + Ki * eInt[0];
           gy = gy + Kp * ey + Ki * eInt[1];
           gz = gz + Kp * ez + Ki * eInt[2];

           // Integrate rate of change of quaternion
           pa = q2;
           pb = q3;
           pc = q4;
           q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
           q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
           q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
           q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

           // Normalise quaternion
           norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
           norm = 1.0f / norm;
           q[0] = q1 * norm;
           q[1] = q2 * norm;
           q[2] = q3 * norm;
           q[3] = q4 * norm;

       }

       // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
       // In this coordinate system, the positive z-axis is down toward Earth.
       // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination),
       // looking down on the sensor positive yaw is counterclockwise.
       // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
       // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
       // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
       // Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get the correct orientation the rotations must be
       // applied in the correct order which for this configuration is yaw, pitch, and then roll.
       // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

       float getRoll() // returns roll in radians
       {
         float i  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
         return i;
       }

       float getPitch() // returns pitch in radians
       {
         float j = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
         return j;
       }

       float getYaw() // returns yaw in radians 0-PI, 0 is magnetic north
       {
         float k   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
         return k;
       }
