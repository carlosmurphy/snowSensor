# snowSensor
Measure snow pack density and stability with allll the sensors

Hardware:
Adafuit Feather M0 datalogger
LSM9DS1 - 9 DOF IMU/temp
HTU21D - humidity/temp
TSL2561 - lux + EO/IR
BMP180 - pressure + altitude/temp
TMP006 - IR infrared temp / die temp
 xxxxxxxx - capacitance

Developed using the Arduino IDE (i'll get away from it some day)
Mechanical work done in SolidWorks
Electrical design done in Circuit Maker (Altium)

Using the Magdwick AHRS scheme to guess the orientation of the sensor
Going to implement an EKF to estimate sensor depth in snow pack
This should be a better option than some mechanical depth sensor IF possible, big if
Otherwise the depth sensor will be a simple encoder with a string connected to it.
