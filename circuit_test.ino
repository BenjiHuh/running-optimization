#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  // Send data to serial monitor
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  int8_t temperature = myIMU.getTemp();
  Serial.println(temperature);
  // Use the crystal on the board instead of the chip
  myIMU.setExtCrystalUse(true); 
}

void loop() {
  // Get data from accelerometer, gyroscope and magnetometer
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); 

  // Accelerometer
  Serial.print(acc.x());
  Serial.print(",");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print(acc.z());
  Serial.print(",");

  // Delay so that the loop does not run faster than 
  // the speed of the sensor
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
