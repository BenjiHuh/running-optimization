#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <CircularBuffer.h>
#include <EEPROM.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

const int BUFFER_SIZE = 200;
int bufferX[BUFFER_SIZE];
int itemCount = 0;
const int STARTING_EEPROM_ADDRESS = 0;

Adafruit_BNO055 myIMU = Adafruit_BNO055();

bool bufferCleared = false;
bool bufferSaved = false;

void setup() {
  // Send data to serial monitor
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  // Use the crystal on the board instead of the chip
  myIMU.setExtCrystalUse(true); 
}

void loop() {
  // Get data from accelerometer, gyroscope and magnetometer
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER); 

  if (!bufferCleared) {
    EEPROM.get(STARTING_EEPROM_ADDRESS, bufferX);
    for (int i = 0; i < BUFFER_SIZE; i++) {
      Serial.println(bufferX[i]);
    }
    bufferCleared = true;
  } else {
    // Accelerometer
    if (itemCount < BUFFER_SIZE) {
      bufferX[itemCount] = (int)(acc.z() * 100);
      itemCount++;
    } else if (!bufferSaved) {
      EEPROM.put(STARTING_EEPROM_ADDRESS, bufferX);
      bufferSaved = true;
    } else {
      Serial.println("!");
    }
  }

  // Delay so that the loop does not run faster than 
  // the speed of the sensor
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
