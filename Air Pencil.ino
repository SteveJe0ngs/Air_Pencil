#include <Wire.h>
#include "Kalman.h"
#include <Mouse.h>

const int BUTTON = 9;

int SumX = 0;
int SumZ = 0;

int16_t gyroX, gyroZ;
int Sensitivity = 600;
int delayi = 3;

uint32_t timer;

uint8_t i2cData[14];                // Buffer for I2C data
const uint8_t IMUAddress = 0x68;    // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000;  // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop);  // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  return Wire.endTransmission(sendStop);  // Returns 0 on success
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false))                       // Don't release the bus
    return 1;                                            // Error in communication
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t) true);  // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available())
        ;
      if (Wire.available())
        data[i] = Wire.read();
      else
        return 2;  // Error in communication
    }
  }
  return 0;  // Success
}

void setup() {
  pinMode(BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  Wire.begin();

  i2cData[0] = 7;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to ±2g

  while (i2cWrite(0x19, i2cData, 4, false))
    ;  // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true))
    ;  // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1))
    ;

  if (i2cData[0] != 0x68) {  // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }

  delay(100);  // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6))
    ;
  timer = micros();

  Mouse.begin();
}


void loop() {

  while (i2cRead(0x3B, i2cData, 14))
    ;

  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  gyroX = gyroX / Sensitivity / 1.1 * -1;
  gyroZ = gyroZ / Sensitivity * -1;

  Mouse.move(gyroZ, -gyroX);

  if (digitalRead(BUTTON) == LOW) {
    Mouse.press();
  } else
    Mouse.release();

  delay(delayi);
}