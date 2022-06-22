#include <Wire.h>
#include <SoftwareSerial.h>
#include "MPU6050.h"

MPU6050 mpu;

TaskHandle_t keepTrackJob;

// Gyro and Timing Globals ------------------------------------------------
const int timeBetweenLoopsms = 3;
const double deltaT = timeBetweenLoopsms / 1000.0;

double xGyroOffset = 0, yGyroOffset = 0, zGyroOffset = 0;

double currentGyroXAngle = 0, currentGyroYAngle = 0, currentGyroZAngle = 0;

double xGyroRead = 0, yGyroRead = 0, zGyroRead = 0;

long int loopCount = 0;
// ------------------------------------------------------------------------

void calibrate() {
  xGyroOffset = mpu.getGyroX();
  yGyroOffset = mpu.getGyroY();
  zGyroOffset = mpu.getGyroZ();
}

void keepGyro(void *) {
  while(true) {
    double xGyroRead = mpu.getGyroX() - xGyroOffset;
    double yGyroRead = mpu.getGyroY() - yGyroOffset;
    double zGyroRead = mpu.getGyroZ() - zGyroOffset;

    if(xGyroRead < 0.01 && xGyroRead > -0.01)
      xGyroRead = 0;

    if(yGyroRead < 0.01 && yGyroRead > -0.01)
      yGyroRead = 0;

    if(zGyroRead < 0.01 && zGyroRead > -0.01)
      zGyroRead = 0;

    currentGyroXAngle += xGyroRead * deltaT;
    currentGyroYAngle += yGyroRead * deltaT;
    currentGyroZAngle += zGyroRead * deltaT;
  }
}

void setup(void) {
  Serial.begin(115200);

  // Gyro init ---------------------------------------

  mpu.initMPU();
  delay(100);
  calibrate();

  xTaskCreatePinnedToCore(keepGyro, "keepGyro", 50000, NULL, 0, &keepTrackJob, 1);

  // -------------------------------------------------
  
  delay(1000);
}

void loop() {

  Serial.println("x " + String(currentGyroXAngle) + " y " + String(currentGyroYAngle) + " z " + String(currentGyroZAngle));
  delay(100);
}
