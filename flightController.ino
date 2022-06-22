#include "MPU6050.h"

MPU6050 mpu;

TaskHandle_t keepTrackJob;

void keepTrack(void *) {
  mpu.startComplementaryFilter();
}

void setup(void) {
  Serial.begin(115200);

  // Gyro init ---------------------------------------

  mpu.initMPU();
  delay(100);
  mpu.calibrate();

  xTaskCreate(keepTrack, "keepTrack", 50000, NULL, 0, &keepTrackJob);

  // -------------------------------------------------
  
  delay(1000);
}

void loop() {

  double * valueArray = mpu.getAngleArray();

  Serial.println("x " + String(valueArray[0]) + " y " + String(valueArray[1]) + " z " + String(valueArray[2]));

  delay(100);
}
