#include "MPU6050.h"

MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);

  // MPU init ---------------------------------------

  mpu.initMPU();

  // -------------------------------------------------
  
  delay(1000);
}

void loop() {

  double * valueArray = mpu.getAngleArray();

  Serial.println("x " + String(valueArray[0]) + " y " + String(valueArray[1]) + " z " + String(valueArray[2]));

  delay(100);
}
