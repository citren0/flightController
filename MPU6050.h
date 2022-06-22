#ifndef MPU6050_h
#define MPU6050_h

#define MPUaddr 0x68

#include <Wire.h>

class MPU6050 {
  public:

    inline MPU6050() {}

    inline void initMPU() {
      Wire.begin();
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x6B);
      Wire.write(0x00);
      Wire.endTransmission(true);
    }

    inline double getGyroX() {
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return ((int16_t)(Wire.read() << 8 | Wire.read()))/131.0;
    }

    inline double getGyroY() {
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x45);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return ((int16_t)(Wire.read() << 8 | Wire.read()))/131.0;
    }

    inline double getGyroZ() {
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x47);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return ((int16_t)(Wire.read() << 8 | Wire.read()))/131.0;
    }

};

#endif
