#ifndef MPU6050_h
#define MPU6050_h

#define MPUaddr 0x68

#include <Wire.h>

TaskHandle_t keepTrackJob;

// TODO
// 1) implement altitude estimate with z accel reading.

class MPU6050 {
  private:
    double xGyroOffset = 0, yGyroOffset = 0, zGyroOffset = 0;
    double xAccelOffset = 0, yAccelOffset = 0;
    const int calibrateLength = 1000;
    double angles[3] = {0.0, 0.0, 0.0};
    int tDelay = 4; //ms, takes about 4 on an esp32
    double deltaT = tDelay / 1000.0;

    double gyroWeight = 0.98;
    double accelerometerWeight = 0.02;

    static void startWrapper(void * pvArgument) {
      reinterpret_cast<MPU6050*>(pvArgument)->startComplementaryFilter();
    }

    inline void start() {
      xTaskCreate(startWrapper, "keepTrack", 50000, this, 0, &keepTrackJob);
    }

    void startComplementaryFilter() { //should take about 4ms
      while(true) {
        float xDeltaAngle = getGyroX() * deltaT;
        float yDeltaAngle = getGyroY() * deltaT;
        float zDeltaAngle = getGyroZ() * deltaT;

        angles[0] = ((xDeltaAngle + angles[0]) * gyroWeight) + (accelerometerWeight * getAccelAngleX());
        angles[1] = ((yDeltaAngle + angles[1]) * gyroWeight) + (accelerometerWeight * getAccelAngleY());
        angles[2] += zDeltaAngle;
      }     
    }
  
  public:

    inline MPU6050() {}

    inline void initMPU() {
      Wire.begin();
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x6B);
      Wire.write(0x00);
      Wire.endTransmission(true);
      calibrate();
      start();
      
    }

    void calibrate() {
      double tempXGyroTot = 0, tempYGyroTot = 0, tempZGyroTot = 0;
      double tempXAccelTot = 0, tempYAccelTot = 0;
  
      for(int i = 0; i < calibrateLength; i++) {
        tempXGyroTot += getRawGyroX();
        tempYGyroTot += getRawGyroY();
        tempZGyroTot += getRawGyroZ();
        tempXAccelTot += getRawAccelX();
        tempYAccelTot += getRawAccelY();
      }
      
      xGyroOffset = tempXGyroTot / calibrateLength;
      yGyroOffset = tempYGyroTot / calibrateLength;
      zGyroOffset = tempZGyroTot / calibrateLength;
      xAccelOffset = tempXAccelTot / calibrateLength;
      yAccelOffset = tempYAccelTot / calibrateLength;
      
    }

    double * getAngleArray() { //returns ptr to array that stores up to date angle values in x,y,z
      return angles;
    }

    inline double getGyroX() { // returns Deg / s
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0) - xGyroOffset;
    }

    inline double getGyroY() { // returns Deg / s
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x45);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0) - yGyroOffset;
    }

    inline double getGyroZ() { // returns Deg / s
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x47);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0) - zGyroOffset;
    }

    inline double getAccelAngleX() { // returns Deg
      return (atan(getRawAccelX() / sqrt(pow(getRawAccelY(), 2) + pow(getRawAccelZ(), 2))) * 180 / PI) - xAccelOffset;
    }

    inline double getAccelAngleY() { // returns Deg
      return (atan(getRawAccelY() / sqrt(pow(getRawAccelX(), 2) + pow(getRawAccelZ(), 2))) * 180 / PI) - yAccelOffset;
    }

    // RAW GETTERS ------------------------------------------------------------------------------------------------------
    inline double getRawGyroX() { // returns Deg / s
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0);
    }

    inline double getRawGyroY() { // returns Deg / s
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x45);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0);
    }

    inline double getRawGyroZ() { // returns Deg / s
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x47);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0);
    }

    inline double getRawAccelX() {
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x3D);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/16384.0);
    }

    inline double getRawAccelY() { // returns Deg
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/16384.0);
    }

    inline double getRawAccelZ() { // returns Deg
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x3F);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/16384.0);
    }
    

};

#endif
