#ifndef MPU6050_h
#define MPU6050_h

#define MPUaddr 0x68

#include <Wire.h>

TaskHandle_t keepTrackJob;

// TODO
// 1) implement altitude estimate with z accel reading.

class MPU6050 {
  private:
    // These are the values the calibrator will set.
    double xGyroOffset = 0, yGyroOffset = 0, zGyroOffset = 0;
    double xAccelOffset = 0, yAccelOffset = 0;

    // This sets the number of loops the calibrator runs through before averaging the values.
    const int calibrateLength = 1000;

    // The angles will be stored in this array in real time. A reference to this array can be grabbed via the getAngleArray function.
    double angles[3] = {0.0, 0.0, 0.0};

    // ms it takes to complete a loop of the complementary filter, takes 4 on an esp32
    int tDelay = 4;
    double deltaT = tDelay / 1000.0;

    // Weight of the gyro and accelerometer components of the complementary filter.
    double gyroWeight = 0.98;
    double accelerometerWeight = 0.02;


    /*
      Needed for xTaskCreate
      Parameters: This
    */
    static void startWrapper(void * pvArgument) {
      reinterpret_cast<MPU6050*>(pvArgument)->startComplementaryFilter();
    }


    /*
      Starts the task job and calls the startWrapper function
    */
    inline void start() {
      xTaskCreate(startWrapper, "keepTrack", 50000, this, 0, &keepTrackJob);
    }


    /*
      Runs in a loop in the background, operates the complementary filter
    */
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

    /*
      Does nothing
    */
    inline MPU6050() {}


    /*
      One function to rule them all. Initializes I2C, calibrates the MPU, and starts the complementary filter.
    */
    inline void initMPU() {
      Wire.begin();
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x6B);
      Wire.write(0x00);
      Wire.endTransmission(true);
      calibrate();
      start();
    }


    /*
      Takes an average of the raw gyrometer and accelerometer data to find an offset.
      Most useful if called while the MPU is stationary and facing upright.
    */
    void calibrate() {
      delay(2000);
      
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

      delay(1000);
      
    }


    /*
      Returns a pointer to an array that stores the processed angle values in real time.
    */
    double * getAngleArray() {
      return angles;
    }


    /*
      Next 3 functions return readings in Deg / second from the gyrometer.
    */
    inline double getGyroX() {
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0) - xGyroOffset;
    }

    inline double getGyroY() {
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x45);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0) - yGyroOffset;
    }

    inline double getGyroZ() {
      Wire.beginTransmission(MPUaddr);
      Wire.write(0x47);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUaddr, 2, true);
      return (((int16_t)(Wire.read() << 8 | Wire.read()))/131.0) - zGyroOffset;
    }


    /*
      Next 2 functions return degree readings from the accelerometer.
    */
    inline double getAccelAngleX() { // returns Deg
      return (atan(getRawAccelX() / sqrt(pow(getRawAccelY(), 2) + pow(getRawAccelZ(), 2))) * 180 / PI) - xAccelOffset;
    }

    inline double getAccelAngleY() { // returns Deg
      return (atan(getRawAccelY() / sqrt(pow(getRawAccelX(), 2) + pow(getRawAccelZ(), 2))) * 180 / PI) - yAccelOffset;
    }



    /*
      Next 5 do the same as the previous 5 but without the offsets or conversion to degrees.
    */
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
