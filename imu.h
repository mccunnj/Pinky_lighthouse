#ifndef IMU_H
#define IMU_H

//IMU
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>

extern Adafruit_BNO055 bno;

void setupIMU();
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
void displaySensorStatus();
double* accelCovariance(double lin_accel_x, double lin_accel_y, double lin_accel_z);

#endif