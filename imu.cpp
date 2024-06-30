#include "imu.h"
#include "oled.h"// Include oled.h for display functions

Adafruit_BNO055 bno = Adafruit_BNO055(55);  // Define bno instance

Preferences myPrefs;

void setupIMU() {
  Wire.begin();
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    printFit("BNO055 not detected");
    while(1);
  }
    
  bno.setExtCrystalUse(true);

  // Load offsets from flash
  myPrefs.begin("offsets", true); // Opens flash in Read only
  if (myPrefs.isKey("accel_offset_x")) {
    adafruit_bno055_offsets_t oldCalib;
    oldCalib.accel_offset_x = myPrefs.getShort("accel_offset_x");
    oldCalib.accel_offset_y = myPrefs.getShort("accel_offset_y");
    oldCalib.accel_offset_z = myPrefs.getShort("accel_offset_z");
    oldCalib.mag_offset_x = myPrefs.getShort("mag_offset_x");
    oldCalib.mag_offset_y = myPrefs.getShort("mag_offset_y");
    oldCalib.mag_offset_z = myPrefs.getShort("mag_offset_z");
    oldCalib.gyro_offset_x = myPrefs.getShort("gyro_offset_x");
    oldCalib.gyro_offset_y = myPrefs.getShort("gyro_offset_y");
    oldCalib.gyro_offset_z = myPrefs.getShort("gyro_offset_z");
    oldCalib.accel_radius = myPrefs.getShort("accel_radius");
    oldCalib.mag_radius = myPrefs.getShort("mag_radius");
    displaySensorOffsets(oldCalib);
    bno.setSensorOffsets(oldCalib);
    myPrefs.end();
  } else {
    Serial.println("No calibration data.. Please calibrate and store offsets. See other program.");
    printFit("No calibration data.. Please calibrate and store offsets.");
  }
  // Implement warm-up procedure for the IMU if needed
  // This function ensures IMU is ready before proceeding
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  while(system_status < 5) {
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  }
  Serial.print("IMU warm-up complete. System status: ");
  Serial.println(system_status, HEX);
  printFit("IMU warm-up complete.");
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData){
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.println(calibData.mag_radius);
}

void displaySensorStatus(void){
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.print("Mode: ");
  Serial.println(bno.getMode());
  Serial.println("");
  //delay(500);
}

double* accelCovariance(double lin_accel_x, double lin_accel_y, double lin_accel_z){
  // Assuming noise_density is given in µg/√Hz
  double acceleration_noise_density = 150.0;  // µg/√Hz
  double bandwidth = 62.5;  // Default bandwidth for BNO055 accelerometer in Hz
  int zero_g_offset = 80;

  // Convert µg/√Hz to m/s^2/√Hz (1 µg = 9.81e-6 m/s^2)
  double noise_density_mps2 = acceleration_noise_density * 9.81e-6;

  // Calculate the covariance value
  double covariance_value_nd = noise_density_mps2 / sqrt(bandwidth);

  // Fill in the covariance matrix
  static double linear_acceleration_covariance[9];
  linear_acceleration_covariance[0] = pow((lin_accel_x - zero_g_offset), 2) + pow(covariance_value_nd, 2);
  linear_acceleration_covariance[4] = pow((lin_accel_y - zero_g_offset), 2) + pow(covariance_value_nd, 2);
  linear_acceleration_covariance[8] = pow((lin_accel_z - zero_g_offset), 2) + pow(covariance_value_nd, 2);
  return linear_acceleration_covariance;
}