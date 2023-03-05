/*
 * Class for gyroscope, extends sensor class
 * Periodically measures imu data & magnetometer
 */
#ifndef GYROSCOPE_H
#define GYROSCOPE_H


#include "LSM6.h"
#include "Sensor.h"


#define CALIBRATION_SAMPLE_NUM 10000

#define ACCEL_RANGE (double)2
#define GYRO_RANGE  (double)250

#define GYRO_X_OFFSET gyro_offsets[0]
#define GYRO_Y_OFFSET gyro_offsets[1]
#define GYRO_Z_OFFSET gyro_offsets[2]


#define ACCEL_X_OFFSET accel_offsets[0]
#define ACCEL_Y_OFFSET accel_offsets[1]
#define ACCEL_Z_OFFSET accel_offsets[2]

#define ACCEL_GAIN(x) (double)x * (double)(ACCEL_RANGE / (double)32768)
#define GYRO_GAIN(x) (double)x * (double)(gyro_scale_ / (double)32768)

#define MAX_ANG_PER_SAMPLE 10


typedef enum gyro_scale {
  SCALE_245 = 0x0,
  SCALE_500 = 0x1,
  SCALE_1000 = 0x2,
  SCALE_2000 = 0x3,
} gyro_scale_t;

class Imu : public Sensor {
  public:
  /*
   * imu type to hold data
  */
  typedef struct imu_data {
    float a_x;
    float a_y;
    float a_z;
    float g_x;
    float g_y;
    float g_z;
    float heading;  
  } imu_data_t;

  /*
   * Constructor for imu
   * @param i2cbus for sensor 
   * @param id of sensor
  */
  Imu(TwoWire *bus, String id=""):Sensor(id) {
    imu_sensor.setBus(bus);
    initSensor();
  };

  void setGyroScale(gyro_scale_t scale);

  void setTargetHeading(int angle);

  int getTargetHeading();

  /*
   * Returns imu data
   * @return imu data
  */
  imu_data_t getData();

  /*
   * Function called by scheduler
   * Implementation for hardware read
  */
  int readSensor() override;
  
  private:
  LSM6 imu_sensor;
  int gyro_scale_;
  int target_heading_;
  double curr_heading_;
  unsigned long prev_time_;
  imu_data_t data_;
  double accel_offsets[3] = {0, 0, 0};
  double gyro_offsets[3] = {0, 0, 0};
  double mag_offsets[3] = {0, 0, 0};

  /*
   * initialize imu
  */
  void initSensor() override;
  /*
   * calibrate accelerometer and gyroscope
  */
  void calibrateImu();

};


#endif