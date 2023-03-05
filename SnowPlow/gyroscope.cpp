#include "gyroscope.h"


int gyro_scales_[4] = {250, 500, 1000, 2000};


int Imu::readSensor() {
  imu_sensor.read();
  unsigned long curr_time = millis();
  double time_delta_ms = (curr_time - prev_time_);
  // integrate new heading
  double ang = time_delta_ms * (GYRO_GAIN(imu_sensor.g.z)-GYRO_Z_OFFSET) / 1000.0;
  if (time_delta_ms <= this->getInterruptInterval()*3){
    curr_heading_ += ang;
  }
  prev_time_ = curr_time;
  data_.heading = curr_heading_;
  //Serial.println(curr_heading_);
  data_.a_x = imu_sensor.a.x;
  data_.a_y = imu_sensor.a.y;
  data_.a_z = imu_sensor.a.z;
  data_.g_x = imu_sensor.g.x;
  data_.g_y = imu_sensor.g.y;
  data_.g_z = imu_sensor.g.z;
  data_ready_ = true;
}

void Imu::initSensor() {
  if (!imu_sensor.init()) {
    Serial.println("Failed to init gyro + accel!");
    while(1);
  }
  data_ready_ = false;
  imu_sensor.enableDefault();
  gyro_scale_ = gyro_scales_[0];
  setGyroScale(SCALE_245);
  calibrateImu();
  curr_heading_ = 0;
}

void Imu::setGyroScale(gyro_scale_t scale) {
  // set to high performance 1.66khz
  uint8_t write_val = scale << 2;
  write_val |= 0x40; // set odr to 104 hz
  this->imu_sensor.writeReg(LSM6::CTRL2_G, write_val);
  gyro_scale_ = gyro_scales_[scale];
}

Imu::imu_data_t Imu::getData() {
  this->disableInterrupts();
  imu_data_t tof_data = data_;
  this->enableInterrupts();
  this->data_ready_ = false; // data was read
  return tof_data;
}


void Imu::setTargetHeading(int angle) {
  curr_heading_ = 0; // reset relative angle
  target_heading_ = angle;
}

int Imu::getTargetHeading() {
  return target_heading_;
}

void Imu::calibrateImu() {
  delay(1000);
  Serial.println("Starting IMU calibration");
  for (int i = 0; i < CALIBRATION_SAMPLE_NUM; i++){
    imu_sensor.read();
    gyro_offsets[0] += GYRO_GAIN(imu_sensor.g.x);
    gyro_offsets[1] += GYRO_GAIN(imu_sensor.g.y);
    gyro_offsets[2] += GYRO_GAIN(imu_sensor.g.z);
    accel_offsets[0] += ACCEL_GAIN(imu_sensor.a.x);
    accel_offsets[1] += ACCEL_GAIN(imu_sensor.a.y);
    accel_offsets[2] += ACCEL_GAIN(imu_sensor.a.z);
  }
  for (int i = 0; i < 3; i++){
    gyro_offsets[i] = gyro_offsets[i] / CALIBRATION_SAMPLE_NUM;
    accel_offsets[i] = accel_offsets[i] / CALIBRATION_SAMPLE_NUM;
  }
}