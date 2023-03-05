#include "tof_sensor.h"


volatile static int i2c_addr = 0x2A;

TofSensor::tof_data_t TofSensor::getData() {
  this->disableInterrupts(); 
  tof_data_t tof_data = data_;
  this->enableInterrupts();
  this->data_ready_ = false; // data was read
  return tof_data;
}

int TofSensor::readSensor() {
  if (!is_initialized_){
    return -1;
  }
  data_.range =  tof_sensor_.read(); // grab latest data
  if(data_.range >= MAX_DIST){
    data_.range = 0;
  }
  data_ready_= true;
}

void TofSensor::initSensor() { //turn off sensor
  pinMode(xshut_pin_, OUTPUT);
  digitalWrite(xshut_pin_, LOW);
  this->data_ready_ = false;
}

void TofSensor::startTof(){
  pinMode(xshut_pin_, INPUT); //turn on sensor
  delay(10);
  tof_sensor_.setTimeout(500); //set sensor timeout 
  if (!tof_sensor_.init()) // check sensor if it's initialized 
  {
    Serial.println("Failed to detect and initialize TOF sensor");
    Precheck::fail();
  }
  // rest of config
  tof_sensor_.setAddress(i2c_addr++); //change to new address
  tof_sensor_.startContinuous(20); // start sensor
  tof_sensor_.setDistanceMode(TOF_RANGE_MODE);
  is_initialized_ = true;
}