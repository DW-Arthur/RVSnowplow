#include "ultrasonic.h"


int Ultrasonic::readSensor(){
  data_.range =  ultrasonic_->measureDistanceCm(); // read sensor
  this->data_ready_ = true; // data ready to be read
}

void Ultrasonic::initSensor(){
  //create new ultrasonic class instance
  ultrasonic_ = new UltraSonicDistanceSensor(trigger_pin_, echo_pin_, MAX_RANGE);
  this->data_ready_ = false;
}

Ultrasonic::ultrsonic_data_t Ultrasonic::getData(){
  this->disableInterrupts();
  ultrsonic_data_t uData = data_; // get latest data measurements
  this->enableInterrupts();
  this->data_ready_ = false; // data was read
  return uData;
}