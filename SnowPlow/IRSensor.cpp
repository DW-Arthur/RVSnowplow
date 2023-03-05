#include "IRSensor.h"


void IRSensor::initSensor() {
  pinMode(this->ir_pin_, INPUT);
  this->data_ready_ = false;
}

int IRSensor::readSensor() {
  data_.obstacle = !digitalRead(this->ir_pin_);
  this->data_ready_ = true;
}

IRSensor::ir_data_t IRSensor::getData() {
  this->disableInterrupts();
  ir_data_t iData = data_;
  this->enableInterrupts();
  this->data_ready_ = false; // data was read
  return iData;
}

