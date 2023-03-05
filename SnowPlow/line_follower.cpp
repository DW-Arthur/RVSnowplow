#include "line_follower.h"


int LineSensor::readSensor() {
  data_.val_left = analogRead(PIN_LINE_L) > thresh_ ? BLACK : WHITE;
  data_.val_mid = analogRead(PIN_LINE_M) > thresh_ ? BLACK : WHITE;
  data_.val_right = analogRead(PIN_LINE_R) > thresh_ ? BLACK : WHITE;
  this->data_ready_ = true; // data was read
}

LineSensor::line_data_t LineSensor::getData() {
  this->disableInterrupts();
  line_data_t line_data = data_;
  this->enableInterrupts();
  this->data_ready_ = false; // data was read
  return line_data;
}

void LineSensor::initSensor() {
  pinMode(PIN_LINE_L, INPUT);
  pinMode(PIN_LINE_M, INPUT);
  pinMode(PIN_LINE_R, INPUT);
}
