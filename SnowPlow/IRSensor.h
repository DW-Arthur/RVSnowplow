#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "Sensor.h"

class IRSensor : public Sensor {
  public:
  typedef struct ir_data {
    bool obstacle;
  }ir_data_t;

  IRSensor(int pin, String id=""):Sensor(id) {
    ir_pin_ = pin;
  }

  ir_data_t getData();

  int readSensor() override;

  private:
    int ir_pin_;
    ir_data_t data_;
    void initSensor();
};




#endif