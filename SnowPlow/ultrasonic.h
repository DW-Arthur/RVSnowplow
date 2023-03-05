#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "Sensor.h"
#include <HCSR04.h> //martinsos

#define MAX_RANGE 40

class Ultrasonic: public Sensor {
public:
  /*
   * Data type for ultrasonic sensor, range in cm
   */
  typedef struct ultrsonic_data {
    float range;
  } ultrsonic_data_t;

  /*
   * Constructor for Ultrasonic, sets triggerpin and echopin
   * @param pin for trigger
   * @param pin for echo
   * @param id of sensor
   */
  Ultrasonic(int triggerpin, int echopin, String id=""):Sensor(id){
    this->trigger_pin_ = triggerpin;
    this->echo_pin_ = echopin;
    initSensor();
  };

  /*
   * returns the most recent data
   * @return ultrasonic data
   */
  ultrsonic_data_t getData();

  /*
   * callback for scheduler, updates data
   */
  int readSensor() override;

  private:
  int trigger_pin_;
  int echo_pin_;
  ultrsonic_data_t data_;
  UltraSonicDistanceSensor  *ultrasonic_;

  /*
   * hardware initializer for sensor
   */
  void initSensor() override;
  
};
#endif