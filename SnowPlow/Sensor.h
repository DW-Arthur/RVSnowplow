#ifndef SENSOR_H
#define SENSOR_H


#include "Arduino.h"

/*
 * Base class for sensor
 */
class Sensor {
  public:
  /*
   * Constructor for sensor class
   */

  Sensor(String id);

  /*
   * Function to update sensor data
   */
  virtual int readSensor()=0;


  /*
   * Return the id of the sensors
   * @return sensor id (string)
   */
  String getSensorId();

  /*
   * Enable scheduler to update sensor data
   */
  void enableInterrupts();

  /*
   * Stop scheduler from reading sensor
   */
  void disableInterrupts();

  /*
   * Return status of data 
   * @return true if ready to read
   */
  inline bool isDataReady() __attribute__((always_inline)) {
    return data_ready_;
  }

  /*
   * Return if interrupts are enabled
   * @return true if sensor interrupts are enabled
   */
  inline bool interruptsEnabled() __attribute__((always_inline)) {
    return interrupts_enabled_;
  }

  /*
   * set the time until next sensor read
   * @param optional offswet
   */
  inline void setTimerExp(int offset=0) __attribute__((always_inline)) {
    exp_time_ = millis() + interrupt_interval_ + offset - 1; // sub 1 for 0 index timer
  }

  /* 
   * Get sensor should be read at
   * @return time (millis) when timer expires
   */
  inline uint64_t getTimerExp() __attribute__((always_inline)) {
    return exp_time_;
  }

  /*
   * Set interval at which sensor should be read
   * @param interval ms
   */
  inline void setInterruptInterval(uint32_t interval_ms) __attribute__((always_inline)) {
    interrupt_interval_ = interval_ms;
  }

  /*
   * Get interval at which sensor should be read
   * @param interval ms
   */
  inline uint32_t getInterruptInterval() __attribute__((always_inline)) {
    return interrupt_interval_;
  }

  protected:
  bool data_ready_;
  /*
   * Function to initialize sensor
   */
  virtual void initSensor()=0;
  

  private:
  String sensor_id_;
  uint64_t exp_time_;
  uint64_t interrupt_interval_;
  bool interrupts_enabled_;
};


#endif