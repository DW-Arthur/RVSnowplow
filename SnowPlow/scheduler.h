#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "Sensor.h"
#include "DueTimer.h"
#include "Arduino.h"
#include "fsm.h"

#define MAX_SENSORS        10
#define SCHEDULE_PERIOD    10000  // 10ms


/*
 * No need for a scheduler class, will only have 1
 * Scheduler namespace offers methods to register new sensors
 * Adding sensors will schedule them to run at a set frequency
 *
 * Uses 1 hardware timer, creates soft timers for each sensor
 */
class Scheduler {
  public:
  /*
   * Add a new sensor to run every x ms
   * @param Sensor : sensor to schedule
   * @param interval_ms : time between each sensor read
   */
  static void addSensor(Sensor *s, long interval_ms);


  static void setfSM(Fsm *fsm);

  /*
   * Start the scheduler
   */
  static void startScheduler();

};


#endif