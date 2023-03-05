#include "scheduler.h"



// hidden from user
static Fsm *fsm_instance;
Sensor *sensors[MAX_SENSORS];
int num_sensors = 0;
DueTimer timer = DueTimer(5);


/*
 * Add a new sensors to the scheduler o run every interval_ms
 * @param sensor reference
 * @param interval to perform read
 */
void Scheduler::addSensor(Sensor *s, long interval_ms) {
  if (num_sensors > MAX_SENSORS) {
    Serial.println("Max number of sensors added to scheduler!");
    while (1);
  }
  // Add the sensor to the
  sensors[num_sensors] = s;
  s->setInterruptInterval(interval_ms);
  num_sensors++;
}


/*
 * Hidden, not callable by user
 * callback for hardware timer, will loop through
 * sensors and call their readSensor method
 */
void onTimerExpire() {
  for (int i = 0; i < num_sensors; i++) {
    Sensor *sensor = sensors[i];
    if (sensor->interruptsEnabled() && millis() > sensor->getTimerExp()) {
      // execute read sensor
      sensor->readSensor();
      sensor->setTimerExp();
    }
  }
  //Serial.println(fsm_instance->nextState());
  fsm_instance->nextState();
  watchdogReset();
}


void Scheduler::setfSM(Fsm *fsm) {
  fsm_instance = fsm;
}

/*
 * starts the hardware timer, attaches timer callback to check which sensors are
 * scheduled to read
 */
void Scheduler::startScheduler() {
  for (auto *sensor : sensors) {
    sensor->setTimerExp();
  }

  // initialize hardware timer
  timer.attachInterrupt(onTimerExpire);
  timer.start(SCHEDULE_PERIOD);
}
