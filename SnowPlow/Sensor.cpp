#include "Sensor.h"


Sensor::Sensor(String id) {
  sensor_id_ = id;
  if (id != "")
    Serial.print("Initializing ");
    Serial.println(id);
  interrupts_enabled_ = true;
}

void Sensor::enableInterrupts() {
  interrupts_enabled_ = true;
}

void Sensor::disableInterrupts() {
  interrupts_enabled_ = false;
}

String Sensor::getSensorId() {
  return sensor_id_;
}