#include "motor_driver.h"
#include "line_follower.h"
#include "tof_sensor.h"
#include"Sensor.h"
#include "fsm.h"
#include "IRSensor.h"
#include "ultrasonic.h"
#include "scheduler.h"
#include "gyroscope.h"
#include "Wire.h"
#include "precheck.h"
#include "watchdog.h"

void watchdogSetup(void){}

Fsm fsm;
MotorDriver *motors;

// sensors
Ultrasonic *ultrasonic_l;
Ultrasonic *ultrasonic_r;
TofSensor *tof_left;
TofSensor *tof_right;
LineSensor *line_r;
LineSensor *line_l;
IRSensor *ir_r;
IRSensor *ir_l;

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  Wire1.setClock(400000);
  randomSeed(analogRead(0));
  delay(2000); // give time after plugging in
  Precheck::init();
  motors = new MotorDriver();

  ultrasonic_r = new Ultrasonic(33, 32, "UR");
  ultrasonic_l = new Ultrasonic(35, 34, "UL");
  line_l = new LineSensor(A0, A1, A2, 890, "LL");
  line_r = new LineSensor(A3, A4, A5, 870, "LR");
  ir_r = new IRSensor(44);
  ir_l = new IRSensor(45);

  tof_left = new TofSensor(&Wire1, 51, "TL");
  tof_right = new TofSensor(&Wire1, 50, "TR");

  // start tofs, will change address with xshut pin
  tof_right->startTof();
  tof_left->startTof();

  Scheduler::addSensor(ultrasonic_l, 50);
  Scheduler::addSensor(ultrasonic_r, 50);
  Scheduler::addSensor(line_r, 10);
  Scheduler::addSensor(line_l, 50);
  Scheduler::addSensor(tof_right, 50);
  Scheduler::addSensor(tof_left, 50);
  Scheduler::addSensor(ir_r, 50);
  Scheduler::addSensor(ir_l, 50);

  fsm.setSensors(ultrasonic_l, ultrasonic_r, line_l, line_r, tof_right, tof_left, ir_r, ir_l);
  fsm.setMotorDriver(motors);
  // start scheduler
  Scheduler::setfSM(&fsm);
  Scheduler::startScheduler();
  delay(1000); // let sensors start reading
  watchdogEnable(500);
}

void loop() {
  
}
