#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <VL53L1X.h>
#include "Wire.h"
#include "precheck.h"
#include "Sensor.h"

#define MAX_DIST 1000
#define SCAN_INTERVAL_MS  20
#define TOF_RANGE_MODE VL53L1X::Short


class TofSensor: public Sensor {
  public:
  typedef struct tof_data {
    uint16_t range;
  } tof_data_t;

  /*
   * Constructor for the TOF sensor
   * @param i2c bus to use for sensor
   * @param optional sensor name
   */
  TofSensor(TwoWire *bus, int xshut_pin, String id=""):Sensor(id){
    xshut_pin_ = xshut_pin;
    is_initialized_ = false;
    tof_sensor_.setBus(bus);
    initSensor();
  };

  /*
   * MUST BE CALLED AFTER INIT
   * init after the sensor address has been changed using xshut
   */
  void startTof();

  /*
   * get the latest data
   * @return range data
   */
  tof_data getData();

  /*
   * perform sensor read
   */
  int readSensor() override;

  private:
  bool is_initialized_;
  int xshut_pin_;
  tof_data_t data_;
  VL53L1X tof_sensor_;

  /*
   * function to initialize the sensor
   */
  void initSensor() override;
};

#endif