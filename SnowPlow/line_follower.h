#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "Arduino.h"
#include "Sensor.h"

#define NUM_OUT_PINS 3

#define PIN_LINE_L line_pins_[0]
#define PIN_LINE_M line_pins_[1]
#define PIN_LINE_R line_pins_[2]

/*
 * Line follower class, will read value of line sensor under robot
 */
class LineSensor: public Sensor {
  public:
  /*
   * Enum for line sensor value
   */ 
  typedef enum line_col {
    WHITE = 0,
    BLACK = 1,
  } line_col_t;

  /*
   * Data type for line sensor, contains reading for
   * each pin, will either be BLACK or WHITE (1, 0)
   */
  typedef struct line_data {
    line_col_t val_left;
    line_col_t val_mid;
    line_col_t val_right;
  } line_data_t;
  
  /*
   * Constructor for Line sensor, takes pins, and optional sensor id
   * @param pin_l
   * @param pin_m
   * @param pin_r
   */
  LineSensor(int pin_l, int pin_m, int pin_r, int thresh, String id=""):Sensor(id){
    thresh_ = thresh;
    line_pins_[0] = pin_l;
    line_pins_[1] = pin_m;
    line_pins_[2] = pin_r;
    initSensor();
  };

  /*
   * Method to return latest reading from line senor
   * @return line_data_t
   */
  line_data_t getData();

  /*
   * Method to update latest sensor data
   */
  int readSensor()override;
  
  private: 
  int thresh_;
  int line_pins_[NUM_OUT_PINS];
  line_data_t data_;

  void initSensor() override;
};

#endif
