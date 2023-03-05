#ifndef FSM_H
#define FSM_H

#include "random"
#include "motor_driver.h"
#include "tof_sensor.h"
#include "line_follower.h"
#include "gyroscope.h"
#include "ultrasonic.h"
#include "IRSensor.h"

#define DEFAULT_SPEED      100 // speed that robot should move at
#define DEFAULT_TURN_SPEED 185 // speed robot should turn at
#define DEFAULT_REVERSE_CM 5  // how far to reverse on obstacle, cm

#define MIN_ULT_RANGE      25 
#define MIN_TOF_RANGE      95 

#define MIN_TURN_TIME      900
#define MAX_TURN_TIME      2100

#define DELAY_FSM(time_ms) if(!delayFsm(time_ms)) break;

/*
 * States for fsm
 */
typedef enum state {
  FORWARD_STATE,
  REVERSE_STATE,
  TURN_RIGHT_STATE,
  TURN_LEFT_STATE,
  TURN_BOUNDARY_STATE,
  DONE_STATE
}state_t;

class Fsm {
    public:
    
    /*
     * Constructor for fsm
     */
    Fsm();

    /*
     * Add sensors to the fsm, fsm will use sensors to control motor driver
     * @param left ultrasonic reference
     * @param right ultrasonic reference
     * @param left line sensor reference
     * @param right line sensor reference
     * @param front tof sensor reference
     */
    void setSensors(Ultrasonic *ult_left, Ultrasonic *ult_right, LineSensor *line_l, LineSensor *line_r,
                     TofSensor *tof_right, TofSensor *tof_left, IRSensor *ir_r, IRSensor *ir_l);

    /*
     * Set reference to motor driver, fsm will control robot with this
     * @param reference to motor driver
     */
    void setMotorDriver(MotorDriver *motors);
   
    /*
     * update the fsm, also returns the next state
     * @return new state of the fsm
     */
    state_t nextState();

    /*
     * Get the current state of the fsm
     * @return state
     */
    state_t getState();

    private:
    unsigned long delay_time_; // used for non-blocking fsm delays
    state_t curr_state; // hold current state

    // motor driver reference
    MotorDriver *motor_driver_;

    // fsm inputs
    TofSensor *tof_left_;
    TofSensor *tof_right_;
    LineSensor *line_sensor_;
    Imu *imu_sensor_;
    Ultrasonic *ult_left_;
    Ultrasonic *ult_right_;
    LineSensor *line_l_;
    LineSensor *line_r_;
    IRSensor *ir_r_;
    IRSensor *ir_l_;

    bool delayFsm(int time_ms); 
    int rand_delay_ms_;
};
#endif
