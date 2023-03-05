#include "fsm.h"

int rand_dirs[] = {-1, 1}; // pick rand direction for spin

Fsm::Fsm() {
    curr_state = FORWARD_STATE;
    rand_delay_ms_ = 0;
} 

void Fsm::setSensors(Ultrasonic *ult_left, Ultrasonic *ult_right, LineSensor *line_l, LineSensor *line_r,
                     TofSensor *tof_right, TofSensor *tof_left, IRSensor *ir_r, IRSensor *ir_l) {
  this->ult_right_ = ult_right;
  this->ult_left_ = ult_left;
  this->line_l_ = line_l;
  this->line_r_ = line_r;
  this->tof_left_ = tof_left;
  this->tof_right_ = tof_right;
  this->ir_r_ = ir_r;
  this->ir_l_ = ir_l;
}

void Fsm::setMotorDriver(MotorDriver *motors) {
  this->motor_driver_ = motors;
}

bool Fsm::delayFsm(int time_ms) {
  if (delay_time_ == 0){
    delay_time_ = millis() + time_ms;
  }else if (millis() >= delay_time_){
    delay_time_ = 0;
    return true;
  }
  return false;
}

state_t Fsm::nextState(){
  float range_ult;
  uint16_t range_tof;
  bool ir_reading;
  LineSensor::line_data_t line_reading;
  static int line_cnt_left;
  static int line_cnt_right;

  switch (curr_state) {

    // if we are in the done state
    case DONE_STATE:
      motor_driver_->stopRobot(); // stop!
      break;
    
    
    // if we are in the forward state
    case FORWARD_STATE:
      // set speed if not done yet
      if (motor_driver_->getSetSpeed() == 0)
        motor_driver_->setRobotSpeed(DEFAULT_SPEED);

      // check if obstacle infront of right TOF
      if (tof_right_->isDataReady()){
        range_tof = tof_right_->getData().range;
        if(range_tof < MIN_TOF_RANGE && range_tof != 0){
          curr_state = TURN_LEFT_STATE; // turn left
          motor_driver_->stopRobot();
          return curr_state;        
        }
      }

      // check if obstacle infront of left TOF
      if (tof_left_->isDataReady()){
        range_tof = tof_left_->getData().range;
        if(range_tof < MIN_TOF_RANGE && range_tof != 0){
          curr_state = TURN_RIGHT_STATE; // turn right
          motor_driver_->stopRobot();
          return curr_state;        
        }
      }

      // check if obstacle infront of left Ultrasonic
      if(ult_left_->isDataReady()){
        range_ult = ult_left_->getData().range;
        if (range_ult < MIN_ULT_RANGE && range_ult != -1){
          curr_state = TURN_RIGHT_STATE; // turn right
          motor_driver_->stopRobot();
          return curr_state;
        } 
      }

      // check if obstacle infront of right Ultrasonic
      if(ult_right_->isDataReady()){
        range_ult = ult_right_->getData().range;
        if (range_ult < MIN_ULT_RANGE && range_ult != -1){
          curr_state = TURN_LEFT_STATE; // turn left
          motor_driver_->stopRobot();
          return curr_state;
        }
      }

      // check if right ir sensor sees obstacle
      if(ir_r_->isDataReady()){
        ir_reading = ir_r_->getData().obstacle;
        if(ir_reading){
          curr_state = TURN_LEFT_STATE;
          motor_driver_->stopRobot();
          return curr_state;
        }
      }

      // check if left ir sensor sees obstacle
      if(ir_l_->isDataReady()){
        ir_reading = ir_l_->getData().obstacle;
        if(ir_reading){
          curr_state = TURN_RIGHT_STATE;
          motor_driver_->stopRobot();
          return curr_state;
        }
      }

      // check if the left line sensor has an line under it
      if(line_l_->isDataReady()){
        line_reading = line_l_->getData();
        if (line_reading.val_left || line_reading.val_mid || line_reading.val_right){
          line_cnt_left++;
        }
      }

      // check if the right line sensor has an line under it
      if(line_r_->isDataReady()){
        line_reading = line_r_->getData();
        if (line_reading.val_left || line_reading.val_mid || line_reading.val_right){
          line_cnt_right++;
        }
      }

      if(line_cnt_left >= 2 || line_cnt_right >= 2){
        line_cnt_left = 0;
        line_cnt_right = 0;
        curr_state = REVERSE_STATE;
      }
      
      break;
    
  
    // if we are in the turn left state
    case TURN_LEFT_STATE:
      // start the turn (CCW)
      if (motor_driver_->getSetSpeed() != -DEFAULT_TURN_SPEED)
        motor_driver_->turnRobot(-DEFAULT_TURN_SPEED);

      // get latest readings of right sensors
      if(ult_right_->isDataReady() && tof_right_->isDataReady() && ir_r_->isDataReady()){
        range_ult = ult_right_->getData().range;
        range_tof = tof_right_->getData().range;
        ir_reading = ir_r_->getData().obstacle;
        
        // continue turn until no obstacles infront of rigth sensors
        if ((range_ult > MIN_ULT_RANGE || range_ult == -1) && 
          (range_tof > MIN_TOF_RANGE || range_tof == 0) && !ir_reading) {
          motor_driver_->stopRobot();
          curr_state = FORWARD_STATE;
        }
      }

      break;


    // if we are in the turn right state
    case TURN_RIGHT_STATE:
      // start the turn (CW)
      if (motor_driver_->getSetSpeed() != DEFAULT_TURN_SPEED)
        motor_driver_->turnRobot(DEFAULT_TURN_SPEED);

      // get latest readings of left sensors
      if(ult_left_->isDataReady() && tof_left_->isDataReady() && ir_l_->isDataReady()){
        range_ult = ult_left_->getData().range;
        range_tof = tof_left_->getData().range;
        ir_reading = ir_l_->getData().obstacle;

        // continue turn until no obstacles infront of left sensors
        if ((range_ult > MIN_ULT_RANGE || range_ult == -1) &&
        (range_tof > MIN_TOF_RANGE || range_tof == 0) && !ir_reading) {
          motor_driver_->stopRobot();
          curr_state = FORWARD_STATE;
        }
      }
      break;

    
    // if in reverse state
    case REVERSE_STATE:
      if(motor_driver_->getSetSpeed() != -DEFAULT_SPEED)
        motor_driver_->setRobotSpeed(-DEFAULT_SPEED);
      
      DELAY_FSM(475);
      motor_driver_->stopRobot();
      curr_state = TURN_BOUNDARY_STATE;
      break;
      
    
    // if in turn after boundary state, will turn a random amount
    case TURN_BOUNDARY_STATE:
      if(motor_driver_->getSetSpeed() == 0) // spin random direction
        motor_driver_->turnRobot(rand_dirs[random(0,2)] * DEFAULT_TURN_SPEED);
      
      if(rand_delay_ms_ == 0)
        rand_delay_ms_ = random(MIN_TURN_TIME, MAX_TURN_TIME);

      DELAY_FSM(rand_delay_ms_); // make random???

      rand_delay_ms_ = 0;
      motor_driver_->stopRobot();
      curr_state = FORWARD_STATE;
      break;

  return curr_state;
  }
}
