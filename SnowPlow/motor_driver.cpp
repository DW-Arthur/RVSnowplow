#include "motor_driver.h"


// use m defined in main for ISR
extern MotorDriver *motors;

// curr direction = 1 for foward, -1 for backward, so just add
// to encoder count
void encoderLeftISR() {
  static unsigned long prev_time = 0;
  unsigned long isr_time = millis();
  // debounce
  if (isr_time - prev_time > ENCODER_DEBOUNCE_MS){
    // check if target hit
    if (motors->incEncoderLeft() >= motors->getEncoderTargetLeft()){
      motors->stopRobot();
    }
  }
  prev_time = isr_time;
}

void encoderRightISR() {
  static unsigned long prev_time = 0;
  unsigned long isr_time = millis();
  // debounce
  if (isr_time - prev_time > ENCODER_DEBOUNCE_MS){
    // check if target hit
    if (motors->incEncoderRight() >= motors->getEncoderTargetRight()){
      motors->stopRobot();
    }
  }
  prev_time = isr_time;
}

/*
 * offsets to array, hidden in cpp
 */
int motor_offsets[NUM_MOTORS] = {
  MOTOR1_FLIP,
  MOTOR2_FLIP,
  MOTOR3_FLIP,
  MOTOR4_FLIP
};

MotorDriver::MotorDriver() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  motors[0] = new CytronMD(PWM_DIR, MOTOR1_PWM, MOTOR1_DIR_PIN);
  motors[1] = new CytronMD(PWM_DIR, MOTOR2_PWM, MOTOR2_DIR_PIN);
  motors[2] = new CytronMD(PWM_DIR, MOTOR3_PWM, MOTOR3_DIR_PIN);
  motors[3] = new CytronMD(PWM_DIR, MOTOR4_PWM, MOTOR4_DIR_PIN);
  curr_robot_speed_ = 0;
}

void MotorDriver::turnRobot(int16_t speed) {
  setMotorSpeed(MOTOR_FL, -speed);
  setMotorSpeed(MOTOR_BL, -speed);
  setMotorSpeed(MOTOR_FR, speed);
  setMotorSpeed(MOTOR_BR, speed);
  curr_robot_speed_ = speed; // as a temp
}

void MotorDriver::setRobotSpeed(int16_t wheel_speed) {
  for (int i = 0; i < NUM_MOTORS; i++)
    motors[i]->setSpeed(wheel_speed*motor_offsets[i]);
  curr_robot_speed_ = wheel_speed;
}

int16_t MotorDriver::getSetSpeed(){
  return curr_robot_speed_;
}

void MotorDriver::setMotorSpeed(int motor_num, int16_t wheel_speed) {
  if (motor_num >= NUM_MOTORS)
    return;

  motors[motor_num]->setSpeed(wheel_speed*motor_offsets[motor_num]);
}

void MotorDriver::moveRobot(int distance_cm, int16_t speed) {
  encoder_left_count = 0; // reset encoder
  encoder_target_left = DISTCM_TO_ENC(abs(distance_cm));

  // only use 1 encoder, both wheels should move at same speed for forwards
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftISR, RISING);
  if (distance_cm < 0)
    speed = -speed;
    
  curr_robot_speed_ = speed;
  setRobotSpeed(speed);
}

void MotorDriver::stopRobot() {
  curr_robot_speed_ = 0;
  setRobotSpeed(0);
  detachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN));
}

int MotorDriver::incEncoderLeft() {
  encoder_left_count++;
  return encoder_left_count;
}

int MotorDriver::incEncoderRight() {
  encoder_right_count++;
  return encoder_right_count;
}

int MotorDriver::getEncoderCountLeft() {
  return encoder_left_count;
}

int MotorDriver::getEncoderCountRight() {
  return encoder_right_count;
}

int MotorDriver::getEncoderTargetRight(){
  return encoder_target_right;
}

int MotorDriver::getEncoderTargetLeft(){
  return encoder_target_left;
}

direction_t MotorDriver::getCurrDirection() {
  return curr_direction;
}
