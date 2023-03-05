#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "CytronMotorDriver.h"

// wheel control macros
#define NUM_MOTORS              4
#define MOTOR1_DIR_PIN          28 // motor front left
#define MOTOR2_DIR_PIN          29 // motor front right
#define MOTOR3_DIR_PIN          30 // motor back left
#define MOTOR4_DIR_PIN          31 // motor back right
#define MOTOR1_PWM              2 // motor front left
#define MOTOR2_PWM              3 // motor front right
#define MOTOR3_PWM              4 // motor back left
#define MOTOR4_PWM              5 // motor back right


// macros for motor index
#define MOTOR_FL                0
#define MOTOR_BL                1
#define MOTOR_FR                2
#define MOTOR_BR                3


// if wiring flipped, change these
#define MOTOR1_FLIP              -1 // motor front left
#define MOTOR2_FLIP              -1 // motor front right 
#define MOTOR3_FLIP              -1 // motor back left
#define MOTOR4_FLIP              -1 // motor back right


// wheel encoder macros
#define ENCODER_TOOTH_COUNT    (float)10 // teeth per rev
#define WHEEL_CIRC_CM          (float)16 // circumferance of wheel cm
#define ENCODER_LEFT_PIN       44
#define ENCODER_RIGHT_PIN      45

// macro to convert dist CM to encoder count
#define DISTCM_TO_ENC(dist)   (int)(dist / (WHEEL_CIRC_CM / ENCODER_TOOTH_COUNT))


// debouncing constants
#define ENCODER_DEBOUNCE_MS    10

typedef enum direction {
  FORWARD = 1,
  BACKWARD = -1,
} direction_t;


class MotorDriver {
  public:
  MotorDriver();
  
  /*
   * turn the robot to a specified angle
   * TODO: Add imu implementaion
   * @param speed of turn + = cw, - = ccw
   */
  void turnRobot(int16_t speed);

  /* 
   * Set speed of robot
   * @param speed at which it moves, + = forward, - = backwards
   */  
  void setRobotSpeed(int16_t wheel_speed);

  /*
   * Get the speed of the robot
   * @return speed
   */ 
  int16_t getSetSpeed();

  /*
   * Set the speed of a single motor
   * @param which motor
   * @param speed of motor
   */
  void setMotorSpeed(int motor_num, int16_t wheel_speed);
  
  /*
   * move robot a specified distance
   * @param distance (cm)
   * @param speed
   */
  void moveRobot(int distance_cm, int16_t speed);

  /*
   * stop the robot
    */
  void stopRobot();

  /*
   * increment the left encoder count
   * @return new count
   */
  int incEncoderLeft();

  /*
   * increment the right encoder count
   * @return new count
   */
  int incEncoderRight();

  /*
   * Get the encoder count right
   * @return right encoder count
   */
  int getEncoderCountLeft();

  /*
   * Get the encoder count left
   * @return left encoder count
   */
  int getEncoderCountRight();

  /*
   * Get the set target for right encoder
   * @return right encoder count
   */
  int getEncoderTargetRight();

  /*
   * Get the set target for left encoder
   * @return left encoder count
   */
  int getEncoderTargetLeft();

  /*
   * Get direction robot is moving
   * @return current direction
   */
  direction_t getCurrDirection();

  private:
  CytronMD* motors[NUM_MOTORS]; 

  direction_t curr_direction;
  int encoder_left_count;
  int encoder_right_count;

  int encoder_target_left;
  int encoder_target_right;

  int16_t curr_robot_speed_;
};


#endif
