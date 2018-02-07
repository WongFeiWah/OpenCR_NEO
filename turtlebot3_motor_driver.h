

/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */
#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_
// add by huang
#define MOTOR_370_333RPM

#ifdef MOTOR_370_333RPM
//#define KP  0.4
//#define KI  0.04
//#define KD  0.0008

#define KP  2.6
#define KI  0.4
#define KD  0.9
/*
#define KP  2.0
#define KI  0.36
#define KD  0.8*/
#define ROTATE_SPEED 1.0
#endif


#define LEFT                             1
#define RIGHT                            0


#define BUADRATE 115200
#define intervalTime_timer8  150000  //30ms  intervalTime_timer8 should be larger than 100ms without watchdog
#define amount_pulse 5    //arduino calculates the speed of a wheel every "amount_pulse"  pulses
#define dis_pwm  30.0   //the difference of each neighbor pwm is not bigger than 30 : the limitation of acceleration

#define  OUTPUT_READABLE_GYRO
#define  OUTPUT_READABLE_YAWPITCHROLL

// SERVO to control through ROS
//#define SERVO_PIN 4
#ifdef SERVO_PIN
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
#endif

#define DEBUG

#ifdef DEBUG
#define debug_print(x) Serial2.print(x)
#define debug_println(x) Serial2.println(x)
#define debug_print_hex(x) Serial2.print(x, HEX)
#else
#define debug_print(x)
#define debug_println(x)
#define debug_print_hex(x)
#endif

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY            240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0

#define DXL_LEFT_ID                     1       // ID of left motor
#define DXL_RIGHT_ID                    2       // ID of right motor
#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque


// add by huang
//the pin num of motor, two pins for 1 motor,  MOTOR1_A and MOTOR1_B are for the motor1
#define MOTOR1_A  6
#define MOTOR1_B  9
#define MOTOR2_A  10
#define MOTOR2_B  11


//the pin number of encoder, ENCODERX is for motorX
#define ENCODER1_A  5
#define ENCODER1_B  2
#define ENCODER1_R  0
#define ENCODER2_A  12
#define ENCODER2_B  7
#define ENCODER2_R  3


#ifndef BTYE
#define BYTE unsigned char
#endif



typedef struct
{
  int32_t counter;
  int count;
  char dir;
  char encoder_a;
  char encoder_b;
  char encoder_exit;
  unsigned long tick;
}Wheel;

typedef struct
{

    float v_motor1;
    float v_motor2;
 //   float v_motor3;

}RRobotData;

typedef struct{
  TIM_HandleTypeDef tf_TIM;
  TIM_OC_InitTypeDef tf_OC;
  uint32_t tim_ch;
}PWM_CONFIG;

typedef struct
{
	
double output;

double target;
double feedback;
double last_feedback;

double e_0;			//this error
double e_1;			//last error
double e_2;			//pre  error

double Proportion;			//this error
double Integral;			//last error
double Differential;			//pre  error


double Kp;
double Ki;
double Kd;

int motorA;
int motorB;

}MOTOR_PID;

class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  bool init(void);
  bool readEncoder(int32_t &left_value, int32_t &right_value);
  bool speedControl(int64_t left_wheel_value, int64_t right_wheel_value, float left_speed = 0, float right_speed = 0);
  // add by huang
  void Count_and_Direction(Wheel *omni);


  
  RRobotData speed_motor;  
  Wheel omni_wheel[2];
  float ang[2] ;   //the speed of each wheel,r/min
  //MOTOR_PID m_pid[LEFT];
  //MOTOR_PID m_pid[RIGHT];
  MOTOR_PID m_pid[2];
  
 private:
	float	protocol_version_;
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
