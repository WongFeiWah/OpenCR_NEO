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

#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_

#include <math.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>

#include <IMU.h>
#include <RC100.h>

#include "turtlebot3_motor_driver.h"
#include "sonar.h"
#include "Serial_cmd.h"
#define MY_PI                            3.141592653
#define CONTROL_MOTOR_SPEED_PERIOD       15   //hz
#define IMU_PUBLISH_PERIOD               50  //hz
#define SENSOR_STATE_PUBLISH_PERIOD      30   //hz
#define CMD_VEL_PUBLISH_PERIOD           30   //hz
#define DRIVE_INFORMATION_PUBLISH_PERIOD 30   //hz
#define DRIVE_TEST_PERIOD                30   //hz

#define MOTOR_PULSE                      570
#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.185           // meter (BURGER : 0.160, WAFFLE : 0.287)
//#define TURNING_RADIUS                   0.1435          // meter (BURGER : 0.080, WAFFLE : 0.1435)
//#define ROBOT_RADIUS                     0.87           // meter (BURGER : 0.105, WAFFLE : 0.220)
#define ENCODER_MIN                      -2147483648     // raw
#define ENCODER_MAX                      2147483648      // raw



#define LINEAR_SCALE                     1.0
#define ANGULAR_SCALE                    1.0
#define VELOCITY_CONSTANT_VALUE          (1/(2.0*MY_PI*WHEEL_RADIUS/MOTOR_PULSE/0.06))  //V =  r*w = 2*pi*r * RPM/330 / 0.06s = RPM       V = r * w = r * RPM * 0.10472   23.873241468266191543070176641389
                                                         //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                         // Goal RPM = V * 1263.632956882

#define MAX_LINEAR_VELOCITY              0.25   // m/s (BURGER => 0.22, WAFFLE => 0.25)
#define MAX_ANGULAR_VELOCITY             1.82   // rad/s (BURGER => 2.84, WAFFLE => 1.82)
#define VELOCITY_STEP                    0.01   // m/s
#define VELOCITY_LINEAR_X                0.01   // m/s
#define VELOCITY_ANGULAR_Z               0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X          1
#define SCALE_VELOCITY_ANGULAR_Z         1

#define TICK2RAD                         (2.0*MY_PI/MOTOR_PULSE)  //2*pi/330

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TEST_DISTANCE                    3.600     // meter
#define TEST_RADIAN                      3.14      // 180 degree

#define WAIT_FOR_BUTTON_PRESS            0
#define WAIT_SECOND                      1
#define CHECK_BUTTON_RELEASED            2

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

// Function prototypes
void publishImuMsg(void);
void publishSensorStateMsg(void);
void publishDriveInformation(void);
bool updateOdometry(double diff_time);
void updateJoint(void);
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void receiveRemoteControlData(void);
void controlMotorSpeed(void);
uint8_t getButtonPress(void);
void testDrive(void);
void checkPushButtonState(void);
float checkVoltage(void);
void showLedStatus(void);
void updateRxTxLed(void);

#endif // TURTLEBOT3_CORE_CONFIG_H_
