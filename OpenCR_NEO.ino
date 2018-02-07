
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

#include "turtlebot3_core_config.h"

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
HardwareTimer Timer(TIMER_CH1);

/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> init_pose_sub("initialpose", InitPoseCallback);

/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Command velocity of Turtlebot3 using RC100 remote controller
geometry_msgs::Twist cmd_vel_rc100_msg;
ros::Publisher cmd_vel_rc100_pub("cmd_vel_rc100", &cmd_vel_rc100_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

std_msgs::Float64MultiArray sonar1_msg;
ros::Publisher sonar_pub("sonar1_msg", &sonar1_msg);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tfbroadcaster;

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[4];

/*******************************************************************************
* Declaration for motor
*******************************************************************************/
Turtlebot3MotorDriver motor_driver;
bool init_encoder_[2]  = {false, false};
int32_t last_diff_tick_[2];
int32_t last_tick_[2];
double last_rad_[2];
double last_velocity_[2];
double goal_linear_velocity  = 0.0;
double goal_angular_velocity = 0.0;

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
cIMU imu;

/*******************************************************************************
* Declaration for RC100 remote controller
*******************************************************************************/
RC100 remote_controller;
double const_cmd_vel    = 0.6;

/*******************************************************************************
* Declaration for test drive
*******************************************************************************/
bool start_move = false;
bool start_rotate = false;
int32_t last_left_encoder  = 0;
int32_t last_right_encoder = 0;

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint"};
float joint_states_pos[2] = {0.0, 0.0};
float joint_states_vel[2] = {0.0, 0.0};
float joint_states_eff[2] = {0.0, 0.0};

/*******************************************************************************
* Declaration for LED
*******************************************************************************/
#define LED_TXD         0
#define LED_RXD         1
#define LED_LOW_BATTERY 2
#define LED_ROS_CONNECT 3

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
#define BATTERY_POWER_OFF             0
#define BATTERY_POWER_STARTUP         1
#define BATTERY_POWER_NORMAL          2
#define BATTERY_POWER_CHECK           3
#define BATTERY_POWER_WARNNING        4

static bool    setup_end       = false;
static uint8_t battery_voltage = 0;
static float   battery_valtage_raw = 0;
static uint8_t battery_state   = BATTERY_POWER_OFF;

/*******************************************************************************
* Declaration for PID, for speedcontrol
*******************************************************************************/
int timer_timer;
long count_timer1= 0 ; // number of timer2() called
long count_timer2= 0 ; // number of timer2() called

unsigned long watch_dog = 0, last_time = 0;
 
bool pidflag=false;

double test_speed = 0.2;
double test_d = 0.6;
int64_t end_tick = 0;

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{  
  // add by huang
  drv_pwm_release(MOTOR1_A);
  drv_pwm_set_freq(MOTOR1_A,4000);
  drv_pwm_release(MOTOR1_B);
  drv_pwm_set_freq(MOTOR1_B,4000);
  drv_pwm_release(MOTOR2_A);
  drv_pwm_set_freq(MOTOR2_A,4000);
  drv_pwm_release(MOTOR2_B);
  drv_pwm_set_freq(MOTOR2_B,4000);
  
  pidflag=false;watch_dog=0;last_time=0;
  
  motor_driver.speed_motor.v_motor1 = 0;
  motor_driver.speed_motor.v_motor2 = 0;
  
  motor_driver.omni_wheel[0].encoder_a = ENCODER1_A;
  motor_driver.omni_wheel[0].encoder_b = ENCODER1_B;
  motor_driver.omni_wheel[0].encoder_exit = ENCODER1_R;
  motor_driver.omni_wheel[0].counter = 0;
  motor_driver.omni_wheel[0].count = 0;
  motor_driver.omni_wheel[0].dir = 0;
  
  motor_driver.omni_wheel[1].encoder_a = ENCODER2_A;
  motor_driver.omni_wheel[1].encoder_b = ENCODER2_B;
  motor_driver.omni_wheel[1].encoder_exit = ENCODER2_R;
  motor_driver.omni_wheel[1].counter = 0;
  motor_driver.omni_wheel[1].count = 0;
  motor_driver.omni_wheel[1].dir = 0;


  Sonar_Init();

  //****************************************************************************************
  //**    Motor Initialization
  //****************************************************************************************  
  // set motor Pin
  
  //****************************************************************************************
  //**    Encoder Initialization
  //****************************************************************************************  
  // set encoder Pin
  pinMode(ENCODER1_B, INPUT_PULLUP);
   // pinMode(3, INPUT_PULLDOWN);
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  //  pinMode(8, INPUT_PULLDOWN);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  
  // set encoder irp falling   
  attachInterrupt(motor_driver.omni_wheel[0].encoder_exit, EnCoder1_Handle, FALLING);
  attachInterrupt(motor_driver.omni_wheel[1].encoder_exit, EnCoder2_Handle, FALLING);
  
  // set timer8
  Timer.stop();
  Timer.setPeriod(60000);//intervalTime_timer8);           // in microseconds 15ms
  Timer.attachInterrupt(Timer8_Handle);
  Timer.start();  
  
//****************************************************************************************
//**    ROS Initialization
//****************************************************************************************    
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(init_pose_sub);
  nh.advertise(sensor_state_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(sonar_pub);
  tfbroadcaster.init(nh);

  nh.loginfo("Connected to OpenCR board!");

  // Setting for Dynamixel motors
  motor_driver.init();

  // Setting for IMU
  imu.begin();

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  remote_controller.begin(1);  // 57600bps baudrate for RC100 control

  cmd_vel_rc100_msg.linear.x  = 0.0;
  cmd_vel_rc100_msg.angular.z = 0.0;

  // Setting for SLAM and navigation (odometry, joint states, TF)
  odom_pose[0] = 0.0;
  odom_pose[1] = 0.0;
  odom_pose[2] = 0.0;

  joint_states.header.frame_id = "base_footprint";
  joint_states.name            = joint_states_name;

  joint_states.name_length     = 2;
  joint_states.position_length = 2;
  joint_states.velocity_length = 2;
  joint_states.effort_length   = 2;

  prev_update_time = millis();

  pinMode(13, OUTPUT);

  SerialBT2.begin(57600);

  Serial2.begin(115200);

  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  test_Serial_Control();
  // add by huang
  last_time = millis() - watch_dog;
  if ( watch_dog != 0 && last_time > 1300 ) {
    goal_linear_velocity  = 0;
    goal_angular_velocity = 0;
    watch_dog = 0;
  }



  if ((millis()-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_PERIOD))
  {
    tTime[0] = millis();
    //controlMotorSpeed();
  }

  if ((millis()-tTime[1]) >= (1000 / CMD_VEL_PUBLISH_PERIOD))
  {
    cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);

    //Sonar_Update(sonar1_msg);
    //sonar_pub.publish(&sonar1_msg);
    
    tTime[1] = millis();
  }

  if ((millis()-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_PERIOD))
  {
    publishSensorStateMsg();
    publishDriveInformation();
    tTime[2] = millis();
  }

  if ((millis()-tTime[3]) >= (1000 / IMU_PUBLISH_PERIOD))
  {
    publishImuMsg();
    tTime[3] = millis();
  }

  

  // Check push button pressed for simple test drive
  checkPushButtonState();

  // Update the IMU unit
  imu.update();



  // Start Gyro Calibration after ROS connection
  updateGyroCali();

  // Show LED status
  showLedStatus();

  // Update Voltage
  updateVoltageCheck();

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();
}

float velocity_calculate(Wheel *omni) {
  float ang;
  ang = float(omni->count*ROTATE_SPEED);
/*  if (omni->dir == 1) {
    ang = -ang;
  }*/
  omni->count = 0;
  return ang;
}

void Timer8_Handle() {
  
  motor_driver.ang[0] = velocity_calculate(&motor_driver.omni_wheel[0]);
  if (motor_driver.omni_wheel[0].dir == 0) {
    motor_driver.ang[0] = -motor_driver.ang[0];
  }
  motor_driver.ang[1] = velocity_calculate(&motor_driver.omni_wheel[1]);
  if (motor_driver.omni_wheel[1].dir == 1) {
    motor_driver.ang[1] = -motor_driver.ang[1];
  }

  controlMotorSpeed();
}

void EnCoder1_Handle(){
	motor_driver.Count_and_Direction(&motor_driver.omni_wheel[0]);
}

void EnCoder2_Handle(){
	motor_driver.Count_and_Direction(&motor_driver.omni_wheel[1]);
}


/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_linear_velocity  = cmd_vel_msg.linear.x;
  goal_angular_velocity = cmd_vel_msg.angular.z;

  pidflag = true;
  watch_dog = millis();
}

static double last_theta = 0.0;
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    odom_pose[0] = msg.pose.pose.position.x;
    odom_pose[1] = msg.pose.pose.position.y;
    odom_pose[2] = atan2f(msg.pose.pose.orientation.y*msg.pose.pose.orientation.z + msg.pose.pose.orientation.x*msg.pose.pose.orientation.w,
                       0.5f - msg.pose.pose.orientation.z*msg.pose.pose.orientation.z - msg.pose.pose.orientation.w*msg.pose.pose.orientation.w);
    last_theta = odom_pose[2];
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";
  
  imu_msg.angular_velocity.x = imu.SEN.gyroADC[0];
  imu_msg.angular_velocity.y = imu.SEN.gyroADC[1];
  imu_msg.angular_velocity.z = imu.SEN.gyroADC[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = imu.SEN.accADC[0];
  imu_msg.linear_acceleration.y = imu.SEN.accADC[1];
  imu_msg.linear_acceleration.z = imu.SEN.accADC[2];
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = imu.quat[0];
  imu_msg.orientation.x = imu.quat[1];
  imu_msg.orientation.y = imu.quat[2];
  imu_msg.orientation.z = imu.quat[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(&imu_msg);

  tfs_msg.header.stamp    = nh.now();
  tfs_msg.header.frame_id = "base_link";
  tfs_msg.child_frame_id  = "imu_link";
  tfs_msg.transform.rotation.w = imu.quat[0];
  tfs_msg.transform.rotation.x = imu.quat[1];
  tfs_msg.transform.rotation.y = imu.quat[2];
  tfs_msg.transform.rotation.z = imu.quat[3];

  tfs_msg.transform.translation.x = 0.1;
  tfs_msg.transform.translation.y = 0.0;
  tfs_msg.transform.translation.z = 0.068;

  tfbroadcaster.sendTransform(tfs_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  int32_t current_tick;

  sensor_state_msg.stamp = nh.now();
  sensor_state_msg.battery = checkVoltage();
  
  dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
  {
    sensor_state_pub.publish(&sensor_state_msg);
  }
  else
  {
    return;
  }

  current_tick = sensor_state_msg.left_encoder;

  if (!init_encoder_[LEFT])
  {
    last_tick_[LEFT] = current_tick;
    init_encoder_[LEFT] = true;
  }

  last_diff_tick_[LEFT] = current_tick - last_tick_[LEFT];
  last_tick_[LEFT] = current_tick;
  last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];

  current_tick = sensor_state_msg.right_encoder;

  if (!init_encoder_[RIGHT])
  {
    last_tick_[RIGHT] = current_tick;
    init_encoder_[RIGHT] = true;
  }

  last_diff_tick_[RIGHT] = current_tick - last_tick_[RIGHT];
  last_tick_[RIGHT] = current_tick;
  last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = nh.now();

  // odom
  updateOdometry((double)(step_time * 0.001));
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // joint_states
  updateJoint();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);

  // tf
  updateTF(odom_tf);
  tfbroadcaster.sendTransform(odom_tf);
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool updateOdometry(double diff_time)
{
  double odom_vel[3];

  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta;
  
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick_[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick_[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  //delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;
  delta_theta = atan2f(imu.quat[1]*imu.quat[2] + imu.quat[0]*imu.quat[3],
                       0.5f - imu.quat[2]*imu.quat[2] - imu.quat[3]*imu.quat[3]) - last_theta;
  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity_[LEFT]  = wheel_l / step_time;
  last_velocity_[RIGHT] = wheel_r / step_time;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  // We should update the twist of the odometry
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  last_theta = atan2f(imu.quat[1]*imu.quat[2] + imu.quat[0]*imu.quat[3],
                      0.5f - imu.quat[2]*imu.quat[2] - imu.quat[3]*imu.quat[3]);

  return true;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void updateJoint(void)
{
  joint_states_pos[LEFT]  = last_rad_[LEFT];
  joint_states_pos[RIGHT] = last_rad_[RIGHT];

  joint_states_vel[LEFT]  = last_velocity_[LEFT];
  joint_states_vel[RIGHT] = last_velocity_[RIGHT];

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom.header.frame_id = "odom";
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

/*******************************************************************************
* Receive remocon (RC100) data
*******************************************************************************/
void receiveRemoteControlData(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if (received_data & RC100_BTN_U)
    {
      cmd_vel_rc100_msg.linear.x += VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }
    else if (received_data & RC100_BTN_D)
    {
      cmd_vel_rc100_msg.linear.x -= VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    }

    if (received_data & RC100_BTN_L)
    {
      cmd_vel_rc100_msg.angular.z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }
    else if (received_data & RC100_BTN_R)
    {
      cmd_vel_rc100_msg.angular.z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    }

    if (received_data & RC100_BTN_6)
    {
      cmd_vel_rc100_msg.linear.x  = const_cmd_vel;
      cmd_vel_rc100_msg.angular.z = 0.0;
    }
    else if (received_data & RC100_BTN_5)
    {
      cmd_vel_rc100_msg.linear.x  = 0.0;
      cmd_vel_rc100_msg.angular.z = 0.0;
    }

    if (cmd_vel_rc100_msg.linear.x > MAX_LINEAR_VELOCITY)
    {
      cmd_vel_rc100_msg.linear.x = MAX_LINEAR_VELOCITY;
    }

    if (cmd_vel_rc100_msg.angular.z > MAX_ANGULAR_VELOCITY)
    {
      cmd_vel_rc100_msg.angular.z = MAX_ANGULAR_VELOCITY;
    }

    goal_linear_velocity  = cmd_vel_rc100_msg.linear.x;
    goal_angular_velocity = cmd_vel_rc100_msg.angular.z;
    pidflag=true;
  }
}

/*******************************************************************************
* Control motor speed
*******************************************************************************/
void controlMotorSpeed(void)
{
  bool dxl_comm_result = false;

  double wheel_speed_cmd[2];
  double lin_vel1;
  double lin_vel2;

  wheel_speed_cmd[LEFT]  = goal_linear_velocity*LINEAR_SCALE - (goal_angular_velocity * WHEEL_SEPARATION / 2)*ANGULAR_SCALE;
  wheel_speed_cmd[RIGHT] = goal_linear_velocity*LINEAR_SCALE + (goal_angular_velocity * WHEEL_SEPARATION / 2)*ANGULAR_SCALE;

  lin_vel1 = wheel_speed_cmd[LEFT] * VELOCITY_CONSTANT_VALUE;
  if (lin_vel1 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel1 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel1 = -LIMIT_X_MAX_VELOCITY;
  }

  lin_vel2 = wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VALUE;
  if (lin_vel2 > LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 =  LIMIT_X_MAX_VELOCITY;
  }
  else if (lin_vel2 < -LIMIT_X_MAX_VELOCITY)
  {
    lin_vel2 = -LIMIT_X_MAX_VELOCITY;
  }
  wheel_speed_cmd[LEFT] = (int)lin_vel1;
  wheel_speed_cmd[RIGHT] = (int)-lin_vel2;

  if(pidflag)
  {
#if 0
    /*debug_print(abs(motor_driver.speed_motor.v_motor1));
    debug_print(",");
    debug_print(abs(-motor_driver.speed_motor.v_motor2));
    debug_print(",");
    debug_print(abs(motor_driver.ang[0]));
    debug_print(",");
    debug_print(abs(motor_driver.ang[1]));
    debug_print(",");
    debug_print(motor_driver.omni_wheel[0].counter);
    debug_print(",");
    debug_print(motor_driver.omni_wheel[1].counter);
    debug_print(",");
    debug_print(motor_driver.m_pid[LEFT].output);
    debug_print(",");
    debug_print(motor_driver.m_pid[RIGHT].output);
    debug_print(",");
    debug_print(motor_driver.m_pid[LEFT].Integral);
    debug_print(",");
    debug_print(motor_driver.m_pid[RIGHT].Integral);
    debug_println(",0");*/
    //SendVal('1',(motor_driver.ang[0]/100.0));
    DataScope_Get_Channel_Data(motor_driver.ang[LEFT],1);
    DataScope_Get_Channel_Data(motor_driver.ang[RIGHT],2);
    DataScope_Get_Channel_Data(wheel_speed_cmd[LEFT],3);
    DataScope_Get_Channel_Data(wheel_speed_cmd[RIGHT],4);
    DataScope_Get_Channel_Data(motor_driver.m_pid[LEFT].Integral,5);
    DataScope_Get_Channel_Data(motor_driver.m_pid[RIGHT].Integral,6);
    int count = DataScope_Data_Generate(6);
    Serial.write(DataScope_OutPut_Buffer,count);
#endif
		dxl_comm_result = motor_driver.speedControl((int64_t)wheel_speed_cmd[LEFT], (int64_t)wheel_speed_cmd[RIGHT],motor_driver.ang[LEFT],motor_driver.ang[RIGHT]);

  }
  else
  {
    dxl_comm_result = motor_driver.speedControl(0, 0,motor_driver.ang[0],motor_driver.ang[1]);
  }

  if (dxl_comm_result == false)
    return;
}

/*******************************************************************************
* Get Button Press (Push button 1, Push button 2)
*******************************************************************************/
uint8_t getButtonPress(void)
{
  uint8_t button_state = 0;
  static uint32_t t_time[2];
  static uint8_t button_state_num[2] = {0, };

  for (int button_num = 0; button_num < 2; button_num++)
  {
    switch (button_state_num[button_num])
    {
     case WAIT_FOR_BUTTON_PRESS:
       if (getPushButton() & (1 << button_num))
       {
         t_time[button_num] = millis();
         button_state_num[button_num] = WAIT_SECOND;
       }
       break;

     case WAIT_SECOND:
       if ((millis()-t_time[button_num]) >= 1000)
       {
         if (getPushButton() & (1 << button_num))
         {
           button_state_num[button_num] = CHECK_BUTTON_RELEASED;
           button_state |= (1 << button_num);
         }
         else
         {
           button_state_num[button_num] = WAIT_FOR_BUTTON_PRESS;
         }
       }
       break;

     case CHECK_BUTTON_RELEASED:
       if (!(getPushButton() & (1 << button_num)))
         button_state_num[button_num] = WAIT_FOR_BUTTON_PRESS;
       break;

     default :
       button_state_num[button_num] = WAIT_FOR_BUTTON_PRESS;
       break;
    }
  }

  return button_state;
}

/*******************************************************************************
* Turtlebot3 test drive using RC100 remote controller
*******************************************************************************/
void testDrive(void)
{
  int32_t current_tick = millis();
  double diff_encoder = 0.0;
  if (start_move)
  {
    diff_encoder = test_d*1000; // (Circumference) / (The number of tick per revolution)

    if (abs(end_tick - current_tick) <= test_d*4000)
    {
      cmd_vel_rc100_msg.linear.x  = test_speed * SCALE_VELOCITY_LINEAR_X;
      goal_linear_velocity  = cmd_vel_rc100_msg.linear.x;
      pidflag = true;
    }
    else
    {
      cmd_vel_rc100_msg.linear.x  = 0.0;
      goal_linear_velocity  = cmd_vel_rc100_msg.linear.x;
      pidflag = false;
      start_move = false;
    }
  }
  else if (start_rotate)
  {
    diff_encoder = test_d / (0.207 / 330); 
    if (abs(end_tick - current_tick) <= test_d*4000)
    {
      cmd_vel_rc100_msg.linear.x  = -test_speed * SCALE_VELOCITY_LINEAR_X;
      goal_linear_velocity  = cmd_vel_rc100_msg.linear.x;
      pidflag = true;
    }
    else
    {
      cmd_vel_rc100_msg.linear.x  = 0.0;
      goal_linear_velocity  = cmd_vel_rc100_msg.linear.x;
      pidflag = false;
      start_rotate = false;
    }
  }
}

/*******************************************************************************
* Check Push Button State
*******************************************************************************/
void checkPushButtonState()
{
  uint8_t button_state = getButtonPress();

  if (button_state & (1<<0))
  {
    start_move = true;
    last_left_encoder = sensor_state_msg.left_encoder;
    last_right_encoder = sensor_state_msg.right_encoder;
  }

  if (button_state & (1<<1))
  {
    end_tick = millis();
    start_rotate = true;
    last_left_encoder = sensor_state_msg.left_encoder;
    last_right_encoder = sensor_state_msg.right_encoder;
  }

  testDrive();
}

/*******************************************************************************
* Check voltage
*******************************************************************************/
float checkVoltage(void)
{
  float vol_value;

  vol_value = getPowerInVoltage();

  return vol_value;
}

/*******************************************************************************
* Show LED status
*******************************************************************************/
void showLedStatus(void)
{
  static uint32_t t_time = millis();

  if ((millis()-t_time) >= 500)
  {
    t_time = millis();
    digitalWrite(13, !digitalRead(13));
  }

  if (getPowerInVoltage() < 11.1)
  {
    setLedOn(LED_LOW_BATTERY);
  }
  else
  {
    setLedOff(LED_LOW_BATTERY);
  }

  if (nh.connected())
  {
    setLedOn(LED_ROS_CONNECT);
  }
  else
  {
    setLedOff(LED_ROS_CONNECT);
  }

  updateRxTxLed();
}

void updateRxTxLed(void)
{
  static uint32_t rx_led_update_time;
  static uint32_t tx_led_update_time;
  static uint32_t rx_cnt;
  static uint32_t tx_cnt;

  if ((millis()-tx_led_update_time) > 50)
  {
    tx_led_update_time = millis();

    if (tx_cnt != Serial.getTxCnt())
    {
      setLedToggle(LED_TXD);
    }
    else
    {
      setLedOff(LED_TXD);
    }

    tx_cnt = Serial.getTxCnt();
  }

  if ((millis()-rx_led_update_time) > 50)
  {
    rx_led_update_time = millis();

    if (rx_cnt != Serial.getRxCnt())
    {
      setLedToggle(LED_RXD);
    }
    else
    {
      setLedOff(LED_RXD);
    }

    rx_cnt = Serial.getRxCnt();
  }
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void)
{
  static bool gyro_cali = false;
  uint32_t pre_time;
  uint32_t t_time;

  char log_msg[50];

  if (nh.connected())
  {
    if (gyro_cali == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      imu.SEN.gyro_cali_start();

      t_time   = millis();
      pre_time = millis();
      while(!imu.SEN.gyro_cali_get_done())
      {
        imu.update();

        if (millis()-pre_time > 5000)
        {
          break;
        }
        if (millis()-t_time > 100)
        {
          t_time = millis();
          setLedToggle(LED_ROS_CONNECT);
        }
      }
      gyro_cali = true;

      sprintf(log_msg, "Calibrattion End");
      nh.loginfo(log_msg);
    }
  }
  else
  {
    gyro_cali = false;
  }
}

/*******************************************************************************
* updateVoltageCheck
*******************************************************************************/
void updateVoltageCheck(void)
{
  static bool startup = false;
  static int vol_index = 0;
  static int prev_state = 0;
  static int alram_state = 0;
  static int check_index = 0;

  int i;
  float vol_sum;
  float vol_value;

  static uint32_t process_time[8] = {0,};
  static float    vol_value_tbl[10] = {0,};

  float voltage_ref       = 11.0 + 0.0;
  float voltage_ref_warn  = 11.0 + 0.0;


  if (startup == false)
  {
    startup = true;
    for (i=0; i<8; i++)
    {
      process_time[i] = millis();
    }
  }

  if (millis()-process_time[0] > 100)
  {
    process_time[0] = millis();

    vol_value_tbl[vol_index] = getPowerInVoltage();

    vol_index++;
    vol_index %= 10;

    vol_sum = 0;
    for(i=0; i<10; i++)
    {
        vol_sum += vol_value_tbl[i];
    }
    vol_value = vol_sum/10;
    battery_valtage_raw = vol_value;

    //Serial.println(vol_value);

    battery_voltage = vol_value;
  }


  if(millis()-process_time[1] > 1000)
  {
    process_time[1] = millis();

    //Serial.println(battery_state);

    switch(battery_state)
    {
      case BATTERY_POWER_OFF:
        if (setup_end == true)
        {
          alram_state = 0;
          if(battery_valtage_raw > 5.0)
          {
            check_index   = 0;
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_STARTUP;
          }
          else
          {
            noTone(BDPIN_BUZZER);
          }
        }
        break;

      case BATTERY_POWER_STARTUP:
        if(battery_valtage_raw > voltage_ref)
        {
          check_index   = 0;
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
          setPowerOn();
        }

        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if (battery_valtage_raw > 5.0)
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_CHECK;
          }
          else
          {
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_OFF;
          }
        }
        break;

      case BATTERY_POWER_NORMAL:
        alram_state = 0;
        if(battery_valtage_raw < voltage_ref)
        {
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_CHECK;
          check_index   = 0;
        }
        break;

      case BATTERY_POWER_CHECK:
        if(check_index < 5)
        {
          check_index++;
        }
        else
        {
          if(battery_valtage_raw < voltage_ref_warn)
          {
            setPowerOff();
            prev_state    = battery_state;
            battery_state = BATTERY_POWER_WARNNING;
          }
        }
        if(battery_valtage_raw >= voltage_ref)
        {
          setPowerOn();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        break;

      case BATTERY_POWER_WARNNING:
        alram_state ^= 1;
        if(alram_state)
        {
          tone(BDPIN_BUZZER, 1000, 500);
        }

        if(battery_valtage_raw > voltage_ref)
        {
          setPowerOn();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_NORMAL;
        }
        else
        {
          setPowerOff();
        }

        if(battery_valtage_raw < 5.0)
        {
          setPowerOff();
          prev_state    = battery_state;
          battery_state = BATTERY_POWER_OFF;
        }
        break;

      default:
        break;
    }
  }
}

void setPowerOn()
{
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
}

void setPowerOff()
{
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}
