

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

#include "turtlebot3_motor_driver.h"

PWM_CONFIG tim_handle[6] = {0};

static void myPWM_Init(uint32_t pin)
{
  PWM_CONFIG *set;
  TIM_HandleTypeDef  *pTIM;
  TIM_OC_InitTypeDef *pOC;  

  static bool isRunning = false;

  if(!isRunning)
  {
    isRunning = true;
    tim_handle[0].tf_TIM.Instance = TIM3;
    tim_handle[0].tim_ch = TIM_CHANNEL_1;
    
    tim_handle[1].tf_TIM.Instance = TIM1;
    tim_handle[1].tim_ch = TIM_CHANNEL_1;
    
    tim_handle[2].tf_TIM.Instance = TIM2;
    tim_handle[2].tim_ch = TIM_CHANNEL_3;
    
    tim_handle[3].tf_TIM.Instance = TIM9;
    tim_handle[3].tim_ch = TIM_CHANNEL_2;
    
    tim_handle[4].tf_TIM.Instance = TIM11;
    tim_handle[4].tim_ch = TIM_CHANNEL_1;
    
    tim_handle[5].tf_TIM.Instance = TIM12;
    tim_handle[5].tim_ch = TIM_CHANNEL_2;
    
  }
  
  switch(pin)
  {
    case 3:
    set = &tim_handle[0];
    break;
    case 5:
    set = &tim_handle[1];
    break;
    case 6:
    set = &tim_handle[2];
    break;
    case 9:
    set = &tim_handle[3];
    break;
    case 10:
    set = &tim_handle[4];
    break;
    case 11:
    set = &tim_handle[5];
    break;
    default:
    return;
  }

  pTIM = &set->tf_TIM;
  pOC = &set->tf_OC; 
  
  pTIM->Init.Prescaler         = 8-1;//t=216000000/8
  pTIM->Init.Period            = 216000000/8/2000-1;//1khz    

  pTIM->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  pTIM->Init.CounterMode       = TIM_COUNTERMODE_UP;
  pTIM->Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(pTIM);

  memset(pOC, 0, sizeof(TIM_OC_InitTypeDef));

  pOC->OCMode       = TIM_OCMODE_PWM1;
  pOC->OCPolarity   = TIM_OCPOLARITY_HIGH;
  pOC->OCFastMode   = TIM_OCFAST_DISABLE;
  pOC->OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  pOC->OCNIdleState = TIM_OCNIDLESTATE_RESET;
  pOC->OCIdleState  = TIM_OCIDLESTATE_RESET;

  pOC->Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(pTIM, pOC, set->tim_ch);
  HAL_TIM_PWM_Start(pTIM, set->tim_ch); 
}

static void myPWM_Write(uint32_t pin,uint8_t Pulse)
{
  PWM_CONFIG *set;
  TIM_HandleTypeDef  *pTIM;
  TIM_OC_InitTypeDef *pOC;  
  switch(pin)
  {
    case 3:
    set = &tim_handle[0];
    break;
    case 5:
    set = &tim_handle[1];
    break;
    case 6:
    set = &tim_handle[2];
    break;
    case 9:
    set = &tim_handle[3];
    break;
    case 10:
    set = &tim_handle[4];
    break;
    case 11:
    set = &tim_handle[5];
    break;
    default:
    return;
  }

  pTIM = &set->tf_TIM;
  pOC = &set->tf_OC; 
  Pulse = constrain(Pulse, 0, (1<<8)-1);
  pOC->Pulse = map( Pulse, 0, (1<<8)-1, 0, pTIM->Init.Period+1 );
  HAL_TIM_PWM_ConfigChannel(pTIM, pOC, set->tim_ch);
  HAL_TIM_PWM_Start(pTIM, set->tim_ch);
}

void PID_controller(MOTOR_PID *pid)
{
	if(abs(pid->target) <= 200 && abs(pid->target) >= 0)
	{
		double out;  
		pid->Proportion = abs(pid->target) - pid->feedback;
    pid->Integral += (pid->Ki*pid->Proportion);
		pid->Integral = constrain(pid->Integral,0,200);
		pid->Differential = (pid->feedback - pid->last_feedback);

		out = pid->Kp*pid->Proportion + pid->Integral - pid->Kd*pid->Differential; 
/*   
		if (out - pid->output > dis_pwm) {
      out = pid->output + dis_pwm;
    }*/
    pid->output = constrain(out,0,200);
		pid->last_feedback = pid->feedback;
    if(abs(pid->target) >= 1)
    {
      if (pid->target >= 0) 
      {
        myPWM_Write(pid->motorB, LOW);
        myPWM_Write(pid->motorA, (uint8_t)abs(pid->output));
      }
      else 
      {
        myPWM_Write(pid->motorA, LOW);
        myPWM_Write(pid->motorB, (uint8_t)abs(pid->output));
      }
    }
    else
    {
      myPWM_Write(pid->motorA, 256);
      myPWM_Write(pid->motorB, 256);      
    }
  }
  else
  {
      myPWM_Write(pid->motorA, 256);
      myPWM_Write(pid->motorB, 256);
  }
}


Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: protocol_version_(PROTOCOL_VERSION),
  m_pid_l({0}),
  m_pid_r({0})
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
}

bool Turtlebot3MotorDriver::init(void)
{
  /******************************************************************************
   PID Init
  ******************************************************************************/
	// left wheel initialize pid parameter
	m_pid_l.e_0 = 0;
	m_pid_l.e_1 = 0;
	m_pid_l.e_2 = 0;
	m_pid_l.Proportion = 0;
	m_pid_l.Integral = 40.0;
	m_pid_l.Differential = 0;
	m_pid_l.Kp = KP;
	m_pid_l.Ki = KI;
	m_pid_l.Kd = KD;
	m_pid_l.output = 0;
	m_pid_l.target = 0;
	m_pid_l.feedback = 0;
	m_pid_l.last_feedback = 0;
	m_pid_l.motorA = MOTOR1_A;
	m_pid_l.motorB = MOTOR1_B;
  myPWM_Init(m_pid_l.motorA);
  myPWM_Init(m_pid_l.motorB);
	// right wheel initialize pid parameter
	m_pid_r.e_0 = 0;
	m_pid_r.e_1 = 0;
	m_pid_r.e_2 = 0;
	m_pid_r.Proportion = 0;
	m_pid_r.Integral = 40.0;
	m_pid_r.Differential = 0;
	m_pid_r.Kp = KP;
	m_pid_r.Ki = KI;
	m_pid_r.Kd = KD;
	m_pid_r.output = 0;
	m_pid_r.target = 0;
	m_pid_r.feedback = 0;
	m_pid_r.last_feedback = 0;
	m_pid_r.motorA = MOTOR2_A;
	m_pid_r.motorB = MOTOR2_B;
  myPWM_Init(m_pid_r.motorA);
  myPWM_Init(m_pid_r.motorB);
  return true;
}

bool Turtlebot3MotorDriver::readEncoder(int32_t &left_value, int32_t &right_value)
{
  noInterrupts();
  left_value  =-omni_wheel[0].counter;
  right_value=omni_wheel[1].counter;
  interrupts();
  return true;
}

void  Turtlebot3MotorDriver::Count_and_Direction(Wheel *omni) {
	// feedback speed counter
  omni->count += 1;
  // odom counter
  if (digitalRead(omni->encoder_a) == HIGH) {

    if (digitalRead(omni->encoder_b) == LOW) {
      omni->dir = 0; //forward
      omni->counter ++;
    }
    else {
      omni->dir = 1; //backward
      omni->counter -= 1;
    }
  }
  else if (digitalRead(omni->encoder_b) == HIGH) {
    omni->dir = 0; //forward
    omni->counter ++;
  }
  else {
    omni->dir = 1; //backward
    omni->counter -= 1;
  }

}

bool Turtlebot3MotorDriver::speedControl(int64_t left_wheel_value, int64_t right_wheel_value, float left_speed, float right_speed)
{
	m_pid_l.feedback = abs(left_speed);
	m_pid_r.feedback = abs(right_speed);
  m_pid_l.target = left_wheel_value;
  m_pid_r.target = right_wheel_value;
  PID_controller(&m_pid_r);
  PID_controller(&m_pid_l);
  
  return true;
}
