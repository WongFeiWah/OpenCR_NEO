#ifndef SERIAL_CMD_H_
#define SERIAL_CMD_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include "turtlebot3_motor_driver.h"

extern unsigned char DataScope_OutPut_Buffer[42];     //待发送帧数据缓存区


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 
 

void test_Serial_Control();
void SendVal(char mode, float val);

#endif
