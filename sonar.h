#ifndef SONAR_H_
#define SONAR_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

void Sonar_Init();
void Sonar_Update(std_msgs::Float64MultiArray &sonar_msg);

#endif
