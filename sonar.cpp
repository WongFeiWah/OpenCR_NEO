#include "sonar.h"

unsigned long Time_Echo_us = 0;

unsigned long Len_mm_X100  = 0;

unsigned long Len_Integer = 0; //

unsigned int Len_Fraction = 0;

unsigned int EchoPin[5] = {51,53,55,57,59};

unsigned int TrigPin[5] = {50,52,54,56,58};

double range[5] = {0};

double* GetRang()
{
  static double _len = 0.0;
  int i = 2;
  for(i = 2; i<5; i++)
  {
    digitalWrite(TrigPin[i], HIGH);
    delayMicroseconds(50);
    digitalWrite(TrigPin[i], LOW);
    Time_Echo_us = pulseIn(EchoPin[i], HIGH, 10000);//60ms
  
    if((Time_Echo_us < 10000) && (Time_Echo_us > 1))
    {
  
      Len_mm_X100 = (Time_Echo_us*34)/2;
      range[i] = Len_mm_X100/100000.0;
  
    }
    else
    {
      range[i] = 100.0;
    }    
  }

  
  return range;
}


void Sonar_Init() {
  // put your setup code here, to run once:
  for(int i = 0;i < 5; i++)
  {
    pinMode(EchoPin[i], INPUT);
    pinMode(TrigPin[i], OUTPUT);    
  }

}

void Sonar_Update(std_msgs::Float64MultiArray &sonar_msg) {

  GetRang();
  sonar_msg.data_length = 5;
  sonar_msg.data[0] = range[0];
  sonar_msg.data[1] = range[1];
  sonar_msg.data[2] = range[2];
  sonar_msg.data[3] = range[3];
  sonar_msg.data[4] = range[4];
}
