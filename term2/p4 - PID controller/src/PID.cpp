#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double throttle, double speed_p, double speed_d, double k_angle) {
  this->Kp=Kp;
  this->Ki=Ki;
  this->Kd=Kd;
  this->max_throttle = throttle;
  this->speed_p=speed_p;
  this->speed_d=speed_d;
  this->K_angle=k_angle;

  p_error=0;
  d_error=0;
  i_error=0;
  steps = 0;
  average_cte = 0;
  total_cte =0;
  average_speed = 0;
  max_speed = 0;
  max_error = 0;
  speed_sum = 0;

}

void PID::UpdateError(double cte) {
   double previous_cte = p_error;
   p_error=cte;
   i_error+=cte;
   d_error= cte - previous_cte;
   std::cout << "p_error (cte): " << p_error << std::endl;
   std::cout << "d_error      : " << d_error << std::endl;

}

double PID::TotalError() {
  std::cout << "Kp * p_error: " << Kp * p_error << std::endl;
  std::cout << "Kd * d_error: " << Kd * d_error << std::endl;
  double total = Kp * p_error + Kd * d_error;
   //+ Ki * i_error
  return total;
}
double PID::SpeedError(double angle){
  //double speed = 0.7 + speed_p * p_error + speed_d * d_error ;
  double speed = max_throttle + speed_p * p_error + speed_d * d_error + fabs(angle) * K_angle;
  std::cout  << "speed_p * p_error " << speed_p * p_error << std::endl;
  std::cout << "speed_d * d_error " << speed_d * d_error << std::endl;
  std::cout << "speed correction by steer_value  " << fabs(angle) * K_angle << std::endl;
  return speed;
}
