#include "pid_controller.h"
#include <iostream>

namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki,
                             const double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) {
  double current_output_=0.0;
  double derror_=0;
  //std::cout<<"error value is "<<error<<std::endl;
  integral_+=error*dt;
  if(integral_>5)
  {
    integral_=5;
  } 
  if(integral_<(-5))
  {
    integral_=(-5);
  }
  // std::cout<<"intergral value is "<<integral_<<std::endl;
  derror_ = (error-previous_error_)/dt;
  previous_error_=error;
  //std::cout<<"derror value is "<<derror_<<std::endl;
  current_output_ = kp_*error+ki_*integral_+kd_*derror_;
  // std::cout<<"current_output value is "<<current_output_<<std::endl;
  return current_output_;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {
 kp_=0;
 ki_=0;
 kd_=0; 
}

}  // namespace control
}  // namespace shenlan