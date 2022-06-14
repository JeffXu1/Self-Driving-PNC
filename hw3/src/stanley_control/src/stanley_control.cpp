#include "stanley_control.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include <math.h>

using namespace std;

namespace shenlan {
namespace control {

double atan2_to_PI(const double atan2) {
  return atan2 * M_PI / 180;
}

double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.x - x;
  const double dy = point.y - y;
  return dx * dx + dy * dy;
}

void StanleyController::LoadControlConf() {
  k_y_ = 0.5;
}

// /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
// 提示，在该函数中你需要调用计算误差
void StanleyController::ComputeControlCmd(
    const VehicleState &vehicle_state,
    const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) {
    double e_theta_;
    double e_y_;
    double y_theta;
    trajectory_points_=planning_published_trajectory.trajectory_points;
    ComputeLateralErrors(vehicle_state.x,vehicle_state.y,vehicle_state.heading,e_y_,e_theta_);
    y_theta = atan(k_y_*e_y_/(vehicle_state.velocity));
    cout<<"y_theta value "<<y_theta<<endl;
    cmd.steer_target=e_theta_+y_theta;
    if(cmd.steer_target>atan2_to_PI(u_max_))
      cmd.steer_target=atan2_to_PI(u_max_);
    if(cmd.steer_target<atan2_to_PI(u_min_))
      cmd.steer_target=atan2_to_PI(u_min_);
    cout<<"cmd.steer_target value "<<cmd.steer_target<<endl;
}

// /** to-do **/ 计算需要的误差，包括横向误差，纵向误差

void StanleyController::ComputeLateralErrors(const double x, const double y,
                                             const double theta, double &e_y,
                                             double &e_theta) {
    double theta_ref;
    double flag_etheta;
    
    TrajectoryPoint NearestPoint;
    NearestPoint=QueryNearestPointByPosition(x,y);
    theta_ref = NearestPoint.heading;
    
    cout<<"vehicle_state heading"<<theta<<endl; 
    cout<<"NearestPoint heading"<<theta_ref<<endl;
    if(abs(theta_ref-theta)>=M_PI){
      if((theta_ref<0)&&(theta>0)){
        e_theta=-((M_PI-theta)+(M_PI+theta_ref));
        flag_etheta=1;
      }
      else{
        e_theta=(M_PI+theta+M_PI-theta_ref);
        flag_etheta=2;
      }
      
    }
    else{
      e_theta = -(theta_ref-theta);
      flag_etheta=3;
    }
    
    double point_y_in_vehicle = -(NearestPoint.x-x)*sin(theta)+(NearestPoint.y-y)*cos(theta);
    if(point_y_in_vehicle>=0){
      e_y = -(sqrt(pow(x-NearestPoint.x,2)+pow(y-NearestPoint.y,2)));
    }
         
    else{
      e_y = (sqrt(pow(x-NearestPoint.x,2)+pow(y-NearestPoint.y,2)));
    }
    cout<<"e_theta value"<<e_theta<<endl;
    cout<<"flag_etheta value "<<flag_etheta<<endl; 
    cout<<"e_y value "<<e_y<<endl;                                  
  
}

TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x,
                                                               const double y) {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 1; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  // cout << " index_min: " << index_min << endl;
  //cout << "tarjectory.heading: " << trajectory_points_[index_min].heading << endl;
  theta_ref_ = trajectory_points_[index_min].heading;

  return trajectory_points_[index_min];
}

}  // namespace control
}  // namespace shenlan
