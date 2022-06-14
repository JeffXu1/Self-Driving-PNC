#include "stanley_control.h"
#include "pid_controller.h"

// Input
VehicleState vehicle_state_;

using namespace std;

bool first_record_ = false;
double V_set_ = 5.0;

double wheelbase_ = 2.852;
double car_length_ = 2.852;

shenlan::control::PIDController speed_pid_controller(0.5, 0.0, 0.0);

double PointDistance(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.x - x;
  const double dy = point.y - y;
  return  sqrt(dx * dx + dy * dy);
}

double pid_control() {
  double ego_speed = std::sqrt(vehicle_state_.vx * vehicle_state_.vx +  // 本车速度
                                vehicle_state_.vy * vehicle_state_.vy);

  // 位置误差
  double v_err = V_set_ - ego_speed;                // 速度误差
  
  cout << "v_err: " << v_err << endl;
  
  double acceleration_cmd = speed_pid_controller.Control(v_err, 0.01);
  return acceleration_cmd;
};

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if(!first_record_) first_record_ = true; 
  // ROS_ERROR("I heard: [%f]", msg->pose.pose.position.x);

  vehicle_state_.vx = msg->twist.twist.linear.x;
  vehicle_state_.vy = msg->twist.twist.linear.y;

  // 将orientation(四元数)转换为欧拉角(roll, pitch, yaw)
  tf::Quaternion q; 
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(vehicle_state_.roll, vehicle_state_.pitch, vehicle_state_.yaw);
  
  vehicle_state_.heading = vehicle_state_.yaw;    // pose.orientation是四元数
  // cout << "vehicle_state_.heading: " << vehicle_state_.heading << endl;

  // 将位置转移到前车轮的中心点
  vehicle_state_.x = msg->pose.pose.position.x + std::cos(vehicle_state_.heading) * 0.5 * car_length_; 
  vehicle_state_.y = msg->pose.pose.position.y + std::sin(vehicle_state_.heading) * 0.5 * wheelbase_;

  // cout << "vehicle_state_.heading: " << vehicle_state_.heading << endl;
  vehicle_state_.velocity =
      std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                msg->twist.twist.linear.y * msg->twist.twist.linear.y);
  vehicle_state_.angular_velocity =
      std::sqrt(msg->twist.twist.angular.x * msg->twist.twist.angular.x +
                msg->twist.twist.angular.y * msg->twist.twist.angular.y);
  vehicle_state_.acceleration = 0.0;
}

int main(int argc, char** argv) {
  // // Construct the reference line
  // std::shared_ptr<zjlmap::Map> map_data_ = std::make_shared<zjlmap::Map>();
  // int handle = 0;
  // zjlmap::ErrorCode ec =
  //   map_data_->load("src/maps/TestMap.xodr", handle);
  // ROS_ERROR("I heard: [%d]", ec);
  // zjlmap::LaneId lane_id = {3, 0, 1};
  // zjlmap::LaneInfo lane_info = map_data_->query_lane_info(lane_id);
  // std::vector<zjlmap::TracePoint> reference_line;
  // map_data_->calc_lane_center_line_curv(lane_id, lane_info.begin,
  // lane_info.end, 0.25, reference_line);

  // lane_id = {7, 0, -1};
  // lane_info = map_data_->query_lane_info(lane_id);
  // std::vector<zjlmap::TracePoint> reference_line2;
  // map_data_->calc_lane_center_line_curv(lane_id, lane_info.begin,
  // lane_info.end, 0.25, reference_line2);

  // reference_line.insert(reference_line.end(), reference_line2.begin(),
  // reference_line2.end());

  // for (size_t i=0; i<reference_line.size(); i++)
  // {
  //     ROS_ERROR("reference pt %d,  x: %f, y: %f", i, reference_line[i].x,
  //     reference_line[i].y);
  // }

  // std::ofstream target_line_cout(
  //     "src/stanley_control/data/reference_line.txt");
  // target_line_cout.setf(std::ios::fixed, std::ios::floatfield);
  // target_line_cout.precision(2);
  // for (size_t i = 0; i < reference_line.size(); i++) {
  //     target_line_cout << reference_line[i].x << "  "
  //                     << reference_line[i].y << std::endl;
  // }
  // target_line_cout.close();

  // Read the reference_line txt
  std::ifstream infile;
  infile.open(
      "src/stanley_control/data/cube_town_reference_line.txt");  //将文件流对象与文件连接起来
  assert(infile.is_open());  //若失败,则输出错误消息,并终止程序运行

  std::vector<std::pair<double, double>> xy_points;
  std::string s;
  std::string x;
  std::string y;
  while (getline(infile, s)) {
    std::stringstream word(s);
    word >> x;
    word >> y;
    double pt_x = std::atof(x.c_str());
    double pt_y = std::atof(y.c_str());
    xy_points.push_back(std::make_pair(pt_x, pt_y));
  }
  infile.close();

  // Construct the reference_line path profile
  std::vector<double> headings;
  std::vector<double> accumulated_s;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::unique_ptr<shenlan::control::ReferenceLine> reference_line =
      std::make_unique<shenlan::control::ReferenceLine>(xy_points);
  reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas,
                                     &dkappas);

  for (size_t i = 0; i < headings.size(); i++) {
    std::cout << "pt " << i << " heading: " << headings[i]
              << " acc_s: " << accumulated_s[i] << " kappa: " << kappas[i]
              << " dkappas: " << dkappas[i] << std::endl;
  }

  // Construct the planning trajectory
  TrajectoryData planning_published_trajectory;
  for (size_t i = 0; i < headings.size(); i++) {
    TrajectoryPoint trajectory_pt;
    trajectory_pt.x = xy_points[i].first;
    trajectory_pt.y = xy_points[i].second;
    trajectory_pt.v = 2.0;
    trajectory_pt.a = 0.0;
    trajectory_pt.heading = headings[i];
    trajectory_pt.kappa = kappas[i];

    planning_published_trajectory.trajectory_points.push_back(trajectory_pt);
  }

  TrajectoryPoint goal_point = planning_published_trajectory.trajectory_points.back();

  ros::init(argc, argv, "control_pub");
  ros::NodeHandle nh;
  ROS_ERROR("init !");
  ros::Subscriber sub = nh.subscribe("/odom", 10, odomCallback);
  ros::Publisher control_pub =
      nh.advertise<lgsvl_msgs::VehicleControlData>("/vehicle_cmd", 1000);

  // Lqr control part
  ControlCmd cmd;
  std::unique_ptr<shenlan::control::StanleyController> stanley_controller =
      std::make_unique<shenlan::control::StanleyController>();
  stanley_controller->LoadControlConf();

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if(first_record_) { 
      //距离终点0.5m停止
      double LeftDistance;
      if(PointDistance(goal_point, vehicle_state_.x, vehicle_state_.y) < 0.5){
          V_set_ = 0;
          cout << "get goal point " << endl;
      }
      LeftDistance=PointDistance(goal_point, vehicle_state_.x, vehicle_state_.y);
      cout << "LeftDistance value " <<LeftDistance<<endl;
      stanley_controller->ComputeControlCmd(vehicle_state_,
                                          planning_published_trajectory, cmd);
      

      lgsvl_msgs::VehicleControlData control_cmd;
      control_cmd.header.stamp = ros::Time::now();
      
      double acc_cmd = pid_control();

      control_cmd.acceleration_pct = acc_cmd;
      control_cmd.target_gear = lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
      control_cmd.target_wheel_angle = cmd.steer_target;

      control_pub.publish(control_cmd);
    }
    //cout << "control_cmd.target_wheel_angle: " << control_cmd.target_wheel_angle << endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
