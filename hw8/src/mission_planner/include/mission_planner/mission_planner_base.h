#ifndef MISSION_PLANNER_MISSION_PLANNER_BASE_H
#define MISSION_PLANNER_MISSION_PLANNER_BASE_H

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_planning_msgs/Route.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

// others
#include <string>
#include <vector>

namespace mission_planner {
class MissionPlanner {
 protected:
  MissionPlanner();

  geometry_msgs::PoseStamped goal_pose_;
  geometry_msgs::PoseStamped start_pose_;
  std::vector<geometry_msgs::PoseStamped> checkpoints_;

  std::string base_link_frame_;
  std::string map_frame_;

  ros::NodeHandle pnh_;

  ros::Publisher marker_publisher_;
  ros::Publisher global_path_publisher_;

  virtual bool isRoutingGraphReady() const = 0;
  virtual autoware_planning_msgs::Route planRoute(
      geometry_msgs::PoseArray& global_path) = 0;
  virtual void visualizeRoute(
      const autoware_planning_msgs::Route& route) const = 0;
  virtual void publishRoute(autoware_planning_msgs::Route& route,
                            geometry_msgs::PoseArray& global_path);
  virtual void visualizeGlobalPath(geometry_msgs::PoseArray& global_path) = 0;
  virtual void publishGlobalPath(geometry_msgs::PoseArray& global_path);

 private:
  ros::Publisher route_publisher_;
  ros::Subscriber goal_subscriber_;
  ros::Subscriber checkpoint_subscriber_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool getEgoVehiclePose(geometry_msgs::PoseStamped* ego_vehicle_pose);
  void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg_ptr);
  void checkpointCallback(
      const geometry_msgs::PoseStampedConstPtr& checkpoint_msg_ptr);
  bool transformPose(const geometry_msgs::PoseStamped& input_pose,
                     geometry_msgs::PoseStamped* output_pose,
                     const std::string target_frame);
};

}  // namespace mission_planner
#endif  // MISSION_PLANNER_MISSION_PLANNER_BASE_H
