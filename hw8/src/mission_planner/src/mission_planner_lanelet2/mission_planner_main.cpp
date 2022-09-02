#include <mission_planner/lanelet2_impl/mission_planner_lanelet2.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_planner_node");

  mission_planner::MissionPlannerLanelet2 mission_planner;

  ros::spin();

  return 0;
}
