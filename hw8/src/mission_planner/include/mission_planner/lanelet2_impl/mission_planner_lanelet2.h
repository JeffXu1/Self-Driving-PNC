#ifndef MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H
#define MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_lanelet2_msgs/MapBin.h>
#include <mission_planner/lanelet2_impl/route_handler.h>
#include <mission_planner/mission_planner_base.h>

// lanelet
#include <geometry_msgs/PoseArray.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <nav_msgs/Path.h>

// others
#include <string>

using RouteSections = std::vector<autoware_planning_msgs::RouteSection>;

namespace mission_planner {
class MissionPlannerLanelet2 : public MissionPlanner {
 public:
  MissionPlannerLanelet2();

 private:
  bool is_graph_ready_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  ros::Subscriber map_subscriber_;

  void mapCallback(const autoware_lanelet2_msgs::MapBin& msg);
  bool isGoalValid() const;

  // virtual functions
  bool isRoutingGraphReady() const;
  autoware_planning_msgs::Route planRoute(
      geometry_msgs::PoseArray& global_path);
  void visualizeRoute(const autoware_planning_msgs::Route& route) const;
  void visualizeGlobalPath(geometry_msgs::PoseArray& global_path);
  int clacCloestPathNodeID(
  const lanelet::ConstLineString2d& center_line,
  const lanelet::BasicPoint2d& point);

  // routing
  bool planPathBetweenCheckpoints(
      const geometry_msgs::PoseStamped& start_checkpoint,
      const geometry_msgs::PoseStamped& goal_checkpoint,
      lanelet::ConstLanelets* path_lanelets_ptr) const;
  lanelet::ConstLanelets getMainLanelets(
      const lanelet::ConstLanelets& path_lanelets,
      const RouteHandler& lanelet_sequence_finder);
  RouteSections createRouteSections(const lanelet::ConstLanelets& main_path,
                                    const RouteHandler& route_handler);
};
}  // namespace mission_planner

#endif  // MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H
