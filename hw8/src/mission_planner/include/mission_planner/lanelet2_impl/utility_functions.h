#ifndef MISSION_PLANNER_LANELET2_IMPL_UTILITY_FUNCTIONS_H
#define MISSION_PLANNER_LANELET2_IMPL_UTILITY_FUNCTIONS_H
#include <string>

#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <unordered_set>
#include <vector>
bool exists(const std::unordered_set<lanelet::Id>& set, const lanelet::Id& id);

template <typename T>
bool exists(const std::vector<T>& vectors, const T& item) {
  for (const auto& i : vectors) {
    if (i == item) {
      return true;
    }
  }
  return false;
}

void setColor(std_msgs::ColorRGBA* cl, double r, double g, double b, double a);
void insertMarkerArray(visualization_msgs::MarkerArray* a1,
                       const visualization_msgs::MarkerArray& a2);
std::string toString(const geometry_msgs::Pose& pose);
bool getClosestLanelet(const geometry_msgs::Pose& search_pose,
                       const lanelet::LaneletMapPtr& lanelet_map,
                       lanelet::Lanelet* closest_lanelet,
                       double distance_thresh = 10.0);
#endif  // MISSION_PLANNER_LANELET2_IMPL_UTILITY_FUNCTIONS_H
