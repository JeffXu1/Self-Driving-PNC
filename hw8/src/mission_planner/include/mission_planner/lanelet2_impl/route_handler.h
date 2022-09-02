#ifndef MISSION_PLANNER_LANELET_IMPL_ROUTE_HANDLER_H
#define MISSION_PLANNER_LANELET_IMPL_ROUTE_HANDLER_H

// Autoware
#include <lanelet2_extension/utility/query.h>
#include <mission_planner/lanelet2_impl/utility_functions.h>

// lnanelet
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>

#include <vector>

namespace mission_planner {
class RouteHandler {
 private:
  lanelet::LaneletMapConstPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;

  void setRouteLanelets(
      const lanelet::LaneletMapConstPtr& lanelet_map_ptr,
      const lanelet::routing::RoutingGraphPtr& routing_graph_ptr,
      const lanelet::ConstLanelets& path_lanelets);
  bool isBijectiveConnection(
      const lanelet::ConstLanelets& lanelet_section1,
      const lanelet::ConstLanelets& lanelet_section2) const;

 public:
  bool getPreviousLaneletWithinRoute(const lanelet::ConstLanelet& lanelet,
                                     lanelet::ConstLanelet* prev_lanelet) const;
  bool getNextLaneletWithinRoute(const lanelet::ConstLanelet& lanelet,
                                 lanelet::ConstLanelet* next_lanelet) const;
  RouteHandler(const lanelet::LaneletMapConstPtr& lanelet_map_ptr,
               const lanelet::routing::RoutingGraphPtr& routing_graph,
               const lanelet::ConstLanelets& path_lanelets);
  lanelet::ConstLanelets getRouteLanelets() const;

  lanelet::ConstLanelets getLaneletSequence(
      const lanelet::ConstLanelet& lanelet) const;
  lanelet::ConstLanelets getLaneletSequenceUpTo(
      const lanelet::ConstLanelet& lanelet) const;
  lanelet::ConstLanelets getLaneletSequenceAfter(
      const lanelet::ConstLanelet& lanelet) const;
  lanelet::ConstLanelets getPreviousLaneletSequence(
      const lanelet::ConstLanelets& lanelet_sequence) const;

  lanelet::ConstLanelets getLaneSequenceUpTo(
      const lanelet::ConstLanelet& lanelet) const;
  lanelet::ConstLanelets getLaneSequenceAfter(
      const lanelet::ConstLanelet& lanelet) const;
  lanelet::ConstLanelets getLaneSequence(
      const lanelet::ConstLanelet& lanelet) const;
  lanelet::ConstLanelets getNeighborsWithinRoute(
      const lanelet::ConstLanelet& lanelet) const;
  std::vector<lanelet::ConstLanelets> getLaneSection(
      const lanelet::ConstLanelet& lanelet) const;
  lanelet::ConstLanelets getNextLaneSequence(
      const lanelet::ConstLanelets& lane_sequence) const;
};
}  // namespace mission_planner
#endif  // MISSION_PLANNER_LANELET_IMPL_ROUTE_HANDLER_H
