#include <ros/ros.h>
#include "dummy_perception_publisher/node.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "dummy_perception_publisher");
  DummyPerceptionPublisherNode node;

  ros::spin();

  return 0;
};