#pragma once
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <dummy_perception_publisher/Object.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <random>
#include <tuple>

class DummyPerceptionPublisherNode {
 private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher dynamic_object_pub_;
  ros::Subscriber object_sub_;
  ros::Timer timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<dummy_perception_publisher::Object> objects_;
  double visible_range_;
  double detection_successful_rate_;
  bool enable_ray_tracing_;
  std::mt19937 random_generator_;
  void timerCallback(const ros::TimerEvent &);
  void createObjectPointcloud(
      const double length, const double width, const double height,
      const double std_dev_x, const double std_dev_y, const double std_dev_z,
      const tf2::Transform &tf_base_link2moved_object,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud_ptr);
  void objectCallback(const dummy_perception_publisher::Object::ConstPtr &msg);

 public:
  DummyPerceptionPublisherNode();
  ~DummyPerceptionPublisherNode(){};
};