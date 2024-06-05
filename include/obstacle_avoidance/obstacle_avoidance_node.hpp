// Copyright 2024 MJ
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_NODE_HPP_
#define OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_NODE_HPP_

#define M_PI       3.14159265358979323846  /* pi */

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"



#include "obstacle_avoidance/obstacle_avoidance.hpp"

namespace obstacle_avoidance
{
using ObstacleAvoidancePtr = std::unique_ptr<obstacle_avoidance::ObstacleAvoidance>;

class OBSTACLE_AVOIDANCE_PUBLIC ObstacleAvoidanceNode : public rclcpp::Node
{
public:
  explicit ObstacleAvoidanceNode(const rclcpp::NodeOptions & options);
  void subscribeToLaserScan();
  void subscribeToTrajectory();
  void subscribeToCarPose();
  void on_timer();


  geometry_msgs::msg::Pose getPose(const autoware_auto_planning_msgs::msg::Trajectory & traj, const int idx);
  double calcDist2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);
  double normalizeEulerAngle(double euler);
  bool calcClosestIndex(const autoware_auto_planning_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & pose,
  size_t & output_closest_idx, const double dist_thr = 10.0, const double angle_thr = M_PI_4);

private:
  ObstacleAvoidancePtr obstacle_avoidance_{nullptr};
  int64_t param_name_{123};
  autoware_auto_planning_msgs::msg::Trajectory trajectory_;
  std::array<geometry_msgs::msg::Pose, 270> poses_array;
  // std::array<float, 1080> distances_array; //1080 pktow +- 135 stopni
  geometry_msgs::msg::Pose car_pose;
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_publisher_;

  // tf2 listener components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic tf2 updates
};
}  // namespace obstacle_avoidance

#endif  // OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_NODE_HPP_
