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

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

private:
  ObstacleAvoidancePtr obstacle_avoidance_{nullptr};
  int64_t param_name_{123};
  std::array<geometry_msgs::msg::Pose, 10> poses_array;
  // std::array<float, 1080> distances_array; //1080 pktow +- 135 stopni
  geometry_msgs::msg::Pose car_pose;
  

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_pose_subscriber_;

};
}  // namespace obstacle_avoidance

#endif  // OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_NODE_HPP_
