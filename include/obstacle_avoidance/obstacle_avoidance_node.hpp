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

#define M_PI 3.14159265358979323846 /* pi */
#define THRESHOLD 50

#include "obstacle_avoidance/obstacle_avoidance.hpp"
#include "occupancy_grid.hpp"
#include "random"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualizer.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <memory>

namespace obstacle_avoidance
{
using ObstacleAvoidancePtr = std::unique_ptr<obstacle_avoidance::ObstacleAvoidance>;

struct Node_struct
{
  double x, y;
  double cost;  // only used for RRT*
  int parent;   // index of parent node in the tree vector
  std::vector<int> children;
  bool is_root = false;
};

class OBSTACLE_AVOIDANCE_PUBLIC ObstacleAvoidanceNode : public rclcpp::Node
{
public:
  explicit ObstacleAvoidanceNode(const rclcpp::NodeOptions & options);
  void subscribeToLaserScan();
  void subscribeToTrajectory();
  void subscribeToCarPose();
  double calcDist2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);
  double calculate_dist2(double x1, double x2, double y1, double y2);
  bool calcClosestIndex(
    const std::vector<geometry_msgs::msg::Pose> & waypoints, const geometry_msgs::msg::Pose & pose,
    size_t & output_closest_idx, const double dist_thr = 10.0, const double angle_thr = M_PI_4);
  std::vector<double> sample_point();
  int nearest(std::vector<Node_struct> & tree, std::vector<double> & sampled_point);
  Node_struct steer(Node_struct & nearest_node, std::vector<double> & sampled_point);
  bool check_collision(Node_struct & nearest_node, Node_struct & new_node);
  bool check_collision_2(
    geometry_msgs::msg::Point & nearest_node, geometry_msgs::msg::Point & new_node);
  std::vector<int> near(std::vector<Node_struct> & tree, Node_struct & node);
  double line_cost(Node_struct & n1, Node_struct & n2);
  bool is_goal(Node_struct & latest_added_node, double goal_x, double goal_y);
  std::vector<Node_struct> find_path(std::vector<Node_struct> & tree, Node_struct & node);
  void visualize_tree(vector<Node_struct> & tree);
  void initialize_marker(
    visualization_msgs::msg::Marker & marker, onst std::string & frame_id, const std::string & ns,
    int32_t id, int32_t type, const std_msgs::msg::ColorRGBA & color, double scale);
  void load_waypoints(const std::string & filename);
  void visualize_map();
  void get_current_goal();
  void reset_goal();
  void advance_goal();
  int find_closest_waypoint(
    const vector<geometry_msgs::msg::Pose> & waypoints, const geometry_msgs::msg::Pose & pose);
  autoware_auto_planning_msgs::msg::Trajectory createTrajectory(
    const std::vector<Node_struct> & points);
  double calculateYaw(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

private:
  ObstacleAvoidancePtr obstacle_avoidance_{nullptr};
  autoware_auto_planning_msgs::msg::Trajectory trajectory_;
  autoware_auto_planning_msgs::msg::Trajectory avoidance_trajectory_;
  std::vector<geometry_msgs::msg::Pose> waypoints_;
  geometry_msgs::msg::Pose car_pose;
  size_t self_idx;
  size_t target_idx;
  size_t curr_goal_ind_;
  double last_time_;
  Node_struct start_node;
  std::mt19937 gen_;
  bool path_found_flag = false;
  bool obstacle_detected = false;

  visualization_msgs::msg::Marker goal_marker;
  visualization_msgs::msg::Marker tree_nodes;
  visualization_msgs::msg::Marker tree_branch;
  visualization_msgs::msg::Marker path_dots;

  std_msgs::msg::ColorRGBA red;
  std_msgs::msg::ColorRGBA green;
  std_msgs::msg::ColorRGBA blue;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    trajectory_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_pose_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_update_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_branches_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_viz_pub_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    obstacle_avoidance_trajectory_pub_;

  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::OccupancyGrid map_updated_;
  nav_msgs::msg::OccupancyGrid map_updated_2_;
};
}  // namespace obstacle_avoidance

#endif  // OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_NODE_HPP_
