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

#include "obstacle_avoidance/obstacle_avoidance_node.hpp"

namespace obstacle_avoidance
{

const int MAX_ITER = 350;
const int MIN_ITER = 300;
const int COLLISION_HORIZON = 30;
const double STD = 1.5;
const double STEER_RANGE = 0.3;
const float NEAR_RANGE = 1.0;
const double GOAL_THRESHOLD = 0.15;
const float GOAL_AHEAD_DIST = 4.5;
const double X_SAMPLE_RANGE = 3;
const double Y_SAMPLE_RANGE = 3;
const float MARGIN = 0.01;
const float MARGIN_2 = 0.18;
const float DETECTED_OBS_MARGIN = 0.01;
const float DETECTED_OBS_MARGIN_2 = 0.3;
const float VELOCITY = 2;

ObstacleAvoidanceNode::ObstacleAvoidanceNode(const rclcpp::NodeOptions & options)
: Node("obstacle_avoidance", options)
{
  obstacle_avoidance_ = std::make_unique<obstacle_avoidance::ObstacleAvoidance>();
  const std::string csv_path = this->declare_parameter("csv_path", "");

  map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      map_ = *msg;
      if (map_updated_.data.empty()) {
        map_updated_ = map_;
        map_updated_2_ = map_;
        occupancy_grid::inflate_map(map_updated_, MARGIN);
        occupancy_grid::inflate_map(map_updated_2_, MARGIN_2);
      }
    });

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_found", 10);
  goal_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal", 10);
  tree_nodes_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tree_nodes", 10);
  tree_branches_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("/tree_branches", 10);
  map_update_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_updated", 10);
  obstacle_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle_viz", 10);
  obstacle_avoidance_trajectory_pub_ =
    this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
      "/planning/racing_planner/avoidance/trajectory", 10);

  // Initialize colors
  red.r = 1.0;
  red.a = 1.0;
  green.g = 1.0;
  green.a = 1.0;
  blue.b = 1.0;
  blue.a = 1.0;

  // Set up markers
  initialize_marker(
    goal_marker, "map", "goal", 0, visualization_msgs::msg::Marker::SPHERE, green, 0.3);
  initialize_marker(
    tree_nodes, "map", "nodes", 5, visualization_msgs::msg::Marker::POINTS, red, 0.05);
  initialize_marker(
    tree_branch, "map", "branch", 6, visualization_msgs::msg::Marker::LINE_LIST, blue, 0.01);

  subscribeToCarPose();
  subscribeToLaserScan();
  subscribeToTrajectory();

  visualize_map();
  if (waypoints_.empty()) {
    load_waypoints(csv_path);
  }
  reset_goal();
  last_time_ = this->now().seconds();
}

void ObstacleAvoidanceNode::load_waypoints(const std::string & filename)
{
  std::ifstream file(filename);
  std::string line;

  // Skip the first line
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    // Skip the first two elements
    std::getline(ss, token, ';');
    // Extract x and y
    std::getline(ss, token, ';');
    double x = std::stod(token);
    std::getline(ss, token, ';');
    double y = std::stod(token);
    std::getline(ss, token, ';');
    double yaw = std::stod(token);

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.z = yaw;
    pose.orientation.w = 1.0;
    waypoints_.push_back(pose);
  }
}

void ObstacleAvoidanceNode::visualize_map()
{
  visualization_msgs::msg::Marker dots;
  std_msgs::msg::ColorRGBA color;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 1.0;
  this->initialize_marker(
    dots, "map", "obstacle", 0, visualization_msgs::msg::Marker::POINTS, color, 0.04);

  for (size_t i = 0; i < map_.data.size(); i++) {
    if (map_.data[i] == 100) {
      geometry_msgs::msg::Point p;
      p.x = occupancy_grid::ind2x(map_, i);
      p.y = occupancy_grid::ind2y(map_, i);
      p.z = 0.0;
      dots.points.push_back(p);
    }
  }
  obstacle_viz_pub_->publish(dots);
}

bool comp_cost(Node_struct & n1, Node_struct & n2)
{
  return n1.cost < n2.cost;
}

inline double ObstacleAvoidanceNode::calculate_dist2(double x1, double x2, double y1, double y2)
{
  // considering that doing sqrt is expensive
  return pow(x1 - x2, 2) + pow(y1 - y2, 2);
}

void ObstacleAvoidanceNode::get_current_goal()
{
  float dist_to_goal2 = calculate_dist2(
    waypoints_.at(curr_goal_ind_).position.x, car_pose.position.x,
    waypoints_.at(curr_goal_ind_).position.y, car_pose.position.y);
  // goal out of range, reset goal
  if (dist_to_goal2 > pow(GOAL_AHEAD_DIST, 2)) {
    reset_goal();
  }
  if (occupancy_grid::is_xy_occupied(
        map_updated_, waypoints_.at(curr_goal_ind_).position.x,
        waypoints_.at(curr_goal_ind_).position.y)) {
    advance_goal();
  }
  // enough progress made, advance goal
  else if (dist_to_goal2 < pow(GOAL_AHEAD_DIST * 0.75, 2)) {
    advance_goal();
  }
}

void ObstacleAvoidanceNode::reset_goal()
{
  if (
    (car_pose.position.x == -0.0097019 && car_pose.position.y == -0.580195) ||
    (car_pose.position.x == 0 && car_pose.position.y == 0)) {
    RCLCPP_INFO(this->get_logger(), "Failed to receive car's pose");
  } else {
    int closest_ind = find_closest_waypoint(waypoints_, car_pose);
    float closest_dist2 = calculate_dist2(
      waypoints_.at(closest_ind).position.x, car_pose.position.x,
      waypoints_.at(closest_ind).position.y, car_pose.position.y);
    if (closest_dist2 > pow(GOAL_AHEAD_DIST, 2)) {
      throw std::runtime_error(
        "Couldn't find a goal in range. Reposition the car somewhere near the waypoints");
    }
    // advance from closest waypoint until one that is around 0.9 GOAL_AHEAD_DIST away
    curr_goal_ind_ = closest_ind;
    advance_goal();
  }
}

void ObstacleAvoidanceNode::advance_goal()
{
  // advance in waypoints from current goal to a point that is around 0.9*MAX_GOAL_AHEAD distance ahead
  using namespace occupancy_grid;
  int curr_ind = curr_goal_ind_;
  if (curr_ind >= waypoints_.size()) {
    curr_ind = 0;
  }
  float pose_x = car_pose.position.x;
  float pose_y = car_pose.position.y;
  float curr_dist2 = calculate_dist2(
    waypoints_.at(curr_ind).position.x, pose_x, waypoints_.at(curr_ind).position.y, pose_y);

  while (curr_dist2 < pow(GOAL_AHEAD_DIST * 0.9, 2) ||
         is_xy_occupied(
           map_updated_, waypoints_.at(curr_ind).position.x, waypoints_.at(curr_ind).position.y)) {
    curr_dist2 = calculate_dist2(
      waypoints_.at(curr_ind).position.x, pose_x, waypoints_.at(curr_ind).position.y, pose_y);
    curr_ind++;
    if (curr_ind >= waypoints_.size()) {
      curr_ind = 0;
    }
    if (
      curr_dist2 > pow(GOAL_AHEAD_DIST, 2) &&
      is_xy_occupied(
        map_updated_, waypoints_.at(curr_ind).position.x, waypoints_.at(curr_ind).position.y)) {
      break;
    }
  }
  curr_goal_ind_ = max(0, curr_ind - 1);
}

int ObstacleAvoidanceNode::find_closest_waypoint(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, const geometry_msgs::msg::Pose & pose)
{
  float min_dist2 = 100000.0;
  int min_ind = -1;
  for (int i = 0; i < waypoints.size(); i++) {
    float dist2 = calculate_dist2(
      waypoints.at(i).position.x, pose.position.x, waypoints.at(i).position.y, pose.position.y);
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      min_ind = i;
    }
  }
  return min_ind;
}

std::vector<double> ObstacleAvoidanceNode::sample_point()
{
  // This method returns a sampled point from the free space
  std::vector<double> sampled_point;

  std::normal_distribution<double> norm_dist_x(
    0.6 * waypoints_.at(curr_goal_ind_).position.x + 0.4 * car_pose.position.x, STD);
  std::normal_distribution<double> norm_dist_y(
    0.6 * waypoints_.at(curr_goal_ind_).position.y + 0.4 * car_pose.position.y, STD);
  double x = norm_dist_x(gen_);
  double y = norm_dist_y(gen_);

  // sample recursively until one in the free space gets returned
  if (!occupancy_grid::is_xy_occupied(map_updated_, x, y)) {
    sampled_point.push_back(x);
    sampled_point.push_back(y);
    return sampled_point;
  } else {
    return sample_point();
  }
}

int ObstacleAvoidanceNode::nearest(
  std::vector<Node_struct> & tree, std::vector<double> & sampled_point)
{
  // This method returns the nearest node on the tree to the sampled point
  // Args:
  //     tree (std::vector<Node>): the current RRT tree
  //     sampled_point (std::vector<double>): the sampled point in free space
  // Returns:
  //     nearest_node (int): index of nearest node on the tree
  int nearest_node = 0;
  double min_dist = 100000.0;
  for (int i = 0; i < int(tree.size()); i++) {
    double dist = calculate_dist2(tree.at(i).x, sampled_point[0], tree.at(i).y, sampled_point[1]);
    if (dist < min_dist) {
      nearest_node = i;
      min_dist = dist;
    }
  }
  return nearest_node;
}

Node_struct ObstacleAvoidanceNode::steer(
  Node_struct & nearest_node, std::vector<double> & sampled_point)
{
  // The function steer:(x,y)->z returns a point such that z is “closer”
  // to y than x is. The point z returned by the function steer will be
  // such that z minimizes ||z−y|| while at the same time maintaining
  //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

  // basically, expand the tree towards the sample point (within a max dist)

  // Args:
  //    nearest_node (Node): nearest node on the tree to the sampled point
  //    sampled_point (std::vector<double>): the sampled point in free space
  // Returns:
  //    new_node (Node): new node created from steering
  Node_struct new_node;
  double dist =
    sqrt(calculate_dist2(nearest_node.x, sampled_point[0], nearest_node.y, sampled_point[1]));

  new_node.x = nearest_node.x + min(STEER_RANGE, dist) * (sampled_point[0] - nearest_node.x) / dist;
  new_node.y = nearest_node.y + min(STEER_RANGE, dist) * (sampled_point[1] - nearest_node.y) / dist;

  return new_node;
}

bool ObstacleAvoidanceNode::check_collision(Node_struct & nearest_node, Node_struct & new_node)
{
  // This method returns a boolean indicating if the path between the
  // nearest node and the new node created from steering is collision free

  bool collision = false;

  // Calculate differences
  double x_diff = new_node.x - nearest_node.x;
  double y_diff = new_node.y - nearest_node.y;

  // Calculate the number of steps needed based on the maximum difference
  double resolution = map_updated_.info.resolution;
  int x_steps = std::ceil(std::abs(x_diff) / resolution);
  int y_steps = std::ceil(std::abs(y_diff) / resolution);
  int num_steps = std::max(x_steps, y_steps);

  // Calculate step increments
  double x_increment = x_diff / num_steps;
  double y_increment = y_diff / num_steps;

  // Iterate over the line and check for collisions
  for (int i = 0; i <= num_steps; ++i) {
    double x = nearest_node.x + i * x_increment;
    double y = nearest_node.y + i * y_increment;

    if (occupancy_grid::is_xy_occupied(map_updated_2_, x, y)) {
      collision = true;
      break;
    }
  }
  return collision;
}

bool ObstacleAvoidanceNode::check_collision_2(
  geometry_msgs::msg::Point & nearest_node, geometry_msgs::msg::Point & new_node)
{
  // This method returns a boolean indicating if the path between the
  // nearest node and the new node created from steering is collision free

  bool collision = false;

  // Calculate differences
  double x_diff = new_node.x - nearest_node.x;
  double y_diff = new_node.y - nearest_node.y;

  // Calculate the number of steps needed based on the maximum difference
  double resolution = map_updated_.info.resolution;
  int x_steps = std::ceil(std::abs(x_diff) / resolution);
  int y_steps = std::ceil(std::abs(y_diff) / resolution);
  int num_steps = std::max(x_steps, y_steps);

  // Calculate step increments
  double x_increment = x_diff / num_steps;
  double y_increment = y_diff / num_steps;

  // Iterate over the line and check for collisions
  for (int i = 0; i <= num_steps; ++i) {
    double x = nearest_node.x + i * x_increment;
    double y = nearest_node.y + i * y_increment;

    if (occupancy_grid::is_xy_occupied(map_updated_, x, y)) {
      collision = true;
      break;
    }
  }
  return collision;
}

bool ObstacleAvoidanceNode::is_goal(Node_struct & latest_added_node, double goal_x, double goal_y)
{
  // This method checks if the latest node added to the tree is close enough
  return calculate_dist2(goal_x, latest_added_node.x, goal_y, latest_added_node.y) <
         pow(GOAL_THRESHOLD, 2);
}

std::vector<Node_struct> ObstacleAvoidanceNode::find_path(
  std::vector<Node_struct> & tree, Node_struct & node)
{
  // This method traverses the tree from the node that has been determined as goal
  std::vector<Node_struct> found_path;
  Node_struct current = node;
  while (!current.is_root) {
    found_path.push_back(current);
    current = tree.at(current.parent);
  }
  found_path.push_back(current);  // add start node
  reverse(found_path.begin(), found_path.end());
  return found_path;
}

// RRT* methods
double ObstacleAvoidanceNode::line_cost(Node_struct & n1, Node_struct & n2)
{
  // This method returns the cost of the straight line path between two nodes
  return sqrt(calculate_dist2(n1.x, n2.x, n1.y, n2.y));
}

std::vector<int> ObstacleAvoidanceNode::near(std::vector<Node_struct> & tree, Node_struct & node)
{
  // This method returns the set of Nodes in the neighborhood of a node.
  // Returns:
  //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood
  std::vector<int> neighborhood;
  neighborhood.clear();
  for (int i = 0; i < tree.size(); i++) {
    if (line_cost(tree.at(i), node) < NEAR_RANGE) {
      neighborhood.push_back(i);
    }
  }
  return neighborhood;
}

void ObstacleAvoidanceNode::visualize_tree(std::vector<Node_struct> & tree)
{
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.pose.position.x = waypoints_.at(curr_goal_ind_).position.x;
  goal_marker.pose.position.y = waypoints_.at(curr_goal_ind_).position.y;
  goal_marker.pose.position.z = 0.0;
  goal_viz_pub_->publish(goal_marker);

  // Plot tree
  tree_nodes.points.clear();
  tree_branch.points.clear();

  for (size_t i = 0; i < tree.size(); i++) {
    geometry_msgs::msg::Point p;
    p.x = tree.at(i).x;
    p.y = tree.at(i).y;
    p.z = 0.0;
    tree_nodes.points.push_back(p);
    for (size_t j = 0; j < tree.at(i).children.size(); j++) {
      tree_branch.points.push_back(p);
      geometry_msgs::msg::Point p_child;
      p_child.x = tree.at(tree.at(i).children.at(j)).x;
      p_child.y = tree.at(tree.at(i).children.at(j)).y;
      p_child.z = 0.0;
      tree_branch.points.push_back(p_child);
    }
  }
  tree_branches_pub_->publish(tree_branch);
  tree_nodes_pub_->publish(tree_nodes);
}

void ObstacleAvoidanceNode::initialize_marker(
  visualization_msgs::msg::Marker & marker, const std::string & frame_id, const std::string & ns,
  int32_t id, int32_t type, const std_msgs::msg::ColorRGBA & color, double scale)
{
  marker.header.frame_id = frame_id;
  marker.header.stamp = this->now();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = scale;
  marker.color = color;
}

inline double ObstacleAvoidanceNode::calcDist2d(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot((a.x - b.x), (a.y - b.y));
}

bool ObstacleAvoidanceNode::calcClosestIndex(
  const std::vector<geometry_msgs::msg::Pose> & waypoints, const geometry_msgs::msg::Pose & pose,
  size_t & output_closest_idx, const double dist_thr, const double angle_thr)
{
  double dist_min = std::numeric_limits<double>::max();
  int closest_idx = -1;
  for (int i = 0; i < waypoints.size(); i++) {
    const double dist = calcDist2d(waypoints[i].position, pose.position);

    if (dist < dist_min) {
      dist_min = dist;
      closest_idx = static_cast<int>(i);
    }
  }
  output_closest_idx = static_cast<size_t>(closest_idx);

  return closest_idx != -1;
}

void ObstacleAvoidanceNode::subscribeToCarPose()
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));  // Depth: KEEP_LAST (5)
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  car_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/cartographer/pose",
    qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      car_pose = msg->pose;

      start_node.x = car_pose.position.x;
      start_node.y = car_pose.position.y;
      start_node.parent = 0;
      start_node.cost = 0.0;
      start_node.is_root = true;
      // Get Goal
      get_current_goal();

      std::vector<Node_struct> tree;
      std::vector<Node_struct> nodes_near_goal;

      tree.clear();
      tree.push_back(start_node);

      if (car_pose.position.x != -0.0097019 and car_pose.position.y != -0.580195) {
        calcClosestIndex(waypoints_, car_pose, self_idx);
      }
      
      if (!obstacle_detected)  // obstacle is not detected, trying to detect
      {
        for (int i = self_idx; i < (self_idx + COLLISION_HORIZON) % 270; i++) {
          path_found_flag = false;
          if (check_collision_2(waypoints_.at(i).position, waypoints_.at(i + 1).position)) {
            obstacle_detected = true;
            target_idx = (curr_goal_ind_ + 4) % waypoints_.size();
            break;
          }
        }
        obstacle_avoidance_trajectory_pub_->publish(trajectory_);
      } else  // obstacle is detected
      {
        if (waypoints_.size() > 0 && car_pose.position.x != 0 && car_pose.position.y != 0) {
          for (int iter = 0; iter < MAX_ITER; iter++) {
            std::vector<double> sampled_point = sample_point();
            int nearest_ind = nearest(tree, sampled_point);
            Node_struct new_node = steer(tree.at(nearest_ind), sampled_point);
            if (!check_collision(tree.at(nearest_ind), new_node)) {
              std::vector<int> nodes_near = near(tree, new_node);
              tree.push_back(new_node);
              // connect new_node to the node in the neighborhood with the minimum cost 
              int min_cost_node_ind = nearest_ind;
              float min_cost =
                tree.at(nearest_ind).cost + line_cost(tree.at(nearest_ind), new_node);
              for (int i = 0; i < nodes_near.size(); i++) {
                if (!check_collision(tree.at(nodes_near.at(i)), new_node)) {
                  float cost =
                    tree.at(nodes_near.at(i)).cost + line_cost(tree.at(nodes_near.at(i)), new_node);
                  if (cost < min_cost) {
                    min_cost_node_ind = nodes_near.at(i);
                    min_cost = cost;
                  }
                }
              }

              tree.back().is_root = false;
              tree.back().cost = min_cost;
              // add edge
              tree.back().parent = min_cost_node_ind;
              tree.at(min_cost_node_ind).children.push_back(tree.size() - 1);

              /** Rewiring **/
              int rewire_count = 0;

              for (int j = 0; j < int(nodes_near.size()); j++) {
                float new_cost = tree.back().cost + line_cost(new_node, tree.at(nodes_near.at(j)));
                if (new_cost < tree.at(nodes_near.at(j)).cost) {
                  if (!check_collision(tree.at(nodes_near.at(j)), new_node)) {
                    // rewire: update parent, cost and costs of all children;
                    float cost_change = new_cost - tree.at(nodes_near.at(j)).cost;
                    tree.at(nodes_near.at(j)).cost = new_cost;
                    // assign new_node to be its new parent
                    int old_parent = tree.at(nodes_near.at(j)).parent;
                    tree.at(nodes_near.at(j)).parent = tree.size() - 1;
                    tree.back().children.push_back(nodes_near.at(j));
                    // remove it from its old parent's children list
                    std::vector<int>::iterator start = tree.at(old_parent).children.begin();
                    std::vector<int>::iterator end = tree.at(old_parent).children.end();
                    tree.at(old_parent).children.erase(remove(start, end, nodes_near.at(j)), end);
                  }
                }
                rewire_count++;
              }
              if (is_goal(
                    tree.back(), waypoints_.at(curr_goal_ind_).position.x,
                    waypoints_.at(curr_goal_ind_).position.y)) {
                nodes_near_goal.push_back(tree.back());
              }
            }
            /** check if goal reached and recover path with the minimum cost**/
            if (iter > MIN_ITER && !nodes_near_goal.empty()) {
              Node_struct best =
                *min_element(nodes_near_goal.begin(), nodes_near_goal.end(), comp_cost);
              std::vector<Node_struct> path_found = find_path(tree, nodes_near_goal.back());

              initialize_marker(
                path_dots, "map", "path", 20, visualization_msgs::msg::Marker::POINTS, green, 0.04);

              for (int i = 0; i < path_found.size(); i++) {
                geometry_msgs::msg::Point p;
                p.x = path_found.at(i).x;
                p.y = path_found.at(i).y;
                path_dots.points.push_back(p);
              }

              double RRT_INTERVAL = 0.1;
              std::vector<geometry_msgs::msg::Point> path_processed;
              for (int i = 0; i < path_dots.points.size() - 1; i++) {
                path_processed.push_back(path_dots.points[i]);
                double dist = sqrt(
                  pow(path_dots.points[i + 1].x - path_dots.points[i].x, 2) +
                  pow(path_dots.points[i + 1].y - path_dots.points[i].y, 2));
                if (dist < RRT_INTERVAL) continue;
                int num = static_cast<int>(ceil(dist / RRT_INTERVAL));
                for (int j = 1; j < num; j++) {
                  geometry_msgs::msg::Point p;
                  p.x = path_dots.points[i].x +
                        j * ((path_dots.points[i + 1].x - path_dots.points[i].x) / num);
                  p.y = path_dots.points[i].y +
                        j * ((path_dots.points[i + 1].y - path_dots.points[i].y) / num);
                  path_processed.push_back(p);
                }
              }
              path_dots.points = path_processed;
              marker_pub_->publish(path_dots);
              visualize_tree(tree);

              avoidance_trajectory_ = createTrajectory(path_found);
              path_found_flag = true;
              break;
            }
            if (path_found_flag) {
              obstacle_avoidance_trajectory_pub_->publish(avoidance_trajectory_);
              if (self_idx == target_idx) {
                obstacle_detected = false;
              }
            }
          }

          if (nodes_near_goal.empty()) {
            // std::cout << "Couldn't find a path" << std::endl;
          }
        }
      }
    });
}

// other subsctiptions
void ObstacleAvoidanceNode::subscribeToLaserScan()
{
  rclcpp::QoS qos(rclcpp::KeepLast(5));  // Depth: KEEP_LAST (5)
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // Utwórz subskrybenta dla tematu /sensing/lidar/scan
  laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/sensing/lidar/scan",
    qos,  // Rozmiar kolejki subskrybenta
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // only reset map when the car has made enough progress
      if (this->now().seconds() - last_time_ > 0.2) {
        map_updated_ = map_;  // might be expensive to copy
        map_updated_2_ = map_;
        occupancy_grid::inflate_map(map_updated_, MARGIN);
        occupancy_grid::inflate_map(map_updated_2_, MARGIN_2);
        last_time_ = this->now().seconds();
      }

      float angle_min = msg->angle_min;
      float angle_increment = msg->angle_increment;

      tf2::Quaternion q(
        car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z,
        car_pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      for (int i = 300; i < msg->ranges.size() - 300; i++) {
        float range = msg->ranges.at(i);
        if (!std::isnan(range) && !std::isinf(range)) {
          float angle = angle_min + angle_increment * i;
          tf2::Vector3 pos_in_car(range * cos(angle) + 0.35, range * sin(angle), 0.0);
          tf2::Vector3 pos_in_map(
            (pos_in_car.x() * cos(yaw) - pos_in_car.y() * sin(yaw)) + car_pose.position.x,
            (pos_in_car.x() * sin(yaw) + pos_in_car.y() * cos(yaw)) + car_pose.position.y, 0.0);

          if (!occupancy_grid::is_xy_occupied(map_, pos_in_map.x(), pos_in_map.y())) {
            occupancy_grid::inflate_cell(
              map_updated_, occupancy_grid::xy2ind(map_updated_, pos_in_map.x(), pos_in_map.y()),
              DETECTED_OBS_MARGIN, 100);
            occupancy_grid::inflate_cell(
              map_updated_2_,
              occupancy_grid::xy2ind(map_updated_2_, pos_in_map.x(), pos_in_map.y()),
              DETECTED_OBS_MARGIN_2, 100);
          }
        }
      }
      // free the cells in which the car occupies (dynamic layer)
      occupancy_grid::inflate_cell(
        map_updated_,
        occupancy_grid::xy2ind(map_updated_, car_pose.position.x, car_pose.position.y), 0.25, 0);
      map_update_pub_->publish(map_updated_);
    });
}

double ObstacleAvoidanceNode::calculateYaw(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  // Calculate the vectors between the points
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  double angleRadians = atan2(dy, dx);
  return angleRadians;
}

autoware_auto_planning_msgs::msg::Trajectory ObstacleAvoidanceNode::createTrajectory(
  const std::vector<Node_struct> & points)
{
  autoware_auto_planning_msgs::msg::Trajectory trajectory;

  trajectory.header.frame_id = "map";

  for (size_t i = 0; i < points.size() - 1; i++) {
    geometry_msgs::msg::Point p1;
    p1.x = points[i].x;
    p1.y = points[i].y;
    geometry_msgs::msg::Point p2;
    p2.x = points[i + 1].x;
    p2.y = points[i + 1].y;

    autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_point;
    geometry_msgs::msg::Pose pose;
    pose.position.x = p2.x;
    pose.position.y = p2.y;
    pose.position.z = 0.0;

    pose.orientation.z = calculateYaw(p1, p2);
    // pose.orientation.w = 1.0;
    trajectory_point.pose = pose;
    trajectory_point.longitudinal_velocity_mps = VELOCITY;

    trajectory.points.push_back(trajectory_point);
  }

  return trajectory;
}

void ObstacleAvoidanceNode::subscribeToTrajectory()
{
  trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/racing_planner/trajectory",
    300,  // Rozmiar kolejki subskrybenta
    [this](const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
      trajectory_ = *msg;
    });
}

}  // namespace obstacle_avoidance

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_avoidance::ObstacleAvoidanceNode)