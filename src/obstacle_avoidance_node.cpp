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

const int MAX_ITER = 30;
const int MIN_ITER = 20;
const double STD = 1.5;  // standard deviation for normal distribution
const double STEER_RANGE = 0.3;
const float NEAR_RANGE = 1.0;
const double GOAL_THRESHOLD = 0.15;
const float GOAL_AHEAD_DIST = 2.5;
const double X_SAMPLE_RANGE = 2;
const double Y_SAMPLE_RANGE = 2;
const float SCAN_RANGE = 3.5;
const float MARGIN = 0.18;
const float DETECTED_OBS_MARGIN = 0.2;
const string file_name =
  "/home/max/autoware/src/universe/external/trajectory_loader/data/trajectory.csv";

ObstacleAvoidanceNode::ObstacleAvoidanceNode(const rclcpp::NodeOptions & options)
: Node("obstacle_avoidance", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  obstacle_avoidance_ = std::make_unique<obstacle_avoidance::ObstacleAvoidance>();
  param_name_ = this->declare_parameter("param_name", 456);
  obstacle_avoidance_->foo(param_name_);

  

  map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  "/map", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    map_updated_ = map_;
    occupancy_grid::inflate_map(map_, MARGIN);
        // Open a file in write mode
    // std::ofstream outfile("map.txt", std::ios_base::app); // Append mode

    // // Check if the file is open
    // if (!outfile.is_open())
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to open file");
    //   return;
    // }

    // // Write the data to the file
    // outfile << "Map Info: \n";
    // outfile << "Width: " << map_.info.width << "\n";
    // outfile << "Height: " << map_.info.height << "\n";
    // outfile << "Resolution: " << map_.info.resolution << "\n";
    // outfile << "Origin: [" << map_.info.origin.position.x << ", " << map_.info.origin.position.y << ", " << map_.info.origin.position.z << "]\n";
    // outfile << "Data: \n";

    // for (size_t i = 0; i < map_.data.size(); ++i)
    // {
    //   outfile << static_cast<int>(map_.data[i]) << " ";
    //   if ((i + 1) % map_.info.width == 0)
    //   {
    //     outfile << "\n";
    //   }
    // }

    // // Close the file
    // outfile.close();
  });

  obstacle_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/obstacle_point", 100);
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_found", 10);
  goal_viz_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal", 10);
  tree_nodes_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tree_nodes", 10);
  tree_branches_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tree_branches", 10);
  map_update_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_updated", 10);
  obstacle_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle_viz", 10);

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

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&ObstacleAvoidanceNode::on_timer, this));
  subscribeToCarPose();
  // subscribeToTrajectory();
  subscribeToLaserScan();
  // subscribeToPredictedTrajectory();

  visualize_map();
  if (waypoints_.empty()) {
    load_waypoints(
      "/home/max/autoware/src/universe/external/trajectory_loader/data/trajectory.csv");
  }
  reset_goal();

}

// make function to load waypoints from .csv file where x and y are separated by ; and are on the
// same line and are 2nd and 3rd element of the line
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

    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    waypoints_.push_back(point);
  }
}

void ObstacleAvoidanceNode::visualize_map() {
    // Tworzenie markera
    visualization_msgs::msg::Marker dots;
    
    // Definiowanie kolorów (czerwony z pełną przezroczystością)
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;

    // Inicjalizacja markera przy użyciu funkcji initialize_marker
    this->initialize_marker(dots, "map", "obstacle", 0, visualization_msgs::msg::Marker::POINTS, color, 0.04);

    // Dodawanie punktów przeszkód do markera
    for (size_t i = 0; i < map_.data.size(); i++) {
        if (map_.data[i] == 100) {
            geometry_msgs::msg::Point p;
            p.x = occupancy_grid::ind2x(map_, i);
            p.y = occupancy_grid::ind2y(map_, i);
            p.z = 0.0;
            dots.points.push_back(p);
        }
    }

    // Publikowanie markera
    obstacle_viz_pub_->publish(dots);
}

bool comp_cost(Node_struct & n1, Node_struct & n2)
{
  return n1.cost < n2.cost;
}

void ObstacleAvoidanceNode::on_timer()
{
  // Lookup transform and update car pose
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
  }

  // car_pose.position.x = transform_stamped.transform.translation.x;
  // car_pose.position.y = transform_stamped.transform.translation.y;

  // std::cout << "Car pose: " << car_pose.position.x << " " << car_pose.position.y << std::endl;
  // start_node.x = car_pose.position.x;
  // start_node.y = car_pose.position.y;
  // start_node.parent = 0;
  // start_node.cost = 0.0;
  // start_node.is_root = true;
  // // Get Goal
  // get_current_goal();

  // std::vector<Node_struct> tree;
  // std::vector<Node_struct> nodes_near_goal;

  // tree.clear();
  // tree.push_back(start_node);
  // if (waypoints_.size() > 0) {
  //   for (int iter = 0; iter < MAX_ITER; iter++) {
  //     std::vector<double> sampled_point = sample();
  //     int nearest_ind = nearest(tree, sampled_point);
  //     Node_struct new_node = steer(tree.at(nearest_ind), sampled_point);
  //     if (!check_collision(tree.at(nearest_ind), new_node)) {
  //       std::vector<int> nodes_near = near(tree, new_node);
  //       tree.push_back(new_node);

  //       /** connect new_node to the node in the neighborhood with the minimum cost **/
  //       int min_cost_node_ind = nearest_ind;
  //       float min_cost = tree.at(nearest_ind).cost + line_cost(tree.at(nearest_ind), new_node);
  //       std::cout << "nodes_near.size(): " << nodes_near.size() << std::endl;
  //       std::cout << "tree.size(): " << tree.size() << std::endl;
  //       for (int i = 0; i < nodes_near.size(); i++) {
  //         if (!check_collision(tree.at(nodes_near.at(i)), new_node)) {
  //           float cost =
  //             tree.at(nodes_near.at(i)).cost + line_cost(tree.at(nodes_near.at(i)), new_node);
  //           if (cost < min_cost) {
  //             min_cost_node_ind = nodes_near.at(i);
  //             min_cost = cost;
  //           }
  //         }
  //       }

  //       tree.back().is_root = false;
  //       tree.back().cost = min_cost;
  //       // add edge
  //       tree.back().parent = min_cost_node_ind;
  //       tree.at(min_cost_node_ind).children.push_back(tree.size() - 1);

  //       /** Rewiring **/
  //       int rewire_count = 0;
  //       for (int j = 0; j < int(nodes_near.size()); j++) {
  //         float new_cost = tree.back().cost + line_cost(new_node, tree.at(nodes_near.at(j)));
  //         if (new_cost < tree.at(nodes_near.at(j)).cost) {
  //           if (!check_collision(tree.at(nodes_near.at(j)), new_node)) {
  //             // rewire: update parent, cost and costs of all children;
  //             float cost_change = new_cost - tree.at(nodes_near.at(j)).cost;
  //             tree.at(nodes_near.at(j)).cost = new_cost;
  //             // assign new_node to be its new parent
  //             int old_parent = tree.at(nodes_near.at(j)).parent;
  //             tree.at(nodes_near.at(j)).parent = tree.size() - 1;
  //             tree.back().children.push_back(nodes_near.at(j));
  //             // remove it from its old parent's children list
  //             std::vector<int>::iterator start = tree.at(old_parent).children.begin();
  //             std::vector<int>::iterator end = tree.at(old_parent).children.end();
  //             tree.at(old_parent).children.erase(remove(start, end, nodes_near.at(j)), end);
  //             // update_children_cost(tree, nodes_near.at(j), cost_change); //
  //             optional(expensive)
  //           }
  //         }
  //         rewire_count++;
  //       }
  //       // cout<<"rewire: "<<rewire_count<<endl;
  //       if (is_goal(
  //             tree.back(), waypoints_[predicted_self_idx].x, waypoints_[predicted_self_idx].y)) {
  //         nodes_near_goal.push_back(tree.back());
  //       }
  //     }
  //     /** check if goal reached and recover path with the minimum cost**/
  //     if (iter > MIN_ITER && !nodes_near_goal.empty()) {
  //       Node_struct best = *min_element(nodes_near_goal.begin(), nodes_near_goal.end(),
  //       comp_cost); std::vector<Node_struct> path_found = find_path(tree,
  //       nodes_near_goal.back());

  //       visualization_msgs::msg::Marker path_dots;
  //       path_dots.header.frame_id = "map";
  //       path_dots.id = 20;
  //       path_dots.ns = "path";
  //       path_dots.type = visualization_msgs::msg::Marker::POINTS;
  //       path_dots.scale.x = path_dots.scale.y = path_dots.scale.z = 0.08;
  //       path_dots.action = visualization_msgs::msg::Marker::ADD;
  //       path_dots.pose.orientation.w = 1.0;
  //       path_dots.color.g = 0.0;
  //       path_dots.color.r = 1.0;
  //       path_dots.color.a = 1.0;

  //       for (int i = 0; i < path_found.size(); i++) {
  //         geometry_msgs::msg::Point p;
  //         p.x = path_found.at(i).x;
  //         p.y = path_found.at(i).y;
  //         path_dots.points.push_back(p);
  //       }
  //       double RRT_INTERVAL = 0.2;
  //       std::vector<geometry_msgs::msg::Point> path_processed;
  //       for (int i = 0; i < path_dots.points.size() - 1; i++) {
  //         path_processed.push_back(path_dots.points[i]);
  //         double dist = sqrt(
  //           pow(path_dots.points[i + 1].x - path_dots.points[i].x, 2) +
  //           pow(path_dots.points[i + 1].y - path_dots.points[i].y, 2));
  //         if (dist < RRT_INTERVAL) continue;
  //         int num = static_cast<int>(ceil(dist / RRT_INTERVAL));
  //         for (int j = 1; j < num; j++) {
  //           geometry_msgs::msg::Point p;
  //           p.x = path_dots.points[i].x +
  //                 j * ((path_dots.points[i + 1].x - path_dots.points[i].x) / num);
  //           p.y = path_dots.points[i].y +
  //                 j * ((path_dots.points[i + 1].y - path_dots.points[i].y) / num);
  //           path_processed.push_back(p);
  //         }
  //       }

  //       path_dots.points = path_processed;
  //       marker_publisher_->publish(path_dots);
  //       //            track_path(path);
  //       visualize_tree(tree);
  //       std::cout << "path found" << std::endl;
  //       break;
  //     }
  //   }

  //   if (nodes_near_goal.empty()) {
  //     std::cout << "Couldn't find a path" << std::endl;
  //   }
  // }
}

inline double ObstacleAvoidanceNode::calculate_dist2(double x1, double x2, double y1, double y2)
{
  // considering that doing sqrt is expensive
  return pow(x1 - x2, 2) + pow(y1 - y2, 2);
}

void ObstacleAvoidanceNode::get_current_goal()
{
  float dist_to_goal2 = calculate_dist2(
    waypoints_.at(curr_goal_ind_).x, car_pose.position.x, waypoints_.at(curr_goal_ind_).y,
    car_pose.position.y);
  // std::cout << "dist_to_goal2: " << dist_to_goal2 << std::endl;
  // goal out of range, reset goal
  if (dist_to_goal2 > pow(GOAL_AHEAD_DIST, 2)) {
    reset_goal();
  }
  if (occupancy_grid::is_xy_occupied(
        map_updated_, waypoints_.at(curr_goal_ind_).x, waypoints_.at(curr_goal_ind_).y)) {
    advance_goal();
  }
  // enough progress made, advance goal
  else if (dist_to_goal2 < pow(GOAL_AHEAD_DIST * 0.75, 2)) {
    advance_goal();
  }
}

void ObstacleAvoidanceNode::reset_goal()
{
  // boost::shared_ptr<nav_msgs::msg::Odometry const> pose_ptr;
  // pose_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>(pose_topic, ros::Duration(5.0));

  if (
    (car_pose.position.x == -0.0097019 && car_pose.position.y == -0.580195) ||
    (car_pose.position.x == 0 && car_pose.position.y == 0)) {
    RCLCPP_INFO(this->get_logger(), "Failed to receive car's pose");
  } else {
    int closest_ind = find_closest_waypoint(waypoints_, car_pose);
    float closest_dist2 = calculate_dist2(
      waypoints_.at(closest_ind).x, car_pose.position.x, waypoints_.at(closest_ind).y,
      car_pose.position.y);
    if (closest_dist2 > pow(GOAL_AHEAD_DIST, 2)) {
      throw std::runtime_error(
        "Couldn't find a goal in range. Reposition the car somewhere near the waypoints");
    }
    // advance from closest waypoint until one that is around 0.9 GOAL_AHEAD_DIST away
    curr_goal_ind_ = closest_ind;
    // std::cout << "curr_goal_ind_: " << curr_goal_ind_ << std::endl;
    advance_goal();
  }
}

void ObstacleAvoidanceNode::advance_goal()
{
  // advance in waypoints from current goal to a point that is around 0.9*MAX_GOAL_AHEAD distance
  // ahead
  using namespace occupancy_grid;
  int curr_ind = curr_goal_ind_;
  if (curr_ind >= waypoints_.size()) {
    curr_ind = 0;
  }
  float pose_x = car_pose.position.x;
  float pose_y = car_pose.position.y;
  float curr_dist2 =
    calculate_dist2(waypoints_.at(curr_ind).x, pose_x, waypoints_.at(curr_ind).y, pose_y);

  while (curr_dist2 < pow(GOAL_AHEAD_DIST * 0.9, 2) ||
         is_xy_occupied(map_updated_, waypoints_.at(curr_ind).x, waypoints_.at(curr_ind).y)) {
    curr_dist2 =
      calculate_dist2(waypoints_.at(curr_ind).x, pose_x, waypoints_.at(curr_ind).y, pose_y);
    curr_ind++;
    if (curr_ind >= waypoints_.size()) {
      curr_ind = 0;
    }
    if (
      curr_dist2 > pow(GOAL_AHEAD_DIST, 2) &&
      is_xy_occupied(map_updated_, waypoints_.at(curr_ind).x, waypoints_.at(curr_ind).y)) {
      break;
    }
  }
  curr_goal_ind_ = max(0, curr_ind - 1);
  // std::cout << "curr_goal_ind_: " << curr_goal_ind_ << std::endl;
}

int ObstacleAvoidanceNode::find_closest_waypoint(
  const std::vector<geometry_msgs::msg::Point> & waypoints, const geometry_msgs::msg::Pose & pose)
{
  float min_dist2 = 100000.0;
  int min_ind = -1;
  for (int i = 0; i < waypoints.size(); i++) {
    float dist2 =
      calculate_dist2(waypoints.at(i).x, pose.position.x, waypoints.at(i).y, pose.position.y);
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      min_ind = i;
    }
  }
  // std::cout << "min_ind: " << min_ind << std::endl;
  return min_ind;
}

std::vector<double> ObstacleAvoidanceNode::sample_point()
{
  // This method returns a sampled point from the free space

  std::vector<double> sampled_point;

  // double x = car_pose.position.x + uni_dist_(gen_)*X_SAMPLE_RANGE;
  // double y = car_pose.position.y + uni_dist_(gen_)*Y_SAMPLE_RANGE;

  // std::cout << "widać że się wywołuje" << std::endl;
  std::normal_distribution<double> norm_dist_x(
    0.6 * waypoints_.at(curr_goal_ind_).x + 0.4 * car_pose.position.x, STD);
  std::normal_distribution<double> norm_dist_y(
    0.6 * waypoints_.at(curr_goal_ind_).y + 0.4 * car_pose.position.y, STD);
  double x = norm_dist_x(gen_);
  double y = norm_dist_y(gen_);

  // std::cout << "nadal widać" << std::endl;

  // sample recursively until one in the free space gets returned
  if (!occupancy_grid::is_xy_occupied(map_updated_, x, y)) {
    sampled_point.push_back(x);
    sampled_point.push_back(y);
    // std::cout << "nadal widać jak znalazło" << std::endl;
    return sampled_point;
  } else {
    // std::cout << "nadal widać jak nie znalazło" << std::endl;
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
      // std::cout << "i: " << i << std::endl;
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
  int x_cell_diff = abs(ceil((nearest_node.x - new_node.x) / map_updated_.info.resolution));
  int y_cell_diff = abs(ceil((nearest_node.y - new_node.y) / map_updated_.info.resolution));

  double dt = 1.0 / max(x_cell_diff, y_cell_diff);
  double t = 0.0;
  for (int i = 0; i <= max(x_cell_diff, y_cell_diff); i++) {
    double x = nearest_node.x + t * (new_node.x - nearest_node.x);
    double y = nearest_node.y + t * (new_node.y - nearest_node.y);
    if (occupancy_grid::is_xy_occupied(map_updated_, x, y)) {
      collision = true;
      break;
    }
    t += dt;
  }
  return collision;
}

bool ObstacleAvoidanceNode::is_goal(Node_struct & latest_added_node, double goal_x, double goal_y)
{
  // This method checks if the latest node added to the tree is close
  // enough
  return calculate_dist2(goal_x, latest_added_node.x, goal_y, latest_added_node.y) <
         pow(GOAL_THRESHOLD, 2);
}

std::vector<Node_struct> ObstacleAvoidanceNode::find_path(
  std::vector<Node_struct> & tree, Node_struct & node)
{
  // This method traverses the tree from the node that has been determined
  // as goal
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
      // std::cout << "i neighborhood: " << i << std::endl;
      neighborhood.push_back(i);
    }
  }
  return neighborhood;
}

void ObstacleAvoidanceNode::visualize_tree(std::vector<Node_struct> & tree)
{
  // geometry_msgs::msg::Pose goal_pose;
  // goal_pose.orientation.w = 1.0;
  // goal_pose.position.x = waypoints_.at(curr_goal_ind_).x;
  // goal_pose.position.y = waypoints_.at(curr_goal_ind_).y;

  goal_marker.pose.orientation.w = 1.0;
  goal_marker.pose.position.x = waypoints_.at(curr_goal_ind_).x;
  goal_marker.pose.position.y = waypoints_.at(curr_goal_ind_).y;
  goal_marker.pose.position.z = 0.0;
  // goal_viz_publisher_->set_pose(goal_pose);
  goal_viz_publisher_->publish(goal_marker);

  // Plot tree
  tree_nodes.points.clear();
  tree_branch.points.clear();

  for (size_t i = 0; i < tree.size(); i++) {
    geometry_msgs::msg::Point p;
    p.x = tree.at(i).x;
    p.y = tree.at(i).y;
    p.z = 0.0;  // Set z to 0 or any other value as needed
    tree_nodes.points.push_back(p);
    for (size_t j = 0; j < tree.at(i).children.size(); j++) {
      tree_branch.points.push_back(p);
      geometry_msgs::msg::Point p_child;
      p_child.x = tree.at(tree.at(i).children.at(j)).x;
      p_child.y = tree.at(tree.at(i).children.at(j)).y;
      p_child.z = 0.0;  // Set z to 0 or any other value as needed
      tree_branch.points.push_back(p_child);
    }
  }
  tree_branches_pub_->publish(tree_branch);
  tree_nodes_pub_->publish(tree_nodes);
}

// void ObstacleAvoidanceNode::track_path(const nav_msgs::msg::Path& path){
//     //use pure pursuit to track the path planned by RRT
//     int i =0;
//     while (i<path.poses.size()-1){
//         float x = path.poses.at(i).pose.position.x;
//         float y = path.poses.at(i).pose.position.y;
//         float x_car = car_pose.position.x;
//         float y_car = car_pose.position.y;
//         if (calculate_dist2(x, x_car, y, y_car) > pow(LOOK_AHEAD_DIST, 2)){
//             break;
//         }
//         i++;
//     }
//     //calculate setpoint for pure pursuit to track
//     tf::Vector3 p1(path.poses.at(i).pose.position.x, path.poses.at(i).pose.position.y, 0.0);
//     tf::Vector3 p2(path.poses.at(max(0,i-1)).pose.position.x,
//     path.poses.at(max(0,i-1)).pose.position.y, 0.0); pos_sp_ = tf_.inverse() * ((p1 + p2) / 2.0);
//     float curvature = 2*abs(pos_sp_.getY())/(LOOK_AHEAD_DIST * LOOK_AHEAD_DIST);

//     // publish drive cmds
//     float steering_cmd =  P_GAIN * curvature;
//     if (pos_sp_.getY()<0){steering_cmd *= -1.0;}
//     publish_cmd(steering_cmd);

//     // visualize setpoint for tracking
//     geometry_msgs::Pose pose;
//     pose.orientation.w = 1.0;
//     pose.position.x = pos_sp_.x(); pose.position.y = pos_sp_.y();
//     pos_sp_viz->set_pose(pose);
//     pos_sp_viz->publish_marker();
// }

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

// //calcClosestIndex section
geometry_msgs::msg::Pose ObstacleAvoidanceNode::getPose(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const int idx)
{
  return traj.points.at(idx).pose;
}

inline double ObstacleAvoidanceNode::calcDist2d(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot((a.x - b.x), (a.y - b.y));
}

double ObstacleAvoidanceNode::normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI) {
    res -= (2.0 * M_PI);
  }
  while (res < -M_PI) {
    res += 2.0 * M_PI;
  }

  return res;
}

bool ObstacleAvoidanceNode::calcClosestIndex(
  const std::vector<geometry_msgs::msg::Point>& waypoints, const geometry_msgs::msg::Pose &
  pose, size_t & output_closest_idx, const double dist_thr, const double angle_thr)
{
  double dist_min = std::numeric_limits<double>::max();
  // const double yaw_pose = tf2::getYaw(pose.orientation);
  int closest_idx = -1;
  // std::cout << "waypoints_ point:" << waypoints[0].x << " " << waypoints[0].y << std::endl;
  // std::cout << "pose point:" << pose.position.x << " " << pose.position.y << std::endl;
  for (int i = 0; i < waypoints.size(); i++) {
    const double dist = calcDist2d(waypoints[i], pose.position);

    /* check distance threshold */
    // if (dist > dist_thr) {
    //   continue;
    // }

    /* check angle threshold */
    // double yaw_i = tf2::getYaw(getPose(traj, i).orientation);
    // double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_i);

    // if (std::fabs(yaw_diff) > angle_thr) {
    //   continue;
    // }

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

  // Utwórz subskrybenta dla tematu /sensing/lidar/scan
  car_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/cartographer/pose",
    qos,  // Rozmiar kolejki subskrybenta
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
      car_pose = msg->pose;
      // std::cout << "Car pose: " << car_pose.position.x << " " << car_pose.position.y << std::endl;

      // calcClosestIndex(waypoints_, car_pose, self_idx);
      // std::cout << "self_idx: " << self_idx << std::endl;

      // std::cout << "Car pose: " << car_pose.position.x << " " << car_pose.position.y << std::endl;
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
      if (waypoints_.size() > 0 && car_pose.position.x > 1 && car_pose.position.y != 0) {
        for (int iter = 0; iter < MAX_ITER; iter++) {
          // std::cout << "początek pętli" << std::endl; //to widać
          std::vector<double> sampled_point = sample_point();
          // std::cout << "essa, środek pętli?" << std::endl; //tego już nie widać
          int nearest_ind = nearest(tree, sampled_point);
          Node_struct new_node = steer(tree.at(nearest_ind), sampled_point);
          if (!check_collision(tree.at(nearest_ind), new_node)) {
            std::vector<int> nodes_near = near(tree, new_node);
            tree.push_back(new_node);
            /** connect new_node to the node in the neighborhood with the minimum cost **/
            int min_cost_node_ind = nearest_ind;
            float min_cost = tree.at(nearest_ind).cost + line_cost(tree.at(nearest_ind), new_node); 
            // std::cout << "nodes_near.size(): " << nodes_near.size() << std::endl;
            // std::cout << "tree.size(): " << tree.size() << std::endl;
            // std::cout << "iter: " << iter << std::endl;
            for (int i = 0; i < nodes_near.size(); i++) {
              if (!check_collision(tree.at(nodes_near.at(i)), new_node)) {
                float cost = tree.at(nodes_near.at(i)).cost + line_cost(tree.at(nodes_near.at(i)), new_node);
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
            // std::cout << "int <><><>nodes_near.size(): " << int(nodes_near.size()) << std::endl;
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
                  // std::cout << "tree.at(nodes_near.at(j)).children.size(): " << tree.at(nodes_near.at(j)).children.size() << std::endl;
                  // std::cout << "tree.at(old_parent).children.size(): " << tree.at(old_parent).children.size() << std::endl;
                  // remove it from its old parent's children list
                  std::vector<int>::iterator start = tree.at(old_parent).children.begin();
                  std::vector<int>::iterator end = tree.at(old_parent).children.end();
                  tree.at(old_parent).children.erase(remove(start, end, nodes_near.at(j)), end);
                  // update_children_cost(tree, nodes_near.at(j), cost_change); //optional(expensive)
                }
              }
              rewire_count++;
            }
            // std::cout << "rewire: " << rewire_count << std::endl;
            // std::cout << "nodes_near_goal.size(): " << nodes_near_goal.size() << std::endl;
            // std::cout << "waypoints_.size(): " << waypoints_.size() << std::endl;
            // std::cout << "curr_goal_ind_: " << curr_goal_ind_ << std::endl;
            // std::cout << "new_node.children.size(): " << new_node.children.size() << std::endl;
            // std::cout << "Car pose: " << car_pose.position.x << " " << car_pose.position.y << std::endl;
            // std::cout << "path_found.size(): " << path_found.size() << std::endl;
            // std::cout << "path_processed.size(): " << path_processed.size() << std::endl;


            if (is_goal(tree.back(), waypoints_.at(curr_goal_ind_).x, waypoints_.at(curr_goal_ind_).y))
            {
              nodes_near_goal.push_back(tree.back());
            }
          }
          // std::cout << "koniec pętli" << std::endl;
          /** check if goal reached and recover path with the minimum cost**/
          if (iter > MIN_ITER && !nodes_near_goal.empty()) {
            Node_struct best = *min_element(nodes_near_goal.begin(), nodes_near_goal.end(), comp_cost); 
            std::vector<Node_struct> path_found = find_path(tree, nodes_near_goal.back());

            // visualization_msgs::msg::Marker path_dots;
            // path_dots.header.frame_id = "map";
            // path_dots.id = 20;
            // path_dots.ns = "path";
            // path_dots.type = visualization_msgs::msg::Marker::POINTS;
            // path_dots.scale.x = path_dots.scale.y = path_dots.scale.z = 0.08;
            // path_dots.action = visualization_msgs::msg::Marker::ADD;
            // path_dots.pose.orientation.w = 1.0;
            // path_dots.color.g = 0.0;
            // path_dots.color.r = 1.0;
            // path_dots.color.a = 1.0;


            initialize_marker(
              path_dots, "map", "path", 20, visualization_msgs::msg::Marker::POINTS, red, 0.08);

            for (int i = 0; i < path_found.size(); i++) {
              geometry_msgs::msg::Point p;
              p.x = path_found.at(i).x;
              p.y = path_found.at(i).y;
              path_dots.points.push_back(p);
            }
            double RRT_INTERVAL = 0.2;
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
                p.x = path_dots.points[i].x + j * ((path_dots.points[i + 1].x - path_dots.points[i].x) / num);
                p.y = path_dots.points[i].y + j * ((path_dots.points[i + 1].y - path_dots.points[i].y) / num);
                path_processed.push_back(p);
              }
            }

            path_dots.points = path_processed;
            marker_publisher_->publish(path_dots);
            //track_path(path);
            visualize_tree(tree);
            std::cout << "path found" << std::endl;
            break;
          }
        }

        if (nodes_near_goal.empty()) {
          std::cout << "Couldn't find a path" << std::endl;
        }
      }
    });
}

//other subsctiptions
void ObstacleAvoidanceNode::subscribeToLaserScan()
{
  rclcpp::QoS qos(rclcpp::KeepLast(5)); // Depth: KEEP_LAST (5)
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Utwórz subskrybenta dla tematu /sensing/lidar/scan
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/sensing/lidar/scan",
        qos, // Rozmiar kolejki subskrybenta
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            
      
      map_updated_ = map_;

      float angle_min = msg->angle_min;
      float angle_increment = msg->angle_increment;

      tf2::Quaternion q(
        car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z,
        car_pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      // for (int i = 360; i < msg->ranges.size() - 360; i++) //90 stopni na środku lidara
      for (int i = 0; i < msg->ranges.size(); i++)
      {
          float range = msg->ranges.at(i);
          // if (range > SCAN_RANGE) {
          //     continue;
          // }
          if (!std::isnan(range) && !std::isinf(range)) {
            float angle = angle_min + angle_increment * i;
            tf2::Vector3 pos_in_car(range*cos(angle) + 0.35, range*sin(angle), 0.0);
            tf2::Vector3 pos_in_map((pos_in_car.x()*cos(yaw)-pos_in_car.y()*sin(yaw)) + car_pose.position.x, (pos_in_car.x()*sin(yaw)+pos_in_car.y()*cos(yaw)) + car_pose.position.y, 0.0);

            if (!occupancy_grid::is_xy_occupied(map_, pos_in_map.x(), pos_in_map.y())){
                occupancy_grid::inflate_cell(map_updated_, occupancy_grid::xy2ind(map_updated_, pos_in_map.x(), pos_in_map.y()), DETECTED_OBS_MARGIN, 100);
            }
          }
      }
      // free the cells in which the car occupies (dynamic layer)
      occupancy_grid::inflate_cell(map_updated_, occupancy_grid::xy2ind(map_updated_, car_pose.position.x, car_pose.position.y), 0.25, 0);
      map_update_pub_->publish(map_updated_);




      // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
      // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
      // distances_array = msg->ranges;
      // for (int i = 0; i < msg->ranges.size(); i++)
      // {
      //   distances_array[i] = msg->ranges[i];
      // }

      // std::cout << "before calc idx" << car_pose.position.x << " " << car_pose.position.y <<
      // std::endl; define scan start global angle float start_angle = msg->angle_min -
      // car_pose.orientation.z;
      // float angle_increment = msg->angle_increment;
      auto obstacle_point_msg = geometry_msgs::msg::PointStamped();
      // convert quaternion to euler
      // tf2::Quaternion q(
      //   car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z,
      //   car_pose.orientation.w);
      // tf2::Matrix3x3 m(q);
      // double roll, pitch, yaw;
      // m.getRPY(roll, pitch, yaw);
      // size_t self_idx;
      // const auto & current_pose = car_pose;
      // const auto & trajectory = trajectory_;
      // autoware_auto_planning_msgs::msg::Trajectory trajectory = trajectory_;

      // if car pose is not x=-0.0097019 and y=-0.580195, calculate the closest index
      if (car_pose.position.x != -0.0097019 and car_pose.position.y != -0.580195) {
        calcClosestIndex(waypoints_, car_pose, self_idx);
      }
      // calcClosestIndex(trajectory, current_pose, self_idx);
      // current_pose = trajectory.at(self_idx+30).pose;

      // std::cout << "self_idx: " << self_idx << std::endl;
      // std::cout << car_pose.position.x << " " << car_pose.position.y << std::endl;
      // std::cout << "predicted_self_idx: " << predicted_self_idx << std::endl;
      // std::cout << "waypoints_[predicted_self_idx]: " << waypoints_[predicted_self_idx].x << " "
      //           << waypoints_[predicted_self_idx].y << std::endl;

      // trajectory points
      for (int j = self_idx; j < (self_idx + 30) % 270; j++) {
        auto temp_point = waypoints_.at(j);
        // std::cout << "tr_point: " << j << std::endl;
        // create float vector of points (x,y)
        // std::vector<std::pair<float, float>> points;

        // laser scan points
        for (int i = 420; i < msg->ranges.size() - 420; i++) {
          int counter = 0;
          float angle = i * angle_increment + msg->angle_min;
          // std::cout << "i: " << i << " angle: " << angle << std::endl;

          float x = msg->ranges[i] * cos(angle) + 0.35;  // car
          float y = msg->ranges[i] * sin(angle);         // car
          // rotate the point to the car frame
          float x_car = x * cos(yaw) - y * sin(yaw);
          float y_car = x * sin(yaw) + y * cos(yaw);

          x = x_car + car_pose.position.x;
          y = y_car + car_pose.position.y;

          // points.push_back(std::make_pair(x, y));

          float distance = sqrt(pow(x - temp_point.x, 2) + pow(y - temp_point.y, 2));
          if (distance < 0.05) {
            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            // add obstacle point to the vector
            obstacle_points.push_back(point);
          }

          // check least 5 points from vector obstacle_points and calculate distance between the two
          // furthest from each other
          if (obstacle_points.size() > 5) {
            float max_distance = 0;
            for (int i = obstacle_points.size() - 5; i < obstacle_points.size(); i++) {
              for (int j = i + 1; j < obstacle_points.size(); j++) {
                float distance = sqrt(
                  pow(obstacle_points[i].x - obstacle_points[j].x, 2) +
                  pow(obstacle_points[i].y - obstacle_points[j].y, 2));
                if (distance > max_distance) {
                  max_distance = distance;
                }
              }
            }
            // if the distance is smaller than 0.02m, publish the point
            if (max_distance < 0.02) {
              obstacle_point_msg.header.stamp = msg->header.stamp;
              obstacle_point_msg.header.frame_id = "map";
              obstacle_point_msg.point.x = obstacle_points[obstacle_points.size()-1].x;
              obstacle_point_msg.point.y = obstacle_points[obstacle_points.size()-1].y;
              obstacle_point_msg.point.z = 0.0;
              obstacle_publisher_->publish(obstacle_point_msg);
            }
          }
        }
      }
    });
}

// void ObstacleAvoidanceNode::subscribeToTrajectory()
// {
//   trajectory_subscriber_ =
//   this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
//     "/planning/racing_planner/trajectory",
//     300,  // Rozmiar kolejki subskrybenta
//     [this](const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
//       // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
//       // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
//       trajectory_ = *msg;
//     });
// }

// // make subscriber to /control/trajectory_follower/lateral/predicted_trajectory
// void ObstacleAvoidanceNode::subscribeToPredictedTrajectory()
// {
//   // Utwórz subskrybenta dla tematu /control/trajectory_follower/lateral/predicted_trajectory
//   predicted_trajectory_subscriber_ =
//     this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
//       "/control/trajectory_follower/lateral/predicted_trajectory",
//       10,  // Rozmiar kolejki subskrybenta
//       [this](const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
//         // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
//         // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
//         predicted_trajectory_ = *msg;

//         // auto obstacle_point_msg = geometry_msgs::msg::PointStamped();

//         for (int i = 0; i < msg->points.size(); i++) {
//           // tutaj jest 10 punktów, globalne współrzędne aktualnej przewidywanej trajektorii
//           predicted_poses_array[i] = msg->points[i].pose;
//         }
//         calcClosestIndex(waypoints_, predicted_poses_array[9], predicted_self_idx);
//         // obstacle_point_msg.header.stamp = msg->header.stamp;
//         // obstacle_point_msg.header.frame_id = "map";
//         // obstacle_point_msg.point.x = predicted_poses_array[9].position.x;
//         // obstacle_point_msg.point.y = predicted_poses_array[9].position.y;
//         // obstacle_point_msg.point.z = 0.0;
//         // obstacle_publisher_->publish(obstacle_point_msg);
//       });
// }

}  // namespace obstacle_avoidance

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_avoidance::ObstacleAvoidanceNode)