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

const int MAX_ITER = 1200;
const int MIN_ITER = 1000;
const double STD = 1.5;   // standard deviation for normal distribution
const double STEER_RANGE = 0.3;
const float NEAR_RANGE = 1.0;
const double GOAL_THRESHOLD = 0.15;

ObstacleAvoidanceNode::ObstacleAvoidanceNode(const rclcpp::NodeOptions & options)
:  Node("obstacle_avoidance", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  obstacle_avoidance_ = std::make_unique<obstacle_avoidance::ObstacleAvoidance>();
  param_name_ = this->declare_parameter("param_name", 456);
  obstacle_avoidance_->foo(param_name_);

  obstacle_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/obstacle_point", 100);
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_found", 1);
  goal_viz_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal", 1);
  // goal_viz_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/goal", 1);
  // tree_nodes_pub_ = create_publisher<visualization_msgs::msg::Marker>("tree_nodes", 1);
  // tree_branches_pub_ = create_publisher<visualization_msgs::msg::Marker>("tree_branches", 1);
  tree_nodes_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tree_nodes", 1);
  tree_branches_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tree_branches", 1);

  // Initialize colors
  std_msgs::msg::ColorRGBA red;
  red.r = 1.0; red.a = 1.0;
  std_msgs::msg::ColorRGBA green;
  green.g = 1.0; green.a = 1.0;
  std_msgs::msg::ColorRGBA blue;
  blue.b = 1.0; blue.a = 1.0;

  // Set up markers
  initialize_marker(goal_marker, "map", "goal", 0, visualization_msgs::msg::Marker::SPHERE, green, 0.3);
  initialize_marker(tree_nodes, "map", "nodes", 5, visualization_msgs::msg::Marker::POINTS, red, 0.05);
  initialize_marker(tree_branch, "map", "branch", 6, visualization_msgs::msg::Marker::LINE_LIST, blue, 0.01);
  // goal_viz_ = new MarkerVisualizer(goal_viz_pub_, "goal", "map", green, 0.3, visualization_msgs::msg::Marker::SPHERE);
  // map_updates_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_updates", 10);
  
  // Subskrybuj temat /sensing/lidar/scan
  // subscribeToCarPose();
  subscribeToLaserScan();
  subscribeToPredictedTrajectory();
  subscribeToTrajectory(); // 10 punktów, globalne współrzędne 

  // Initialize tf2 listener timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ObstacleAvoidanceNode::on_timer, this));
  // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());


  map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      1,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          occupancy_grid_.info = msg->info;
          occupancy_grid_.data.resize(msg->data.size());
          std::copy(msg->data.begin(), msg->data.end(), occupancy_grid_.data.begin());
      });
}

geometry_msgs::msg::Pose ObstacleAvoidanceNode::getPose(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const int idx)
{
  return traj.points.at(idx).pose;
}

inline double ObstacleAvoidanceNode::calcDist2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot((a.x - b.x), (a.y - b.y));
}

inline double ObstacleAvoidanceNode::calculate_dist2(double x1, double x2, double y1, double y2){
  // considering that doing sqrt is expensive
  return pow(x1-x2, 2) + pow(y1-y2,2);
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
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & pose,
  size_t & output_closest_idx, const double dist_thr, const double angle_thr)
{
  double dist_min = std::numeric_limits<double>::max();
  const double yaw_pose = tf2::getYaw(pose.orientation);
  int closest_idx = -1;

  for (int i = 0; i < static_cast<int>(traj.points.size()); ++i) {
    // if (pose.position.x == -0.0097019 and pose.position.y == -0.580195)
    // {
    //   continue;
    // }
    const double dist = calcDist2d(getPose(traj, i).position, pose.position);

    /* check distance threshold */
    // if (dist > dist_thr) {
    //   continue;
    // }

    /* check angle threshold */
    double yaw_i = tf2::getYaw(getPose(traj, i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_i);

    // if (std::fabs(yaw_diff) > angle_thr) {
    //   continue;
    // }

    if (dist < dist_min) {
      dist_min = dist;
      closest_idx = i;
    }
  }

  output_closest_idx = static_cast<size_t>(closest_idx);

  return closest_idx != -1;
}

void ObstacleAvoidanceNode::subscribeToLaserScan()
{
    rclcpp::QoS qos(rclcpp::KeepLast(5)); // Depth: KEEP_LAST (5)
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Utwórz subskrybenta dla tematu /sensing/lidar/scan
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/sensing/lidar/scan",
        qos, // Rozmiar kolejki subskrybenta
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {

            // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
            // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
          // distances_array = msg->ranges;
          // for (int i = 0; i < msg->ranges.size(); i++)
          // {
          //   distances_array[i] = msg->ranges[i];
          // }

          // std::cout << "before calc idx" << car_pose.position.x << " " << car_pose.position.y << std::endl;
          // define scan start global angle
          // float start_angle = msg->angle_min - car_pose.orientation.z;
          float angle_increment = msg->angle_increment;
          auto obstacle_point_msg = geometry_msgs::msg::PointStamped();
          //convert quaternion to euler
          tf2::Quaternion q(car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z, car_pose.orientation.w);
          tf2::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          // size_t self_idx;
          const auto & current_pose = car_pose;  
          const auto & trajectory = trajectory_;
          // autoware_auto_planning_msgs::msg::Trajectory trajectory = trajectory_;
          
          //if car pose is not x=-0.0097019 and y=-0.580195, calculate the closest index
          if (car_pose.position.x != -0.0097019 and car_pose.position.y != -0.580195)
          {
            calcClosestIndex(trajectory, current_pose, self_idx);
          }
          // calcClosestIndex(trajectory, current_pose, self_idx);
          // current_pose = trajectory.at(self_idx+30).pose;

          std::cout << "self_idx: " << self_idx << std::endl;
          std::cout << car_pose.position.x << " " << car_pose.position.y << std::endl;
          std::cout << "predicted_self_idx: " << predicted_self_idx << std::endl;
          
          
          //trajectory points
          for (int j = self_idx; j < (self_idx+30)%270; j++)
          {
            auto temp_pose = trajectory.points.at(j).pose;
            // std::cout << "tr_point: " << j << std::endl;
            //create float vector of points (x,y)
            // std::vector<std::pair<float, float>> points;

            // laser scan points
            for (int i = 420; i < msg->ranges.size()-420; i++)
            {
              int counter = 0;
              float angle = i * angle_increment + msg->angle_min;
              // std::cout << "i: " << i << " angle: " << angle << std::endl;

              float x = msg->ranges[i] * cos(angle) + 0.35; //car 
              float y = msg->ranges[i] * sin(angle); // car
              //rotate the point to the car frame
              float x_car = x * cos(yaw) - y * sin(yaw);
              float y_car = x * sin(yaw) + y * cos(yaw);

              x = x_car + car_pose.position.x;
              y = y_car + car_pose.position.y;

              // points.push_back(std::make_pair(x, y));
              
              float distance = sqrt(pow(x - temp_pose.position.x, 2) + pow(y - temp_pose.position.y, 2));
              if (distance < 0.05)
              {
                geometry_msgs::msg::Point point;
                point.x = x;
                point.y = y;
                //add obstacle point to the vector
                obstacle_points.push_back(point);
              }

                //check least 5 points from vector obstacle_points and calculate distance between the two furthest from each other
              if (obstacle_points.size() > 5)
              {
                float max_distance = 0;
                for (int i = obstacle_points.size()-5; i < obstacle_points.size(); i++)
                {
                  for (int j = i+1; j < obstacle_points.size(); j++)
                  {
                    float distance = sqrt(pow(obstacle_points[i].x - obstacle_points[j].x, 2) + pow(obstacle_points[i].y - obstacle_points[j].y, 2));
                    if (distance > max_distance)
                    {
                      max_distance = distance;
                    }
                  }
                }
                //if the distance is smaller than 0.02m, publish the point
                if (max_distance < 0.02)
                {
                  // obstacle_point_msg.header.stamp = msg->header.stamp;
                  // obstacle_point_msg.header.frame_id = "map";
                  // obstacle_point_msg.point.x = obstacle_points[obstacle_points.size()-1].x;
                  // obstacle_point_msg.point.y = obstacle_points[obstacle_points.size()-1].y;
                  // obstacle_point_msg.point.z = 0.0;
                  // obstacle_publisher_->publish(obstacle_point_msg);



                }
              }

              
              
            }
          }
          
          
        });
}

void ObstacleAvoidanceNode::subscribeToTrajectory()
{
    trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        "/planning/racing_planner/trajectory",
        10, // Rozmiar kolejki subskrybenta
        [this](const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
            // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
            // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
          trajectory_ = *msg;
          if(false == is_waypoints_set)
          {
            for (int i = 0; i < msg->points.size(); i++)
            {
              poses_array[i] = msg->points[i].pose;
              waypoints_.push_back(msg->points[i].pose.position);
            }
            is_waypoints_set = true;
          }
        });
}

//make subscriber to /control/trajectory_follower/lateral/predicted_trajectory
void ObstacleAvoidanceNode::subscribeToPredictedTrajectory()
{
    // Utwórz subskrybenta dla tematu /control/trajectory_follower/lateral/predicted_trajectory
    predicted_trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        "/control/trajectory_follower/lateral/predicted_trajectory",
        10, // Rozmiar kolejki subskrybenta
        [this](const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
            // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
            // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
          predicted_trajectory_ = *msg;

          // auto obstacle_point_msg = geometry_msgs::msg::PointStamped();
          
          for (int i = 0; i < msg->points.size(); i++)
          {
            //tutaj jest 10 punktów, globalne współrzędne aktualnej przewidywanej trajektorii
            predicted_poses_array[i] = msg->points[i].pose; 

          }
          calcClosestIndex(trajectory_, predicted_poses_array[9], predicted_self_idx);
          // obstacle_point_msg.header.stamp = msg->header.stamp;
          // obstacle_point_msg.header.frame_id = "map";
          // obstacle_point_msg.point.x = predicted_poses_array[9].position.x;
          // obstacle_point_msg.point.y = predicted_poses_array[9].position.y;
          // obstacle_point_msg.point.z = 0.0;
          // obstacle_publisher_->publish(obstacle_point_msg);
        });
}

// void ObstacleAvoidanceNode::subscribeToCarPose()
// {
//     // Utwórz subskrybenta dla tematu /localization/cartographer/pose
//     car_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//         "/localization/cartographer/pose",
//         10, // Rozmiar kolejki subskrybenta
//         [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//             // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
//             // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
//           // car_pose = msg->pose;
//         });
// }


std::vector<double> ObstacleAvoidanceNode::sample() {
    // This method returns a sampled point from the free space

    std::vector<double> sampled_point;

   // double x = car_pose.position.x + uni_dist_(gen_)*X_SAMPLE_RANGE;
   // double y = car_pose.position.y + uni_dist_(gen_)*Y_SAMPLE_RANGE;
    std::normal_distribution<double> norm_dist_x(0.6*waypoints_.at(predicted_self_idx).x+0.4*car_pose.position.x, STD);
    std::normal_distribution<double> norm_dist_y(0.6*waypoints_.at(predicted_self_idx).y+0.4*car_pose.position.y, STD);
    double x = norm_dist_x(gen_);
    double y = norm_dist_y(gen_);

    // sample recursively until one in the free space gets returned
    if (!occupancy_grid::is_xy_occupied(occupancy_grid_,x, y)){
        sampled_point.push_back(x);
        sampled_point.push_back(y);
        return sampled_point;
    }
    else{
        return sample();
    }
}

int ObstacleAvoidanceNode::nearest(std::vector<Node_struct> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double min_dist = 100000.0;
    for (int i=0; i<int(tree.size()); i++){
        double dist = calculate_dist2(tree.at(i).x, sampled_point[0], tree.at(i).y, sampled_point[1]);
        if (dist<min_dist){
            nearest_node = i;
            min_dist = dist;
        }
    }
    return nearest_node;
}

Node_struct ObstacleAvoidanceNode::steer(Node_struct &nearest_node, std::vector<double> &sampled_point) {
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
  double dist = sqrt(calculate_dist2(nearest_node.x, sampled_point[0], nearest_node.y, sampled_point[1]));

  new_node.x = nearest_node.x + min(STEER_RANGE, dist)*(sampled_point[0]-nearest_node.x)/dist;
  new_node.y = nearest_node.y + min(STEER_RANGE, dist)*(sampled_point[1]-nearest_node.y)/dist;

  return new_node;
}

bool ObstacleAvoidanceNode::check_collision(Node_struct &nearest_node, Node_struct &new_node) {
  // This method returns a boolean indicating if the path between the
  // nearest node and the new node created from steering is collision free

  bool collision = false;
  int x_cell_diff = abs(ceil((nearest_node.x-new_node.x)/occupancy_grid_.info.resolution));
  int y_cell_diff = abs(ceil((nearest_node.y-new_node.y)/occupancy_grid_.info.resolution));

  double dt = 1.0/max(x_cell_diff, y_cell_diff);
  double t = 0.0;
  for (int i=0;i<= max(x_cell_diff, y_cell_diff); i++){
      double x = nearest_node.x + t*(new_node.x-nearest_node.x);
      double y = nearest_node.y + t*(new_node.y-nearest_node.y);
      if (occupancy_grid::is_xy_occupied(occupancy_grid_, x, y)){
          collision = true;
          break;
      }
      t+=dt;
  }
  return collision;
}

std::vector<int> ObstacleAvoidanceNode::near(std::vector<Node_struct> &tree, Node_struct& node) {
  // This method returns the set of Nodes in the neighborhood of a node.
  // Returns:
  //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood
  std::vector<int> neighborhood;
  neighborhood.clear();
  for (int i=0; i<tree.size(); i++){
      if (line_cost(tree.at(i), node) < NEAR_RANGE){
          neighborhood.push_back(i);
      }
  }
  return neighborhood;
}

double ObstacleAvoidanceNode::line_cost(Node_struct &n1, Node_struct &n2) {
  // This method returns the cost of the straight line path between two nodes

  return sqrt(calculate_dist2(n1.x, n2.x, n1.y, n2.y));
}

bool ObstacleAvoidanceNode::is_goal(Node_struct &latest_added_node, double goal_x, double goal_y) {
  // This method checks if the latest node added to the tree is close
  // enough
  return calculate_dist2(goal_x, latest_added_node.x, goal_y, latest_added_node.y) < pow(GOAL_THRESHOLD, 2);
}

std::vector<Node_struct> ObstacleAvoidanceNode::find_path(std::vector<Node_struct> &tree, Node_struct& node) {
  // This method traverses the tree from the node that has been determined
  // as goal
  std::vector<Node_struct> found_path;
  Node_struct current = node;
  while (!current.is_root){
      found_path.push_back(current);
      current = tree.at(current.parent);
  }
  found_path.push_back(current); // add start node
  reverse(found_path.begin(), found_path.end());
  return found_path;
}

bool comp_cost(Node_struct& n1, Node_struct& n2){
  return n1.cost < n2.cost;
}

void ObstacleAvoidanceNode::initialize_marker(visualization_msgs::msg::Marker &marker,
                           const std::string &frame_id,
                           const std::string &ns,
                           int32_t id,
                           int32_t type,
                           const std_msgs::msg::ColorRGBA &color,
                           double scale) {
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

void ObstacleAvoidanceNode::visualize_tree(std::vector<Node_struct>& tree){
  // Plot goal first
  // geometry_msgs::msg::Pose goal_pose;
  // goal_pose.orientation.w = 1.0;
  // goal_pose.position.x = waypoints_.at(predicted_self_idx).x;
  // goal_pose.position.y = waypoints_.at(predicted_self_idx).y;

  // visualization_msgs::msg::Marker goal_marker;
  // goal_marker.header.frame_id = "map";  // Adjust the frame_id as needed
  // goal_marker.header.stamp = this->get_clock()->now();
  // goal_marker.ns = "goal";
  // goal_marker.id = 0;
  // goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  // goal_marker.action = visualization_msgs::msg::Marker::ADD;
  // goal_marker.pose.orientation.w = 1.0;
  // goal_marker.pose.position.x = waypoints_.at(predicted_self_idx).x;
  // goal_marker.pose.position.y = waypoints_.at(predicted_self_idx).y;
  // goal_marker.pose.position.z = 0;  // Adjust the z position as needed
  // goal_marker.scale.x = 0.2;
  // goal_marker.scale.y = 0.2;
  // goal_marker.scale.z = 0.2;
  // goal_marker.color.a = 1.0;  // Don't forget to set the alpha!
  // goal_marker.color.r = 1.0;
  // goal_marker.color.g = 0.0;
  // goal_marker.color.b = 0.0;

  goal_marker.pose.orientation.w = 1.0;
  goal_marker.pose.position.x = waypoints_.at(predicted_self_idx).x;
  goal_marker.pose.position.y = waypoints_.at(predicted_self_idx).y;
  goal_marker.pose.position.z = 0.0;

  // goal_viz_publisher_->set_pose(goal_pose);
  goal_viz_publisher_->publish(goal_marker);

  // Plot tree
  tree_nodes.points.clear();
  tree_branch.points.clear();

  for (size_t i = 0; i < tree.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = tree[i].x; 
    p.y = tree[i].y;
    p.z = 0.0;  // Set z to 0 or any other value as needed
    tree_nodes.points.push_back(p);
    for (size_t j = 0; j < tree[i].children.size(); ++j) {
        tree_branch.points.push_back(p);
        geometry_msgs::msg::Point p_child;
        p_child.x = tree[tree[i].children[j]].x;
        p_child.y = tree[tree[i].children[j]].y;
        p_child.z = 0.0;  // Set z to 0 or any other value as needed
        tree_branch.points.push_back(p_child);
    }
  }

  tree_branches_pub_->publish(tree_branch);
  tree_nodes_pub_->publish(tree_nodes);
}

void ObstacleAvoidanceNode::on_timer()
{
  // Lookup transform and update car pose
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "map", "base_link", tf2::TimePointZero);


    // car_pose.position.z = transform_stamped.transform.translation.z;
    // car_pose.orientation = transform_stamped.transform.rotation;

    // RCLCPP_INFO(this->get_logger(), "Updated car pose from tf2: (%.2f, %.2f)",
    //             car_pose.position.x, car_pose.position.y);
    // std::cout << "Updated car pose from tf2: (" << car_pose.position.x << ", " << car_pose.position.y << ")" << std::endl;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
  }
  car_pose.position.x = transform_stamped.transform.translation.x;
  car_pose.position.y = transform_stamped.transform.translation.y;


  start_node.x = car_pose.position.x;
  start_node.y = car_pose.position.y;
  start_node.parent = 0;
  start_node.cost = 0.0;
  start_node.is_root = true;


  std::vector<Node_struct> tree;
  std::vector<Node_struct> nodes_near_goal;

  tree.clear();
  tree.push_back(start_node);

  for (int iter=0; iter<MAX_ITER; iter++){
      std::vector<double> sampled_point = sample();
      int nearest_ind = nearest(tree, sampled_point);
      Node_struct new_node = steer(tree.at(nearest_ind), sampled_point);
      if (!check_collision(tree.at(nearest_ind), new_node)){
          std::vector<int> nodes_near = near(tree, new_node);
          tree.push_back(new_node);

          /** connect new_node to the node in the neighborhood with the minimum cost **/
          int min_cost_node_ind = nearest_ind;
          float min_cost = tree.at(nearest_ind).cost + line_cost(tree.at(nearest_ind), new_node);
          for (int i=0; i<nodes_near.size(); i++){
              if(!check_collision(tree.at(nodes_near.at(i)), new_node)){
                  float cost = tree.at(nodes_near.at(i)).cost + line_cost(tree.at(nodes_near.at(i)), new_node);
                  if(cost < min_cost) {
                      min_cost_node_ind = nodes_near.at(i);
                      min_cost = cost;
                  }
              }
          }

          tree.back().is_root = false;
          tree.back().cost = min_cost;
          // add edge
          tree.back().parent = min_cost_node_ind;
          tree.at(min_cost_node_ind).children.push_back(tree.size()-1);

          /** Rewiring **/
          int rewire_count = 0;
          for (int j=0; j<int(nodes_near.size()); j++) {
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
                      // update_children_cost(tree, nodes_near.at(j), cost_change); // optional(expensive)
                  }
              }
              rewire_count ++;
          }
          //cout<<"rewire: "<<rewire_count<<endl;
          if (is_goal(tree.back(), waypoints_.at(predicted_self_idx).x, waypoints_.at(predicted_self_idx).y)){
              nodes_near_goal.push_back(tree.back());
          }
      }
      /** check if goal reached and recover path with the minimum cost**/
      if(iter>MIN_ITER && !nodes_near_goal.empty()){
          Node_struct best = *min_element(nodes_near_goal.begin(), nodes_near_goal.end(), comp_cost);
          std::vector<Node_struct> path_found = find_path(tree, nodes_near_goal.back());

          visualization_msgs::msg::Marker path_dots;
          path_dots.header.frame_id = "map";
          path_dots.id = 20;
          path_dots.ns = "path";
          path_dots.type = visualization_msgs::msg::Marker::POINTS;
          path_dots.scale.x = path_dots.scale.y = path_dots.scale.z = 0.08;
          path_dots.action = visualization_msgs::msg::Marker::ADD;
          path_dots.pose.orientation.w = 1.0;
          path_dots.color.g = 0.0;
          path_dots.color.r = 1.0;
          path_dots.color.a = 1.0;

          for (int i=0; i<path_found.size(); i++){
              geometry_msgs::msg::Point p;
              p.x = path_found.at(i).x;
              p.y = path_found.at(i).y;
              path_dots.points.push_back(p);
          }
          double RRT_INTERVAL = 0.2;
          std::vector<geometry_msgs::msg::Point> path_processed;
          for (int i=0; i< path_dots.points.size()-1; i++){
              path_processed.push_back(path_dots.points[i]);
              double dist = sqrt(pow(path_dots.points[i+1].x-path_dots.points[i].x, 2)
                                  +pow(path_dots.points[i+1].y-path_dots.points[i].y, 2));
              if (dist < RRT_INTERVAL) continue;
              int num = static_cast<int>(ceil(dist/RRT_INTERVAL));
              for(int j=1; j< num; j++){
                  geometry_msgs::msg::Point p;
                  p.x = path_dots.points[i].x + j*((path_dots.points[i+1].x - path_dots.points[i].x)/num);
                  p.y = path_dots.points[i].y + j*((path_dots.points[i+1].y - path_dots.points[i].y)/num);
                  path_processed.push_back(p);
              }
          }

          path_dots.points = path_processed;
          marker_publisher_->publish(path_dots);
//            track_path(path);
          visualize_tree(tree);
          //ROS_INFO("path found");
          break;
      }
  }

  if (nodes_near_goal.empty()){
      std::cout << "Couldn't find a path" << std::endl;
  }
}



}  // namespace obstacle_avoidance

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_avoidance::ObstacleAvoidanceNode)