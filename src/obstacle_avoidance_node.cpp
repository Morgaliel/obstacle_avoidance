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

ObstacleAvoidanceNode::ObstacleAvoidanceNode(const rclcpp::NodeOptions & options)
:  Node("obstacle_avoidance", options)
{
  obstacle_avoidance_ = std::make_unique<obstacle_avoidance::ObstacleAvoidance>();
  param_name_ = this->declare_parameter("param_name", 456);
  obstacle_avoidance_->foo(param_name_);

  obstacle_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/obstacle_point", 100);
  
  // Subskrybuj temat /sensing/lidar/scan
  subscribeToCarPose();
  subscribeToLaserScan();
  subscribeToTrajectory(); // 10 punktów, globalne współrzędne 
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
    const double dist = calcDist2d(getPose(traj, i).position, pose.position);

    /* check distance threshold */
    if (dist > dist_thr) {
      continue;
    }

    /* check angle threshold */
    double yaw_i = tf2::getYaw(getPose(traj, i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_i);

    if (std::fabs(yaw_diff) > angle_thr) {
      continue;
    }

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

          // define scan start global angle
          // float start_angle = msg->angle_min - car_pose.orientation.z;
          float angle_increment = msg->angle_increment;
          auto obstacle_point_msg = geometry_msgs::msg::PointStamped();
          //convert quaternion to euler
          tf2::Quaternion q(car_pose.orientation.x, car_pose.orientation.y, car_pose.orientation.z, car_pose.orientation.w);
          tf2::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);

          

          for (int i = 0; i < msg->ranges.size(); i++)
          {
            float angle = i * angle_increment + msg->angle_min;

            
            float x = msg->ranges[i] * cos(angle) + 0.35; //car 
            float y = msg->ranges[i] * sin(angle); // car



            //rotate the point to the car frame
            float x_car = x * cos(yaw) - y * sin(yaw);
            float y_car = x * sin(yaw) + y * cos(yaw);

            x = x_car + car_pose.position.x;
            y = y_car + car_pose.position.y;

            // std::cout << "car orientation: " << car_pose.orientation.z << std::endl;
            // std::cout << "yaw: " << yaw << std::endl;
            // std::cout << "angle: " << angle << std::endl;
            // std::cout << "range: " << msg->ranges[i] << std::endl;
            // std::cout << "x_car: " << x_car << " y_car: " << y_car << std::endl;
            // std::cout << "car x: " << car_pose.position.x << " car y: " << car_pose.position.y << std::endl;
            // std::cout << "x: " << x << " y: " << y << std::endl;
            // std::cout << "----------------" << std::endl;

            // check if point is in the trajectory
            for (int j = 0; j < poses_array.size(); j++)
            {
              float distance = sqrt(pow(x - poses_array[j].position.x, 2) + pow(y - poses_array[j].position.y, 2));
              if (distance < 0.05)
              {
                // obstacle detected
                // publish stop signal
                // stop the car
                // obstacle_point_msg.header.frame_id = "obstacle_avoidance";
                obstacle_point_msg.header.stamp = msg->header.stamp;
                obstacle_point_msg.header.frame_id = "map";
                obstacle_point_msg.point.x = x;
                obstacle_point_msg.point.y = y;
                obstacle_point_msg.point.z = 0.0;

                // std::cout << "Obstacle detected at x: " << x << " y: " << y << std::endl;
                obstacle_publisher_->publish(obstacle_point_msg);

              }
            }
          }
          
          
        });
}

void ObstacleAvoidanceNode::subscribeToTrajectory()
{
    // Utwórz subskrybenta dla tematu /sensing/lidar/scan
    trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        "/planning/racing_planner/trajectory",
        10, // Rozmiar kolejki subskrybenta
        [this](const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
            // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
            // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
          // trajectory_ = msg;
          size_t self_idx;
          const auto & current_pose = car_pose;  
          const auto & trajectory = *msg;
          // autoware_auto_planning_msgs::msg::Trajectory trajectory = trajectory_;
          
          calcClosestIndex(trajectory, current_pose, self_idx);
          current_pose = trajectory.at(self_idx+30).pose;

          std::cout << "self_idx: " << self_idx << std::endl;

          for (int i = 0; i < msg->points.size(); i++)
          {
            poses_array[i] = msg->points[i].pose;
          }
        });
}

void ObstacleAvoidanceNode::subscribeToCarPose()
{
    // Utwórz subskrybenta dla tematu /localization/cartographer/pose
    car_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization/cartographer/pose",
        10, // Rozmiar kolejki subskrybenta
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
            // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
          car_pose = msg->pose;
        });
}




}  // namespace obstacle_avoidance

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_avoidance::ObstacleAvoidanceNode)