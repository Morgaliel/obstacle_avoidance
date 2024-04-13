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

// #include <tier4_api_utils/tier4_api_utils.hpp>
// #include <tier4_external_api_msgs/srv/set_emergency.hpp>

namespace obstacle_avoidance
{

ObstacleAvoidanceNode::ObstacleAvoidanceNode(const rclcpp::NodeOptions & options)
:  Node("obstacle_avoidance", options)
{
  obstacle_avoidance_ = std::make_unique<obstacle_avoidance::ObstacleAvoidance>();
  param_name_ = this->declare_parameter("param_name", 456);
  obstacle_avoidance_->foo(param_name_);


  
  // Subskrybuj temat /sensing/lidar/scan
  subscribeToCarPose();
  subscribeToLaserScan();
  subscribeToTrajectory(); // 10 punktów, globalne współrzędne
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
          float start_angle = msg->angle_min - car_pose.orientation.z;
          float angle_increment = msg->angle_increment;

          for (int i = 0; i < msg->ranges.size(); i++)
          {
            float angle = start_angle + i * angle_increment;
            float x = msg->ranges[i] * cos(angle); //global
            float y = msg->ranges[i] * sin(angle); //global
            std::cout << "im in" << std::endl;
            // check if point is in the trajectory
            for (int j = 0; j < poses_array.size(); j++)
            {
              float distance = sqrt(pow(x - poses_array[j].position.x, 2) + pow(y - poses_array[j].position.y, 2));
              if (distance < 0.5)
              {
                // obstacle detected
                // publish stop signal
                // stop the car
                std::cout << "obstacle detected" << std::endl;
                // bool emergency = true;

                // RCLCPP_INFO(get_logger(), "%s emergency stop", emergency ? "Set" : "Clear");

                // auto request = std::make_shared<tier4_external_api_msgs::srv::SetEmergency::Request>();
                // request->emergency = emergency;

                // client_emergency_stop_->async_send_request(
                //   request, [this, emergency](
                //             rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedFuture result) {
                //     auto response = result.get();
                //     if (tier4_api_utils::is_success(response->status)) {
                //       RCLCPP_INFO(get_logger(), "service succeeded");
                //     } else {
                //       RCLCPP_WARN(get_logger(), "service failed: %s", response->status.message.c_str());
                //     }
                //   });
              }
            }
          }

        });
}

void ObstacleAvoidanceNode::subscribeToTrajectory()
{
    // Utwórz subskrybenta dla tematu /sensing/lidar/scan
    trajectory_subscriber_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        "/control/trajectory_follower/lateral/predicted_trajectory",
        10, // Rozmiar kolejki subskrybenta
        [this](const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
            // Funkcja zwrotna wywoływana przy otrzymaniu nowej wiadomości
            // Możesz umieścić tutaj logikę przetwarzania wiadomości LaserScan
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