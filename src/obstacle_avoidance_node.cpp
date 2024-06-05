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
:  Node("obstacle_avoidance", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  obstacle_avoidance_ = std::make_unique<obstacle_avoidance::ObstacleAvoidance>();
  param_name_ = this->declare_parameter("param_name", 456);
  obstacle_avoidance_->foo(param_name_);

  obstacle_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/obstacle_point", 100);
  
  // Subskrybuj temat /sensing/lidar/scan
  // subscribeToCarPose();
  subscribeToLaserScan();
  subscribeToTrajectory(); // 10 punktów, globalne współrzędne 

  // Initialize tf2 listener timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ObstacleAvoidanceNode::on_timer, this));
  // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

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

          std::cout << "before calc idx" << car_pose.position.x << " " << car_pose.position.y << std::endl;
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
                  obstacle_point_msg.header.stamp = msg->header.stamp;
                  obstacle_point_msg.header.frame_id = "map";
                  obstacle_point_msg.point.x = obstacle_points[obstacle_points.size()-1].x;
                  obstacle_point_msg.point.y = obstacle_points[obstacle_points.size()-1].y;
                  obstacle_point_msg.point.z = 0.0;
                  obstacle_publisher_->publish(obstacle_point_msg);
                }
              }

                // obstacle_point_msg.header.stamp = msg->header.stamp;
                // obstacle_point_msg.header.frame_id = "map";
                // obstacle_point_msg.point.x = x;
                // obstacle_point_msg.point.y = y;
                // obstacle_point_msg.point.z = 0.0;
                // obstacle_publisher_->publish(obstacle_point_msg);
              
              
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
          trajectory_ = *msg;
          
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
          // car_pose = msg->pose;
        });
}

void ObstacleAvoidanceNode::on_timer()
{
  // Lookup transform and update car pose
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      "map", "base_link", tf2::TimePointZero);

    car_pose.position.x = transform_stamped.transform.translation.x;
    car_pose.position.y = transform_stamped.transform.translation.y;
    // car_pose.position.z = transform_stamped.transform.translation.z;
    // car_pose.orientation = transform_stamped.transform.rotation;

    // RCLCPP_INFO(this->get_logger(), "Updated car pose from tf2: (%.2f, %.2f)",
    //             car_pose.position.x, car_pose.position.y);
    // std::cout << "Updated car pose from tf2: (" << car_pose.position.x << ", " << car_pose.position.y << ")" << std::endl;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
  }
}


}  // namespace obstacle_avoidance

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_avoidance::ObstacleAvoidanceNode)