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

#include "obstacle_avoidance/obstacle_avoidance.hpp"

namespace obstacle_avoidance
{
using ObstacleAvoidancePtr = std::unique_ptr<obstacle_avoidance::ObstacleAvoidance>;

class OBSTACLE_AVOIDANCE_PUBLIC ObstacleAvoidanceNode : public rclcpp::Node
{
public:
  explicit ObstacleAvoidanceNode(const rclcpp::NodeOptions & options);

private:
  ObstacleAvoidancePtr obstacle_avoidance_{nullptr};
  int64_t param_name_{123};
};
}  // namespace obstacle_avoidance

#endif  // OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_NODE_HPP_
