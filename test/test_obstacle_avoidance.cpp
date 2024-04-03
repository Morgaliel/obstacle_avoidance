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

#include "gtest/gtest.h"
#include "obstacle_avoidance/obstacle_avoidance.hpp"

TEST(TestObstacleAvoidance, TestHello) {
  std::unique_ptr<obstacle_avoidance::ObstacleAvoidance> obstacle_avoidance_ =
    std::make_unique<obstacle_avoidance::ObstacleAvoidance>();
  auto result = obstacle_avoidance_->foo(999);
  EXPECT_EQ(result, 999);
}
