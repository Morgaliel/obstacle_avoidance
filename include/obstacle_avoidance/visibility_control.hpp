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

#ifndef OBSTACLE_AVOIDANCE__VISIBILITY_CONTROL_HPP_
#define OBSTACLE_AVOIDANCE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(OBSTACLE_AVOIDANCE_BUILDING_DLL) || defined(OBSTACLE_AVOIDANCE_EXPORTS)
    #define OBSTACLE_AVOIDANCE_PUBLIC __declspec(dllexport)
    #define OBSTACLE_AVOIDANCE_LOCAL
  #else  // defined(OBSTACLE_AVOIDANCE_BUILDING_DLL) || defined(OBSTACLE_AVOIDANCE_EXPORTS)
    #define OBSTACLE_AVOIDANCE_PUBLIC __declspec(dllimport)
    #define OBSTACLE_AVOIDANCE_LOCAL
  #endif  // defined(OBSTACLE_AVOIDANCE_BUILDING_DLL) || defined(OBSTACLE_AVOIDANCE_EXPORTS)
#elif defined(__linux__)
  #define OBSTACLE_AVOIDANCE_PUBLIC __attribute__((visibility("default")))
  #define OBSTACLE_AVOIDANCE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define OBSTACLE_AVOIDANCE_PUBLIC __attribute__((visibility("default")))
  #define OBSTACLE_AVOIDANCE_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // OBSTACLE_AVOIDANCE__VISIBILITY_CONTROL_HPP_
