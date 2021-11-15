// Copyright 2021 Factor Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TOF_SENSE_ROS2__VISIBILITY_CONTROL_HPP_
#define TOF_SENSE_ROS2__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TOF_SENSE_ROS2_EXPORT __attribute__((dllexport))
#define TOF_SENSE_ROS2_IMPORT __attribute__((dllimport))
#else
#define TOF_SENSE_ROS2_EXPORT __declspec(dllexport)
#define TOF_SENSE_ROS2_IMPORT __declspec(dllimport)
#endif
#ifdef TOF_SENSE_ROS2_BUILDING_LIBRARY
#define TOF_SENSE_ROS2_PUBLIC TOF_SENSE_ROS2_EXPORT
#else
#define TOF_SENSE_ROS2_PUBLIC TOF_SENSE_ROS2_IMPORT
#endif
#define TOF_SENSE_ROS2_PUBLIC_TYPE TOF_SENSE_ROS2_PUBLIC
#define TOF_SENSE_ROS2_LOCAL
#else
#define TOF_SENSE_ROS2_EXPORT __attribute__((visibility("default")))
#define TOF_SENSE_ROS2_IMPORT
#if __GNUC__ >= 4
#define TOF_SENSE_ROS2_PUBLIC __attribute__((visibility("default")))
#define TOF_SENSE_ROS2_LOCAL __attribute__((visibility("hidden")))
#else
#define TOF_SENSE_ROS2_PUBLIC
#define TOF_SENSE_ROS2_LOCAL
#endif
#define TOF_SENSE_ROS2_PUBLIC_TYPE
#endif

#endif  // TOF_SENSE_ROS2__VISIBILITY_CONTROL_HPP_
