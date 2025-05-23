// Copyright 2022 Tier IV, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/route_handler/route_handler.hpp"
#include "type_alias.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::static_centerline_generator
{
namespace utils
{
geometry_msgs::msg::Pose create_pose(
  const double px, const double py, const double pz, const double qx, const double qy,
  const double qz, const double qw);

nav_msgs::msg::Odometry::ConstSharedPtr convert_to_odometry(const geometry_msgs::msg::Pose & pose);

rclcpp::QoS create_transient_local_qos();

lanelet::ConstLanelets get_lanelets_from_ids(
  const RouteHandler & route_handler, const std::vector<lanelet::Id> & lane_ids);

lanelet::ConstLanelets get_lanelets_from_route(
  const RouteHandler & route_handler, const LaneletRoute & route);

geometry_msgs::msg::Pose get_center_pose(
  const RouteHandler & route_handler, const lanelet::Id lanelet_id);

PathWithLaneId get_path_with_lane_id(
  const RouteHandler & route_handler, const lanelet::ConstLanelets lanelets,
  const geometry_msgs::msg::Pose & start_pose, const double ego_nearest_dist_threshold,
  const double ego_nearest_yaw_threshold);

void update_centerline(
  lanelet::LaneletMapPtr lanelet_map_ptr, const std::vector<TrajectoryPoint> & new_centerline,
  const std::vector<lanelet::Id> & centerline_lane_ids);

Marker create_footprint_marker(
  const std::string & ns, const LinearRing2d & footprint_poly, const double width, const double r,
  const double g, const double b, const double a, const rclcpp::Time & now, const size_t idx);

Marker create_text_marker(
  const std::string & ns, const geometry_msgs::msg::Pose & pose, const double value, const double r,
  const double g, const double b, const double a, const rclcpp::Time & now, const size_t idx);

void create_points_marker(
  MarkerArray & marker_array, const std::string & ns,
  const std::vector<std::vector<geometry_msgs::msg::Point>> & points_vec, const double width,
  const rclcpp::Time & now);

MarkerArray create_delete_all_marker_array(
  const std::vector<std::string> & ns_vec, const rclcpp::Time & now);

}  // namespace utils
}  // namespace autoware::static_centerline_generator

#endif  // UTILS_HPP_
