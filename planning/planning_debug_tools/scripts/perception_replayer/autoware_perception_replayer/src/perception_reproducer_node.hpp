#ifndef AUTOWARE_PERCEPTION_REPLAYER__PERCEPTION_REPRODUCER_NODE_HPP_
#define AUTOWARE_PERCEPTION_REPLAYER__PERCEPTION_REPRODUCER_NODE_HPP_

#include "common.hpp"
#include <deque>
#include <algorithm>

namespace autoware_perception_replayer
{

class PerceptionReproducerNode : public PerceptionReplayerCommon
{
public:
  PerceptionReproducerNode(const rclcpp::NodeOptions & options);

private:
  void on_timer();
  void on_timer_kill_perception();
  void kill_online_perception_node();
  
  // Helper functions
  size_t find_nearest_ego_odom_index(const geometry_msgs::msg::Pose & ego_pose);
  std::vector<size_t> find_nearby_ego_odom_indices(const std::vector<geometry_msgs::msg::Pose> & ego_poses, double search_radius);
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_check_perception_process_;
  
  double ego_odom_search_radius_;
  double rosbag_ego_odom_search_radius_;
  double reproduce_cool_down_;
  
  std::deque<size_t> reproduce_sequence_indices_;
  std::deque<size_t> cool_down_indices_;
  std::map<size_t, rclcpp::Time> ego_odom_id2last_published_timestamp_;
  
  geometry_msgs::msg::Pose last_sequenced_ego_pose_;
  bool has_last_sequenced_ego_pose_ = false;
  
  // Previous messages to handle gaps
  DetectedObjects prev_detected_objects_msg_;
  TrackedObjects prev_tracked_objects_msg_;
  PredictedObjects prev_predicted_objects_msg_;
  TrafficSignals prev_traffic_signals_msg_;
  Odometry::SharedPtr prev_ego_odom_msg_;
};

} // namespace autoware_perception_replayer

#endif // AUTOWARE_PERCEPTION_REPLAYER__PERCEPTION_REPRODUCER_NODE_HPP_
