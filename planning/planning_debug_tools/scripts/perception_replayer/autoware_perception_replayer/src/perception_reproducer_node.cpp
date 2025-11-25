#include "perception_reproducer_node.hpp"
#include <cstdlib>
#include <signal.h>

namespace autoware_perception_replayer
{

PerceptionReproducerNode::PerceptionReproducerNode(const rclcpp::NodeOptions & options)
: PerceptionReplayerCommon("perception_reproducer", ReplayerOptions())
{
  // Parse parameters
  options_.bag_path = declare_parameter<std::string>("bag_path", "");
  options_.detected_object = declare_parameter<bool>("detected_object", false);
  options_.tracked_object = declare_parameter<bool>("tracked_object", false);
  options_.rosbag_format = declare_parameter<std::string>("rosbag_format", "db3");
  options_.verbose = declare_parameter<bool>("verbose", false);
  options_.search_radius = declare_parameter<double>("search_radius", 1.5);
  options_.reproduce_cool_down = declare_parameter<double>("reproduce_cool_down", 80.0);
  options_.noise = declare_parameter<bool>("noise", true);

  rosbag_ego_odom_search_radius_ = options_.search_radius;
  ego_odom_search_radius_ = rosbag_ego_odom_search_radius_;
  reproduce_cool_down_ = (options_.search_radius != 0.0) ? options_.reproduce_cool_down : 0.0;

  // Re-initialize base class with parsed options (a bit hacky but needed since base ctor runs first)
  // Actually, we should have parsed params in base or passed them.
  // Since we already constructed base with empty options, we need to re-run load_rosbag and re-create publishers if options changed.
  // But publishers are created in base ctor.
  // Better approach: Parse params in a static helper or in the derived class before calling base ctor?
  // No, we can't.
  // Let's just re-create them or move the init logic to a separate init() method.
  // For now, I will just manually re-do the setup here since it's cleaner than refactoring the whole base class structure.
  
  // Re-create publishers based on params
  if (options_.detected_object) {
    objects_pub_ = create_publisher<DetectedObjects>("/perception/object_recognition/detection/objects", 1);
  } else if (options_.tracked_object) {
    objects_pub_ = create_publisher<TrackedObjects>("/perception/object_recognition/tracking/objects", 1);
  } else {
    objects_pub_ = create_publisher<PredictedObjects>("/perception/object_recognition/objects", 1);
  }
  
  // Reload rosbag
  rosbag_detected_objects_data_.clear();
  rosbag_tracked_objects_data_.clear();
  rosbag_predicted_objects_data_.clear();
  rosbag_traffic_signals_data_.clear();
  rosbag_ego_odom_data_.clear();
  load_rosbag(options_.bag_path);

  if (rosbag_ego_odom_data_.empty()) {
    RCLCPP_ERROR(get_logger(), "No ego odom data found in rosbag!");
    return;
  }

  // Calculate average interval
  double total_duration = 0;
  if (rosbag_ego_odom_data_.size() > 1) {
    for (size_t i = 1; i < rosbag_ego_odom_data_.size(); ++i) {
      total_duration += (rosbag_ego_odom_data_[i].first - rosbag_ego_odom_data_[i-1].first).seconds();
    }
    double avg_interval = total_duration / (rosbag_ego_odom_data_.size() - 1);
    timer_ = create_wall_timer(std::chrono::duration<double>(avg_interval), std::bind(&PerceptionReproducerNode::on_timer, this));
  } else {
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PerceptionReproducerNode::on_timer, this));
  }

  timer_check_perception_process_ = create_wall_timer(
    std::chrono::seconds(3), std::bind(&PerceptionReproducerNode::on_timer_kill_perception, this));

  RCLCPP_INFO(get_logger(), "Perception Reproducer initialized");
}

void PerceptionReproducerNode::on_timer_kill_perception()
{
  kill_online_perception_node();
}

void PerceptionReproducerNode::kill_online_perception_node()
{
  std::string kill_process_name;
  if (options_.detected_object) {
    kill_process_name = "dummy_perception_publisher_node";
  } else if (options_.tracked_object) {
    kill_process_name = "multi_object_tracker";
  } else {
    kill_process_name = "map_based_prediction";
  }

  // Using system command to kill process - similar to python implementation
  // Note: This is platform specific (Linux)
  std::string cmd = "pidof " + kill_process_name;
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) return;
  
  char buffer[128];
  std::string result = "";
  while (!feof(pipe)) {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }
  pclose(pipe);

  if (!result.empty()) {
    int pid = std::atoi(result.c_str());
    if (pid > 0) {
      kill(pid, SIGTERM);
    }
  }
}

void PerceptionReproducerNode::on_timer()
{
  auto timestamp = now();
  
  // Publish empty pointcloud if detected object mode
  if (options_.detected_object) {
    auto pc_msg = create_empty_pointcloud(timestamp);
    pointcloud_pub_->publish(pc_msg);
  }

  if (!ego_odom_) {
    // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No ego odom received yet.");
    return;
  }

  auto ego_pose = ego_odom_->pose.pose;
  
  double dist_moved = 999.0;
  if (has_last_sequenced_ego_pose_) {
    dist_moved = std::hypot(
      ego_pose.position.x - last_sequenced_ego_pose_.position.x,
      ego_pose.position.y - last_sequenced_ego_pose_.position.y);
  }

  if (dist_moved > ego_odom_search_radius_) {
    last_sequenced_ego_pose_ = ego_pose;
    has_last_sequenced_ego_pose_ = true;

    // Find nearby ego odoms
    std::vector<geometry_msgs::msg::Pose> ego_poses = {ego_pose};
    auto nearby_indices = find_nearby_ego_odom_indices(ego_poses, ego_odom_search_radius_);
    
    std::vector<geometry_msgs::msg::Pose> nearby_poses;
    for (auto idx : nearby_indices) {
      nearby_poses.push_back(rosbag_ego_odom_data_[idx].second.pose.pose);
    }

    if (nearby_poses.empty()) {
      size_t nearest_idx = find_nearest_ego_odom_index(ego_pose);
      nearby_poses.push_back(rosbag_ego_odom_data_[nearest_idx].second.pose.pose);
    }

    auto ego_odom_indices = find_nearby_ego_odom_indices(nearby_poses, rosbag_ego_odom_search_radius_);

    // Update cool down
    while (!cool_down_indices_.empty()) {
      size_t idx = cool_down_indices_.front();
      if (ego_odom_id2last_published_timestamp_.count(idx)) {
        double elapsed = (timestamp - ego_odom_id2last_published_timestamp_[idx]).seconds();
        if (elapsed > reproduce_cool_down_) {
          cool_down_indices_.pop_front();
        } else {
          break;
        }
      } else {
        cool_down_indices_.pop_front();
      }
    }

    // Filter out cool down indices
    std::vector<size_t> filtered_indices;
    for (auto idx : ego_odom_indices) {
      bool in_cool_down = false;
      for (auto cd_idx : cool_down_indices_) {
        if (idx == cd_idx) {
          in_cool_down = true;
          break;
        }
      }
      if (!in_cool_down) {
        filtered_indices.push_back(idx);
      }
    }
    std::sort(filtered_indices.begin(), filtered_indices.end());
    
    reproduce_sequence_indices_.clear();
    for (auto idx : filtered_indices) {
      reproduce_sequence_indices_.push_back(idx);
    }
  }

  bool repeat_flag = reproduce_sequence_indices_.empty();

  // Speed check logic
  if (!repeat_flag) {
    double ego_speed = std::hypot(ego_odom_->twist.twist.linear.x, ego_odom_->twist.twist.linear.y);
    size_t ego_odom_idx = reproduce_sequence_indices_.front();
    auto & rosbag_odom = rosbag_ego_odom_data_[ego_odom_idx].second;
    double rosbag_speed = std::hypot(rosbag_odom.twist.twist.linear.x, rosbag_odom.twist.twist.linear.y);
    
    double dist = std::hypot(
      ego_pose.position.x - rosbag_odom.pose.pose.position.x,
      ego_pose.position.y - rosbag_odom.pose.pose.position.y);
      
    if (rosbag_speed > ego_speed * 2.0 && rosbag_speed > 3.0 && dist > ego_odom_search_radius_) {
      repeat_flag = true;
    }
  }

  Odometry::SharedPtr current_ego_odom_msg;
  
  // Objects and Traffic Signals pointers (using void* or std::any would be generic, but we know types)
  // We need to handle different types.
  // Let's use local variables.
  DetectedObjects current_detected_objects;
  TrackedObjects current_tracked_objects;
  PredictedObjects current_predicted_objects;
  bool has_objects = false;
  
  TrafficSignals current_traffic_signals;
  bool has_traffic_signals = false;

  if (!repeat_flag) {
    size_t ego_odom_idx = reproduce_sequence_indices_.front();
    reproduce_sequence_indices_.pop_front();
    
    rclcpp::Time pose_timestamp = rosbag_ego_odom_data_[ego_odom_idx].first;
    current_ego_odom_msg = std::make_shared<Odometry>(rosbag_ego_odom_data_[ego_odom_idx].second);
    
    // Find objects
    if (options_.detected_object) {
      current_detected_objects = find_msg_by_timestamp(rosbag_detected_objects_data_, pose_timestamp);
      has_objects = true;
    } else if (options_.tracked_object) {
      current_tracked_objects = find_msg_by_timestamp(rosbag_tracked_objects_data_, pose_timestamp);
      has_objects = true;
    } else {
      current_predicted_objects = find_msg_by_timestamp(rosbag_predicted_objects_data_, pose_timestamp);
      has_objects = true;
    }
    
    // Find traffic signals
    current_traffic_signals = find_msg_by_timestamp(rosbag_traffic_signals_data_, pose_timestamp);
    has_traffic_signals = true; // Assuming we always get something or empty

    ego_odom_id2last_published_timestamp_[ego_odom_idx] = timestamp;
    cool_down_indices_.push_back(ego_odom_idx);
  } else {
    current_ego_odom_msg = prev_ego_odom_msg_;
    if (options_.detected_object) current_detected_objects = prev_detected_objects_msg_;
    else if (options_.tracked_object) current_tracked_objects = prev_tracked_objects_msg_;
    else current_predicted_objects = prev_predicted_objects_msg_;
    has_objects = true;
    
    current_traffic_signals = prev_traffic_signals_msg_;
    has_traffic_signals = true;
  }

  // Publish Ego Odom
  if (current_ego_odom_msg) {
    prev_ego_odom_msg_ = current_ego_odom_msg;
    recorded_ego_pub_->publish(*current_ego_odom_msg);
  }

  // Publish Objects
  if (has_objects) {
    if (options_.detected_object) {
      prev_detected_objects_msg_ = current_detected_objects;
      current_detected_objects.header.stamp = timestamp;
      // Noise logic could be added here
      translate_objects_coordinate(ego_pose, current_ego_odom_msg->pose.pose, current_detected_objects);
      std::static_pointer_cast<rclcpp::Publisher<DetectedObjects>>(objects_pub_)->publish(current_detected_objects);
    } else if (options_.tracked_object) {
      prev_tracked_objects_msg_ = current_tracked_objects;
      current_tracked_objects.header.stamp = timestamp;
      std::static_pointer_cast<rclcpp::Publisher<TrackedObjects>>(objects_pub_)->publish(current_tracked_objects);
    } else {
      prev_predicted_objects_msg_ = current_predicted_objects;
      current_predicted_objects.header.stamp = timestamp;
      std::static_pointer_cast<rclcpp::Publisher<PredictedObjects>>(objects_pub_)->publish(current_predicted_objects);
    }
  }

  // Publish Traffic Signals
  if (has_traffic_signals) {
    prev_traffic_signals_msg_ = current_traffic_signals;
    current_traffic_signals.stamp = timestamp;
    traffic_signals_pub_->publish(current_traffic_signals);
  }
}

size_t PerceptionReproducerNode::find_nearest_ego_odom_index(const geometry_msgs::msg::Pose & ego_pose)
{
  double min_dist_sq = std::numeric_limits<double>::max();
  size_t min_idx = 0;
  
  for (size_t i = 0; i < rosbag_ego_odom_data_.size(); ++i) {
    double dx = rosbag_ego_odom_data_[i].second.pose.pose.position.x - ego_pose.position.x;
    double dy = rosbag_ego_odom_data_[i].second.pose.pose.position.y - ego_pose.position.y;
    double dist_sq = dx*dx + dy*dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      min_idx = i;
    }
  }
  return min_idx;
}

std::vector<size_t> PerceptionReproducerNode::find_nearby_ego_odom_indices(
  const std::vector<geometry_msgs::msg::Pose> & ego_poses, double search_radius)
{
  std::vector<size_t> indices;
  double radius_sq = search_radius * search_radius;
  
  for (size_t i = 0; i < rosbag_ego_odom_data_.size(); ++i) {
    const auto & p = rosbag_ego_odom_data_[i].second.pose.pose;
    for (const auto & target : ego_poses) {
      double dx = p.position.x - target.position.x;
      double dy = p.position.y - target.position.y;
      if (dx*dx + dy*dy <= radius_sq) {
        indices.push_back(i);
        break; // Found match for this odom, no need to check other targets
      }
    }
  }
  return indices;
}

} // namespace autoware_perception_replayer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_perception_replayer::PerceptionReproducerNode)
