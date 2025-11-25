#ifndef AUTOWARE_PERCEPTION_REPLAYER__COMMON_HPP_
#define AUTOWARE_PERCEPTION_REPLAYER__COMMON_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <deque>
#include <string>
#include <vector>
#include <utility>
#include <memory>

#include "utils.hpp"

namespace autoware_perception_replayer
{

struct ReplayerOptions {
  std::string bag_path;
  bool detected_object = false;
  bool tracked_object = false;
  std::string rosbag_format = "db3";
  bool verbose = false;
  double search_radius = 1.5;
  double reproduce_cool_down = 80.0;
  bool noise = false;
};

class PerceptionReplayerCommon : public rclcpp::Node
{
public:
  PerceptionReplayerCommon(const std::string & node_name, const ReplayerOptions & options)
  : Node(node_name), options_(options)
  {
    // Publishers
    if (options_.detected_object) {
      objects_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
        "/perception/object_recognition/detection/objects", 1);
    } else if (options_.tracked_object) {
      objects_pub_ = create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
        "/perception/object_recognition/tracking/objects", 1);
    } else {
      objects_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
        "/perception/object_recognition/objects", 1);
    }

    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/perception/obstacle_segmentation/pointcloud", 1);
    
    recorded_ego_pub_as_initialpose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1);
    
    recorded_ego_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/perception_reproducer/rosbag_ego_odom", 1);
    
    goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", 1);
    
    traffic_signals_pub_ = create_publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>(
      "/perception/traffic_light_recognition/traffic_signals", 1);

    // Subscribers
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", 1,
      std::bind(&PerceptionReplayerCommon::on_odom, this, std::placeholders::_1));

    load_rosbag(options_.bag_path);
  }

protected:
  ReplayerOptions options_;
  nav_msgs::msg::Odometry::SharedPtr ego_odom_;

  // Data storage
  // Using generic shared pointers or specific types?
  // Since we need to support different object types, we might need a variant or template.
  // For now, let's store serialized messages or use a base class if possible?
  // No, they don't share a common base.
  // We can store them as `rclcpp::SerializedMessage` but deserializing every time is slow?
  // The python code stores deserialized objects.
  // In C++, we can use `std::any` or a variant, or just separate vectors.
  // Since `objects_pub_` is typed, we know the type at runtime but it's fixed for the run.
  // But we can't easily template the class member without templating the whole class.
  // Let's use a helper struct or just separate vectors and use the one that is active.
  
  using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
  using TrackedObjects = autoware_perception_msgs::msg::TrackedObjects;
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using TrafficSignals = autoware_perception_msgs::msg::TrafficLightGroupArray;
  using Odometry = nav_msgs::msg::Odometry;

  std::vector<std::pair<rclcpp::Time, DetectedObjects>> rosbag_detected_objects_data_;
  std::vector<std::pair<rclcpp::Time, TrackedObjects>> rosbag_tracked_objects_data_;
  std::vector<std::pair<rclcpp::Time, PredictedObjects>> rosbag_predicted_objects_data_;
  
  std::vector<std::pair<rclcpp::Time, TrafficSignals>> rosbag_traffic_signals_data_;
  std::vector<std::pair<rclcpp::Time, Odometry>> rosbag_ego_odom_data_;

  rclcpp::PublisherBase::SharedPtr objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr recorded_ego_pub_as_initialpose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr recorded_ego_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<TrafficSignals>::SharedPtr traffic_signals_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    ego_odom_ = msg;
  }

  void load_rosbag(const std::string & path)
  {
    RCLCPP_INFO(get_logger(), "Started loading rosbag: %s", path.c_str());
    
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = path;
    storage_options.storage_id = "sqlite3"; // Default, maybe infer?

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader;
    try {
      reader.open(storage_options, converter_options);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open rosbag: %s", e.what());
      return;
    }

    std::string objects_topic;
    if (options_.detected_object) objects_topic = "/perception/object_recognition/detection/objects";
    else if (options_.tracked_object) objects_topic = "/perception/object_recognition/tracking/objects";
    else objects_topic = "/perception/object_recognition/objects";

    std::string ego_odom_topic = "/localization/kinematic_state";
    std::string traffic_signals_topic = "/perception/traffic_light_recognition/traffic_signals";

    rosbag2_storage::StorageFilter filter;
    filter.topics = {objects_topic, ego_odom_topic, traffic_signals_topic};
    reader.set_filter(filter);

    rclcpp::Serialization<DetectedObjects> ser_detected;
    rclcpp::Serialization<TrackedObjects> ser_tracked;
    rclcpp::Serialization<PredictedObjects> ser_predicted;
    rclcpp::Serialization<TrafficSignals> ser_traffic;
    rclcpp::Serialization<Odometry> ser_odom;

    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      rclcpp::Time stamp(bag_message->time_stamp);
      
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

      if (bag_message->topic_name == objects_topic) {
        if (options_.detected_object) {
          DetectedObjects msg;
          ser_detected.deserialize_message(&serialized_msg, &msg);
          rosbag_detected_objects_data_.emplace_back(stamp, msg);
        } else if (options_.tracked_object) {
          TrackedObjects msg;
          ser_tracked.deserialize_message(&serialized_msg, &msg);
          rosbag_tracked_objects_data_.emplace_back(stamp, msg);
        } else {
          PredictedObjects msg;
          ser_predicted.deserialize_message(&serialized_msg, &msg);
          rosbag_predicted_objects_data_.emplace_back(stamp, msg);
        }
      } else if (bag_message->topic_name == traffic_signals_topic) {
        TrafficSignals msg;
        ser_traffic.deserialize_message(&serialized_msg, &msg);
        rosbag_traffic_signals_data_.emplace_back(stamp, msg);
      } else if (bag_message->topic_name == ego_odom_topic) {
        Odometry msg;
        ser_odom.deserialize_message(&serialized_msg, &msg);
        rosbag_ego_odom_data_.emplace_back(stamp, msg);
      }
    }
    RCLCPP_INFO(get_logger(), "Ended loading rosbag");
  }

  template <typename T>
  typename T::value_type::second_type find_msg_by_timestamp(const T & data, const rclcpp::Time & timestamp)
  {
    if (data.empty()) return {};

    auto it = std::lower_bound(data.begin(), data.end(), timestamp,
      [](const auto & pair, const rclcpp::Time & t) {
        return pair.first < t;
      });

    if (it == data.end()) {
      return data.back().second;
    }
    if (it == data.begin()) {
      return data.front().second;
    }

    // Check which one is closer
    auto prev = std::prev(it);
    if ((timestamp - prev->first).nanoseconds() < (it->first - timestamp).nanoseconds()) {
      return prev->second;
    }
    return it->second;
  }
  
  // Helper to get objects message regardless of type (returns void* or uses template in caller)
  // Actually, we can just have a virtual function or similar if we want to be generic, 
  // but since we only use one type per run, we can just use the specific vector.
};

} // namespace autoware_perception_replayer

#endif // AUTOWARE_PERCEPTION_REPLAYER__COMMON_HPP_
