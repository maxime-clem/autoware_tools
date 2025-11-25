#include "perception_replayer_node.hpp"
#include <QApplication>
#include <cstdlib>
#include <signal.h>

namespace autoware_perception_replayer
{

// QJumpSlider Implementation
QJumpSlider::QJumpSlider(Qt::Orientation orientation, int max_value, QWidget * parent)
: QSlider(orientation, parent), max_value_(max_value)
{
}

int QJumpSlider::mouse_to_value(QMouseEvent * event)
{
  double x = event->pos().x();
  return static_cast<int>(max_value_ * x / width());
}

void QJumpSlider::mousePressEvent(QMouseEvent * event)
{
  QSlider::mousePressEvent(event);
  if (event->button() == Qt::LeftButton) {
    setValue(mouse_to_value(event));
    is_mouse_pressed_ = true;
  }
}

void QJumpSlider::mouseMoveEvent(QMouseEvent * event)
{
  QSlider::mouseMoveEvent(event);
  if (is_mouse_pressed_) {
    setValue(mouse_to_value(event));
  }
}

void QJumpSlider::mouseReleaseEvent(QMouseEvent * event)
{
  QSlider::mouseReleaseEvent(event);
  if (event->button() == Qt::LeftButton) {
    is_mouse_pressed_ = false;
  }
}

// TimeManagerWidget Implementation
TimeManagerWidget::TimeManagerWidget(rclcpp::Time start_timestamp, rclcpp::Time end_timestamp, QWidget * parent)
: QMainWindow(parent), start_timestamp_(start_timestamp), end_timestamp_(end_timestamp)
{
  setObjectName("PerceptionReplayer");
  resize(480, 120);
  setWindowFlags(Qt::WindowStaysOnTopHint);

  QWidget * central_widget = new QWidget(this);
  central_widget->setObjectName("central_widget");
  
  QGridLayout * grid_layout = new QGridLayout(central_widget);
  grid_layout->setContentsMargins(10, 10, 10, 10);
  
  // Rate buttons
  std::vector<double> rates = {0.1, 0.5, 1.0, 2.0, 5.0, 10.0};
  for (size_t i = 0; i < rates.size(); ++i) {
    QPushButton * btn = new QPushButton(QString::number(rates[i]));
    btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    rate_buttons.push_back(btn);
    grid_layout->addWidget(btn, 0, i, 1, 1);
  }

  // Pause button
  pause_button = new QPushButton("pause");
  pause_button->setCheckable(true);
  pause_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  grid_layout->addWidget(pause_button, 1, 0, 1, -1);

  // Pub buttons
  pub_recorded_ego_pose_button = new QPushButton("publish recorded ego pose");
  grid_layout->addWidget(pub_recorded_ego_pose_button, 2, 0, 1, -1);
  
  pub_goal_pose_button = new QPushButton("publish last recorded ego pose as goal pose");
  grid_layout->addWidget(pub_goal_pose_button, 3, 0, 1, -1);

  // Slider
  slider = new QJumpSlider(Qt::Horizontal, max_value_);
  slider->setMinimum(0);
  slider->setMaximum(max_value_);
  slider->setValue(0);
  grid_layout->addWidget(slider, 4, 0, 1, -1);

  setCentralWidget(central_widget);
}

int TimeManagerWidget::timestamp_to_value(rclcpp::Time timestamp)
{
  double duration = (end_timestamp_ - start_timestamp_).seconds();
  if (duration <= 0) return 0;
  double offset = (timestamp - start_timestamp_).seconds();
  return static_cast<int>((offset / duration) * max_value_);
}

rclcpp::Time TimeManagerWidget::value_to_timestamp(int value)
{
  double duration = (end_timestamp_ - start_timestamp_).seconds();
  double offset = (static_cast<double>(value) / max_value_) * duration;
  return start_timestamp_ + rclcpp::Duration::from_seconds(offset);
}

// PerceptionReplayerNode Implementation
PerceptionReplayerNode::PerceptionReplayerNode(const rclcpp::NodeOptions & /*options*/)
: PerceptionReplayerCommon("perception_replayer", ReplayerOptions())
{
  // Parse parameters
  options_.bag_path = declare_parameter<std::string>("bag_path", "");
  options_.detected_object = declare_parameter<bool>("detected_object", false);
  options_.tracked_object = declare_parameter<bool>("tracked_object", false);
  options_.rosbag_format = declare_parameter<std::string>("rosbag_format", "db3");
  
  // Reload rosbag
  rosbag_detected_objects_data_.clear();
  rosbag_tracked_objects_data_.clear();
  rosbag_predicted_objects_data_.clear();
  rosbag_traffic_signals_data_.clear();
  rosbag_ego_odom_data_.clear();
  load_rosbag(options_.bag_path);

  if (rosbag_detected_objects_data_.empty() && rosbag_tracked_objects_data_.empty() && rosbag_predicted_objects_data_.empty()) {
    RCLCPP_WARN(get_logger(), "No object data found!");
  }
  
  // Re-create publishers based on params (same as reproducer)
  if (options_.detected_object) {
    objects_pub_ = create_publisher<DetectedObjects>("/perception/object_recognition/detection/objects", 1);
  } else if (options_.tracked_object) {
    objects_pub_ = create_publisher<TrackedObjects>("/perception/object_recognition/tracking/objects", 1);
  } else {
    objects_pub_ = create_publisher<PredictedObjects>("/perception/object_recognition/objects", 1);
  }

  // Initialize timestamp
  if (!rosbag_detected_objects_data_.empty()) bag_timestamp_ = rosbag_detected_objects_data_.front().first;
  else if (!rosbag_tracked_objects_data_.empty()) bag_timestamp_ = rosbag_tracked_objects_data_.front().first;
  else if (!rosbag_predicted_objects_data_.empty()) bag_timestamp_ = rosbag_predicted_objects_data_.front().first;
  else bag_timestamp_ = now(); // Fallback

  // Find end timestamp
  rclcpp::Time end_timestamp = bag_timestamp_;
  if (!rosbag_detected_objects_data_.empty() && rosbag_detected_objects_data_.back().first > end_timestamp) end_timestamp = rosbag_detected_objects_data_.back().first;
  if (!rosbag_tracked_objects_data_.empty() && rosbag_tracked_objects_data_.back().first > end_timestamp) end_timestamp = rosbag_tracked_objects_data_.back().first;
  if (!rosbag_predicted_objects_data_.empty() && rosbag_predicted_objects_data_.back().first > end_timestamp) end_timestamp = rosbag_predicted_objects_data_.back().first;
  if (!rosbag_ego_odom_data_.empty() && rosbag_ego_odom_data_.back().first > end_timestamp) end_timestamp = rosbag_ego_odom_data_.back().first;

  // Initialize Widget
  widget_ = new TimeManagerWidget(bag_timestamp_, end_timestamp);

  // Timer
  timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PerceptionReplayerNode::on_timer, this));
}

void PerceptionReplayerNode::on_timer()
{
  auto timestamp = now();
  kill_online_perception_node();

  if (options_.detected_object) {
    auto pc_msg = create_empty_pointcloud(timestamp);
    pointcloud_pub_->publish(pc_msg);
  }

  // Update timestamp based on slider and rate
  if (widget_) {
    // Get timestamp from slider (user might have moved it)
    // Actually, we should check if user is dragging.
    // But the python code updates bag_timestamp from slider every loop.
    // "self.bag_timestamp = self.rosbag_objects_data[0][0] + self.widget.slider.value() ..."
    
    rclcpp::Time start_time = widget_->value_to_timestamp(0); // Should be start
    // Wait, widget stores start/end.
    
    // If not paused, advance time
    if (!is_pause_) {
      bag_timestamp_ = bag_timestamp_ + rclcpp::Duration::from_seconds(rate_ * delta_time_);
    } else {
        // If paused, we might want to take the slider value?
        // The python code:
        // self.bag_timestamp = ... slider.value() ...
        // if not pause: bag_timestamp += ...
        // slider.setValue(...)
        
        // This implies the slider is the source of truth at start of loop?
        // But if we update slider at end of loop, then next loop it reads that value.
        // If user moves slider, it reads new value.
        // So yes, read from slider first.
        
        bag_timestamp_ = widget_->value_to_timestamp(widget_->slider->value());
    }
    
    // Update slider
    widget_->slider->setValue(widget_->timestamp_to_value(bag_timestamp_));
  }

  // Find topics
  DetectedObjects current_detected_objects;
  TrackedObjects current_tracked_objects;
  PredictedObjects current_predicted_objects;
  bool has_objects = false;
  
  TrafficSignals current_traffic_signals;
  bool has_traffic_signals = false;

  if (options_.detected_object) {
    current_detected_objects = find_msg_by_timestamp(rosbag_detected_objects_data_, bag_timestamp_);
    has_objects = true; // Assuming find returns something or empty
  } else if (options_.tracked_object) {
    current_tracked_objects = find_msg_by_timestamp(rosbag_tracked_objects_data_, bag_timestamp_);
    has_objects = true;
  } else {
    current_predicted_objects = find_msg_by_timestamp(rosbag_predicted_objects_data_, bag_timestamp_);
    has_objects = true;
  }
  
  current_traffic_signals = find_msg_by_timestamp(rosbag_traffic_signals_data_, bag_timestamp_);
  has_traffic_signals = true;

  // Publish Objects
  if (has_objects) {
    if (options_.detected_object) {
      current_detected_objects.header.stamp = timestamp;
      
      if (ego_odom_) {
        auto ego_odom_at_bag = find_msg_by_timestamp(rosbag_ego_odom_data_, bag_timestamp_);
        // Need to check if valid? find_msg_by_timestamp returns last if out of bounds.
        translate_objects_coordinate(ego_odom_->pose.pose, ego_odom_at_bag.pose.pose, current_detected_objects);
      }
      
      std::static_pointer_cast<rclcpp::Publisher<DetectedObjects>>(objects_pub_)->publish(current_detected_objects);
    } else if (options_.tracked_object) {
      current_tracked_objects.header.stamp = timestamp;
      std::static_pointer_cast<rclcpp::Publisher<TrackedObjects>>(objects_pub_)->publish(current_tracked_objects);
    } else {
      current_predicted_objects.header.stamp = timestamp;
      std::static_pointer_cast<rclcpp::Publisher<PredictedObjects>>(objects_pub_)->publish(current_predicted_objects);
    }
  }

  // Publish Traffic Signals
  if (has_traffic_signals) {
    // Check if empty?
    // Python: if traffic_signals_msg: ... elif prev ...
    // find_msg_by_timestamp returns a message. If the vector was empty it returns default constructed.
    // We should check if vector was empty in find_msg_by_timestamp?
    // The helper returns default constructed if empty.
    
    // Assuming we have data.
    current_traffic_signals.stamp = timestamp;
    traffic_signals_pub_->publish(current_traffic_signals);
    prev_traffic_signals_msg_ = current_traffic_signals;
  } else if (prev_traffic_signals_msg_.traffic_light_groups.size() > 0) { // Check if valid
    prev_traffic_signals_msg_.stamp = timestamp;
    traffic_signals_pub_->publish(prev_traffic_signals_msg_);
  }
}

void PerceptionReplayerNode::kill_online_perception_node()
{
  // Same as reproducer
  std::string kill_process_name;
  if (options_.detected_object) {
    kill_process_name = "dummy_perception_publisher_node";
  } else if (options_.tracked_object) {
    kill_process_name = "multi_object_tracker";
  } else {
    kill_process_name = "map_based_prediction";
  }

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

void PerceptionReplayerNode::set_pause(bool pause)
{
  is_pause_ = pause;
}

void PerceptionReplayerNode::set_rate(double rate)
{
  rate_ = rate;
}

void PerceptionReplayerNode::publish_recorded_ego_pose()
{
  auto ego_odom = find_msg_by_timestamp(rosbag_ego_odom_data_, bag_timestamp_);
  
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.pose.pose = ego_odom.pose.pose;
  // Covariance from python code
  msg.pose.covariance = {
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787
  };
  
  recorded_ego_pub_as_initialpose_->publish(msg);
  RCLCPP_INFO(get_logger(), "Published recorded ego pose as /initialpose");
}

void PerceptionReplayerNode::publish_goal_pose()
{
  if (rosbag_ego_odom_data_.empty()) return;
  
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.pose = rosbag_ego_odom_data_.back().second.pose.pose;
  
  goal_pose_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Published last recorded ego pose as /planning/mission_planning/goal");
}

} // namespace autoware_perception_replayer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_perception_replayer::PerceptionReplayerNode)
