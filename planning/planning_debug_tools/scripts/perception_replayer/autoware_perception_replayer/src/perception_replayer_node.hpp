#ifndef AUTOWARE_PERCEPTION_REPLAYER__PERCEPTION_REPLAYER_NODE_HPP_
#define AUTOWARE_PERCEPTION_REPLAYER__PERCEPTION_REPLAYER_NODE_HPP_

#include "common.hpp"
#include <QMainWindow>
#include <QPushButton>
#include <QSlider>
#include <QGridLayout>
#include <QLabel>
#include <QTimer>
#include <QMouseEvent>

namespace autoware_perception_replayer
{

class QJumpSlider : public QSlider
{
  Q_OBJECT
public:
  QJumpSlider(Qt::Orientation orientation, int max_value, QWidget * parent = nullptr);

protected:
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  int mouse_to_value(QMouseEvent * event);
  int max_value_;
  bool is_mouse_pressed_ = false;
};

class TimeManagerWidget : public QMainWindow
{
  Q_OBJECT
public:
  TimeManagerWidget(rclcpp::Time start_timestamp, rclcpp::Time end_timestamp, QWidget * parent = nullptr);
  
  QPushButton * pause_button;
  QPushButton * pub_recorded_ego_pose_button;
  QPushButton * pub_goal_pose_button;
  QJumpSlider * slider;
  std::vector<QPushButton*> rate_buttons;

  int timestamp_to_value(rclcpp::Time timestamp);
  rclcpp::Time value_to_timestamp(int value);

private:
  rclcpp::Time start_timestamp_;
  rclcpp::Time end_timestamp_;
  int max_value_ = 1000000;
};

class PerceptionReplayerNode : public PerceptionReplayerCommon
{
public:
  PerceptionReplayerNode(const rclcpp::NodeOptions & options);
  
  // Public method to be called by main loop or timer
  void update();
  
  rclcpp::Time get_bag_timestamp() const { return bag_timestamp_; }
  
  // Setters for GUI interaction
  void set_pause(bool pause);
  void set_rate(double rate);
  void on_slider_moved(int value);
  void publish_recorded_ego_pose();
  void publish_goal_pose();

  TimeManagerWidget * widget_; // Pointer to GUI

private:
  void on_timer();
  void kill_online_perception_node();
  
  rclcpp::TimerBase::SharedPtr timer_;
  double delta_time_ = 0.1;
  
  rclcpp::Time bag_timestamp_;
  bool is_pause_ = false;
  double rate_ = 1.0;
  
  TrafficSignals prev_traffic_signals_msg_;
};

} // namespace autoware_perception_replayer

#endif // AUTOWARE_PERCEPTION_REPLAYER__PERCEPTION_REPLAYER_NODE_HPP_
