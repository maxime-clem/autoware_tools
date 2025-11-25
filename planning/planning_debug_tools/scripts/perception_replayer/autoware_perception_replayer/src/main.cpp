#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "perception_replayer_node.hpp"
#include "perception_reproducer_node.hpp"

int main(int argc, char ** argv)
{
  // Check executable name to decide mode
  std::string exe_name = argv[0];
  bool is_replayer = false;
  if (exe_name.find("perception_replayer") != std::string::npos) {
    is_replayer = true;
  }

  // If replayer, we need QApplication
  std::shared_ptr<QApplication> app;
  if (is_replayer) {
    // QApplication requires argc/argv
    // We need to pass them before rclcpp::init removes ros args?
    // rclcpp::init modifies argv? No, it parses them.
    // Qt also parses them.
    // Let's init Qt first.
    app = std::make_shared<QApplication>(argc, argv);
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  if (is_replayer) {
    auto node = std::make_shared<autoware_perception_replayer::PerceptionReplayerNode>(options);
    
    // Setup GUI connections
    // Widget is already created in constructor
    if (node->widget_) {
      node->widget_->show();
    }
    
    // We need end timestamp for the slider.
    // Let's update PerceptionReplayerCommon to store start/end time.
    // For now, just use a default or get it from data.
    rclcpp::Time start_time = node->get_bag_timestamp();
    rclcpp::Time end_time = start_time;
    
    // Find max timestamp in data
    // This is inefficient but works for now.
    // Actually, the vectors are sorted by time.
    // So we can check the back() of each vector.
    
    // Accessing protected members?
    // PerceptionReplayerNode inherits from Common, so it can access them.
    // But main() cannot.
    // We should move GUI init inside the Node constructor or a method.
    // But TimeManagerWidget is a QMainWindow, so it should be shown.
    
    // Let's move the widget creation inside PerceptionReplayerNode constructor or an init method.
    // I already have `widget_` member in PerceptionReplayerNode.
    // But I need to pass `app`? No, `new QWidget` doesn't need app passed explicitly.
    
    // Refactor: Move widget creation to PerceptionReplayerNode.
    // But PerceptionReplayerNode is a ROS node, it shouldn't necessarily know about GUI details if we want to keep it clean?
    // But I already included QMainWindow in the header.
    // So let's do it in the constructor of PerceptionReplayerNode.
    
    // Wait, I already put `widget_` as a public member pointer in the header, but didn't initialize it in the cpp constructor.
    // I should initialize it in the constructor.
    
    // Let's fix PerceptionReplayerNode.cpp to initialize the widget.
    // But I need `app` to be running.
    
    node->widget_->show();
    
    // Connect signals
    QObject::connect(node->widget_->pause_button, &QPushButton::clicked, [node](bool checked){
      node->set_pause(checked);
    });
    
    for (auto btn : node->widget_->rate_buttons) {
      QObject::connect(btn, &QPushButton::clicked, [node, btn](){
        node->set_rate(btn->text().toDouble());
      });
    }
    
    QObject::connect(node->widget_->pub_recorded_ego_pose_button, &QPushButton::clicked, [node](){
      node->publish_recorded_ego_pose();
    });
    
    QObject::connect(node->widget_->pub_goal_pose_button, &QPushButton::clicked, [node](){
      node->publish_goal_pose();
    });
    
    // Slider connection?
    // The node updates the slider in on_timer.
    // If user moves slider, we need to update node.
    // QJumpSlider doesn't emit valueChanged when set programmatically?
    // It does. We need to distinguish.
    // Or just read slider value in timer loop (which I implemented).
    
    // Run loop
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    
    // We need to spin ROS and Qt.
    // Use a QTimer to spin ROS.
    QTimer * timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, [&exec](){
      exec.spin_some();
    });
    timer->start(10); // 100Hz
    
    app->exec();
    
  } else {
    // Reproducer
    auto node = std::make_shared<autoware_perception_replayer::PerceptionReproducerNode>(options);
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
