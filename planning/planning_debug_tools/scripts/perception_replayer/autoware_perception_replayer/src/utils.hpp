#ifndef AUTOWARE_PERCEPTION_REPLAYER__UTILS_HPP_
#define AUTOWARE_PERCEPTION_REPLAYER__UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

namespace autoware_perception_replayer
{

inline sensor_msgs::msg::PointCloud2 create_empty_pointcloud(const rclcpp::Time & timestamp)
{
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pointcloud_msg.header.stamp = timestamp;
  pointcloud_msg.header.frame_id = "map";
  pointcloud_msg.height = 1;
  pointcloud_msg.is_dense = true;
  pointcloud_msg.point_step = 16;
  pointcloud_msg.width = 0; // Empty

  sensor_msgs::msg::PointField field_x;
  field_x.name = "x";
  field_x.offset = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count = 1;
  pointcloud_msg.fields.push_back(field_x);

  sensor_msgs::msg::PointField field_y;
  field_y.name = "y";
  field_y.offset = 4;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_y.count = 1;
  pointcloud_msg.fields.push_back(field_y);

  sensor_msgs::msg::PointField field_z;
  field_z.name = "z";
  field_z.offset = 8;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_z.count = 1;
  pointcloud_msg.fields.push_back(field_z);

  return pointcloud_msg;
}

inline double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & orientation)
{
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

inline geometry_msgs::msg::Quaternion get_quaternion_from_yaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

template <typename T>
void translate_objects_coordinate(
  const geometry_msgs::msg::Pose & ego_pose,
  const geometry_msgs::msg::Pose & log_ego_pose,
  T & objects_msg)
{
  double log_ego_yaw = get_yaw_from_quaternion(log_ego_pose.orientation);
  double ego_yaw = get_yaw_from_quaternion(ego_pose.orientation);

  // Transformation matrices (simplified 2D + yaw for this use case)
  // We need to transform object from log_ego frame to map frame, then to current ego frame?
  // The python code does: object_pos_vec = inv(ego_pose_trans_mat) * log_ego_pose_trans_mat * log_object_pos_vec
  // This transforms the object from the log's world frame to the current ego's local frame?
  // Wait, the python code comments say: "translate object pose from ego pose in log to ego pose in simulation"
  // But the matrices are constructed from map-to-ego (or map-to-log-ego).
  // Let's follow the math:
  // log_ego_pose_trans_mat: T_map_logego
  // ego_pose_trans_mat: T_map_ego
  // log_object_pos_vec: P_map_obj (since objects are usually in map frame in autoware?)
  //
  // Python code:
  // log_object_pose = o.kinematics.pose_with_covariance.pose
  // log_object_pos_vec = [x, y, 1]
  // object_pos_vec = inv(T_map_ego) * T_map_logego * log_object_pos_vec
  //
  // If objects are in map frame, then `log_object_pos_vec` is P_map.
  // Then `T_map_logego * P_map` would be wrong if T_map_logego is the transform of the ego in map.
  //
  // Actually, in Autoware, objects are usually in "map" frame if they are tracked/predicted?
  // Or "base_link" if detected?
  // The python code uses `log_object_pose` directly.
  //
  // If the objects are in map frame, then we just want to shift them so they appear relative to the NEW ego pose
  // in the same way they were relative to the OLD ego pose.
  // i.e. P_obj_rel_ego = inv(T_map_logego) * P_obj_map_log
  //      P_obj_map_new = T_map_ego * P_obj_rel_ego
  //      P_obj_map_new = T_map_ego * inv(T_map_logego) * P_obj_map_log
  //
  // The python code does: inv(T_map_ego) * T_map_logego * P_obj
  // This looks like it's calculating P_obj_rel_ego_new?
  //
  // Let's look at the python code again:
  // ego_pose_trans_mat = [[cos, -sin, x], [sin, cos, y], [0,0,1]] -> This is T_map_ego (transform from ego to map? No, usually rotation matrix R is [[cos, -sin], [sin, cos]] which rotates vector in local to global. And translation is added. So yes, this is T_map_ego.
  //
  // object_pos_vec = inv(T_map_ego) * T_map_logego * log_object_pos_vec
  //
  // If `log_object_pos_vec` is in map frame.
  // Then `T_map_logego * log_object_pos_vec` is weird.
  //
  // Maybe `log_object_pos_vec` is in `base_link` frame?
  // If it is detected objects, they are usually in `base_link`.
  // If so, `T_map_logego * log_object_pos_vec` transforms it to map frame (P_map).
  // Then `inv(T_map_ego) * P_map` transforms it back to the NEW ego's `base_link` frame.
  //
  // So if the input objects are in base_link, and we want the output objects to be in base_link (relative to the new ego),
  // then this math is correct.
  //
  // However, the python code says:
  // "translate object pose from ego pose in log to ego pose in simulation"
  //
  // If the objects are `DetectedObjects`, they are in `base_link`.
  // If they are `TrackedObjects` or `PredictedObjects`, they are usually in `map`.
  //
  // The python script handles `detected_object` arg.
  // If `detected_object` is true, it calls `translate_objects_coordinate`.
  // So this is indeed for detected objects in `base_link`.
  //
  // So:
  // 1. Transform log object (base_link) to map using log ego pose.
  // 2. Transform map object to new ego (base_link) using new ego pose.
  //
  // Wait, `inv(T_map_ego)` transforms Map -> Ego.
  // `T_map_logego` transforms Ego -> Map.
  // So `inv(T_map_ego) * (T_map_logego * P_log_obj)`
  // = T_ego_map * T_map_logego * P_log_obj
  // = T_ego_logego * P_log_obj
  // = P_new_obj
  //
  // Yes, this is correct for base_link objects.

  double c_log = cos(log_ego_yaw);
  double s_log = sin(log_ego_yaw);
  double x_log = log_ego_pose.position.x;
  double y_log = log_ego_pose.position.y;

  double c_ego = cos(ego_yaw);
  double s_ego = sin(ego_yaw);
  double x_ego = ego_pose.position.x;
  double y_ego = ego_pose.position.y;

  // T_map_logego
  // [ c_log -s_log  x_log ]
  // [ s_log  c_log  y_log ]
  // [ 0      0      1     ]

  // T_map_ego
  // [ c_ego -s_ego  x_ego ]
  // [ s_ego  c_ego  y_ego ]
  // [ 0      0      1     ]

  // We want inv(T_map_ego) * T_map_logego
  // Let T_diff = inv(T_map_ego) * T_map_logego
  //
  // inv(T_map_ego) is:
  // [ c_ego   s_ego   -x_ego*c_ego - y_ego*s_ego ]
  // [ -s_ego  c_ego    x_ego*s_ego - y_ego*c_ego ]
  // [ 0       0        1                         ]

  // This seems complicated to implement manually.
  // But since it's 2D, we can just do:
  // P_map = T_map_logego * P_log
  // P_new = inv(T_map_ego) * P_map

  for (auto & obj : objects_msg.objects) {
    // Assuming detected objects, so kinematics.pose_with_covariance.pose is the one to change.
    // Note: Tracked/Predicted objects have different structure but this function is likely only used for DetectedObjects in the python script.
    // The python script checks `if self.args.detected_object:` before calling this.

    auto & pose = obj.kinematics.pose_with_covariance.pose;
    double obj_x = pose.position.x;
    double obj_y = pose.position.y;
    double obj_yaw = get_yaw_from_quaternion(pose.orientation);

    // 1. To Map (using log ego)
    double map_x = c_log * obj_x - s_log * obj_y + x_log;
    double map_y = s_log * obj_x + c_log * obj_y + y_log;
    double map_yaw = obj_yaw + log_ego_yaw;

    // 2. To New Ego (using current ego)
    // P_new = R_ego^T * (P_map - T_ego)
    double dx = map_x - x_ego;
    double dy = map_y - y_ego;

    double new_x = c_ego * dx + s_ego * dy;
    double new_y = -s_ego * dx + c_ego * dy;
    double new_yaw = map_yaw - ego_yaw;

    pose.position.x = new_x;
    pose.position.y = new_y;
    pose.orientation = get_quaternion_from_yaw(new_yaw);
  }
}

} // namespace autoware_perception_replayer

#endif // AUTOWARE_PERCEPTION_REPLAYER__UTILS_HPP_
