#include "robograsp_vision_bridge/piper_control_interface.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace robograsp_vision_bridge
{

PiperControlInterface::PiperControlInterface(
  rclcpp::Node * node,
  const std::string & target_topic,
  const std::string & target_frame)
: node_(node)
, target_topic_(target_topic)
, target_frame_(target_frame)
{
  publisher_ = node_->create_publisher<robograsp_interfaces::msg::ObjectInfo>(
    target_topic_, 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

bool PiperControlInterface::send_object_info(
  const robograsp_interfaces::msg::PerceptionResult & perception)
{
  robograsp_interfaces::msg::ObjectInfo msg;
  msg.header.stamp = node_->now();
  msg.object_class = perception.object_class;
  msg.confidence = perception.confidence;
  msg.bbox_size = perception.bbox_size;

  std::string src_frame = perception.header.frame_id;
  if (src_frame.empty()) {
    src_frame = "camera_link";
  }

  geometry_msgs::msg::PointStamped point_in;
  point_in.header.frame_id = src_frame;
  point_in.header.stamp = perception.header.stamp;
  point_in.point = perception.position;

  geometry_msgs::msg::Point transformed_point;
  std::string resolved_frame = src_frame;

  try {
    auto point_out = tf_buffer_->transform(point_in, target_frame_);
    transformed_point = point_out.point;
    resolved_frame = target_frame_;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "TF from '%s' to '%s' unavailable: %s. Using original coordinates.",
      src_frame.c_str(), target_frame_.c_str(), ex.what());
    transformed_point = perception.position;
  }

  msg.header.frame_id = resolved_frame;
  msg.bottom_center.x = transformed_point.x;
  msg.bottom_center.y = transformed_point.y;
  msg.bottom_center.z = transformed_point.z - perception.bbox_size[2] / 2.0f;

  publisher_->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
    "Sent object '%s' to %s [%s]: bottom=(%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f)",
    msg.object_class.c_str(),
    target_topic_.c_str(),
    resolved_frame.c_str(),
    msg.bottom_center.x,
    msg.bottom_center.y,
    msg.bottom_center.z,
    msg.bbox_size[0],
    msg.bbox_size[1],
    msg.bbox_size[2]);
  return true;
}

void PiperControlInterface::cancel()
{
  RCLCPP_WARN(node_->get_logger(), "Cancel not implemented for piper_control");
}

}  // namespace robograsp_vision_bridge
