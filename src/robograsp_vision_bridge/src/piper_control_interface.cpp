#include "robograsp_vision_bridge/piper_control_interface.hpp"

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
}

bool PiperControlInterface::send_object_info(
  const robograsp_interfaces::msg::PerceptionResult & perception)
{
  robograsp_interfaces::msg::ObjectInfo msg;
  msg.header.stamp = node_->now();
  msg.object_class = perception.object_class;
  msg.confidence = perception.confidence;
  msg.bbox_size = perception.bbox_size;

  const bool bbox_invalid = (msg.bbox_size[0] <= 0.0f ||
                             msg.bbox_size[1] <= 0.0f ||
                             msg.bbox_size[2] <= 0.0f);
  if (bbox_invalid) {
    if (perception.object_class == "cube") {
      msg.bbox_size = {0.04f, 0.04f, 0.04f};
    } else if (perception.object_class == "cylinder") {
      msg.bbox_size = {0.07f, 0.07f, 0.08f};
    } else if (perception.object_class == "box") {
      msg.bbox_size = {0.05f, 0.05f, 0.05f};
    } else if (perception.object_class == "sphere") {
      msg.bbox_size = {0.04f, 0.04f, 0.04f};
    } else {
      msg.bbox_size = {0.04f, 0.04f, 0.04f};
    }
    RCLCPP_WARN(node_->get_logger(),
      "bbox_size from perception [%.2f, %.2f, %.2f] is invalid for '%s', using defaults",
      perception.bbox_size[0], perception.bbox_size[1], perception.bbox_size[2],
      perception.object_class.c_str());
  }

  // Perception now outputs in target_frame; use position directly
  msg.header.frame_id = target_frame_;
  msg.bottom_center.x = perception.position.x;
  msg.bottom_center.y = perception.position.y;
  msg.bottom_center.z = perception.position.z - perception.bbox_size[2] / 2.0f;

  publisher_->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
    "Sent object '%s' to %s [%s]: bottom=(%.3f, %.3f, %.3f) size=(%.3f, %.3f, %.3f)",
    msg.object_class.c_str(),
    target_topic_.c_str(),
    target_frame_.c_str(),
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
