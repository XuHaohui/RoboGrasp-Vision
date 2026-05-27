#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robograsp_interfaces/msg/perception_result.hpp"

#include "robograsp_vision_bridge/piper_control_interface.hpp"

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("bridge_node")
  {
    declare_parameter("robot_backend", "piper_control");
    declare_parameter("target_topic", "/target_pose");
    declare_parameter("target_frame", "world");

    std::string backend = get_parameter("robot_backend").as_string();
    std::string target_topic = get_parameter("target_topic").as_string();
    std::string target_frame = get_parameter("target_frame").as_string();

    if (backend == "piper_control") {
      robot_ = std::make_shared<robograsp_vision_bridge::PiperControlInterface>(
        this, target_topic, target_frame);
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown backend: %s", backend.c_str());
      throw std::runtime_error("Unknown robot backend: " + backend);
    }

    percept_sub_ = create_subscription<robograsp_interfaces::msg::PerceptionResult>(
      "/perception/result", 10,
      std::bind(&BridgeNode::perception_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "BridgeNode started: backend=%s, forwarding /perception/result -> %s [%s]",
      backend.c_str(), target_topic.c_str(), target_frame.c_str());
  }

private:
  void perception_cb(
    const robograsp_interfaces::msg::PerceptionResult::SharedPtr msg)
  {
    last_perception_ = *msg;

    if (msg->status != "detected") {
      RCLCPP_DEBUG(get_logger(),
        "Skipping perception '%s' with status '%s'",
        msg->object_class.c_str(), msg->status.c_str());
      return;
    }

    if (!robot_->is_ready()) {
      RCLCPP_WARN(get_logger(),
        "Robot not ready, dropping perception for '%s'",
        msg->object_class.c_str());
      return;
    }

    bool success = robot_->send_object_info(*msg);
    if (success) {
      RCLCPP_INFO(get_logger(),
        "Forwarded object '%s' to robot", msg->object_class.c_str());
    } else {
      RCLCPP_ERROR(get_logger(),
        "Failed to forward object '%s'", msg->object_class.c_str());
    }
  }

  std::shared_ptr<robograsp_vision_bridge::RobotInterface> robot_;
  rclcpp::Subscription<robograsp_interfaces::msg::PerceptionResult>::SharedPtr percept_sub_;
  robograsp_interfaces::msg::PerceptionResult last_perception_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
