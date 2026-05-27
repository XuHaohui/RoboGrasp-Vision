#pragma once

#include "robograsp_vision_bridge/robot_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>

namespace robograsp_vision_bridge
{

class PiperControlInterface : public RobotInterface
{
public:
  PiperControlInterface(
    rclcpp::Node * node,
    const std::string & target_topic = "/target_pose",
    const std::string & target_frame = "world");

  bool send_object_info(
    const robograsp_interfaces::msg::PerceptionResult & perception) override;

  void cancel() override;

  bool is_ready() const override { return ready_; }

  void set_ready(bool ready) { ready_ = ready; }

private:
  rclcpp::Node * node_;
  std::string target_topic_;
  std::string target_frame_;
  rclcpp::Publisher<robograsp_interfaces::msg::ObjectInfo>::SharedPtr publisher_;
  bool ready_{true};
};

}  // namespace robograsp_vision_bridge
