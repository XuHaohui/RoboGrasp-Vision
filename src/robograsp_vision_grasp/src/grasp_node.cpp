#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robograsp_interfaces/msg/perception_result.hpp"
#include "robograsp_interfaces/msg/grasp_command.hpp"

#include "robograsp_vision_grasp/generator.hpp"

class GraspNode : public rclcpp::Node
{
public:
  GraspNode()
  : Node("grasp_node"), sequence_id_(0)
  {
    declare_parameter("approach_offset", 0.10);
    declare_parameter("grasp_width", 0.04);
    declare_parameter("world_frame", "world");
    declare_parameter("min_confidence", 0.5);

    double approach_offset = get_parameter("approach_offset").as_double();
    double grasp_width = get_parameter("grasp_width").as_double();
    std::string world_frame = get_parameter("world_frame").as_string();
    min_confidence_ = get_parameter("min_confidence").as_double();

    generator_ = std::make_shared<robograsp_vision_grasp::SimpleTopDownGenerator>(
      approach_offset, grasp_width, world_frame);

    perception_sub_ = create_subscription<robograsp_interfaces::msg::PerceptionResult>(
      "/perception/result", 10,
      std::bind(&GraspNode::perception_cb, this, std::placeholders::_1));

    grasp_pub_ = create_publisher<robograsp_interfaces::msg::GraspCommand>(
      "/grasp/command", 10);

    RCLCPP_INFO(get_logger(),
      "GraspNode started, listening on /perception/result "
      "(min_confidence=%.2f)", min_confidence_);
  }

private:
  void perception_cb(const robograsp_interfaces::msg::PerceptionResult::SharedPtr msg)
  {
    if (msg->confidence < min_confidence_) {
      RCLCPP_DEBUG(get_logger(),
        "Skipping %s: confidence %.2f < threshold %.2f",
        msg->object_class.c_str(), msg->confidence, min_confidence_);
      return;
    }

    auto cmd = generator_->generate(msg);
    if (!cmd) {
      RCLCPP_WARN(get_logger(), "Failed to generate grasp for %s",
        msg->object_class.c_str());
      return;
    }

    sequence_id_++;
    cmd->sequence_id = sequence_id_;
    cmd->header.stamp = now();

    grasp_pub_->publish(*cmd);

    RCLCPP_INFO(get_logger(),
      "[Seq %u] Generated %s grasp for '%s' at "
      "(%.3f, %.3f, %.3f) quality=%.2f",
      cmd->sequence_id,
      cmd->strategy.c_str(),
      cmd->object_class.c_str(),
      cmd->grasp_pose.pose.position.x,
      cmd->grasp_pose.pose.position.y,
      cmd->grasp_pose.pose.position.z,
      cmd->quality);
  }

  std::shared_ptr<robograsp_vision_grasp::GraspGenerator> generator_;
  rclcpp::Subscription<robograsp_interfaces::msg::PerceptionResult>::SharedPtr perception_sub_;
  rclcpp::Publisher<robograsp_interfaces::msg::GraspCommand>::SharedPtr grasp_pub_;
  double min_confidence_;
  uint32_t sequence_id_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspNode>());
  rclcpp::shutdown();
  return 0;
}
