#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "robograsp_interfaces/msg/perception_result.hpp"

#include "robograsp_vision_perception/mock_detector.hpp"

using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception_node")
  {
    declare_parameter("publish_rate", 5.0);
    declare_parameter("sensor_frame", "camera_link");
    declare_parameter("use_mock", true);

    publish_rate_ = get_parameter("publish_rate").as_double();
    sensor_frame_ = get_parameter("sensor_frame").as_string();
    bool use_mock = get_parameter("use_mock").as_bool();

    if (use_mock) {
      detector_ = std::make_shared<robograsp_vision_perception::MockDetector>();
    } else {
      RCLCPP_WARN(get_logger(),
        "Real detector not implemented, falling back to MockDetector");
      detector_ = std::make_shared<robograsp_vision_perception::MockDetector>();
    }

    publisher_ = create_publisher<robograsp_interfaces::msg::PerceptionResult>(
      "/perception/result", 10);

    auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / publish_rate_));
    timer_ = create_wall_timer(period, std::bind(&PerceptionNode::timer_cb, this));

    RCLCPP_INFO(get_logger(), "PerceptionNode started");
  }

private:
  void timer_cb()
  {
    auto det = detector_->detect();

    robograsp_interfaces::msg::PerceptionResult msg;
    msg.header.stamp = now();
    msg.header.frame_id = sensor_frame_;
    msg.object_class = det.object_class;
    msg.confidence = det.confidence;

    msg.position.x = det.position_3d.x;
    msg.position.y = det.position_3d.y;
    msg.position.z = det.position_3d.z;

    msg.bbox_size = det.bbox_size;
    msg.bbox_2d = det.bbox_2d;
    msg.sensor_frame = sensor_frame_;
    msg.status = det.detected ? "detected" : "lost";

    publisher_->publish(msg);
  }

  std::shared_ptr<robograsp_vision_perception::ObjectDetector> detector_;
  rclcpp::Publisher<robograsp_interfaces::msg::PerceptionResult>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_rate_;
  std::string sensor_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
