#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "robograsp_interfaces/msg/perception_result.hpp"
#include "robograsp_interfaces/srv/detect_objects.hpp"

#include "robograsp_vision_perception/camera_detector.hpp"

using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception_node")
  {
    declare_parameter("publish_rate", 5.0);
    declare_parameter("sensor_frame", "camera_depth_optical_frame");
    declare_parameter("auto_publish", false);

    publish_rate_ = get_parameter("publish_rate").as_double();
    sensor_frame_ = get_parameter("sensor_frame").as_string();
    bool auto_publish = get_parameter("auto_publish").as_bool();

    detector_ = std::make_shared<robograsp_vision_perception::ColorThresholdDetector>(this);

    publisher_ = create_publisher<robograsp_interfaces::msg::PerceptionResult>(
      "/perception/result", 10);

    detect_srv_ = create_service<robograsp_interfaces::srv::DetectObjects>(
      "~/detect_objects",
      std::bind(&PerceptionNode::detect_cb, this,
        std::placeholders::_1, std::placeholders::_2));

    if (auto_publish) {
      auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / publish_rate_));
      timer_ = create_wall_timer(period, std::bind(&PerceptionNode::timer_cb, this));
    }

    RCLCPP_INFO(get_logger(),
      "PerceptionNode started (auto_publish=%s)",
      auto_publish ? "true" : "false");
  }

private:
  void detect_cb(
    const std::shared_ptr<robograsp_interfaces::srv::DetectObjects::Request> /*req*/,
    std::shared_ptr<robograsp_interfaces::srv::DetectObjects::Response> res)
  {
    auto detections = detector_->detect_all();

    for (const auto & det : detections) {
      if (!det.detected) continue;

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
      msg.status = "detected";

      res->results.push_back(msg);
    }

    RCLCPP_INFO(get_logger(),
      "DetectObjects: found %zu object(s)", res->results.size());
  }

  void timer_cb()
  {
    auto det = detector_->detect();

    if (!det.detected) {
      return;
    }

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
    msg.status = "detected";

    publisher_->publish(msg);
  }

  std::shared_ptr<robograsp_vision_perception::ObjectDetector> detector_;
  rclcpp::Publisher<robograsp_interfaces::msg::PerceptionResult>::SharedPtr publisher_;
  rclcpp::Service<robograsp_interfaces::srv::DetectObjects>::SharedPtr detect_srv_;
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
