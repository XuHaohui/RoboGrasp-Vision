#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "robograsp_interfaces/msg/perception_result.hpp"
#include "robograsp_interfaces/srv/detect_objects.hpp"

#include "robograsp_vision_perception/camera_detector.hpp"
#include "robograsp_vision_perception/depth_geometry_detector.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : Node("perception_node")
  {
    declare_parameter("publish_rate", 5.0);
    declare_parameter("sensor_frame", "camera_depth_optical_frame");
    declare_parameter("target_frame", "world");
    declare_parameter("auto_publish", false);
    declare_parameter("detector_backend", "color_threshold");

    publish_rate_ = get_parameter("publish_rate").as_double();
    sensor_frame_ = get_parameter("sensor_frame").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    bool auto_publish = get_parameter("auto_publish").as_bool();
    std::string detector_backend = get_parameter("detector_backend").as_string();

    if (detector_backend == "depth_geometry") {
      detector_ = std::make_shared<robograsp_vision_perception::DepthFirstGeometricDetector>(this);
    } else {
      detector_ = std::make_shared<robograsp_vision_perception::ColorThresholdDetector>(this);
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

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
      "PerceptionNode started (auto_publish=%s, target_frame=%s)",
      auto_publish ? "true" : "false",
      target_frame_.c_str());
  }

private:
  geometry_msgs::msg::Point transform_position(
    const geometry_msgs::msg::Point & point_in_camera,
    const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PointStamped point_in;
    point_in.header.frame_id = sensor_frame_;
    point_in.header.stamp = stamp;
    point_in.point = point_in_camera;

    try {
      auto point_out = tf_buffer_->transform(point_in, target_frame_);
      return point_out.point;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "TF from '%s' to '%s' failed: %s. Using camera-frame coordinates.",
        sensor_frame_.c_str(), target_frame_.c_str(), ex.what());
      return point_in_camera;
    }
  }

  void detect_cb(
    const std::shared_ptr<robograsp_interfaces::srv::DetectObjects::Request> /*req*/,
    std::shared_ptr<robograsp_interfaces::srv::DetectObjects::Response> res)
  {
    auto detections = detector_->detect_all();
    auto stamp = now();

    for (const auto & det : detections) {
      if (!det.detected) continue;

      auto world_pos = transform_position(det.position_3d, stamp);

      robograsp_interfaces::msg::PerceptionResult msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = target_frame_;
      msg.object_class = det.object_class;
      msg.confidence = det.confidence;
      msg.position = world_pos;
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

    auto stamp = now();
    auto world_pos = transform_position(det.position_3d, stamp);

    robograsp_interfaces::msg::PerceptionResult msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = target_frame_;
    msg.object_class = det.object_class;
    msg.confidence = det.confidence;
    msg.position = world_pos;
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
  std::string target_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
