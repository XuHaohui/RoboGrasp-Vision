#include <memory>
#include <thread>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "robograsp_interfaces/msg/perception_result.hpp"

#include "robograsp_vision_perception/mock_detector.hpp"

class MockPerceptionNode : public rclcpp::Node
{
public:
  MockPerceptionNode()
  : Node("mock_perception")
  {
    declare_parameter("sensor_frame", "camera_link");
    declare_parameter("trigger_index", -1);

    sensor_frame_ = get_parameter("sensor_frame").as_string();

    detector_ = std::make_shared<robograsp_vision_perception::MockDetector>();

    publisher_ = create_publisher<robograsp_interfaces::msg::PerceptionResult>(
      "/perception/result", 10);

    trigger_cb_ = add_on_set_parameters_callback(
      std::bind(&MockPerceptionNode::on_parameter_change, this, std::placeholders::_1));

    print_menu();

    keyboard_thread_ = std::thread(&MockPerceptionNode::keyboard_loop, this);

    RCLCPP_INFO(get_logger(),
      "MockPerceptionNode started (keyboard / param trigger mode)");
  }

  ~MockPerceptionNode() override
  {
    running_ = false;
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
      if (param.get_name() == "trigger_index") {
        int idx = param.as_int();
        if (idx >= 1 && idx <= static_cast<int>(detector_->object_count())) {
          publish_detection(static_cast<size_t>(idx - 1));
        } else {
          result.successful = false;
          result.reason = "trigger_index must be 1-" +
            std::to_string(detector_->object_count());
        }
      }
    }
    return result;
  }

  void print_menu()
  {
    std::cout << "================================\n"
              << " Mock Perception (Keyboard & Param)\n"
              << "--------------------------------\n"
              << " Keyboard (ros2 run only):\n";
    for (size_t i = 0; i < detector_->object_count(); ++i) {
      const auto & obj = detector_->object_at(i);
      std::cout << "  Press " << (i + 1) << ": " << obj.name
                << " (" << obj.x << ", " << obj.y << ", " << obj.z << ")\n";
    }
    std::cout << "  Press q: quit\n"
              << "--------------------------------\n"
              << " Param (ros2 launch / run):\n";
    for (size_t i = 0; i < detector_->object_count(); ++i) {
      const auto & obj = detector_->object_at(i);
      std::cout << "  ros2 param set /perception trigger_index "
                << (i + 1) << "  # " << obj.name << "\n";
    }
    std::cout << "================================\n"
              << "> " << std::flush;
  }

  void keyboard_loop()
  {
    std::string line;
    while (running_ && rclcpp::ok()) {
      if (!std::getline(std::cin, line)) {
        break;
      }
      if (line.empty()) {
        std::cout << "> " << std::flush;
        continue;
      }

      char key = line[0];

      if (key == 'q' || key == 'Q') {
        RCLCPP_INFO(get_logger(), "Quit requested, shutting down...");
        rclcpp::shutdown();
        return;
      }

      size_t idx = static_cast<size_t>(key - '1');
      if (idx < detector_->object_count()) {
        publish_detection(idx);
      } else {
        std::cout << "Invalid key. Press 1-" << detector_->object_count()
                  << " or q to quit.\n" << "> " << std::flush;
      }
    }
  }

  void publish_detection(size_t idx)
  {
    auto det = detector_->detect_by_index(idx);

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

    RCLCPP_INFO(get_logger(),
      "Published '%s' at (%.3f, %.3f, %.3f) [%s]",
      det.object_class.c_str(),
      det.position_3d.x, det.position_3d.y, det.position_3d.z,
      sensor_frame_.c_str());

    std::cout << "> " << std::flush;
  }

  std::shared_ptr<robograsp_vision_perception::MockDetector> detector_;
  rclcpp::Publisher<robograsp_interfaces::msg::PerceptionResult>::SharedPtr publisher_;
  std::string sensor_frame_;
  std::thread keyboard_thread_;
  bool running_{true};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr trigger_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
