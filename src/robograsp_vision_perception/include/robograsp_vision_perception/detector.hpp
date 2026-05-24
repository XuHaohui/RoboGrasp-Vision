#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <string>
#include <vector>
#include <memory>

namespace robograsp_vision_perception
{

struct DetectionResult
{
  bool detected;
  std::string object_class;
  float confidence;
  geometry_msgs::msg::Point position_3d;
  std::array<float, 3> bbox_size;
  std::array<float, 4> bbox_2d;
};

class ObjectDetector
{
public:
  using SharedPtr = std::shared_ptr<ObjectDetector>;

  virtual ~ObjectDetector() = default;

  virtual DetectionResult detect() = 0;

  virtual std::vector<DetectionResult> detect_all() = 0;

  virtual void stop() = 0;

  virtual rclcpp::Time latest_image_stamp() const { return rclcpp::Time(); }
};

}  // namespace robograsp_vision_perception
