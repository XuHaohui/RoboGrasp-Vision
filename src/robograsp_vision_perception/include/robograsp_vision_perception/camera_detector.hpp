#pragma once

#include "robograsp_vision_perception/detector.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <mutex>
#include <string>

namespace robograsp_vision_perception
{

class CameraDetector : public ObjectDetector
{
public:
  explicit CameraDetector(rclcpp::Node * node);

  DetectionResult detect() final;
  std::vector<DetectionResult> detect_all() final;
  void stop() override;

protected:
  virtual DetectionResult detect_impl(
      const cv::Mat & hsv, const cv::Mat & depth,
      const sensor_msgs::msg::CameraInfo & cinfo) = 0;

  virtual std::vector<DetectionResult> detect_all_impl(
      const cv::Mat & hsv, const cv::Mat & depth,
      const sensor_msgs::msg::CameraInfo & cinfo) = 0;

  rclcpp::Node * node_;

  mutable std::mutex data_mutex_;

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_sub_;

  cv::Mat latest_rgb_;
  cv::Mat latest_depth_;
  sensor_msgs::msg::CameraInfo latest_cinfo_;
  bool has_rgb_{false};
  bool has_depth_{false};
  bool has_cinfo_{false};

  rclcpp::Time latest_rgb_stamp_;
  rclcpp::Time latest_depth_stamp_;
  rclcpp::Time latest_cinfo_stamp_;

  double stale_timeout_{1.0};
};

class ColorThresholdDetector : public CameraDetector
{
public:
  explicit ColorThresholdDetector(rclcpp::Node * node);

protected:
  DetectionResult detect_impl(
      const cv::Mat & hsv, const cv::Mat & depth,
      const sensor_msgs::msg::CameraInfo & cinfo) override;

  std::vector<DetectionResult> detect_all_impl(
      const cv::Mat & hsv, const cv::Mat & depth,
      const sensor_msgs::msg::CameraInfo & cinfo) override;

private:
  DetectionResult detect_from_mask(
      cv::Mat mask, const cv::Mat & depth,
      const std::string & object_class,
      const sensor_msgs::msg::CameraInfo & cinfo);

  DetectionResult detect_color(
      const cv::Mat & hsv, const cv::Mat & depth,
      const cv::Scalar & lower, const cv::Scalar & upper,
      const std::string & object_class,
      const sensor_msgs::msg::CameraInfo & cinfo);

  DetectionResult detect_color_dual(
      const cv::Mat & hsv, const cv::Mat & depth,
      const cv::Scalar & lower1, const cv::Scalar & upper1,
      const cv::Scalar & lower2, const cv::Scalar & upper2,
      const std::string & object_class,
      const sensor_msgs::msg::CameraInfo & cinfo);
};

}  // namespace robograsp_vision_perception
