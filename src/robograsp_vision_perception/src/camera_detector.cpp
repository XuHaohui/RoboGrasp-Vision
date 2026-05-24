#include "robograsp_vision_perception/camera_detector.hpp"

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <algorithm>
#include <vector>

namespace robograsp_vision_perception
{

CameraDetector::CameraDetector(rclcpp::Node * node)
  : node_(node)
{
  color_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", 10,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        latest_rgb_ = cv_ptr->image;
        latest_stamp_ = msg->header.stamp;
        has_rgb_ = true;
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge error (rgb): %s", e.what());
      }
    });

  depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/depth/image_raw", 10,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        latest_depth_ = cv_ptr->image;
        has_depth_ = true;
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge error (depth): %s", e.what());
      }
    });

  cinfo_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", 10,
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_cinfo_ = *msg;
      has_cinfo_ = true;
    });
}

DetectionResult CameraDetector::detect()
{
  cv::Mat rgb, depth;
  sensor_msgs::msg::CameraInfo cinfo;
  bool ready = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (has_rgb_ && has_depth_ && has_cinfo_) {
      rgb = latest_rgb_.clone();
      depth = latest_depth_.clone();
      cinfo = latest_cinfo_;
      ready = true;
    }
  }

  if (!ready) {
    DetectionResult r;
    r.detected = false;
    return r;
  }

  cv::Mat hsv;
  cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
  return detect_impl(hsv, depth, cinfo);
}

void CameraDetector::stop()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  color_sub_.reset();
  depth_sub_.reset();
  cinfo_sub_.reset();
}

std::vector<DetectionResult> CameraDetector::detect_all()
{
  cv::Mat rgb, depth;
  sensor_msgs::msg::CameraInfo cinfo;
  bool ready = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (has_rgb_ && has_depth_ && has_cinfo_) {
      rgb = latest_rgb_.clone();
      depth = latest_depth_.clone();
      cinfo = latest_cinfo_;
      ready = true;
    }
  }

  if (!ready) {
    return {};
  }

  cv::Mat hsv;
  cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
  return detect_all_impl(hsv, depth, cinfo);
}

ColorThresholdDetector::ColorThresholdDetector(rclcpp::Node * node)
  : CameraDetector(node)
{
}

DetectionResult ColorThresholdDetector::detect_impl(
    const cv::Mat & hsv, const cv::Mat & depth,
    const sensor_msgs::msg::CameraInfo & cinfo)
{
  auto r = detect_color_dual(hsv, depth,
    cv::Scalar(0, 100, 100), cv::Scalar(15, 255, 255),
    cv::Scalar(165, 100, 100), cv::Scalar(180, 255, 255),
    "cube", cinfo);
  if (r.detected) return r;

  r = detect_color(hsv, depth,
    cv::Scalar(45, 100, 100), cv::Scalar(75, 255, 255),
    "cylinder", cinfo);
  if (r.detected) return r;

  r = detect_color(hsv, depth,
    cv::Scalar(105, 100, 100), cv::Scalar(135, 255, 255),
    "box", cinfo);
  if (r.detected) return r;

  DetectionResult no_det;
  no_det.detected = false;
  return no_det;
}

DetectionResult ColorThresholdDetector::detect_from_mask(
    cv::Mat mask, const cv::Mat & depth,
    const std::string & object_class,
    const sensor_msgs::msg::CameraInfo & cinfo)
{
  DetectionResult result;
  result.detected = false;
  result.object_class = object_class;

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) return result;

  auto largest = std::max_element(contours.begin(), contours.end(),
    [](const auto & a, const auto & b) { return cv::contourArea(a) < cv::contourArea(b); });
  double area = cv::contourArea(*largest);
  if (area < 50) return result;

  cv::Moments m = cv::moments(*largest);
  int u = static_cast<int>(m.m10 / m.m00);
  int v = static_cast<int>(m.m01 / m.m00);

  float depth_val = depth.at<float>(v, u);
  if (depth_val <= 0 || std::isnan(depth_val) || std::isinf(depth_val)) return result;

  float cx = cinfo.k[2];
  float cy = cinfo.k[5];
  float fx = cinfo.k[0];
  float fy = cinfo.k[4];

  result.position_3d.x = (u - cx) * depth_val / fx;
  result.position_3d.y = (v - cy) * depth_val / fy;
  result.position_3d.z = depth_val;
  result.confidence = std::min(1.0f, static_cast<float>(area) / 500.0f);
  result.bbox_2d = {0, 0, 0, 0};
  result.bbox_size = {-1, -1, -1};
  result.detected = true;
  return result;
}

DetectionResult ColorThresholdDetector::detect_color(
    const cv::Mat & hsv, const cv::Mat & depth,
    const cv::Scalar & lower, const cv::Scalar & upper,
    const std::string & object_class,
    const sensor_msgs::msg::CameraInfo & cinfo)
{
  cv::Mat mask;
  cv::inRange(hsv, lower, upper, mask);
  return detect_from_mask(mask, depth, object_class, cinfo);
}

DetectionResult ColorThresholdDetector::detect_color_dual(
    const cv::Mat & hsv, const cv::Mat & depth,
    const cv::Scalar & lower1, const cv::Scalar & upper1,
    const cv::Scalar & lower2, const cv::Scalar & upper2,
    const std::string & object_class,
    const sensor_msgs::msg::CameraInfo & cinfo)
{
  cv::Mat mask1, mask2;
  cv::inRange(hsv, lower1, upper1, mask1);
  cv::inRange(hsv, lower2, upper2, mask2);
  cv::Mat mask;
  cv::bitwise_or(mask1, mask2, mask);
  return detect_from_mask(mask, depth, object_class, cinfo);
}

std::vector<DetectionResult> ColorThresholdDetector::detect_all_impl(
    const cv::Mat & hsv, const cv::Mat & depth,
    const sensor_msgs::msg::CameraInfo & cinfo)
{
  std::vector<DetectionResult> results;

  auto r = detect_color_dual(hsv, depth,
    cv::Scalar(0, 100, 100), cv::Scalar(15, 255, 255),
    cv::Scalar(165, 100, 100), cv::Scalar(180, 255, 255),
    "cube", cinfo);
  if (r.detected) results.push_back(r);

  r = detect_color(hsv, depth,
    cv::Scalar(45, 100, 100), cv::Scalar(75, 255, 255),
    "cylinder", cinfo);
  if (r.detected) results.push_back(r);

  r = detect_color(hsv, depth,
    cv::Scalar(105, 100, 100), cv::Scalar(135, 255, 255),
    "box", cinfo);
  if (r.detected) results.push_back(r);

  return results;
}

}  // namespace robograsp_vision_perception
