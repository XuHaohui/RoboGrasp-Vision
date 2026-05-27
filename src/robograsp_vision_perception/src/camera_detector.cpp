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
  node_->declare_parameter("stale_timeout", 1.0);
  stale_timeout_ = node_->get_parameter("stale_timeout").as_double();

  color_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", 10,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        latest_rgb_ = cv_ptr->image;
        latest_rgb_stamp_ = msg->header.stamp;
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
        latest_depth_stamp_ = msg->header.stamp;
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
      latest_cinfo_stamp_ = msg->header.stamp;
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
    auto now = node_->now();
    auto timeout = rclcpp::Duration::from_seconds(stale_timeout_);

    if (has_rgb_ && (now - latest_rgb_stamp_) > timeout) {
      has_rgb_ = false;
    }
    if (has_depth_ && (now - latest_depth_stamp_) > timeout) {
      has_depth_ = false;
    }
    if (has_cinfo_ && (now - latest_cinfo_stamp_) > timeout) {
      has_cinfo_ = false;
    }

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
    auto now = node_->now();
    auto timeout = rclcpp::Duration::from_seconds(stale_timeout_);

    if (has_rgb_ && (now - latest_rgb_stamp_) > timeout) {
      has_rgb_ = false;
    }
    if (has_depth_ && (now - latest_depth_stamp_) > timeout) {
      has_depth_ = false;
    }
    if (has_cinfo_ && (now - latest_cinfo_stamp_) > timeout) {
      has_cinfo_ = false;
    }

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
    cv::Scalar(0, 180, 120), cv::Scalar(10, 255, 255),
    cv::Scalar(170, 180, 120), cv::Scalar(180, 255, 255),
    "cube", cinfo);
  if (r.detected) return r;

  r = detect_color(hsv, depth,
    cv::Scalar(40, 80, 60), cv::Scalar(80, 255, 255),
    "cylinder", cinfo);
  if (r.detected) return r;

  r = detect_color(hsv, depth,
    cv::Scalar(110, 140, 80), cv::Scalar(130, 255, 255),
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

  // ─── Estimate 3D bbox from 2D contour + depth ───
  cv::Rect bbox = cv::boundingRect(*largest);
  result.bbox_2d = {static_cast<float>(bbox.x), static_cast<float>(bbox.y),
                    static_cast<float>(bbox.width), static_cast<float>(bbox.height)};

  // Collect valid depth samples inside the contour (surface)
  std::vector<float> surface_depths;
  surface_depths.reserve((bbox.area() + 3) / 4);
  for (int row = bbox.y; row < bbox.y + bbox.height; row += 2) {
    for (int col = bbox.x; col < bbox.x + bbox.width; col += 2) {
      if (row < 0 || row >= depth.rows || col < 0 || col >= depth.cols) continue;
      if (mask.at<uchar>(row, col) == 0) continue;
      float d = depth.at<float>(row, col);
      if (d > 0.0f && !std::isnan(d) && !std::isinf(d)) {
        surface_depths.push_back(d);
      }
    }
  }

  // Median surface depth (more robust than single-pixel centroid)
  float surface_depth = depth_val;
  if (!surface_depths.empty()) {
    std::nth_element(surface_depths.begin(),
                     surface_depths.begin() + surface_depths.size() / 2,
                     surface_depths.end());
    surface_depth = surface_depths[surface_depths.size() / 2];
  }

  result.position_3d.x = -(u - cx) * surface_depth / fx;
  result.position_3d.y = (v - cy) * surface_depth / fy;
  result.position_3d.z = surface_depth;
  result.confidence = std::min(1.0f, static_cast<float>(area) / 500.0f);

  RCLCPP_INFO(node_->get_logger(),
    "Detect '%s': pixel=(%d,%d) depth_centroid=%.4f depth_median=%.4f "
    "fx=%.1f fy=%.1f -> opt=(%+.3f, %+.3f, %.3f)",
    object_class.c_str(), u, v,
    depth_val, surface_depth, fx, fy,
    result.position_3d.x, result.position_3d.y, result.position_3d.z);

  // Estimate background (table) depth from a dilated border around the object
  cv::Mat dilated_mask;
  cv::dilate(mask, dilated_mask,
             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
  cv::Mat border_mask = dilated_mask - mask;

  std::vector<float> bg_depths;
  bg_depths.reserve(200);
  for (int row = bbox.y - 8; row < bbox.y + bbox.height + 8; row += 2) {
    for (int col = bbox.x - 8; col < bbox.x + bbox.width + 8; col += 2) {
      if (row < 0 || row >= depth.rows || col < 0 || col >= depth.cols) continue;
      if (border_mask.at<uchar>(row, col) == 0) continue;
      float d = depth.at<float>(row, col);
      if (d > 0.0f && !std::isnan(d) && !std::isinf(d)) {
        bg_depths.push_back(d);
      }
    }
  }

  float object_height = 0.02f;
  if (!bg_depths.empty()) {
    std::nth_element(bg_depths.begin(),
                     bg_depths.begin() + bg_depths.size() / 2,
                     bg_depths.end());
    float background_depth = bg_depths[bg_depths.size() / 2];
    object_height = background_depth - surface_depth;
    if (object_height < 0.005f) object_height = 0.02f;
  }

  // width  (optical x): pixel span * depth / fx
  float width_3d = bbox.width * surface_depth / fx;
  // extent (optical y): pixel span * depth / fy
  float extent_y_3d = bbox.height * surface_depth / fy;

  if (width_3d < 0.005f) width_3d = 0.01f;
  if (extent_y_3d < 0.005f) extent_y_3d = 0.01f;

  // bbox_size = {X_width, Y_extent, Z_height} in optical frame
  result.bbox_size = {width_3d, extent_y_3d, object_height};

  RCLCPP_DEBUG(node_->get_logger(),
    "bbox for '%s': wx=%.3f dy=%.3f hz=%.3f (2D: %dx%d surface=%d bg=%d)",
    object_class.c_str(),
    width_3d, extent_y_3d, object_height,
    bbox.width, bbox.height,
    static_cast<int>(surface_depths.size()),
    static_cast<int>(bg_depths.size()));

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
    cv::Scalar(0, 180, 120), cv::Scalar(10, 255, 255),
    cv::Scalar(170, 180, 120), cv::Scalar(180, 255, 255),
    "cube", cinfo);
  if (r.detected) results.push_back(r);

  r = detect_color(hsv, depth,
    cv::Scalar(40, 80, 60), cv::Scalar(80, 255, 255),
    "cylinder", cinfo);
  if (r.detected) results.push_back(r);

  r = detect_color(hsv, depth,
    cv::Scalar(110, 140, 80), cv::Scalar(130, 255, 255),
    "box", cinfo);
  if (r.detected) results.push_back(r);

  return results;
}

}  // namespace robograsp_vision_perception
