#pragma once

#include "robograsp_vision_perception/camera_detector.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <opencv2/core.hpp>

#include <vector>
#include <string>

namespace robograsp_vision_perception
{

struct Cluster3D
{
  cv::Point3f median_position;
  cv::Point3f extent;
  float surface_area{0.0f};
  float depth_stddev{0.0f};
  float depth_gradient{0.0f};
  float circularity{0.0f};
  float aspect_ratio{1.0f};
  float obj_height{0.0f};
  int valid_pixels{0};
  int bbox_pixels{0};
};

class DepthFirstGeometricDetector : public CameraDetector
{
public:
  explicit DepthFirstGeometricDetector(rclcpp::Node * node);

protected:
  DetectionResult detect_impl(
      const cv::Mat & hsv, const cv::Mat & depth,
      const sensor_msgs::msg::CameraInfo & cinfo) override;

  std::vector<DetectionResult> detect_all_impl(
      const cv::Mat & hsv, const cv::Mat & depth,
      const sensor_msgs::msg::CameraInfo & cinfo) override;

private:
  float calibrate_table(const cv::Mat & depth);
  cv::Mat compute_height_map(const cv::Mat & depth, float table_depth);
  cv::Mat extract_foreground(const cv::Mat & height);
  std::vector<cv::Mat> extract_clusters(const cv::Mat & binary);
  Cluster3D analyze_cluster(
      const cv::Mat & mask, const cv::Mat & depth,
      float fx, float fy, float cx, float cy);
  std::string classify_by_color(const cv::Mat & mask, const cv::Mat & hsv);
  float compute_confidence(const Cluster3D & cluster);

  DetectionResult build_result(
      const Cluster3D & cluster,
      const std::string & object_class,
      const cv::Rect & bbox_2d);

  float table_depth_{0.0f};
  bool table_calibrated_{false};

  float table_margin_{0.015f};
  float table_calib_roi_ratio_{0.40f};
  int min_cluster_pixels_{50};
  float circularity_thresh_{0.82f};
  float aspect_ratio_thresh_{1.35f};
  float depth_stddev_thresh_{0.004f};
  float depth_gradient_thresh_{0.0005f};
  float min_height_{0.005f};

  cv::Scalar hsv_cube_lower1_;
  cv::Scalar hsv_cube_upper1_;
  cv::Scalar hsv_cube_lower2_;
  cv::Scalar hsv_cube_upper2_;
  cv::Scalar hsv_cylinder_lower_;
  cv::Scalar hsv_cylinder_upper_;
  cv::Scalar hsv_box_lower_;
  cv::Scalar hsv_box_upper_;
};

}  // namespace robograsp_vision_perception
