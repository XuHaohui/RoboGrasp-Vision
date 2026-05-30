#include "robograsp_vision_perception/depth_geometry_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace robograsp_vision_perception
{

DepthFirstGeometricDetector::DepthFirstGeometricDetector(rclcpp::Node * node)
  : CameraDetector(node)
{
  node_->declare_parameter("table.margin", 0.015f);
  node_->declare_parameter("table.calib_roi_ratio", 0.40f);
  node_->declare_parameter("cluster.min_pixels", 50);
  node_->declare_parameter("shape.circularity_thresh", 0.82f);
  node_->declare_parameter("shape.aspect_ratio_thresh", 1.35f);
  node_->declare_parameter("shape.depth_stddev_thresh", 0.004f);
  node_->declare_parameter("shape.depth_gradient_thresh", 0.0005f);
  node_->declare_parameter("shape.min_height", 0.005f);

  table_margin_ = node_->get_parameter("table.margin").as_double();
  table_calib_roi_ratio_ = node_->get_parameter("table.calib_roi_ratio").as_double();
  min_cluster_pixels_ = node_->get_parameter("cluster.min_pixels").as_int();
  circularity_thresh_ = node_->get_parameter("shape.circularity_thresh").as_double();
  aspect_ratio_thresh_ = node_->get_parameter("shape.aspect_ratio_thresh").as_double();
  depth_stddev_thresh_ = node_->get_parameter("shape.depth_stddev_thresh").as_double();
  depth_gradient_thresh_ = node_->get_parameter("shape.depth_gradient_thresh").as_double();
  min_height_ = node_->get_parameter("shape.min_height").as_double();

  auto load_hsv = [this](const std::string & name,
                          const std::vector<int64_t> & defaults,
                          cv::Scalar & scalar) {
    node_->declare_parameter(name, defaults);
    auto v = node_->get_parameter(name).as_integer_array();
    scalar = cv::Scalar(static_cast<double>(v[0]),
                        static_cast<double>(v[1]),
                        static_cast<double>(v[2]));
  };

  load_hsv("hsv.cube.lower1",   {0,   30,  40}, hsv_cube_lower1_);
  load_hsv("hsv.cube.upper1",   {10,  255, 255}, hsv_cube_upper1_);
  load_hsv("hsv.cube.lower2",   {170, 30,  40}, hsv_cube_lower2_);
  load_hsv("hsv.cube.upper2",   {180, 255, 255}, hsv_cube_upper2_);
  load_hsv("hsv.cylinder.lower", {20,  40,  40}, hsv_cylinder_lower_);
  load_hsv("hsv.cylinder.upper", {40,  255, 255}, hsv_cylinder_upper_);
  load_hsv("hsv.box.lower",      {100, 60,  40}, hsv_box_lower_);
  load_hsv("hsv.box.upper",      {140, 255, 255}, hsv_box_upper_);
}

float DepthFirstGeometricDetector::calibrate_table(const cv::Mat & depth)
{
  int cx = depth.cols / 2;
  int cy = depth.rows / 2;
  int w = static_cast<int>(depth.cols * table_calib_roi_ratio_);
  int h = static_cast<int>(depth.rows * table_calib_roi_ratio_);
  cv::Rect roi(cx - w / 2, cy - h / 2, w, h);

  std::vector<float> samples;
  samples.reserve((roi.area() + 3) / 4);
  for (int r = roi.y; r < roi.y + roi.height; r += 2) {
    for (int c = roi.x; c < roi.x + roi.width; c += 2) {
      if (r < 0 || r >= depth.rows || c < 0 || c >= depth.cols) continue;
      float d = depth.at<float>(r, c);
      if (d > 0.1f && d < 5.0f && !std::isnan(d) && !std::isinf(d)) {
        samples.push_back(d);
      }
    }
  }

  if (samples.empty()) return 1.0f;

  size_t n = samples.size() / 2;
  std::nth_element(samples.begin(), samples.begin() + n, samples.end());
  return samples[n];
}

cv::Mat DepthFirstGeometricDetector::compute_height_map(
    const cv::Mat & depth, float table_depth)
{
  cv::Mat height(depth.size(), CV_32FC1, cv::Scalar(0.0f));
  for (int r = 0; r < depth.rows; ++r) {
    for (int c = 0; c < depth.cols; ++c) {
      float d = depth.at<float>(r, c);
      if (d > 0.0f && d < table_depth) {
        height.at<float>(r, c) = table_depth - d;
      }
    }
  }
  return height;
}

cv::Mat DepthFirstGeometricDetector::extract_foreground(const cv::Mat & height)
{
  cv::Mat binary;
  cv::threshold(height, binary, table_margin_, 255.0, cv::THRESH_BINARY);
  binary.convertTo(binary, CV_8U);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
  return binary;
}

std::vector<cv::Mat> DepthFirstGeometricDetector::extract_clusters(
    const cv::Mat & binary)
{
  cv::Mat labels, stats, centroids;
  int n = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8);

  std::vector<cv::Mat> masks;
  for (int i = 1; i < n; ++i) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < min_cluster_pixels_) continue;
    cv::Mat mask = (labels == i);
    masks.push_back(mask);
  }
  return masks;
}

Cluster3D DepthFirstGeometricDetector::analyze_cluster(
    const cv::Mat & mask, const cv::Mat & depth,
    float fx, float fy, float cx, float cy)
{
  Cluster3D result;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) return result;

  const auto & cnt = *std::max_element(contours.begin(), contours.end(),
    [](const auto & a, const auto & b) {
      return cv::contourArea(a) < cv::contourArea(b);
    });

  double area = cv::contourArea(cnt);
  double perimeter = cv::arcLength(cnt, true);
  result.circularity = (perimeter > 0.0)
    ? static_cast<float>(4.0 * M_PI * area / (perimeter * perimeter)) : 0.0f;

  cv::Rect bbox = cv::boundingRect(cnt);
  float bbox_ar = static_cast<float>(bbox.width) / static_cast<float>(bbox.height);
  result.aspect_ratio = std::max(bbox_ar, 1.0f / bbox_ar);
  result.surface_area = static_cast<float>(area);
  result.bbox_pixels = bbox.area();

  std::vector<float> xs, ys, zs;
  double xs_alloc = std::min(area, 5000.0);
  xs.reserve(static_cast<size_t>(xs_alloc));
  ys.reserve(static_cast<size_t>(xs_alloc));
  zs.reserve(static_cast<size_t>(xs_alloc));

  for (int r = bbox.y; r < bbox.y + bbox.height; ++r) {
    for (int c = bbox.x; c < bbox.x + bbox.width; ++c) {
      if (r < 0 || r >= mask.rows || c < 0 || c >= mask.cols) continue;
      if (mask.at<uchar>(r, c) == 0) continue;
      float d = depth.at<float>(r, c);
      if (d <= 0.0f || std::isnan(d) || std::isinf(d)) continue;

      float x = (static_cast<float>(c) - cx) * d / fx;
      float y = (static_cast<float>(r) - cy) * d / fy;
      float z = d;
      xs.push_back(x);
      ys.push_back(y);
      zs.push_back(z);
    }
  }

  if (xs.empty()) return result;

  result.valid_pixels = static_cast<int>(xs.size());

  auto median = [](std::vector<float> & v) -> float {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
  };
  result.median_position = cv::Point3f(median(xs), median(ys), median(zs));

  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  std::sort(zs.begin(), zs.end());
  int lo = static_cast<int>(xs.size() * 0.025);
  int hi = static_cast<int>(xs.size() * 0.975);
  result.extent = cv::Point3f(
    xs[hi] - xs[lo],
    ys[hi] - ys[lo],
    zs[hi] - zs[lo]);

  float mean_z = 0.0f;
  for (float z : zs) mean_z += z;
  mean_z /= static_cast<float>(zs.size());
  float var = 0.0f;
  for (float z : zs) {
    float dz = z - mean_z;
    var += dz * dz;
  }
  result.depth_stddev = std::sqrt(var / static_cast<float>(zs.size()));

  // Depth gradient: higher for curved surfaces (cylinder) than flat (cube/box)
  cv::Mat depth_roi = depth(bbox);
  cv::Mat mask_roi = mask(bbox);
  cv::Mat grad_x, grad_y;
  cv::Sobel(depth_roi, grad_x, CV_32F, 1, 0, 3);
  cv::Sobel(depth_roi, grad_y, CV_32F, 0, 1, 3);
  float grad_sum = 0.0f;
  int grad_count = 0;
  for (int r = 0; r < bbox.height; ++r) {
    for (int c = 0; c < bbox.width; ++c) {
      if (mask_roi.at<uchar>(r, c) == 0) continue;
      float d = depth_roi.at<float>(r, c);
      if (d <= 0.0f) continue;
      grad_sum += std::abs(grad_x.at<float>(r, c)) + std::abs(grad_y.at<float>(r, c));
      grad_count++;
    }
  }
  result.depth_gradient = (grad_count > 0) ? (grad_sum / static_cast<float>(grad_count)) : 0.0f;

  return result;
}

std::string DepthFirstGeometricDetector::classify_by_color(
    const cv::Mat & mask, const cv::Mat & hsv)
{
  auto count_hits = [&](const cv::Scalar & lower, const cv::Scalar & upper) {
    cv::Mat tmp;
    cv::inRange(hsv, lower, upper, tmp);
    cv::Mat masked;
    cv::bitwise_and(tmp, mask, masked);
    return cv::countNonZero(masked);
  };

  int red_hits = count_hits(hsv_cube_lower1_, hsv_cube_upper1_)
               + count_hits(hsv_cube_lower2_, hsv_cube_upper2_);
  int yellow_hits = count_hits(hsv_cylinder_lower_, hsv_cylinder_upper_);
  int blue_hits = count_hits(hsv_box_lower_, hsv_box_upper_);

  RCLCPP_DEBUG(node_->get_logger(),
    "ColorClassify: red=%d yellow=%d blue=%d",
    red_hits, yellow_hits, blue_hits);

  if (red_hits == 0 && yellow_hits == 0 && blue_hits == 0) {
    return "object";
  }

  if (yellow_hits >= red_hits && yellow_hits >= blue_hits) return "cylinder";
  if (red_hits >= blue_hits) return "cube";
  return "box";
}

float DepthFirstGeometricDetector::compute_confidence(const Cluster3D & cluster)
{
  float coverage = (cluster.bbox_pixels > 0)
    ? static_cast<float>(cluster.valid_pixels) / static_cast<float>(cluster.bbox_pixels)
    : 0.0f;
  float size_score = std::min(1.0f, cluster.surface_area / 200.0f);
  float depth_valid = (coverage > 0.6f) ? 1.0f : (coverage / 0.6f);
  return std::min(1.0f, 0.35f * size_score + 0.35f * coverage + 0.30f * depth_valid);
}

DetectionResult DepthFirstGeometricDetector::build_result(
    const Cluster3D & cluster,
    const std::string & object_class,
    const cv::Rect & bbox_2d)
{
  DetectionResult result;
  result.detected = true;
  result.object_class = object_class;
  result.confidence = compute_confidence(cluster);

  result.position_3d.x = cluster.median_position.x;
  result.position_3d.y = cluster.median_position.y;
  result.position_3d.z = cluster.median_position.z;

  float height = (cluster.obj_height > min_height_) ? cluster.obj_height : 0.02f;
  result.bbox_size = {
    std::max(cluster.extent.x, 0.01f),
    std::max(cluster.extent.y, 0.01f),
    height
  };

  result.bbox_2d = {
    static_cast<float>(bbox_2d.x),
    static_cast<float>(bbox_2d.y),
    static_cast<float>(bbox_2d.width),
    static_cast<float>(bbox_2d.height)
  };

  RCLCPP_DEBUG(node_->get_logger(),
    "Detect '%s': median_opt=(%+.3f, %+.3f, %.3f) "
    "extent=(%.3f, %.3f, %.3f) height=%.3f "
    "circ=%.3f ar=%.3f grad=%.6f px=%d conf=%.2f",
    object_class.c_str(),
    result.position_3d.x, result.position_3d.y, result.position_3d.z,
    cluster.extent.x, cluster.extent.y, cluster.extent.z,
    height,
    cluster.circularity, cluster.aspect_ratio, cluster.depth_gradient,
    cluster.valid_pixels, result.confidence);

  return result;
}

DetectionResult DepthFirstGeometricDetector::detect_impl(
    const cv::Mat & /*hsv*/, const cv::Mat & depth,
    const sensor_msgs::msg::CameraInfo & cinfo)
{
  auto results = detect_all_impl(cv::Mat(), depth, cinfo);
  if (!results.empty()) return results[0];

  DetectionResult no;
  no.detected = false;
  return no;
}

std::vector<DetectionResult> DepthFirstGeometricDetector::detect_all_impl(
    const cv::Mat & hsv, const cv::Mat & depth,
    const sensor_msgs::msg::CameraInfo & cinfo)
{
  std::vector<DetectionResult> results;

  float fx = static_cast<float>(cinfo.k[0]);
  float fy = static_cast<float>(cinfo.k[4]);
  float cx = static_cast<float>(cinfo.k[2]);
  float cy = static_cast<float>(cinfo.k[5]);

  if (!table_calibrated_) {
    table_depth_ = calibrate_table(depth);
    table_calibrated_ = true;
    RCLCPP_INFO(node_->get_logger(),
      "Table depth calibrated: %.3f m (optical frame)", table_depth_);
  }

  cv::Mat height = compute_height_map(depth, table_depth_);
  cv::Mat binary = extract_foreground(height);
  auto masks = extract_clusters(binary);

  for (size_t i = 0; i < masks.size(); ++i) {
    auto cluster = analyze_cluster(masks[i], depth, fx, fy, cx, cy);
    if (cluster.valid_pixels < min_cluster_pixels_) continue;

    cluster.obj_height = table_depth_ - cluster.median_position.z;
    if (cluster.obj_height < 0.0f) cluster.obj_height = 0.0f;

    auto object_class = classify_by_color(masks[i], hsv);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(masks[i], contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Rect bbox_2d;
    if (!contours.empty()) {
      bbox_2d = cv::boundingRect(
        *std::max_element(contours.begin(), contours.end(),
          [](const auto & a, const auto & b) {
            return cv::contourArea(a) < cv::contourArea(b);
          }));
    }

    // Skip clusters that touch the image border (edge artifacts)
    const int edge_margin = 10;
    if (bbox_2d.x <= edge_margin || bbox_2d.y <= edge_margin ||
        bbox_2d.x + bbox_2d.width >= depth.cols - edge_margin ||
        bbox_2d.y + bbox_2d.height >= depth.rows - edge_margin) {
      RCLCPP_DEBUG(node_->get_logger(),
        "Skipping edge cluster at (%d,%d) %dx%d",
        bbox_2d.x, bbox_2d.y, bbox_2d.width, bbox_2d.height);
      continue;
    }

    results.push_back(build_result(cluster, object_class, bbox_2d));
  }

  RCLCPP_INFO_ONCE(node_->get_logger(),
    "DepthGeometry: depth-first detector active (table=%.3f m)", table_depth_);

  return results;
}

}  // namespace robograsp_vision_perception
