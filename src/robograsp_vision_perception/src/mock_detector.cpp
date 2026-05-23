#include "robograsp_vision_perception/mock_detector.hpp"

namespace robograsp_vision_perception
{

MockDetector::MockDetector(
  const std::string & default_object,
  double x, double y, double z)
{
  (void)default_object; (void)x; (void)y; (void)z;
  objects_ = {
    {"cube",     0.35,  0.05, 0.2, 0.85f},
    {"cylinder", 0.35, -0.10, 0.2, 0.90f},
    {"box",      0.25,  0.10, 0.2, 0.95f},
  };
}

DetectionResult MockDetector::detect()
{
  size_t idx = counter_ % objects_.size();
  counter_++;

  const auto & obj = objects_[idx];

  DetectionResult r;
  r.detected = true;
  r.object_class = obj.name;
  r.confidence = obj.confidence;

  r.position_3d.x = obj.x;
  r.position_3d.y = obj.y;
  r.position_3d.z = obj.z;

  r.bbox_size = {0.04f, 0.04f, 0.04f};
  r.bbox_2d = {100.0f, 150.0f, 80.0f, 80.0f};

  return r;
}

DetectionResult MockDetector::detect_by_index(size_t index)
{
  DetectionResult r;
  r.bbox_size = {0.04f, 0.04f, 0.04f};
  r.bbox_2d = {100.0f, 150.0f, 80.0f, 80.0f};

  if (index >= objects_.size()) {
    r.detected = false;
    r.object_class = "unknown";
    r.confidence = 0.0f;
    return r;
  }

  const auto & obj = objects_[index];
  r.detected = true;
  r.object_class = obj.name;
  r.confidence = obj.confidence;
  r.position_3d.x = obj.x;
  r.position_3d.y = obj.y;
  r.position_3d.z = obj.z;

  return r;
}

}  // namespace robograsp_vision_perception
