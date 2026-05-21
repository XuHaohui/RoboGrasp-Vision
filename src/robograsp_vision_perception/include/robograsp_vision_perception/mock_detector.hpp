#pragma once

#include "robograsp_vision_perception/detector.hpp"
#include <vector>
#include <string>

namespace robograsp_vision_perception
{

class MockDetector : public ObjectDetector
{
public:
  MockDetector(
    const std::string & default_object = "cube",
    double x = 0.35, double y = 0.05, double z = 0.02);

  DetectionResult detect() override;

  struct ObjectDef
  {
    std::string name;
    double x, y, z;
    float confidence;
  };

  DetectionResult detect_by_index(size_t index);

  size_t object_count() const { return objects_.size(); }

  const ObjectDef & object_at(size_t i) const { return objects_[i]; }

  void stop() override {}

private:
  std::vector<ObjectDef> objects_;
  size_t counter_{0};
};

}  // namespace robograsp_vision_perception
