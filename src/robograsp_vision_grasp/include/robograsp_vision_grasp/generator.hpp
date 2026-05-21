#pragma once

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "robograsp_interfaces/msg/grasp_command.hpp"
#include "robograsp_interfaces/msg/perception_result.hpp"

#include <memory>
#include <string>
#include <vector>

namespace robograsp_vision_grasp
{

inline geometry_msgs::msg::Quaternion make_identity_quaternion()
{
  geometry_msgs::msg::Quaternion q;
  q.w = 1.0;
  return q;
}

class GraspGenerator
{
public:
  using SharedPtr = std::shared_ptr<GraspGenerator>;

  virtual ~GraspGenerator() = default;

  virtual robograsp_interfaces::msg::GraspCommand::SharedPtr generate(
    const robograsp_interfaces::msg::PerceptionResult::SharedPtr & perception) = 0;

  virtual std::vector<robograsp_interfaces::msg::GraspCommand::SharedPtr>
  generate_candidates(
    const robograsp_interfaces::msg::PerceptionResult::SharedPtr & perception,
    size_t num_candidates = 5) = 0;
};

class SimpleTopDownGenerator : public GraspGenerator
{
public:
  SimpleTopDownGenerator(
    double approach_offset = 0.10,
    double grasp_width = 0.04,
    const std::string & world_frame = "world");

  robograsp_interfaces::msg::GraspCommand::SharedPtr generate(
    const robograsp_interfaces::msg::PerceptionResult::SharedPtr & perception) override;

  std::vector<robograsp_interfaces::msg::GraspCommand::SharedPtr>
  generate_candidates(
    const robograsp_interfaces::msg::PerceptionResult::SharedPtr & perception,
    size_t num_candidates = 5) override;

private:
  double approach_offset_;
  double grasp_width_;
  std::string world_frame_;
};

}  // namespace robograsp_vision_grasp
