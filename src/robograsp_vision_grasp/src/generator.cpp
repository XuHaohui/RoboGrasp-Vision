#include "robograsp_vision_grasp/generator.hpp"

namespace robograsp_vision_grasp
{

SimpleTopDownGenerator::SimpleTopDownGenerator(
  double approach_offset, double grasp_width, const std::string & world_frame)
: approach_offset_(approach_offset)
, grasp_width_(grasp_width)
, world_frame_(world_frame)
{
}

robograsp_interfaces::msg::GraspCommand::SharedPtr
SimpleTopDownGenerator::generate(
  const robograsp_interfaces::msg::PerceptionResult::SharedPtr & perception)
{
  auto cmd = std::make_shared<robograsp_interfaces::msg::GraspCommand>();

  cmd->header.frame_id = world_frame_;
  cmd->object_class = perception->object_class;
  cmd->quality = perception->confidence;

  cmd->grasp_pose.header.frame_id = world_frame_;
  cmd->grasp_pose.pose.position.x = perception->position.x;
  cmd->grasp_pose.pose.position.y = perception->position.y;
  cmd->grasp_pose.pose.position.z = perception->position.z;
  cmd->grasp_pose.pose.orientation = make_identity_quaternion();

  cmd->pre_grasp_pose.header.frame_id = world_frame_;
  cmd->pre_grasp_pose.pose.position.x = perception->position.x;
  cmd->pre_grasp_pose.pose.position.y = perception->position.y;
  cmd->pre_grasp_pose.pose.position.z = perception->position.z + approach_offset_;
  cmd->pre_grasp_pose.pose.orientation = make_identity_quaternion();

  cmd->grasp_width = grasp_width_;
  cmd->approach_direction = {0.0f, 0.0f, -1.0f};
  cmd->strategy = "top";
  cmd->use_cartesian_approach = true;

  cmd->object_position = perception->position;

  return cmd;
}

std::vector<robograsp_interfaces::msg::GraspCommand::SharedPtr>
SimpleTopDownGenerator::generate_candidates(
  const robograsp_interfaces::msg::PerceptionResult::SharedPtr & perception,
  size_t num_candidates)
{
  (void)num_candidates;
  auto cmd = generate(perception);
  if (!cmd) return {};
  return {cmd};
}

}  // namespace robograsp_vision_grasp
