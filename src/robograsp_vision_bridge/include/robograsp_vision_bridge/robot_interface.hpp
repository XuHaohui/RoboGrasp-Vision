#pragma once

#include "robograsp_interfaces/msg/perception_result.hpp"

#include <memory>

namespace robograsp_vision_bridge
{

class RobotInterface
{
public:
  using SharedPtr = std::shared_ptr<RobotInterface>;

  virtual ~RobotInterface() = default;

  virtual bool send_object_info(
    const robograsp_interfaces::msg::PerceptionResult & perception) = 0;

  virtual void cancel() = 0;

  virtual bool is_ready() const = 0;
};

}  // namespace robograsp_vision_bridge
