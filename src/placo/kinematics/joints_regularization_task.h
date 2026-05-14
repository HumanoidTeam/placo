#pragma once

#include <string>
#include <vector>
#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;

/**
 * @brief Velocity regularization task restricted to a list of joints.
 *
 * Same idea as RegularizationTask (penalises q̇² with a single magnitude),
 * but only applies to the joints in `joints` — leaves every other DoF
 * untouched. Useful when one specific joint (e.g. `base_yaw_joint`) is the
 * dominant source of unwanted motion and you don't want to dampen the rest
 * of the chain.
 */
struct JointsRegularizationTask : public Task
{
  /**
   * @brief Names of the joints this task regularises (velocity-only).
   */
  std::vector<std::string> joints;

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics
