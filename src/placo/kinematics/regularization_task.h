#pragma once

#include "placo/kinematics/task.h"
#include <map>
#include <string>

namespace placo::kinematics
{
class KinematicsSolver;
struct RegularizationTask : public Task
{
  /**
   * @brief Joint names to regularize (empty = regularize all joints except floating base)
   */
  std::map<std::string, bool> joints;

  /**
   * @brief Sets a joint to be regularized
   * @param joint joint name
   */
  void set_joint(std::string joint);

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
}  // namespace placo::kinematics