#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;

/**
 * @brief Acceleration regularization task
 * 
 * This task minimizes the joint accelerations by penalizing the change in velocity.
 * It minimizes ||qdd||^2 where qdd = (qd_new - qd_prev) / dt
 * 
 * This is achieved by minimizing ||delta_q - qd_prev * dt||^2 in the QP problem.
 */
struct AccelerationRegularizationTask : public Task
{
  /**
   * @brief Update the task matrices
   */
  virtual void update();
  
  /**
   * @brief Type name of the task
   */
  virtual std::string type_name();
  
  /**
   * @brief Error unit of the task
   */
  virtual std::string error_unit();
};
}  // namespace placo::kinematics 