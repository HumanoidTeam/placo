#include "placo/kinematics/joints_regularization_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void JointsRegularizationTask::update()
{
  // Each row of A picks out one joint's velocity DoF; b is zero so the task
  // penalises ‖q̇_subset‖² with the configured weight.
  int n = static_cast<int>(joints.size());
  A = Eigen::MatrixXd::Zero(n, solver->N);
  b = Eigen::MatrixXd::Zero(n, 1);
  for (int i = 0; i < n; ++i)
  {
    A(i, solver->robot.get_joint_v_offset(joints[i])) = 1.0;
  }
}

std::string JointsRegularizationTask::type_name()
{
  return "joints_regularization";
}

std::string JointsRegularizationTask::error_unit()
{
  return "none";
}
}  // namespace placo::kinematics
