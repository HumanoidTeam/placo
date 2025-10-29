#include "placo/kinematics/regularization_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void RegularizationTask::set_joint(std::string joint)
{
  joints[joint] = true;
}

void RegularizationTask::update()
{
  // Regularization magnitude is handled through the task weight (see add_regularization_task)
  // Floating base is not regularized by this task
  
  if (joints.empty())
  {
    // Default behavior: regularize all joints except floating base
    Eigen::MatrixXd I = Eigen::MatrixXd(solver->N, solver->N);
    I.setIdentity();

    A = Eigen::MatrixXd(solver->N - 6, solver->N);
    A.block(0, 0, solver->N - 6, solver->N) = I.block(6, 0, solver->N - 6, solver->N);

    b = Eigen::MatrixXd(solver->N - 6, 1);
    b.setZero();
  }
  else
  {
    // Selective regularization: only regularize specified joints
    A = Eigen::MatrixXd(joints.size(), solver->N);
    b = Eigen::MatrixXd(joints.size(), 1);
    A.setZero();
    b.setZero();

    int k = 0;
    for (auto& entry : joints)
    {
      A(k, solver->robot.get_joint_v_offset(entry.first)) = 1;
      k += 1;
    }
  }
}

std::string RegularizationTask::type_name()
{
  return "regularization";
}

std::string RegularizationTask::error_unit()
{
  return "none";
}
}  // namespace placo::kinematics