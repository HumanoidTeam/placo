#include "placo/kinematics/task.h"

#include <stdexcept>

#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
Eigen::MatrixXd Task::error()
{
  return b;
}

double Task::error_norm()
{
  return b.norm();
}

void Task::exclude_dof(const std::string& dof)
{
  if (solver == nullptr)
  {
    throw std::runtime_error("Task must belong to a solver before excluding a DoF");
  }
  excluded_dofs_.insert(solver->robot.get_joint_v_offset(dof));
}

void Task::include_dof(const std::string& dof)
{
  if (solver == nullptr)
  {
    throw std::runtime_error("Task must belong to a solver before including a DoF");
  }
  excluded_dofs_.erase(solver->robot.get_joint_v_offset(dof));
}

void Task::clear_excluded_dofs()
{
  excluded_dofs_.clear();
}

const std::set<int>& Task::excluded_dofs() const
{
  return excluded_dofs_;
}
}  // namespace placo::kinematics
