#include "placo/kinematics/acceleration_regularization_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void AccelerationRegularizationTask::update()
{
  // We want to minimize ||qdd||^2 where qdd = (qd_new - qd_prev) / dt
  // Since qd_new = delta_q / dt, we have:
  // qdd = (delta_q / dt - qd_prev) / dt = (delta_q - qd_prev * dt) / dt^2
  // 
  // To minimize ||qdd||^2, we minimize ||(delta_q - qd_prev * dt) / dt^2||^2
  // This translates to the QP form: minimize ||A * delta_q - b||^2
  // where A = I / dt^2 and b = qd_prev / dt
  
  if (solver->dt == 0.)
  {
    throw std::runtime_error("AccelerationRegularizationTask::update: you should set solver.dt");
  }

  // Identity matrix scaled by 1/dt^2
  // Skip the first 6 DOFs (floating base) like the standard regularization task
  int n_dofs = solver->N - 6;
  
  A = Eigen::MatrixXd(n_dofs, solver->N);
  A.setZero();
  
  // Set diagonal block for non-floating-base DOFs
  double dt_squared = solver->dt * solver->dt;
  for (int i = 0; i < n_dofs; i++)
  {
    A(i, i + 6) = 1.0 / dt_squared;
  }
  
  // Target: qd_prev / dt (previous velocity scaled by dt)
  b = Eigen::VectorXd(n_dofs);
  
  // Extract previous velocities from robot state (skip floating base)
  for (int i = 0; i < n_dofs; i++)
  {
    b(i) = solver->robot.state.qd(i + 6) / solver->dt;
  }
}

std::string AccelerationRegularizationTask::type_name()
{
  return "acceleration_regularization";
}

std::string AccelerationRegularizationTask::error_unit()
{
  return "rad/s^2";
}
}  // namespace placo::kinematics 