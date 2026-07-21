#pragma once

#include <set>
#include <string>
#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"
#include "placo/tools/prioritized.h"
#include "placo/tools/utils.h"

namespace placo::kinematics
{
class KinematicsSolver;

/**
 * @brief Represents a task for the kinematics solver.
 *
 * A task is essentially a constraint of the form \f$ Ax = b \f$, where x is the vector of joint
 * delta positions solved by the kinematics solver.
 *
 * The task can be either an equality constraint (hard task) or an objective (soft task).
 *
 * See \ref placo::kinematics::KinematicsSolver
 */
class Task : public tools::Prioritized
{
public:
  /**
   * @brief Instance of kinematics solver
   */
  KinematicsSolver* solver = nullptr;

  /**
   * @brief true if this object memory is in the solver (it will be deleted by the solver)
   */
  bool solver_memory = false;

  /**
   * @brief Matrix A in the task Ax = b, where x are the joint delta positions
   */
  Eigen::MatrixXd A;

  /**
   * @brief Vector b in the task Ax = b, where x are the joint delta positions
   */
  Eigen::MatrixXd b;

  /**
   * @brief Update the task A and b matrices from the robot state and targets
   */
  virtual void update() = 0;

  /**
   * @brief Name of the task type
   * @return string representing the task type
   */
  virtual std::string type_name() = 0;

  /**
   * @brief Unit of the task error
   * @return string representing the task error unit
   */
  virtual std::string error_unit() = 0;

  /**
   * @brief Task errors (vector)
   * @return task errors
   */
  virtual Eigen::MatrixXd error();

  /**
   * @brief The task error norm
   * @return task error norm
   */
  virtual double error_norm();

  /**
   * @brief Excludes a degree of freedom (by joint name) from being used by this task.
   *
   * Unlike \ref KinematicsSolver::mask_dof, which is a solver-global hard constraint
   * (qd[dof] == 0), this is a task-local exclusion: the solver zeroes the corresponding
   * column of this task's \ref A matrix before building the QP. The task still reports the
   * same error \ref b, but it cannot reduce that error using the excluded DoF. Other tasks
   * remain free to move the DoF. This implements task-local DoF ownership.
   * @param dof the joint name to exclude
   */
  void exclude_dof(const std::string& dof);

  /**
   * @brief Re-includes a previously excluded degree of freedom (by joint name).
   * This is an idempotent no-op when the name is not currently excluded.
   * @param dof the joint name to include
   */
  void include_dof(const std::string& dof);

  /**
   * @brief Clears all task-local excluded degrees of freedom.
   */
  void clear_excluded_dofs();

  /**
   * @brief Returns the velocity offsets excluded from this task.
   * @return excluded velocity offsets
   */
  const std::set<int>& excluded_dofs() const;

protected:
  /**
   * @brief Velocity offsets excluded from this task (task-local DoF ownership).
   * Joint names are resolved when the exclusion set changes so solving does not repeat
   * model lookups and invalid names fail immediately.
   */
  std::set<int> excluded_dofs_;
};
}  // namespace placo::kinematics
