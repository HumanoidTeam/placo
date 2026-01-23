#include "placo/kinematics/manipulability_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
ManipulabilityTask::ManipulabilityTask(model::RobotWrapper::FrameIndex frame_index, Type type, double lambda)
  : frame_index(frame_index), lambda(lambda), type(type)
{
}

Eigen::MatrixXd ManipulabilityTask::mask_matrix(Eigen::MatrixXd M)
{
  if (type == POSITION)
  {
    return M.block(0, 6, 3, M.cols() - 6);
  }
  else if (type == ORIENTATION)
  {
    return M.block(3, 6, 3, M.cols() - 6);
  }
  else
  {
    return M.block(0, 6, 6, M.cols() - 6);
  }
}

void ManipulabilityTask::update()
{
  // Computing the Jacobian matrix
  Eigen::MatrixXd J_unmasked = solver->robot.frame_jacobian(frame_index, pinocchio::LOCAL);
  Eigen::MatrixXd J = mask_matrix(J_unmasked);

  if (joint_indices.size() > 0)
  {
    Eigen::MatrixXd J_sub(J.rows(), joint_indices.size());
    for (size_t i = 0; i < joint_indices.size(); i++)
    {
      if (joint_indices[i] >= 6 && (joint_indices[i] - 6) < J.cols())
      {
        J_sub.col(i) = J.col(joint_indices[i] - 6);
      }
      else
      {
        J_sub.col(i).setZero();
      }
    }
    J = J_sub;
  }

  Eigen::MatrixXd JJT_inv = (J * J.transpose()).inverse();
  manipulability = sqrt(fmax(0., (J * J.transpose()).determinant()));
  solver->robot.compute_hessians();

  Eigen::VectorXd manipulability_gradient;
  if (joint_indices.size() > 0)
  {
    manipulability_gradient.resize(joint_indices.size());
  }
  else
  {
    manipulability_gradient.resize(solver->N - 6);
  }
  manipulability_gradient.setZero();

  if (joint_indices.size() > 0)
  {
    for (size_t i = 0; i < joint_indices.size(); i++)
    {
      int dof = joint_indices[i];
      Eigen::MatrixXd H_dof_unmasked = solver->robot.get_frame_hessian(frame_index, dof);
      Eigen::MatrixXd H_dof = mask_matrix(H_dof_unmasked);
      Eigen::MatrixXd H_sub(H_dof.rows(), joint_indices.size());
      for (size_t j = 0; j < joint_indices.size(); j++)
      {
        if (joint_indices[j] >= 6 && (joint_indices[j] - 6) < H_dof.cols())
        {
          H_sub.col(j) = H_dof.col(joint_indices[j] - 6);
        }
        else
        {
          H_sub.col(j).setZero();
        }
      }
      Eigen::MatrixXd JH = J * H_sub.transpose();

      manipulability_gradient(i) = manipulability * JH.cwiseProduct(JJT_inv).sum();
    }

    A = Eigen::MatrixXd(joint_indices.size(), solver->N);
    A.setZero();
    for (size_t i = 0; i < joint_indices.size(); i++)
    {
      if (joint_indices[i] < solver->N)
      {
        A(i, joint_indices[i]) = lambda;
      }
    }
  }
  else
  {
    for (int dof = 6; dof < solver->N; dof++)
    {
      Eigen::MatrixXd H_dof_unmasked = solver->robot.get_frame_hessian(frame_index, dof);
      Eigen::MatrixXd H_dof = mask_matrix(H_dof_unmasked);
      Eigen::MatrixXd JH = J * H_dof.transpose();

      manipulability_gradient(dof - 6) = manipulability * JH.cwiseProduct(JJT_inv).sum();
    }

    // Regularization magnitude is handled through the task weight (see add_regularization_task)
    // Floating base is not regularized by this task
    Eigen::MatrixXd I = Eigen::MatrixXd(solver->N, solver->N);
    I.setIdentity();

    A = Eigen::MatrixXd(solver->N - 6, solver->N);
    A.block(0, 0, solver->N - 6, solver->N) = I.block(6, 0, solver->N - 6, solver->N) * lambda;
  }

  b = (1 / (2. * lambda)) * manipulability_gradient;

  if (minimize)
  {
    b = -b;
  }
}

std::string ManipulabilityTask::type_name()
{
  return "manipulability";
}

std::string ManipulabilityTask::error_unit()
{
  return "none";
}
}  // namespace placo::kinematics