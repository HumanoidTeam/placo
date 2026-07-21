#include "placo/kinematics/frame_task.h"

namespace placo::kinematics
{
FrameTask::FrameTask() : position(nullptr), orientation(nullptr)
{
}

FrameTask::FrameTask(PositionTask* position, OrientationTask* orientation)
  : position(position), orientation(orientation)
{
}

void FrameTask::configure(std::string name, std::string priority, double position_weight, double orientation_weight)
{
  position->configure(name + "_position", priority, position_weight);
  orientation->configure(name + "_orientation", priority, orientation_weight);
}

Eigen::Affine3d FrameTask::get_T_world_frame() const
{
  Eigen::Affine3d T;
  T.translation() = position->target_world;
  T.linear() = orientation->R_world_frame;
  return T;
}

void FrameTask::set_T_world_frame(Eigen::Affine3d T_world_frame)
{
  position->target_world = T_world_frame.translation();
  orientation->R_world_frame = T_world_frame.linear();
}

void FrameTask::exclude_dof(const std::string& dof)
{
  position->exclude_dof(dof);
  orientation->exclude_dof(dof);
}

void FrameTask::include_dof(const std::string& dof)
{
  position->include_dof(dof);
  orientation->include_dof(dof);
}

void FrameTask::clear_excluded_dofs()
{
  position->clear_excluded_dofs();
  orientation->clear_excluded_dofs();
}
}  // namespace placo::kinematics
