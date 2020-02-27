//
// Created by ludovic on 09/11/19.
//

#include "talon_srx_hardware_interface/talon_srx_hardware_interface.h"

namespace talon_srx_hardware_interface
{
bool talonSRXHardwareInterface::init(ros::NodeHandle &hw_nh)
{
  ROS_INFO("TEST READ");
  // Get parameters (joint name, joint type, talonsrx id, etc)
  // Get parameters
  std::vector<std::string> Joints;
  if (!hw_nh.getParam("joints", Joints))
  {
    return false;
  }
  motors_name = Joints[0];
  // Intialize raw data
  this->pos = 0.0;
  this->vel = 0.0;
  this->eff = 0.0;
  this->cmd = 0.0;

  // Initialize interface variables
  hardware_interface::JointStateHandle state_handle(this->motors_name, &pos, &vel, &eff);
  this->joint_state_interface.registerHandle(state_handle);

  hardware_interface::JointHandle vel_handle(joint_state_interface.getHandle(this->motors_name), &cmd);
  joint_velocity_interface.registerHandle(vel_handle);

  // Register interfaces
  registerInterface(&joint_state_interface);
  registerInterface(&joint_velocity_interface);
}  // namespace talon_srx_hardware_interface

void talonSRXHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
{
  ROS_INFO("TEST READ");
}

void talonSRXHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
{
  ROS_INFO("TEST WRITE");
}

}  // namespace talon_srx_hardware_interface

PLUGINLIB_EXPORT_CLASS(talon_srx_hardware_interface::talonSRXHardwareInterface, hardware_interface::RobotHW);
