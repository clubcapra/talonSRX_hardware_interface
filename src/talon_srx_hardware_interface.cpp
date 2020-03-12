//
// Created by ludovic on 09/11/19.
//
#include "hardware_interface/internal/interface_manager.h"
#include "talon_srx_hardware_interface/talon_srx_hardware_interface.h"
namespace talon_srx_hardware_interface
{
talonSRXHardwareInterface::talonSRXHardwareInterface()
{
}

talonSRXHardwareInterface::~talonSRXHardwareInterface()
{
}

bool talonSRXHardwareInterface::init(ros::NodeHandle &hw_nh)
{
  using namespace hardware_interface;
  // Get parameters (joint name, joint type, talonsrx id, etc)
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
  JointStateHandle state_handle(motors_name, &pos, &vel, &eff);
  joint_state_interface.registerHandle(state_handle);

  JointHandle vel_handle(velocity_joint_interface.getHandle(motors_name), &cmd);
  velocity_joint_interface.registerHandle(vel_handle);

  // Register interfaces
  registerInterface(&velocity_joint_interface);
  registerInterface(&joint_state_interface);

  return true;
}

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
