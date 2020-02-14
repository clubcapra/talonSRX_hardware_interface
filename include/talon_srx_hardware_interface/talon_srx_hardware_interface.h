//
// Created by ludovic on 1/14/20.
//
#ifndef SRC_TALON_SRX_HARDWARE_INTERFACE_H
#define SRC_TALON_SRX_HARDWARE_INTERFACE_H
// CTRE includes
#include <dynamic_reconfigure/server.h>
#include "Platform-linux-socket-can.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
// ROS includes
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include "geometry_msgs/Twist.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
// STD includes
#include <string>
#include <vector>

// Robot variable
const int8_t MOTOR_COUNT = 4;  // TODO Change it so it loads from a config file.

namespace talon_srx_hardware_interface
{
class talonSRXHardwareInterface : public hardware_interface::RobotHW
{
public:
  // Functions
  bool init(ros::NodeHandle &robot_hw_nh);

  void read(const ros::Time &time, const ros::Duration &period);

  void write(const ros::Time &time, const ros::Duration &period);

  // Interface variables
  std::vector<std::string> motors_name = { "drive_FL", "drive_FR", "drive_RL",
                                           "drive_RR" };  // TODO Change it so it loads from a config file.
  int FL = 0, FR = 0, RL = 0, RR = 0;
  double cmd[MOTOR_COUNT];
  double pos[MOTOR_COUNT];
  double vel[MOTOR_COUNT];
  double eff[MOTOR_COUNT];

private:
  // Variables
  hardware_interface::VelocityJointInterface joint_velocity_interface;
  hardware_interface::JointStateInterface joint_state_interface;
  std::vector<std::shared_ptr<TalonSRX>> left_track;
  std::vector<std::shared_ptr<TalonSRX>> right_track;
};
}  // namespace talon_srx_hardware_interface
#endif  // SRC_TALON_SRX_HARDWARE_INTERFACE_H
