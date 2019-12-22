//
// Created by ludovic on 09/11/19.
//

#ifndef TALONSRX_HARDWARE_H
#define TALONSRX_HARDWARE_H

#include <dynamic_reconfigure/server.h>
#include "Platform-linux-socket-can.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "geometry_msgs/Twist.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Temperature.h"

namespace talon_srx_hardware_interface
{
class TalonSRXHardwareInterface : public hardware_interface::RobotHW
{
public:
  bool init(ros::NodeHandle &nh);

private:
  std::unique_ptr<TalonSRX> left_track;
  std::unique_ptr<TalonSRX> right_track;
};
}  // namespace talon_srx_hardware_interface

#endif  // TAKIN_ROS_CONTROL_WS_TALONSRX_HARDWARE_H
