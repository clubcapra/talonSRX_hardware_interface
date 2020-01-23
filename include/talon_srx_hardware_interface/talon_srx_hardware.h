//
// Created by ludovic on 09/11/19.
//

#ifndef TALONSRX_HARDWARE_H
#define TALONSRX_HARDWARE_H
// CTRE includes
#include <dynamic_reconfigure/server.h>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "Platform-linux-socket-can.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
// ROS includes
#include "geometry_msgs/Twist.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Temperature.h"
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
// STD includes
#include <vector>
#include <string>

using namespace std;
namespace talon_srx_hardware_interface {
    class TalonSRXHardware : public hardware_interface::RobotHW {
    protected:
        ros::NodeHandle nh;
        // interfaces
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::EffortJointInterface effort_joint_interface_;

        joint_limits_interface::EffortJointSaturationInterface effort_joint_saturation_interface_;
        joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_limits_interface_;
        joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
        joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;
        joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;
        joint_limits_interface::VelocityJointSoftLimitsInterface velocity_joint_limits_interface_;


        int num_joints_;
        int joint_mode_; // position, velocity, or effort
        std::vector<std::string> joint_names_;
        std::vector<int> joint_types_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_effort_command_;
        std::vector<double> joint_lower_limits_;
        std::vector<double> joint_upper_limits_;
        std::vector<double> joint_effort_limits_;

    };
}  // namespace talon_srx_hardware_interface

#endif  // TAKIN_ROS_CONTROL_WS_TALONSRX_HARDWARE_H
