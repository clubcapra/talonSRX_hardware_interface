//
// Created by ludovic on 1/14/20.
//
#ifndef SRC_TALON_SRX_HARDWARE_INTERFACE_H
#define SRC_TALON_SRX_HARDWARE_INTERFACE_H
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
#include <controller_manager/controller_manager.h>
#include "ros/ros.h"
// STD includes
#include <vector>
#include <string>

namespace talon_srx_hardware_interface {
    class talonSRXHardwareInterface : public hardware_interface::RobotHW {
    public:
        // Functions
        bool init(ros::NodeHandle &robot_hw_nh);

        void read(const ros::Time &time, const ros::Duration &period);

        void write(const ros::Time &time, const ros::Duration &period);

        // Interface variables
        std::string name_;
        double cmd_;
        double pos_;
        double vel_;
        double eff_;

    private:
        // Variables
        hardware_interface::VelocityJointInterface joint_velocity_interface_;
        hardware_interface::JointStateInterface joint_state_interface_;
    };
}
#endif //SRC_TALON_SRX_HARDWARE_INTERFACE_H
