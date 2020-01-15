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
// STD includes
#include <vector>
#include <string>

using namespace std;
namespace talon_srx_hardware_interface {
    class TalonSRXHardwareInterface : public hardware_interface::RobotHW {
    public:
        bool init(ros::NodeHandle &nh, int id);
        void read(const ros::Time &time, const ros::Duration &period);
        void write(const ros::Time &time, const ros::Duration &period);

        string name_;
        double cmd_;
        double pos_;
        double vel_;
        double eff_;
    private:
        ros::NodeHandle nh;
        hardware_interface::VelocityJointInterface joint_velocity_interface;
        hardware_interface::PositionJointInterface joint_position_interface ;
        hardware_interface::JointStateInterface joint_state_interface;
        TalonSRX talon_srx;
    };
}  // namespace talon_srx_hardware_interface

#endif  // TAKIN_ROS_CONTROL_WS_TALONSRX_HARDWARE_H
