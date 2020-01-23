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
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Temperature.h"
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include "talon_srx_hardware.h"
// STD includes
#include <vector>
#include <string>


using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace talon_srx_hardware_interface {
    static const double POSITION_STEP_FACTOR = 10;
    static const double VELOCITY_STEP_FACTOR = 10;

    class talonSRXHardwareInterface : public talon_srx_hardware_interface::TalonSRXHardware {
    public:

        talonSRXHardwareInterface(ros::NodeHandle& nh);

        ~talonSRXHardwareInterface();

        void init();

        void update(const ros::TimerEvent &e);

        void read();

        void write(ros::Duration elapsed_time);

    protected:
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration control_period_;
        ros::Duration elapsed_time_;
        PositionJointInterface positionJointInterface;
        PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
        double loop_hz_;
        std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    };
}
#endif //SRC_TALON_SRX_HARDWARE_INTERFACE_H
