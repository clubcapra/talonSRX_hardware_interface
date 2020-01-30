//
// Created by ludovic on 09/11/19.
//

#include "talon_srx_hardware_interface/talon_srx_hardware_interface.h"

namespace talon_srx_hardware_interface {
    bool talonSRXHardwareInterface::init(ros::NodeHandle &hw_nh) {
        // Get parameters (joint name, joint type, talonsrx id, etc)
        // TODO Change so it can be more dynamic
        // front
        if (hw_nh.getParam("drives/drive_FL", this->FL))
            this->left_track.push_back(std::make_shared<TalonSRX>(this->FL));
        if (hw_nh.getParam("drives/drive_FR", this->FR))
            this->right_track.push_back(std::make_shared<TalonSRX>(this->FR));
        // rear
        if (hw_nh.getParam("drives/drive_RL", this->RL))
            this->left_track.push_back(std::make_shared<TalonSRX>(this->RL));
        if (hw_nh.getParam("drives/drive_RR", this->RR))
            this->right_track.push_back(std::make_shared<TalonSRX>(this->RR));

        // Intialize raw data
        std::fill_n(this->pos, MOTOR_COUNT, 0.0);
        std::fill_n(this->vel, MOTOR_COUNT, 0.0);
        std::fill_n(this->eff, MOTOR_COUNT, 0.0);
        std::fill_n(this->cmd, MOTOR_COUNT, 0.0);

        // Initialize interface variables
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            hardware_interface::JointStateHandle state_handle(this->motors_name[i], &pos[i], &vel[i], &eff[i]);
            this->joint_state_interface.registerHandle(state_handle);

            hardware_interface::JointHandle vel_handle(joint_state_interface.getHandle(this->motors_name[i]), &cmd[i]);
            joint_velocity_interface.registerHandle(vel_handle);
        }
        // Register interfaces
        registerInterface(&joint_state_interface);
        registerInterface(&joint_velocity_interface);
    }

    void talonSRXHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
        ROS_INFO("TEST READ");
    }

    void talonSRXHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
        ROS_INFO("TEST WRITE");
    }

}
