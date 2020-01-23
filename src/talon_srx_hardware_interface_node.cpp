//
// Created by ludovic on 1/14/20.
//

#include "talon_srx_hardware_interface/talon_srx_hardware_interface.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "talon_srx_hardware_interface");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    talon_srx_hardware_interface::talonSRXHardwareInterface talonSRXHardwareInterface(nh);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}