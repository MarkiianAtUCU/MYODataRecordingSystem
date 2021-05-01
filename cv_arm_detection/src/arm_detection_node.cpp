//
// Created by mmatsi on 27.04.21.
//

#include "ros/ros.h"
#include "cv_arm_detection/ArmDetection.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cv_arm_detection");
    ros::NodeHandle nh("~");

    ArmDetection ad(nh, 4);
    ad.start();
    ros::waitForShutdown();
    return 0;
}