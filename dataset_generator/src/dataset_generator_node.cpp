//
// Created by mmatsi on 08.05.21.
//

#include "dataset_generator/DatasetGenerator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "dataset_generator");
    ros::NodeHandle nh("~");

    DatasetGenerator dg(nh, 4);
    dg.start();
    ros::waitForShutdown();
    return 0;
}