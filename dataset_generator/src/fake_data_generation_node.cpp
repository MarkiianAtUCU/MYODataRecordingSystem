//
// Created by mmatsi on 08.05.21.
//

#include <ros/ros.h>
#include "sensor_read/MyoDataSingle.h"

ros::Publisher pub;
size_t counter;

void timerCallback(const ros::TimerEvent &) {
    sensor_read::MyoDataSingle msg;
    msg.stamp = ros::Time::now();
    msg.adc1 = counter;
    msg.adc2 = counter;

    pub.publish(msg);
    counter++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fake_data_generation_node");
    ros::NodeHandle nh("~");
    pub = nh.advertise<sensor_read::MyoDataSingle>("/sensor_read/sensor_data_single", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / 5000), timerCallback);


    ros::spin();
    return 0;
}