//
// Created by mmatsi on 28.04.21.
//

#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/String.h"
#include "sensor_read/MyoData.h"
#include "sensor_read/MyoDataSingle.h"
#include <chrono>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_read");
    ros::NodeHandle nh("~");

    ros::Publisher pub = nh.advertise<sensor_read::MyoData>("sensor_data", 10);
    ros::Publisher pub2 = nh.advertise<sensor_read::MyoDataSingle>("sensor_data_single", 10);

    serial::Serial ser;
    try {
        ser.setPort("/dev/ttyACM1");
        ser.setBaudrate(1152000);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port ");
        std::cout << e.what() << std::endl;
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    int counter = 0;
    std::vector<std::string> data_vec;

    sensor_read::MyoData msg;
    std::vector<uint8_t> buffer;

    while (ros::ok()) {
        auto time_now = std::chrono::system_clock::now();
        if (ser.available()) {
            std::vector<uint8_t> buffer;
            ser.read(buffer, 21);
            if (counter == 1000) {
                pub.publish(msg);
                msg = sensor_read::MyoData();
                msg.stamp = ros::Time::now();
                counter = 0;
            }

            msg.adc1.push_back(buffer[0] | buffer[1] << 8);
            msg.adc2.push_back(buffer[2] | buffer[3] << 8);
            msg.adc3.push_back(buffer[4] | buffer[5] << 8);
            msg.adc4.push_back(buffer[6] | buffer[7] << 8);
            msg.adc5.push_back(buffer[8] | buffer[9] << 8);
            msg.adc6.push_back(buffer[10] | buffer[11] << 8);
            msg.adc7.push_back(buffer[12] | buffer[13] << 8);
            msg.adc8.push_back(buffer[14] | buffer[15] << 8);
            msg.adc9.push_back(buffer[16] | buffer[17] << 8);
            msg.adc10.push_back(buffer[18] | buffer[19] << 8);
            counter++;

            sensor_read::MyoDataSingle msg2;
            msg2.adc1=(buffer[0] | buffer[1] << 8);
            msg2.adc2=(buffer[2] | buffer[3] << 8);
            msg2.adc3=(buffer[4] | buffer[5] << 8);
            msg2.adc4=(buffer[6] | buffer[7] << 8);
            msg2.adc5=(buffer[8] | buffer[9] << 8);
            msg2.adc6=(buffer[10] | buffer[11] << 8);
            msg2.adc7=(buffer[12] | buffer[13] << 8);
            msg2.adc8=(buffer[14] | buffer[15] << 8);
            msg2.adc9=(buffer[16] | buffer[17] << 8);
            msg2.adc10=(buffer[18] | buffer[19] << 8);

            pub2.publish(msg2);

        }
    }


    return 0;
}