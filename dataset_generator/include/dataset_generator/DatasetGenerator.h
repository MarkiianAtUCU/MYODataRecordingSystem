//
// Created by mmatsi on 08.05.21.
//

#ifndef DATASET_GENERATOR_DATASETGENERATOR_H
#define DATASET_GENERATOR_DATASETGENERATOR_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include "sensor_read/MyoData.h"
#include "sensor_read/MyoDataSingle.h"
#include <fstream>
#include <mutex>

typedef struct {
    sensor_read::MyoDataSingle adc;
    sensor_msgs::JointState ground_truth;
} DataPoint;

class DatasetGenerator {
private:
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;

    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_emg_data_;
    ros::ServiceServer srv_save_data_;

    void initializeCommunication();

    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

    void emgDataCallback(const sensor_read::MyoDataSingleConstPtr &msg);

    bool saveDataCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

    std::vector<DataPoint> data_;
    std::vector<DataPoint> data_patch_;
    std::mutex mtx_data_patch_;

    DataPoint old_ground_truth_;
    bool ground_truth_once_received_ = false;
    std::mutex mtx_old_ground_truth_;

public:
    DatasetGenerator(const ros::NodeHandle &nh, int num_threads);

    void start();

};


#endif //DATASET_GENERATOR_DATASETGENERATOR_H
