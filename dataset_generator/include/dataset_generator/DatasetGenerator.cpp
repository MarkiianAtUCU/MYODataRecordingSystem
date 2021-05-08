//
// Created by mmatsi on 08.05.21.
//

#include "DatasetGenerator.h"

DatasetGenerator::DatasetGenerator(const ros::NodeHandle &nh, int num_threads) : nh_(nh), spinner_(num_threads) {
    initializeCommunication();
}

void DatasetGenerator::initializeCommunication() {
    sub_joint_state_ = nh_.subscribe("joint_state_data", 1, &DatasetGenerator::jointStateCallback, this);
    sub_emg_data_ = nh_.subscribe("sensor_data", 1, &DatasetGenerator::emgDataCallback, this);
    srv_save_data_ = nh_.advertiseService("save_data", &DatasetGenerator::saveDataCallback, this);
}

inline float interpolate(float from, float to, float amount) {
    return from + ((to - from) * amount);
}

void DatasetGenerator::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg) {
    for (const auto &e: msg->position) {
        if (isnan(e))
            return;
    }

    if (!ground_truth_once_received_) {
        old_ground_truth_.ground_truth = *msg;
        ground_truth_once_received_ = true;

        return;
    }

    // interpolate and add to data
    {
        std::scoped_lock<std::mutex> lk(mtx_data_patch_);

        for (int i = 0; i < data_patch_.size() - 1; ++i) {
            if ((data_patch_[i].adc.stamp < old_ground_truth_.ground_truth.header.stamp) && ( old_ground_truth_.ground_truth.header.stamp < data_patch_[i+1].adc.stamp)) {
                std::cout << "Found frame START ts:" << i << std::endl;
            }

            if ((data_patch_[i].adc.stamp < msg->header.stamp) && (msg->header.stamp < data_patch_[i+1].adc.stamp)) {
                std::cout << "Found frame END ts:" << i << std::endl;
            }
            std::cout << "Len: " << std::endl;
        }

        std::cout << "======" << std::endl;
        for (int i = 0; i < data_patch_.size(); ++i) {
            data_patch_[i].ground_truth.position.emplace_back(
                    interpolate(old_ground_truth_.ground_truth.position[0], msg->position[0],
                                static_cast<float>(i) * 1.0f / data_patch_.size()));
        }


        data_.insert(data_.end(), data_patch_.begin(), data_patch_.end());
        data_patch_.clear();

    }


    old_ground_truth_.ground_truth = *msg;
    ground_truth_once_received_ = true;

}

void DatasetGenerator::emgDataCallback(const sensor_read::MyoDataSingleConstPtr &msg) {
    if (!ground_truth_once_received_) {
        ROS_WARN_THROTTLE(1, "Ground truth never rerceived, EMG data skipped");
        return;
    }

    {
        std::scoped_lock<std::mutex> lk(mtx_data_patch_);
        data_patch_.emplace_back();
        data_patch_.back().adc = *msg;
    }
}

void DatasetGenerator::start() {
    spinner_.start();
}

bool DatasetGenerator::saveDataCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    {
        std::scoped_lock<std::mutex> lk(mtx_data_patch_);
        std::stringstream ss;
        ss << "adc_1,adc_2,angle,ts\n";

        for (auto &e: data_) {
            ss << e.adc.adc1 << "," << e.adc.adc2 << "," << e.ground_truth.position[0] << "\n";
        }
        ss << std::endl;

        std::ofstream outFile;
        outFile.open("/home/mmatsi/diploma_ws/src/dataset_generator/out_data/data.csv");
        outFile << ss.rdbuf();
        outFile.close();

        std::cout << "Wrote file" << std::endl;
    }

    return true;
}
