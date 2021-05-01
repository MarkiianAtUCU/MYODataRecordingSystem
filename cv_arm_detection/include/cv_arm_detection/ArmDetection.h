//
// Created by mmatsi on 27.04.21.
//

#ifndef CV_ARM_DETECTION_ARMDETECTION_H
#define CV_ARM_DETECTION_ARMDETECTION_H

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

#include <std_srvs/Trigger.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <mutex>

#include "ocv_utils.h"

#include <cv_arm_detection/ArmDetectionConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>

class ArmDetection {
    typedef struct {
        cv::Scalar low;
        cv::Scalar high;

        double area_high;
        double area_low;
    } HSV_params;


    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    image_transport::ImageTransport it_;

    dynamic_reconfigure::Server<cv_arm_detection::ArmDetectionConfig> reconfigure_server_;


    image_geometry::PinholeCameraModel rgb_camera_model_;
    image_geometry::PinholeCameraModel depth_camera_model_;

    image_transport::Subscriber sub_rgb_image_;
    image_transport::Subscriber sub_depth_image_;
    ros::ServiceServer srv_configure_;

    image_transport::Publisher pub_overlay_image_;
    image_transport::Publisher pub_thresholded_mask_;
    ros::Publisher pub_marker_poses_;

    cv_bridge::CvImageConstPtr current_depth_frame_;
    std::mutex mtx_depth_image_;

    bool configure_flag_ = false;
    std::mutex mtx_configure_flag_;

    HSV_params _hsv_params_;
public:
    ArmDetection(const ros::NodeHandle &nh, int num_threads);

    void start();

private:
    void loadParams();

    void initializeCommunication();

    void rgbImageCallback(const sensor_msgs::ImageConstPtr &msg);

    void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);

    bool configureCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

    void reconfigureCallback(cv_arm_detection::ArmDetectionConfig &config, uint32_t level);

};


#endif //CV_ARM_DETECTION_ARMDETECTION_H
