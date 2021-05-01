//
// Created by mmatsi on 27.04.21.
//

#include "cv_arm_detection/ArmDetection.h"

ArmDetection::ArmDetection(const ros::NodeHandle &nh, int num_threads) : nh_(nh), spinner_(num_threads), it_(nh) {
    loadParams();
    initializeCommunication();
    ROS_INFO("Node started");
}

void ArmDetection::initializeCommunication() {
    ROS_INFO("Waiting for camera info");
    rgb_camera_model_.fromCameraInfo(
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>(nh_.resolveName("rgb_camera_info"))
    );
    ROS_INFO("RGB camera info received");
    depth_camera_model_.fromCameraInfo(
            ros::topic::waitForMessage<sensor_msgs::CameraInfo>(nh_.resolveName("depth_camera_info"))
    );
    ROS_INFO("Depth camera info received");

    sub_rgb_image_ = it_.subscribe("image", 1, &ArmDetection::rgbImageCallback, this);
    sub_depth_image_ = it_.subscribe("depth", 1, &ArmDetection::depthImageCallback, this);

    pub_overlay_image_ = it_.advertise("debug/overlay", 1);
    pub_thresholded_mask_ = it_.advertise("debug/mask", 1);
    pub_marker_poses_ = nh_.advertise<geometry_msgs::PoseArray>("marker_poses", 1);

    srv_configure_ = nh_.advertiseService("configure", &ArmDetection::configureCallback, this);

    reconfigure_server_.setCallback(boost::bind(&ArmDetection::reconfigureCallback, this, _1, _2));

}

typedef struct {
    cv::Point2d coord;
    double r_sq;

    cv::Point a;
    cv::Point b;
    cv::Point c;
} LineFitRes;

struct LineFitResCompare {
    inline bool operator()(const LineFitRes &struct1, const LineFitRes &struct2) {
        return (struct1.r_sq < struct2.r_sq);
    }
};

LineFitRes fitLine(cv::Point a, cv::Point b, cv::Point c) {
    // https://nbviewer.jupyter.org/gist/anonymous/eb643d8ea14eb58aa6f853aa67e3d4f5
    double x_bar = (a.x + b.x + c.x) / 3.0;
    double y_bar = (a.y + b.y + c.y) / 3.0;
    double n = 3.0;

    double numer = (a.x * a.y + b.x * b.y + c.x * c.y) - n * x_bar * y_bar;
    double denum = (a.x * a.x + b.x * b.x + c.x * c.x) - n * x_bar * x_bar;

    double lb = numer / denum;
    double la = y_bar - lb * x_bar;
    LineFitRes res;
    res.coord = cv::Point2d(la, lb);

    double denom = sqrt(lb * lb + 1);
    res.r_sq = (pow(lb * a.x - a.y + la, 2) + pow(lb * b.x - b.y + la, 2) + pow(lb * c.x - c.y + la, 2)) / denom;

    res.a = a;
    res.b = b;
    res.c = c;
    return res;
}


void ArmDetection::rgbImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    if (!current_depth_frame_) {
        ROS_ERROR_THROTTLE(5, "Depth image never received");
        return;
    }

    auto current_rgb_frame_ = cv_bridge::toCvShare(msg, "bgr8");


    bool local_configure_flag;
    {
        std::scoped_lock<std::mutex> lk(mtx_configure_flag_);
        local_configure_flag = configure_flag_;
    }

    cv::Mat hsv_frame;
    cv::cvtColor(current_rgb_frame_->image, hsv_frame, cv::COLOR_BGR2HSV);

    if (local_configure_flag) {
        auto interested_rect = my_cv::selectROI("rgb", current_rgb_frame_->image);

        std::vector<cv::Mat> img_channels;
        cv::split(hsv_frame(interested_rect), img_channels);
        cv::imshow("ROI", current_rgb_frame_->image(interested_rect));

        cv::Mat b_hist, g_hist, r_hist;
        int histSize = 256;
        float range[] = {0, 256}; //the upper boundary is exclusive
        const float *histRange = {range};

        cv::calcHist(&img_channels[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange);
        cv::calcHist(&img_channels[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange);
        cv::calcHist(&img_channels[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange);

        int hist_w = 512, hist_h = 400;
        int bin_w = cvRound((double) hist_w / histSize);
        cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
        cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
        cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

        for (int i = 0; i < 256; i += 20) {
            cv::line(histImage, cv::Point(hist_w / 256 * i, 0), cv::Point(hist_w / 256 * i, hist_h - 1),
                     {100, 100, 100});
            cv::putText(histImage, std::to_string(i), cv::Point(hist_w / 256 * i, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        {100, 100, 100});

        }

        for (int i = 1; i < histSize; i++) {
            cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
                     cv::Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
                     cv::Scalar(255, 0, 0), 2, 8, 0);
            cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
                     cv::Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
                     cv::Scalar(0, 255, 0), 2, 8, 0);
            cv::line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
                     cv::Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
                     cv::Scalar(0, 0, 255), 2, 8, 0);
        }

        cv::imshow("hist", histImage);
    }


    cv::Mat thresholdedImg;
    cv::inRange(hsv_frame, _hsv_params_.low, _hsv_params_.high, thresholdedImg);

//    cv::Mat imageYCrCb;
//    cv::cvtColor(current_rgb_frame_->image, imageYCrCb, cv::COLOR_BGR2YCrCb);

//    cv::Mat thresholdedSkin;
//    cv::inRange(imageYCrCb, cv::Scalar(0, 133, 77), cv::Scalar(235, 173, 127), thresholdedSkin);
//    cv::Mat notThresholdedSkin;
//    cv::bitwise_not(thresholdedSkin, notThresholdedSkin);
//    cv::Mat ThresholdedCombined;
//    cv::bitwise_and(thresholdedImg, notThresholdedSkin, ThresholdedCombined);

    cv::Mat ed_kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

//    cv::erode(thresholdedImg, thresholdedImg, ed_kernel);
//    cv::dilate(thresholdedImg, thresholdedImg, ed_kernel);
    sensor_msgs::ImagePtr msg_thresholdedImg = cv_bridge::CvImage(msg->header, "mono8", thresholdedImg).toImageMsg();
    pub_thresholded_mask_.publish(msg_thresholdedImg);


    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(thresholdedImg, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Moments> mu(contours.size());

    std::vector<cv::Point> detected_markers;
    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(current_rgb_frame_->image, contours, i, {0, 255, 0}, 2);
        mu[i] = cv::moments(contours[i], false);
        double dM01 = mu[i].m01;
        double dM10 = mu[i].m10;
        double dArea = mu[i].m00;

        if (dArea < _hsv_params_.area_low || dArea > _hsv_params_.area_high)
            continue;

        int posX = dM10 / dArea;
        int posY = dM01 / dArea;

        cv::circle(current_rgb_frame_->image, cv::Point(posX, posY), 5, {255, 0, 0}, 3);
        cv::putText(current_rgb_frame_->image, std::to_string(dArea), cv::Point(posX, posY), cv::FONT_HERSHEY_SIMPLEX,
                    1,
                    {255, 0, 0}, 2);

        detected_markers.emplace_back(posX, posY);
    }

    {
        std::scoped_lock<std::mutex> lk(mtx_depth_image_);
        if (abs((current_rgb_frame_->header.stamp - current_depth_frame_->header.stamp).toSec()) > 0.05) {
            ROS_WARN_THROTTLE(1, "Depth and RGB image not in sync");
        }


        geometry_msgs::PoseArray msg_marker_poses;
        msg_marker_poses.header = msg->header;
        for (auto &marker: detected_markers) {
            auto z = depthInNeighborhood(current_depth_frame_->image, marker.x, marker.y, 5, 100);
            const cv::Point2d p(marker.x, marker.y);
            auto object_ray = rgb_camera_model_.projectPixelTo3dRay(rgb_camera_model_.rectifyPoint(p));

            auto object_ray_mag = sqrt(
                    object_ray.x * object_ray.x + object_ray.y * object_ray.y + object_ray.z * object_ray.z);

            auto object_coordinates = (object_ray / object_ray_mag) * z;

            msg_marker_poses.poses.emplace_back();
            msg_marker_poses.poses.back().position.x = object_coordinates.x;
            msg_marker_poses.poses.back().position.y = object_coordinates.y;
            msg_marker_poses.poses.back().position.z = object_coordinates.z;
        }
        if (!msg_marker_poses.poses.empty())
            pub_marker_poses_.publish(msg_marker_poses);
//        cv::imshow("depth", current_depth_frame_->image);
    }


    std::vector<std::vector<int>> permutations = {
            {0, 1, 2},
            {0, 1, 3},
            {0, 1, 4},
            {0, 2, 3},
            {0, 2, 4},
            {0, 3, 4},
            {1, 2, 3},
            {1, 2, 4},
            {1, 3, 4},
            {2, 3, 4}
    };

    if (detected_markers.size() == 5) {
        std::vector<LineFitRes> fitted_lines;
        for (auto &p : permutations) {
            fitted_lines.push_back(fitLine(detected_markers[p[0]], detected_markers[p[1]], detected_markers[p[2]]));
        }

        std::sort(fitted_lines.begin(), fitted_lines.end(), LineFitResCompare());

        for (int i = 0; i < 2; ++i) {
            cv::line(current_rgb_frame_->image,
                     cv::Point(fitted_lines[i].a.x,
                               fitted_lines[i].coord.x + fitted_lines[i].coord.y * fitted_lines[i].a.x),
                     cv::Point(fitted_lines[i].b.x,
                               fitted_lines[i].coord.x + fitted_lines[i].coord.y * fitted_lines[i].b.x),

                     {0, 0, 255},
                     3
            );

            cv::line(current_rgb_frame_->image,
                     cv::Point(fitted_lines[i].a.x,
                               fitted_lines[i].coord.x + fitted_lines[i].coord.y * fitted_lines[i].a.x),
                     cv::Point(fitted_lines[i].c.x,
                               fitted_lines[i].coord.x + fitted_lines[i].coord.y * fitted_lines[i].c.x),

                     {0, 0, 255},
                     3
            );
        }



//        Perform angle detection
    }

//    cv::dilate(thresholdedImg, thresholdedImg, ed_kernel);
//    cv::erode(thresholdedImg, thresholdedImg, ed_kernel);




    {
        std::scoped_lock<std::mutex> lk(mtx_configure_flag_);
        configure_flag_ = false;
    }

    sensor_msgs::ImagePtr msg_out_img = cv_bridge::CvImage(msg->header, "bgr8", current_rgb_frame_->image).toImageMsg();
    pub_overlay_image_.publish(msg_out_img);

}

void ArmDetection::depthImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    {
        std::scoped_lock<std::mutex> lk(mtx_depth_image_);
        current_depth_frame_ = cv_bridge::toCvShare(msg);
    }
}

void ArmDetection::start() {
    spinner_.start();
}

bool ArmDetection::configureCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    {
        std::scoped_lock<std::mutex> lk(mtx_configure_flag_);
        if (configure_flag_) {
            res.success = false;
            res.message = "Configuration already in progress";
            return true;
        }
        configure_flag_ = true;
    }
    res.success = true;
    return true;
}

void ArmDetection::reconfigureCallback(cv_arm_detection::ArmDetectionConfig &config, uint32_t level) {
    _hsv_params_.low = cv::Scalar(config.h_low, config.s_low, config.v_low);
    _hsv_params_.high = cv::Scalar(config.h_top, config.s_top, config.v_top);

    _hsv_params_.area_high = config.area_high;
    _hsv_params_.area_low = config.area_low;
}

void ArmDetection::loadParams() {
    int h, s, v;
    nh_.getParam("marker_settings/color_threshold/h/low", h);
    nh_.getParam("marker_settings/color_threshold/s/low", s);
    nh_.getParam("marker_settings/color_threshold/v/low", v);
    _hsv_params_.low = cv::Scalar(h, s, v);

    nh_.getParam("marker_settings/color_threshold/h/high", h);
    nh_.getParam("marker_settings/color_threshold/s/high", s);
    nh_.getParam("marker_settings/color_threshold/v/high", v);
    _hsv_params_.high = cv::Scalar(h, s, v);

    nh_.getParam("marker_settings/area_threshold/low", _hsv_params_.area_low);
    nh_.getParam("marker_settings/area_threshold/high", _hsv_params_.area_high);

    ROS_INFO("Loaded node parameters:");
    ROS_INFO_STREAM("    HSV low: " << _hsv_params_.low);
    ROS_INFO_STREAM("    HSV high: " << _hsv_params_.high);
    ROS_INFO("    Area low %f", _hsv_params_.area_low);
    ROS_INFO("    Area high %f", _hsv_params_.area_high);
}
