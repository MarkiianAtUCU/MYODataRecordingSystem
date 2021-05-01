//
// Created by mmatsi on 27.04.21.
//

#ifndef CV_ARM_DETECTION_OCV_UTILS_H
#define CV_ARM_DETECTION_OCV_UTILS_H

#include <opencv2/opencv.hpp>


namespace my_cv {
    void CallBackFunc(int event, int x, int y, int flags, void *userdata);

    cv::Rect selectROI(const std::string &window_name, const cv::Mat &img);
}


/**
 Function calculates depth to object in desired coordinates

 @param cv_Mat depth image, which will be used to find distance
 @param int x coordinate of object
 @param int y coordinate of object
 @param int delta - size of rect, from which depth will be drawn
 @param double epsilon - deviation from mean, in which depth will be used;
 @return average depth value in desired coordinates in delta rectangle
 */
double depthInNeighborhood(const  cv::Mat &depth_img, int x, int y, int delta, double burnout_epsilon);

/**
 Function clamps ROI to not be outside original image

 @param cv_Mat input image
 @param cv_Rect2d ROI to be clamped
 @return clamped roi
 */
cv::Rect2d constraintROI(const cv::Mat &img, cv::Rect2d src);

#endif //CV_ARM_DETECTION_OCV_UTILS_H
