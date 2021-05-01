//
// Created by mmatsi on 27.04.21.
//

#include "ocv_utils.h"

namespace my_cv {
    struct MyData {
        const cv::Mat &img;
        const std::string &win_name;
        cv::Point2i tl;
        cv::Point2i br;
        bool is_drawing;
        bool should_be_displayed;
    };

    void CallBackFunc(int event, int x, int y, int flags, void *userdata) {

        auto local_userdata = (MyData *) userdata;

        cv::Mat local_frame = local_userdata->img.clone();

        if (event == cv::EVENT_LBUTTONDOWN) {
            local_userdata->is_drawing = true;
            local_userdata->should_be_displayed = true;
            local_userdata->tl.x = x;
            local_userdata->tl.y = y;
            local_userdata->br.x = x;
            local_userdata->br.y = y;
        } else if (event == cv::EVENT_MOUSEMOVE) {
            if (local_userdata->is_drawing) {
                local_userdata->br.x = x;
                local_userdata->br.y = y;
            }
        } else if (event == cv::EVENT_LBUTTONUP) {
            local_userdata->is_drawing = false;
            local_userdata->br.x = x;
            local_userdata->br.y = y;
        }

        if (local_userdata->should_be_displayed) {
            auto r = cv::Rect(local_userdata->tl, local_userdata->br);
            cv::rectangle(local_frame, r, {1}, 3);
            cv::line(local_frame, {r.x, r.height / 2 + r.y}, {r.x + r.width, r.height / 2 + r.y}, {1}, 3);
            cv::line(local_frame, {r.width / 2 + r.x, r.y}, { r.width / 2 + r.x, r.y + r.height}, {1}, 3);
            cv::imshow(local_userdata->win_name, local_frame);
        }
    }

    cv::Rect selectROI(const std::string &window_name, const cv::Mat &img) {
        MyData my_data = MyData{img, window_name};

        cv::namedWindow(window_name);
        cv::setMouseCallback(window_name, CallBackFunc, &my_data);

        cv::imshow(window_name, img);


        while (true) {
            int key_out = cv::waitKey(0) % 256;
            // 32 - space
            // 10 - enter
            if ((key_out == 32) || (key_out == 10))
                break;
        }
        cv::setMouseCallback(window_name, NULL, NULL);

        return cv::Rect{my_data.tl, my_data.br};
    }
}


double depthInNeighborhood(const cv::Mat &depth_img, int x, int y, int delta, double burnout_epsilon) {
    cv::Mat roi = depth_img(constraintROI(depth_img, cv::Rect2i(x, y, delta, delta)));
    double mean = cv::mean(roi)[0];
    double sum = 0;
    int n = 0;
    for (int i = 0; i < roi.rows; ++i) {
        const uint16_t *Mi = roi.ptr<uint16_t>(i);
        for (int j = 0; j < roi.cols; ++j) {
            if (Mi[j] > mean - burnout_epsilon && Mi[j] < mean + burnout_epsilon) {
                sum += Mi[j];
                n++;
            }
        }
    }
    //    TODO:  different config for simul and no
    return sum / n / 1000;
}

cv::Rect2d constraintROI(const cv::Mat &img, cv::Rect2d src) {
    int top_left_x = src.tl().x;
    int top_left_y = src.tl().y;

    int bottom_right_x = src.br().x;
    int bottom_right_y = src.br().y;

    int width = src.width;
    int height = src.height;
    cv::Mat output;

    if (top_left_x < 0 || top_left_y < 0 || bottom_right_x > img.cols || bottom_right_y > img.rows) {

        if (top_left_x < 0) {
            width = width + top_left_x;
            top_left_x = 0;
        }
        if (top_left_y < 0) {
            height = height + top_left_y;
            top_left_y = 0;
        }
        if (bottom_right_x > img.cols) {
            width = width - (bottom_right_x - img.cols);
        }
        if (bottom_right_y > img.rows) {
            height = height - (bottom_right_y - img.rows);
        }

        cv::Rect R(top_left_x, top_left_y, width, height);
        return R;
    } else {
        return src;
    }
}
