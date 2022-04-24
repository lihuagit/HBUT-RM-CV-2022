//
// Created by Harry-hhj on 2021/5/4.
//

#ifndef CVRM2021_AUTOAIM_HPP
#define CVRM2021_AUTOAIM_HPP

#include <array>
#include <opencv2/opencv.hpp>
#include "detector/detector.h"

struct Detection_pack{
    /*
     * 打包数据结构，将识别结果、对应的图像、陀螺仪和时间戳对应
     */
    bbox_t detection;
    cv::Mat img;
    std::array<double, 4> q;
    double timestamp;
};

void background_predict_EKF_run();
void background_predict_run();
void background_none_predict_run();
void background_find_light_blobs_run();
void background_find_armor_boxs_run(const string &paras_folder);

#endif //CVRM2021_AUTOAIM_HPP
