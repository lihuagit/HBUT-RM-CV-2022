/**
 * @file MindVision.hpp
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */

#ifndef CVRM2021_MINDVISION_HPP
#define CVRM2021_MINDVISION_HPP

#include "MVSDK/CameraApi.h"
#include <opencv2/core.hpp>

class MindVision {
public:
    explicit MindVision(const char *camera_name = "", const char *camera_cfg = "");

    ~MindVision();

    bool open();

    bool close();

    bool isOpen() const;

    bool read(cv::Mat &img) const;

    bool read(cv::Mat &img, double &timestamp_ms) const;

    bool get_exposure_us(double &us) const;

    bool set_exposure_us(double us) const;

private:
    const std::string camera_name;
    const std::string camera_cfg;
    CameraHandle handle;
    std::function<void(cv::Mat, tSdkFrameHead *)> callback;
};


#endif //CVRM2021_MINDVISION_HPP
