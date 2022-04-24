/**
 * @file sensors.hpp
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */
#ifndef CVRM2022_SENSORS_HPP
#define CVRM2022_SENSORS_HPP

#include <opencv2/core.hpp>
#include <options.h>
#include "Video_mode/video_wrapper.h"

struct AutoExposureParam {
    double min_brightness = 60;
    double max_brightness = 100;
    double step_exposure_us = 200;
    double min_exposure_us = 1000;
    double max_exposure_us = 9000;
};

struct SensorsData {
    cv::Mat im;
    std::array<double, 4> q;
    double timestamp; // ms
};

struct SensorParam {
    cv::Mat K;
    cv::Mat D;
    cv::Mat Tcb;
};
void background_sensors_io_auto_restart(const std::string &camera_name,
                                        const std::string &camera_cfg,
                                        const std::string &sensor_param_file,
                                        const std::string &imu_usb_hid ,
                                        int sync_period_ms);
void background_sensors_io_video_auto_restart(const std::string &video_name); 

#endif //CVRM2022_SENSORS_HPP
