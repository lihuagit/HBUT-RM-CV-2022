//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#include "autoaim.hpp"
#include <umt/umt.hpp>
#include <sensors.hpp>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <string>
#include <cmath>
#include <predictor/PredictorKalman.h>
#include <predictor/PredictorAdaptiveEKF.h>

using namespace std::chrono;

static bool debug = true;

void none_predict_run() {
    /*
     * 仅具有跟随的效果，没有抬枪补偿，用来查看电控是否正确
     */
    if (debug) std::cout << "============ none_predict_run ===========" << std::endl;
    umt::Subscriber<Detection_pack> detections_sub("detections_pack");
    auto sensor_param = umt::ObjManager<SensorParam>::find_or_create("sensor_param");
    
    while( (sensor_param->Tcb).empty() )
            std::this_thread::sleep_for(500ms);
    const cv::Mat& K = sensor_param->K;
    const cv::Mat& D = sensor_param->D;
    const cv::Mat& Tcb = sensor_param->Tcb;
    const double fx = K.at<double>(0, 0);
    const double fy = K.at<double>(1, 1);
    const double cx = K.at<double>(0, 2);
    const double cy = K.at<double>(1, 2);
    umt::Publisher<RobotCmd> robot_cmd_pub("robot_cmd");

    while (true) {
        try {
            const auto &detections_pack = detections_sub.pop();
            auto detection = detections_pack.detection;

            auto &target = detection;
            float x = (target.pts[0].x + target.pts[1].x + target.pts[2].x + target.pts[3].x) / 4.f;
            float y = (target.pts[0].y + target.pts[1].y + target.pts[2].y + target.pts[3].y) / 4.f;
            RobotCmd robot_cmd;
            robot_cmd.pitch_angle = (float)atan2(y - cy, fy);
            robot_cmd.yaw_angle = (float)atan2(x - cx, fx);
            LOGM("[MSG] yaw: %f,   pitch: %f ", robot_cmd.yaw_angle, robot_cmd.pitch_angle);
            robot_cmd.pitch_speed = 0;
            robot_cmd.yaw_speed = 0;
            robot_cmd.distance = 0.;
            robot_cmd_pub.push(robot_cmd);
        } catch (umt::MessageError &e) {
            LOGE("[ERROR] 'detections_sub' { %s }",e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void background_none_predict_run() {
    if (debug) std::cout << "============ background_none_predict_run ===========" << std::endl;

    std::thread([]() {
        none_predict_run();
    }).detach();
}

void predict_run() {
    /*
     * 使用传统 Kalman 预测
     */
    if (debug) std::cout << "============ predict_run ===========" << std::endl;

    umt::Publisher<cv::Mat> webview_predictions("prediction");

    umt::Subscriber<Detection_pack> detections_sub("detections_pack");
    umt::Publisher<RobotCmd> robot_cmd_pub("robot_cmd");
    PredictorKalman predictor;

    int fps = 0, fps_count = 0;
    auto t1 = system_clock::now();

    while (true) {
        try {
            auto detections = detections_sub.pop_for(50);
            RobotCmd robot_cmd;
            cv::Mat im2show;
            // bool ok = predictor.predict(detections, robot_cmd, im2show);
            bool ok = predictor.predict_2(detections, robot_cmd, im2show);
            
            if(!ok) {
                std::this_thread::sleep_for(500ms);
            }
            cv::imshow("pre",im2show);
            robot_cmd_pub.push(robot_cmd);
        } catch (umt::MessageError_Timeout &e) {
            // RobotCmd robot_cmd;  // TODO
            // robot_cmd.shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
            // robot_cmd_pub.push(robot_cmd);
        } catch (umt::MessageError &e) {
            LOGE("[WARNING] 'detections_sub' { %s }",e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void background_predict_run() {
    if (debug) std::cout << "============ background_predict_run ===========" << std::endl;

    std::thread([]() {
        predict_run();
        std::cout<<"In background-predict-run" << std::endl;
    }).detach();
}

void predict_EKF_run() {
    /*
     * 使用 EKF 预测
     */
    if (debug) std::cout << "============ predict_EKF_run ===========" << std::endl;

    umt::Publisher<cv::Mat> webview_predictions("predictionEKF");

    umt::Subscriber<Detection_pack> detections_sub("detections_pack");
    umt::Publisher<RobotCmd> robot_cmd_pub("robot_cmd");
    PredictorAdaptiveEKF predictor;

    int fps = 0, fps_count = 0;
    auto t1 = system_clock::now();

    bool last_mode_is_autoaim = true;

    while (true) {
        try {
            auto detections = detections_sub.pop_for(50);
            RobotCmd robot_cmd;
            cv::Mat im2show;

            if (!last_mode_is_autoaim) predictor.clear();
            last_mode_is_autoaim = true;

            bool ok = predictor.predict(detections, robot_cmd, im2show, false);

            if(!ok) {
                std::this_thread::sleep_for(500ms);
            }

            robot_cmd_pub.push(robot_cmd);
        } catch (umt::MessageError_Timeout &e) {
            // RobotCmd robot_cmd;  // TODO
            // robot_cmd.shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
            // robot_cmd_pub.push(robot_cmd);
        } catch (umt::MessageError &e) {
            LOGE("[WARNING] 'detections_sub' { %s }",e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void background_predict_EKF_run() {
    if (debug) std::cout << "============ background_predict_EKF_run ===========" << std::endl;

    std::thread([]() {
        predict_EKF_run();
    }).detach();
}