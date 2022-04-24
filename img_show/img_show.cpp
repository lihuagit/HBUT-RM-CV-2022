/**
 * @file img_show.cpp
 * @brief 显示图片
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */
#include "img_show.h"
#include <thread>
#include <chrono>
using namespace std::chrono;

bool debug=true;

void img_show_raw_run() {
    /*
     * 显示图像
     */
    if (debug) std::cout << "============ img_show_raw_run ===========" << std::endl;

    umt::Subscriber<cv::Mat> raw_sub("raw",50);

    while (true) {
        try {
            auto im2show = raw_sub.pop_for(50);
            cv::imshow("raw",im2show);

            cv::waitKey(1);

        } catch (umt::MessageError_Timeout &e) {
            // 似乎不需要做什么
            continue;
        } catch (umt::MessageError &e) {
            LOGE("[WARNING] 'raw_sub' { %s }",e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void img_show_detections_run() {
    /*
     * 显示图像
     */
    if (debug) std::cout << "============ img_show_detections_run ===========" << std::endl;

    umt::Subscriber<cv::Mat> img_detections_sub("detections",50);
    // std::string path="../pic/";
    // int cnt=1;
    while (true) {
        try {
            auto im2show = img_detections_sub.pop_for(50);
            cv::imshow("detections",im2show);
            // std::string tmp=path+std::to_string(cnt++)+".jpg";
            // cv::imwrite(tmp,im2show);

            cv::waitKey(1);

        } catch (umt::MessageError_Timeout &e) {
            // 似乎不需要做什么
            continue;
        } catch (umt::MessageError &e) {
            LOGE("[WARNING] 'detections_sub' { %s }",e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void img_show_light_run() {
    /*
     * 显示图像
     */
    if (debug) std::cout << "============ img_show_light_run ===========" << std::endl;

    umt::Subscriber<cv::Mat> img_light_sub("img_light",50);

    while (true) {
        try {
            auto im2show = img_light_sub.pop_for(50);
            cv::imshow("img_light",im2show);

            cv::waitKey(1);

        } catch (umt::MessageError_Timeout &e) {
            // 似乎不需要做什么
            continue;
        } catch (umt::MessageError &e) {
            LOGE("[WARNING] 'detections_sub' { %s }",e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void img_show_run(){
    /*
     * 显示图像
     */
    if(show_origin)
        std::thread([]() {
            img_show_raw_run();
        }).detach();
    
    if(show_armor_boxes){
        std::thread([]() {
            img_show_detections_run();
        }).detach();
    }

    if(show_light_blobs){
        std::thread([]() {
            img_show_light_run();
        }).detach();
    }
}
void background_img_show_run() {
    if (debug) std::cout << "============ background_img_show_run ===========" << std::endl;

    std::thread([]() {
        img_show_run();
    }).detach();
}