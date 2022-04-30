//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#include <umt/umt.hpp>
#include <sensors.hpp>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <string>
#include <cmath>
#include "detector.h"
#include "../autoaim.hpp"
#include "classifier/ArmorNumClassifier.h"

using namespace std::chrono;

/////////////////////// debug //////////////////////

std::map<int, string> id2name = {                               //装甲板id到名称的map
        {-1, "OO"},{ 0, "NO"},
        { 1, "B1"},{ 2, "B2"},{ 3, "B3"},{ 4, "B4"},{ 5, "B5"},{ 6, "B7"},{ 7, "B8"},
        { 8, "R1"},{ 9, "R2"},{10, "R3"},{11, "R4"},{12, "R5"},{13, "R7"},{14, "R8"},
};

std::map<string, int> name2id = {                               //装甲板名称到id的map
        {"OO", -1},{"NO",  0},
        {"B1",  1},{"B2",  2},{"B3",  3},{"B4",  4},{"B5",  5},{"B7",  6},{"B8",  7},
        {"R1",  8},{"R2",  9},{"R3", 10},{"R4", 11},{"R5", 12},{"R7", 13},{"R8", 14},
};

std::map<string, int> prior_blue = {
        {"B8", 0}, {"B1", 1}, {"B3", 2}, {"B4", 2}, {"B5", 2}, {"B7", 3}, {"B2", 4},
        {"R8", 5}, {"R1", 6}, {"R3", 7}, {"R4", 7}, {"R5", 7}, {"R7", 8}, {"R2", 9},
        {"NO", 10},
};

std::map<string, int> prior_red = {
        {"R8", 0}, {"R1", 1}, {"R3", 2}, {"R4", 2}, {"R5", 2}, {"R7", 3}, {"R2", 4},
        {"B8", 5}, {"B1", 6}, {"B3", 7}, {"B4", 7}, {"B5", 7}, {"B7", 8}, {"B2", 9},
        {"NO", 10},
};
static bool debug = true;

///////////////////// debug ////////////////////////

void find_light_blobs_run() {
    /*
     * 识别灯条
     */
    if (debug) std::cout << "============ find_light_blobs_run ===========" << std::endl;

    umt::Subscriber<SensorsData> sensor_sub("sensors_data");

    umt::Publisher<Light_blobs_pack> light_blobs_pub("light_blobs_pack");
    umt::Publisher<cv::Mat> img_light_pub("img_light");

    int cnt_useless = -1;
    int fps = 0, fps_count = 0;
    auto t1 = system_clock::now();

    const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};

    while (true) {
        try {
            const auto& [img, q, timestamp] = sensor_sub.pop_for(10);
            if(img.empty()){
                continue;
            }
            LightBlobs light_blobs;
            findLightBlobs(img, light_blobs);

            /* publish light_blobs results */
            if (!light_blobs.empty()) {
                light_blobs_pub.push(Light_blobs_pack{light_blobs, img, q, timestamp});  // 打包数据
            } else 
                if (++cnt_useless == 50) {  // 避免输出太多
                    LOGM("No light_blobs detected!");
                    cnt_useless = -1;
                }
            // show light
            if(show_light_blobs){
                cv::Mat im2show = img.clone();
                for (const auto &b: light_blobs) {
                    cv::rectangle(im2show, b.rect.boundingRect(), cv::Scalar(0, 255, 0), 1);
                }
                fps_count++;
                auto t2 = system_clock::now();
                if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                    fps = fps_count;
                    fps_count = 0;
                    t1 = t2;
                }
                char TextFPS[20];
                sprintf(TextFPS,"fps={ %d }\0",fps);
                cv::putText(im2show, TextFPS, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
                img_light_pub.push(im2show);
                // cv::imshow("dddddd",im2show);
                // cv::waitKey(1);
            }
        } catch (umt::MessageError_Timeout &e) {
            LOGW("[WARNING] 'sensor_sub' { %s }",e.what());
            // 似乎不需要做什么
            continue;
        } catch (umt::MessageError &e) {
            LOGE("[WARNING] 'sensor_sub' { %s }",e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void background_find_light_blobs_run() {
    if (debug) std::cout << "============ background_find_light_blobs_run ===========" << std::endl;

    std::thread([=]() {
        find_light_blobs_run();
    }).detach();
}

void find_armor_boxs_run(const string &paras_folder) {
    /*
     * 识别装甲板
     */

    if (debug) std::cout << "============ find_armor_boxs_run ===========" << std::endl;

    /// 定义发布器、订阅器
    umt::Subscriber<SensorsData> sensor_sub("sensors_data");
    umt::Publisher<Detection_pack> detections_pub("detections_pack");
    umt::Publisher<cv::Mat> img_detections_pub("detections");

    /// 定义ROI区域
    auto roi_rect = umt::ObjManager<cv::Rect2d>::find_or_create("roi_rect");
    cv::Rect2d def_rect(0,0,640,480);
    *roi_rect=def_rect;

    // tracker对象实例
    cv::Ptr<cv::Tracker> tracker;

    int cnt_useless = -1;
    int fps = 0, fps_count = 0;
    auto t1 = system_clock::now();

    const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};
    Classifier classifier(paras_folder);
    bool flag_track=false;

    while (true) {
        try {
            /// 获取数据
            const auto& [img, q, timestamp] = sensor_sub.pop_for(50);

            // 使用KCFTracker进行追踪
            auto pos = *(roi_rect);
            cv::Rect2i poss=pos;
            if(flag_track)
                if(!tracker->update(img, poss)){ 
                    poss=pos;
                    LOGW("Track fail!");
                }
            pos=poss;
            if((pos & def_rect) != pos){
                *roi_rect=def_rect;
                pos=def_rect;
                LOGW("Track out range!");
            }

            // 获取相较于追踪区域两倍长款的区域，用于重新搜索，获取灯条信息
            cv::Rect2d bigger_rect;
            if(pos!=def_rect){
                bigger_rect.x = pos.x - pos.width / 2.0;
                bigger_rect.y = pos.y - pos.height / 2.0;
                bigger_rect.height = pos.height * 2;
                bigger_rect.width  = pos.width * 2;
                bigger_rect &= def_rect;
            }
            else bigger_rect=pos;
            cv::Mat roi = img(bigger_rect).clone();

            LightBlobs light_blobs;
            findLightBlobs(roi, light_blobs);

            auto detection = findArmorBox_3(roi,light_blobs,classifier);

            double img_w2=img.cols/2;
            double img_h2=img.rows/2;
            /* publish detection results */
            if (detection.tag_id!=0) {

                //　添加roi偏移量
                detection.rect.x += bigger_rect.x;
                detection.rect.y += bigger_rect.y;

                for(int i=0;i<4;i++){
                    detection.pts[i].x+=bigger_rect.x-img_w2;
                    detection.pts[i].y+=bigger_rect.y-img_h2;
                    detection.pts[i].y*=-1;
                }

                // for(int i=0;i<4;i++){
                //     detection.pts[i].y+=bigger_rect.y;
                //     detection.pts[i].x+=bigger_rect.x;
                // }
                // 更新tracker
                tracker = TrackerToUse::create();
                tracker->init(img, detection.rect);
                *roi_rect=detection.rect;
                flag_track=true;
                
                detections_pub.push(Detection_pack{detection, img, q, timestamp});  // 打包数据
                cnt_useless = -1;
            } else 
                if (++cnt_useless == 10) {  // 避免输出太多
                    LOGM("No enemy detected!");
                    cnt_useless = -1;
                    *roi_rect=cv::Rect2d(0,0,640,480);
                    flag_track=false;
                }
                
            if(show_armor_boxes){
                cv::Mat im2show = img.clone();
                for(int i=0;i<4;i++){
                    detection.pts[i].x+=img_w2;
                    detection.pts[i].y*=-1;
                    detection.pts[i].y+=img_h2;
                }

                if (detection.tag_id!=0) {
                    // 画线
                    for (int i = 0; i < 4; i++)
                        cv::line(im2show, detection.pts[i], detection.pts[(i + 1) % 4], colors[2], 2);

                    // 画点
                    for(int i=0;i<4;i++)
                        cv::circle(im2show,detection.pts[i],i*3,cv::Scalar(255,255,255),1);

                    // 画矩形
                    cv::rectangle(im2show, detection.rect, cv::Scalar(0, 255, 0), 1);
                    cv::putText(im2show, id2name[detection.tag_id], detection.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[detection.color_id]);
                }

                cv::line(im2show,cv::Point2d(0,img_h2),cv::Point2d(img_w2*2,img_h2),cv::Scalar(0,0,255),1);
                cv::line(im2show,cv::Point2d(img_w2,0),cv::Point2d(img_w2,img_h2*2),cv::Scalar(0,0,255),1);
                fps_count++;
                auto t2 = system_clock::now();
                if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                    fps = fps_count;
                    fps_count = 0;
                    t1 = t2;
                }
                char TextFPS[20];
                sprintf(TextFPS,"fps={ %d }\0",fps);
                cv::putText(im2show, TextFPS, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
                img_detections_pub.push(im2show);
                // cv::imshow("dddddd",im2show);
                // cv::waitKey(1);
            }
        } catch (umt::MessageError_Timeout &e) {
            LOGW("[WARNING] 'sensor_sub___' { %s }",e.what());
            // 似乎不需要做什么
            continue;
        } catch (umt::MessageError &e) {
            LOGE("[WARNING] 'sensor_sub' { %s }",e.what());
            flag_track=false;
            // *roi_rect=def_rect;
            std::this_thread::sleep_for(5ms);
        }
    }
}

void background_find_armor_boxs_run(const string &paras_folder) {
    if (debug) std::cout << "============ background_find_armor_boxs_run ===========" << std::endl;

    std::thread([=]() {
        while (true) {
            find_armor_boxs_run(paras_folder);
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
}