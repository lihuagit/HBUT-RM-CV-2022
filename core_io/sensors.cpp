/**
 * @file sensors.cpp
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */
#include "sensors.hpp"
#include "MindVision/MindVision.hpp"
#include "ExtImu/ExtImu.hpp"
#include <serial/serial.h>
#include <umt/umt.hpp>
#include <opencv2/opencv.hpp>
#include <log.h>
#include <thread>
#include <chrono>
#include <queue>
#include "../img_show/img_show.h"

using namespace serial;
using namespace std::chrono;

static std::mutex seq_q_mtx;
static std::queue<std::array<double, 4>> seq_q;
static bool imu_running = false;
static bool require_imu_stop = false;

static bool debug = true;

bool sync_once(MindVision &camera, ExtImu &imu, int sync_period_ms) {
    /*
     * 单词时间戳同步，并不是永远同步
     * 重启相机并重新触发
     */
    if (debug) std::cout << "============sync_once===========\n";
    camera.close();
    require_imu_stop = true;
    imu.stop_trigger();
    while (imu_running);
    seq_q = std::queue<std::array<double, 4>>();
    camera.open();
    if (!camera.isOpen()) {
        LOGE("[ERROR]: camera reopen fail!\n");
        return false;
    }
    try {
        // imu.periodic_trigger(sync_period_ms);
    } catch (SerialException &e) {
        LOGE("[ERROR] { %s }",e.what());
        return false;
    }
    require_imu_stop = false;
    return true;
}

static void imu_capture_loop(ExtImu &imu, const bool &require_stop, bool &is_ok) {
    /*
     * 获得陀螺仪数据
     */
    if (debug) std::cout << "============imu_capture_loop===========\n";
    ExtImu::sensor_data data{};
    char buffer[15];
    memset(buffer,0,sizeof buffer);
    int len=1;
    imu_running = true;
    while (!require_stop) {
        if (require_imu_stop) {
            imu.flushInput();
            imu_running = false;
            while (require_imu_stop);
            imu_running = true;
        }
        try {
            // imu.read_sensor(&data);
            // std::cout<<"size:"<<sizeof buffer<<std::endl;
            len=imu.read_word_yaw(buffer, 15);
        } catch (SerialException &e) {
            LOGE("[ERROR] { %s }",e.what());
            is_ok = false;
            break;
        }
        if(len<=5) continue;
        if(len>0){
            // std::cout<<"len:"<<len<<std::endl;
            // std::cout<<"buffer:"<<buffer<<std::endl;
        }
        sscanf(buffer,"%f",&data.q[3]);
        // std::cout<<"data.q[3]=="<<data.q[3]<<std::endl;
        if (data.trigger == 1 || true) {
            //std::cout << "lock acquire" << std::endl;
            std::unique_lock lock(seq_q_mtx);
            //std::cout << "lock release" << std::endl;
            if (!seq_q.empty()) seq_q.pop();
            seq_q.push({data.q[3], data.q[0], data.q[1], data.q[2]});  // wxyz
        }
    }
}

bool sensors_io(const std::string &camera_name = "", const std::string &camera_cfg = "",
                const std::string &sensor_param_file = "",
                const std::string &imu_usb_hid = "", int sync_period_ms = 10) {
    /*
     * 读入外参文件，打包数据发布
     */
    if (debug) std::cout << "============sensors_io===========\n";

    cv::FileStorage ifs;
    if (!ifs.open(sensor_param_file, cv::FileStorage::READ)) {
        LOGE("[ERROR]: sensor parameter read fail!");
        return false;
    }
    auto sensor_param = umt::ObjManager<SensorParam>::find_or_create("sensor_param");
    try {
        ifs["K"] >> sensor_param->K;
        ifs["D"] >> sensor_param->D;
        ifs["Tcb"] >> sensor_param->Tcb;
        if (sensor_param->K.cols != 3 || sensor_param->K.rows != 3) {
            throw std::runtime_error("sensor parameter 'K' invalid format!");
        }
        if (sensor_param->D.cols != 5 || sensor_param->D.rows != 1) {
            throw std::runtime_error("sensor parameter 'D' invalid format!");
        }
        if (sensor_param->Tcb.cols != 3 || sensor_param->Tcb.rows != 3) {
            throw std::runtime_error("sensor parameter 'Tcb' invalid format!");
        }
    } catch (cv::Exception &e) {
        LOGE("[ERROR] { %s }",e.what());
        return false;
    }

    MindVision camera(camera_name.data(), camera_cfg.data());
    camera.open();
    if (!camera.isOpen()) {
        LOGE("[ERROR]: camera init fail!");
        return false;
    }
    ExtImu imu;
    for (const auto &port_info : list_ports()) {
        std::cerr << "hardware_id: " << port_info.hardware_id << ", port: " << port_info.port << std::endl;
        if (port_info.hardware_id == imu_usb_hid) {
	        std::cerr << "enter_if 1" << std::endl;
            imu.setPort(port_info.port);
	        std::cerr << "after set id" << std::endl;
            auto timeout = Timeout::simpleTimeout(Timeout::max());
	        std::cerr << "after get time" << std::endl;
            //imu.setTimeout(timeout);
	        std::cerr << "after set time" << std::endl;
            break;
        }
	    std::cerr << "not in if\n";
    }
    std::cerr << "before open imu\n";
    imu.open();
    std::cerr << "after open imu\n";
    if (!imu.isOpen()) {
        LOGE("[ERROR]: imu init fail!");
        return false;
    }

    umt::Publisher<SensorsData> data_pub("sensors_data");
    // auto webview_checkbox = umt::ObjManager<CheckBox>::find_or_create("show raw");
    umt::Publisher<cv::Mat> raw_sub("raw");
    umt::Publisher<cv::Mat> webview_raw("raw");

    bool imu_require_stop = false;
    bool imu_is_ok = true;
    std::thread imu_capture_thread(imu_capture_loop, std::ref(imu),
                                   std::ref(imu_require_stop), std::ref(imu_is_ok));
    std::cerr << "after create imu_capture_thread\n";

    bool auto_exposure_require_stop = false;
    bool auto_exposure_is_ok = true;
    std::cerr << "after create auto_exposure thread\n";

    auto t1 = high_resolution_clock::now();
    int fps = 0, fps_count = 0;
    // 我们应该不需要这个时间措同步
    std::cerr << "before sync_once judge\n";
    // if (!sync_once(camera, imu, sync_period_ms)) {
    //     goto stop;
    // }
    std::cerr << "after sync_once judge, not goto\n";

    while (imu_is_ok && auto_exposure_is_ok) {
        cv::Mat img;
        double timestamp;
        if (!camera.read(img, timestamp)) {
            LOGE("Camera read error!");
            break;
        }
        timestamp /= 1e3;

        /* publish data */
        {
            std::unique_lock lock(seq_q_mtx);
            // LOGM("push data_pub begin");
            if(!seq_q.empty())
                data_pub.push({img, seq_q.back(), timestamp});
            // LOGM("push data_pub end");
        }
        if(show_origin){
            cv::Mat im2show = img.clone();
            fps_count++;
            auto t2 = high_resolution_clock::now();
            if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                fps = fps_count;
                fps_count = 0;
                t1 = t2;
            }
            char TextFPS[20];
            sprintf(TextFPS,"fps={ %d }\0",fps);
            cv::putText(im2show, TextFPS, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0});
            raw_sub.push(im2show);
        }
    }

    stop:
	std::cerr << "start stop label\n";
        imu_require_stop = true;
        auto_exposure_require_stop = true;
        imu_capture_thread.join();
    std::cerr << "return\n";

    return false;
}


void background_sensors_io_auto_restart(const std::string &camera_name = "", const std::string &camera_cfg = "",
                                        const std::string &sensor_param_file = "",
                                        const std::string &imu_usb_hid = "", int sync_period_ms = 10) {
    if (debug) std::cout << "============background_sensors_io_auto_restart===========\n";
    std::thread([=]() {
        while (!sensors_io(camera_name, camera_cfg, sensor_param_file, imu_usb_hid, sync_period_ms)) {
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
}

/**
 * @brief for video
 */
bool sensors_io_video(const std::string &video_name = "") {
    if (debug) std::cout << "============sensors_io_video_run===========\n";

    // for test
    std::string sensor_param_file=PROJECT_DIR"/asset/camera-param.yml";
    cv::FileStorage ifs;
    if (!ifs.open(sensor_param_file, cv::FileStorage::READ)) {
        LOGE("[ERROR]: sensor parameter read fail!");
        return false;
    }
    auto sensor_param = umt::ObjManager<SensorParam>::find_or_create("sensor_param");
    try {
        ifs["K"] >> sensor_param->K;
        ifs["D"] >> sensor_param->D;
        ifs["Tcb"] >> sensor_param->Tcb;
        if (sensor_param->K.cols != 3 || sensor_param->K.rows != 3) {
            throw std::runtime_error("sensor parameter 'K' invalid format!");
        }
        if (sensor_param->D.cols != 5 || sensor_param->D.rows != 1) {
            throw std::runtime_error("sensor parameter 'D' invalid format!");
        }
        if (sensor_param->Tcb.cols != 3 || sensor_param->Tcb.rows != 3) {
            throw std::runtime_error("sensor parameter 'Tcb' invalid format!");
        }
    } catch (cv::Exception &e) {
        LOGE("[ERROR] { %s }",e.what());
        return false;
    }



    VideoWrapper video(video_name);
    if (!video.isOpen()) {
        LOGE("[ERROR]: video init fail!");
        return false;
    }
    LOGM("[MASSAGE]: video init success!");

    umt::Publisher<SensorsData> data_pub("sensors_data");
    umt::Publisher<cv::Mat> raw_sub("raw");

    auto t1 = high_resolution_clock::now();
    int fps = 0, fps_count = 0;

    while (true) {
        cv::Mat img;
        double timestamp;
        if (!video.read(img)) {
            LOGE("Camera read error!");
            break;
        }
        timestamp /= 1e3;
        /* publish data */
        {
            std::unique_lock lock(seq_q_mtx);
            seq_q.push({0,0,0,0});
            data_pub.push({img, seq_q.back(), timestamp});
        }
        if(show_origin){
            cv::Mat im2show = img.clone();
            fps_count++;
            auto t2 = high_resolution_clock::now();
            if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                fps = fps_count;
                fps_count = 0;
                t1 = t2;
            }
            char TextFPS[20];
            sprintf(TextFPS,"fps={ %d }\0",fps);
            cv::putText(im2show, TextFPS, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0});
            raw_sub.push(im2show);
        }
        std::this_thread::sleep_for(33ms);
    }

    stop:
	std::cerr << "start stop label\n";
    std::cerr << "return\n";

    return false;
}

void background_sensors_io_video_auto_restart(const std::string &video_name = "") {
    if (debug) std::cout << "============background_sensors_io_auto_restart===========\n";
    std::thread([=]() {
        while (!sensors_io_video(video_name)) {
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
}