#include <iostream>
#include <sensors.hpp>
#include <robot.hpp>
#include <autoaim.hpp>
#include <options.h>
#include <img_show.h>
using namespace std;

int main(){
    show_origin=true;
    show_armor_boxes=true;
    show_light_blobs=false;
    enemy_color=ENEMY_RED;
    cout<<PROJECT_DIR<<endl;
    printf("main start!!!\n");
    // background_sensors_io_video_auto_restart(PROJECT_DIR"/video/8-11东大3No.4-装甲板-1.mp4");
    // background_sensors_io_video_auto_restart(PROJECT_DIR"/video/8-11东大3No.4.mp4");
    // background_sensors_io_video_auto_restart(PROJECT_DIR"/video/red_3.mp4");
    background_sensors_io_auto_restart("camera_1",PROJECT_DIR"/asset/armor_lihua_3.config",PROJECT_DIR"/asset/camera-param.yml","USB VID:PID=0483:5740 SNR=207233AB374E",10);
    // background_find_light_blobs_run();
    background_find_armor_boxs_run(PROJECT_DIR"/asset/para/");
    // background_none_predict_run();
    background_predict_run();
    background_robot_io_usb_auto_restart("USB VID:PID=0483:5740 SNR=207233AB374E");
    // background_find_armor_boxs_run(PROJECT_DIR"/asset/123svm.xml");
    background_img_show_run();
    // background_none_predict_run();
    while(true);
    printf("main end!!!\n");
    return 0;
}