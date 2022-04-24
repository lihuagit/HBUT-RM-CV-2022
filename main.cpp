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
    background_sensors_io_video_auto_restart(PROJECT_DIR"/video/red_3.mp4");
    // background_find_light_blobs_run();
    background_find_armor_boxs_run(PROJECT_DIR"/asset/para/");
    background_none_predict_run();
    // background_find_armor_boxs_run(PROJECT_DIR"/asset/123svm.xml");
    background_img_show_run();
    // background_none_predict_run();
    while(true);
    printf("main end!!!\n");
    return 0;
}