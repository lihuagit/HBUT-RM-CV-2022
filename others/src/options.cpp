//
// Created by xinyang on 19-3-27.
//

#include <options.h>
#include <log.h>
#include <cstring>
#include <map>

bool show_armor_box = false;
bool show_armor_boxes = false;
bool show_light_blobs = false;
bool show_origin = false;
bool run_with_camera = false;
bool save_video = false;
bool wait_uart = false;
bool save_labelled_boxes = false;
bool show_process = false;
bool show_energy = false;
bool save_mark = false;
bool show_info = false;
bool run_by_frame = false;
bool is_predictor=false;
bool is_predictorKalman=false;
bool is_predictorEKF=false;
int shoot_delay_t=200;
int robot_speed_mps=15;
int enemy_color=ENEMY_RED;