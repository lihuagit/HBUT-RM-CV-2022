/**
 * @file options.h
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */
#ifndef _OPTIONS_H_
#define _OPTIONS_H_

#define ENEMY_BLUE 0
#define ENEMY_RED  1

extern bool show_armor_box;
extern bool show_armor_boxes;
extern bool show_light_blobs;
extern bool show_origin;
extern bool run_with_camera;
extern bool save_video;
extern bool wait_uart;
extern bool save_labelled_boxes;
extern bool show_process;
extern bool show_energy;
extern bool save_mark;
extern bool show_info;
extern bool run_by_frame;
extern bool is_predictorKalman;
extern bool is_predictorEKF;
extern bool is_predictor;
extern int shoot_delay_t;
extern int robot_speed_mps;
extern int enemy_color;

#endif /* _OPTIONS_H_ */
