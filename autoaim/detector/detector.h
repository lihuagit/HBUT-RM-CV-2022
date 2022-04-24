/**
 * @file detection.h
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */
#ifndef _DETECTION_H_
#define _DETECTION_H_

#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <options.h>
#include <log.h>
#include <Eigen/Eigen>
#include "classifier/classifier.h"
#include "classifier/ArmorNumClassifier.h"

#define BLOB_RED    ENEMY_RED
#define BLOB_BLUE   ENEMY_BLUE

#define BOX_RED     ENEMY_RED
#define BOX_BLUE    ENEMY_BLUE

#define IMAGE_CENTER_X      (320)
#define IMAGE_CENTER_Y      (240-20)

#define DISTANCE_HEIGHT_5MM (10700.0)     // 单位: cm*pixel
#define DISTANCE_HEIGHT     DISTANCE_HEIGHT_5MM

// #define         B1 1
// #define         B2 2
// #define         B3 3
// #define         B4 4
// #define         B5 5
// #define         B7 6
// #define         B8 7
// #define         R1 8
// #define         R2 9
// #define         R3 10
// #define         R4 11
// #define         R5 12
// #define         R7 13
// #define         R8 14

extern std::map<int, string> id2name;   //装甲板id到名称的map
extern std::map<string, int> name2id;   //装甲板名称到id的map
extern std::map<string, int> prior_blue;
extern std::map<string, int> prior_red;

typedef cv::TrackerKCF TrackerToUse;                // Tracker类型定义

/******************* 灯条类定义 ***********************/
class LightBlob {
public:
    cv::RotatedRect rect;   //灯条位置
    double area_ratio;
    double length;          //灯条长度
    uint8_t blob_color;      //灯条颜色

    LightBlob(cv::RotatedRect &r, double ratio, uint8_t color) : rect(r), area_ratio(ratio), blob_color(color) {
        length = max(rect.size.height, rect.size.width);
    };
    LightBlob() = default;
};

typedef std::vector<LightBlob> LightBlobs;



/******************* 装甲板类定义　**********************/
class ArmorBox{
public:
    cv::Rect2d rect;
    cv::Point2f pts[4];
    uint8_t box_color;
    int tag_id;

    // explicit ArmorBox(const cv::Rect &pos=cv::Rect2d(), uint8_t color=0, int i=0);
    ArmorBox() = default;
    bool operator<(const ArmorBox &box) const; // 装甲板优先级比较
};

typedef std::vector<ArmorBox> ArmorBoxes;


/******************* 打包数据　**********************/
struct alignas(4) bbox_t {
    cv::Point2f pts[4]; // [pt0, pt1, pt2, pt3]
    cv::Rect2d rect;
    float confidence;
    int color_id; // 0: blue, 1: red, 2: gray
    int tag_id;   // 0: error, 1-14: number
    bbox_t(){ }
    bool operator==(const bbox_t& a) const {
        return pts==a.pts;
    }
    bool operator!=(const bbox_t& a) const {
        return pts!=a.pts;
    }
};

struct Light_blobs_pack{
    /*
     * 打包数据结构，将识别灯条结果、对应的图像、陀螺仪和时间戳对应
     */
    LightBlobs light_blobs;
    cv::Mat img;
    std::array<double, 4> q;
    double timestamp;
};

bool findLightBlobs(const cv::Mat &src, LightBlobs &light_blobs);
bool findLightBlobs_2(const cv::Mat &src, LightBlobs &light_blobs);
std::vector<bbox_t> findArmorBox(const cv::Mat &src,const LightBlobs &light_blobs,Classifier &classifier);
std::vector<bbox_t> findArmorBox_2(const cv::Mat &src,const LightBlobs &light_blobs,ArmorNumClassifier &classifier);
bbox_t findArmorBox_3(const cv::Mat &src,const LightBlobs &light_blobs,Classifier &classifier);

#endif  // _DETECTION_H_