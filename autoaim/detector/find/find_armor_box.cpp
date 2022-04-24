//
// Created by xinyang on 19-7-18.
//
#include "../detector.h"

#define DO_NOT_CNT_TIME

#include <log.h>

// 判断两个灯条的角度差
static bool angelJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    return abs(angle_i - angle_j) < 20;
}
// 判断两个灯条的高度差
static bool heightJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    return abs(centers.y) < 30;
}
// 判断两个灯条的间距
static bool lengthJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    double side_length;
    cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
    side_length = sqrt(centers.ddot(centers));
    return (side_length / light_blob_i.length < 10 && side_length / light_blob_i.length > 0.5);
}
// 判断两个灯条的长度比
static bool lengthRatioJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    return (light_blob_i.length / light_blob_j.length < 2.5
            && light_blob_i.length / light_blob_j.length > 0.4);
}

/* 判断两个灯条的错位度，不知道英文是什么！！！ */
static bool CuoWeiDuJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0 / 180.0 * 3.14159265459;
    if (abs(angle_i - angle_j) > 90) {
        angle += 3.14159265459 / 2;
    }
    Vector2f orientation(cos(angle), sin(angle));
    Vector2f p2p(light_blob_j.rect.center.x - light_blob_i.rect.center.x,
                 light_blob_j.rect.center.y - light_blob_i.rect.center.y);
    return abs(orientation.dot(p2p)) < 25;
}
// 判断装甲板方向
static bool boxAngleJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
                    light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
                    light_blob_j.rect.angle - 90;
    float angle = (angle_i + angle_j) / 2.0;
    if (abs(angle_i - angle_j) > 90) {
        angle += 90.0;
    }
    return (-120.0 < angle && angle < -60.0) || (60.0 < angle && angle < 120.0);
}
// 判断两个灯条是否可以匹配
static bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, uint8_t enemy_color) {
    return light_blob_i.blob_color == enemy_color &&
           light_blob_j.blob_color == enemy_color &&
           lengthRatioJudge(light_blob_i, light_blob_j) &&
           lengthJudge(light_blob_i, light_blob_j) &&
           //           heightJudge(light_blob_i, light_blob_j) &&
           angelJudge(light_blob_i, light_blob_j) &&
           boxAngleJudge(light_blob_i, light_blob_j) &&
           CuoWeiDuJudge(light_blob_i, light_blob_j);

}
// 匹配所有灯条，得出装甲板候选区
bool matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes) {
    armor_boxes.clear();
    int len=light_blobs.size();
    for (int i = 0; i < len - 1; ++i) {
        for (int j = i + 1; j < len; ++j) {
            if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j), enemy_color)) {
                continue;
            }
            cv::Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(i)).rect.boundingRect();
            cv::Rect2d rect_right = light_blobs.at(static_cast<unsigned long>(j)).rect.boundingRect();
            double min_x, min_y, max_x, max_y;
            min_x = fmin(rect_left.x, rect_right.x) - 4;
            max_x = fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
            min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
            max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) +
                    0.5 * (rect_left.height + rect_right.height) / 2.0;
            if (min_x < 0 || max_x > src.cols || min_y < 0 || max_y > src.rows) {
                continue;
            }
            if ((max_x - min_x) / (max_y - min_y) < 0.8) continue;
            LightBlobs pair_blobs = {light_blobs.at(i), light_blobs.at(j)};
            armor_boxes.emplace_back();
            ArmorBox &box=armor_boxes.back();
            box.rect=cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
            box.box_color=enemy_color;

            // 更精确的框选装甲板
            auto blob=light_blobs[i];
            cv::Point2f cen=blob.rect.center;
            std::vector<cv::Point2f> ans1;
            double ang;
            if(blob.rect.size.width>blob.rect.size.height){
                ang=blob.rect.angle;
            }
            else ang=-blob.rect.angle;
            double heigh=max(blob.rect.size.width,blob.rect.size.height);
            heigh=max( heigh,(double)max(blob.rect.size.width,blob.rect.size.height) );
            double h=sin((ang)*M_PI/180)*heigh;
            h/=2;
            double w=cos((ang)*M_PI/180)*heigh;
            w/=2;
            if(h<w) swap(w,h);
            ans1.push_back({cen.x+w,cen.y+h});
            ans1.push_back({cen.x-w,cen.y-h});
            sort(ans1.begin(),ans1.end(),[](cv::Point2f a,cv::Point2f b){ return a.y<b.y; });
            
            blob=light_blobs[j];
            cen=blob.rect.center;
            std::vector<cv::Point2f> ans2;
            ang;
            if(blob.rect.size.width>blob.rect.size.height){
                ang=blob.rect.angle;
            }
            else ang=-blob.rect.angle;
            // heigh=max(blob.rect.size.width,blob.rect.size.height);
            h=sin((ang)*M_PI/180)*heigh;
            h/=2;
            w=cos((ang)*M_PI/180)*heigh;
            w/=2;
            if(h<w) swap(w,h);
            ans2.push_back({cen.x+w,cen.y+h});
            ans2.push_back({cen.x-w,cen.y-h});
            sort(ans2.begin(),ans2.end(),[](cv::Point2f a,cv::Point2f b){ return a.y<b.y; });

            if(ans1[0].x>ans2[0].x){
                swap(ans1[0],ans2[0]);
                swap(ans1[1],ans2[1]);
            }
            box.pts[0]=ans1[0];
            box.pts[1]=ans1[1];
            box.pts[2]=ans2[1];
            box.pts[3]=ans2[0];
        }
    }
    return !armor_boxes.empty();
}
// 在给定的图像上寻找装甲板
std::vector<bbox_t> findArmorBox(const cv::Mat &src,const LightBlobs &light_blobs,Classifier &classifier) {
    // light_blobs 存储所有可能的灯条
    // armor_boxes 装甲板候选区
    ArmorBoxes may_box;
    std::vector<bbox_t> rst;
    rst.reserve(20);
    
    // 对灯条进行匹配得出装甲板候选区
    matchArmorBoxes(src, light_blobs, may_box);
    
    // 如果分类器可用，则使用分类器对装甲板候选区进行筛选
    if (classifier) {
        for (auto &armor_box : may_box) {
            cv::Mat roi = src(armor_box.rect).clone();
            cv::resize(roi, roi, cv::Size(48, 36));
            int c = classifier(roi);
            armor_box.tag_id = c;
        }
// 按照优先级对装甲板进行排序
        sort(may_box.begin(), may_box.end(), [&](const ArmorBox &a, const ArmorBox &b) { return a < b; });
        for (auto &one_box : may_box) {
            if (one_box.tag_id != 0) {
                rst.emplace_back();
                auto &box=rst.back();
                for(int i=0;i<4;i++)
                    box.pts[i]=one_box.pts[i];
                box.rect=one_box.rect;
                box.tag_id=one_box.tag_id;
                box.color_id=one_box.box_color;
            }
        }
        // if (save_labelled_boxes) {
        //     for (const auto &one_box : may_box) {
        //         char filename[100];
        //         sprintf(filename, PROJECT_DIR"/armor_box_photo/%s_%d.jpg", id2name[one_box.id].data(),
        //                 time(nullptr) + clock());
        //         auto box_roi = src(one_box.rect);
        //         cv::resize(box_roi, box_roi, cv::Size(48, 36));
        //         cv::imwrite(filename, box_roi);
        //     }
        // }
    } else { // 如果分类器不可用，则直接选取候选区中的第一个区域作为目标(往往会误识别)
        LOGE("classifier error!!!");
    }
    return rst;
}



// 在给定的图像上寻找装甲板
std::vector<bbox_t> findArmorBox_2(const cv::Mat &src,const LightBlobs &light_blobs,ArmorNumClassifier &classifier) {
    // light_blobs 存储所有可能的灯条
    // armor_boxes 装甲板候选区
    ArmorBoxes may_box;
    std::vector<bbox_t> rst;
    rst.reserve(20);
    
    // 对灯条进行匹配得出装甲板候选区
    matchArmorBoxes(src, light_blobs, may_box);

    classifier.loadImg(src);
    
    // 如果分类器可用，则使用分类器对装甲板候选区进行筛选
    // if (classifier) {
    if (true) {
        for (auto &armor_box : may_box) {
            cv::Mat roi = src(armor_box.rect).clone();
            cv::resize(roi, roi, cv::Size(48, 36));
            classifier.getArmorImg(roi,armor_box.pts);
            armor_box.tag_id = classifier.setArmorNum(roi);
        }
        for (auto &one_box : may_box) {
            if (one_box.tag_id != 0 || true) {
                rst.emplace_back();
                auto &box=rst.back();
                box.tag_id=one_box.tag_id;
                box.color_id=one_box.box_color;
            }
        }
        // if (save_labelled_boxes) {
        //     for (const auto &one_box : may_box) {
        //         char filename[100];
        //         sprintf(filename, PROJECT_DIR"/armor_box_photo/%s_%d.jpg", id2name[one_box.id].data(),
        //                 time(nullptr) + clock());
        //         auto box_roi = src(one_box.rect);
        //         cv::resize(box_roi, box_roi, cv::Size(48, 36));
        //         cv::imwrite(filename, box_roi);
        //     }
        // }
    } else { // 如果分类器不可用，则直接选取候选区中的第一个区域作为目标(往往会误识别)
        LOGE("classifier error!!!");
    }
    return rst;
}


// 在给定的图像上寻找装甲板
bbox_t findArmorBox_3(const cv::Mat &src,const LightBlobs &light_blobs,Classifier &classifier) {
    // LightBlobs light_blobs; // 存储所有可能的灯条
    ArmorBoxes armor_boxes; // 装甲板候选区

    bbox_t res_bbox;
    res_bbox.tag_id=0;
    
    // 对灯条进行匹配得出装甲板候选区
    matchArmorBoxes(src, light_blobs, armor_boxes);

// 如果分类器可用，则使用分类器对装甲板候选区进行筛选
    if (classifier) {
        for (auto &armor_box : armor_boxes) {
            cv::Mat roi = src(armor_box.rect).clone();
            cv::resize(roi, roi, cv::Size(48, 36));
            int c = classifier(roi);
            armor_box.tag_id = c;
        }
// 按照优先级对装甲板进行排序
        sort(armor_boxes.begin(), armor_boxes.end(), [&](const ArmorBox &a, const ArmorBox &b) { return a < b; });
        for (auto &one_box : armor_boxes) {
            if (one_box.tag_id != 0) {
                res_bbox.rect=one_box.rect;
                res_bbox.tag_id=one_box.tag_id;
                res_bbox.color_id=one_box.box_color;
                for(int i=0;i<4;i++)
                    res_bbox.pts[i]=one_box.pts[i];
                break;
            }
        }
        if (save_labelled_boxes) {
            for (const auto &one_box : armor_boxes) {
                char filename[100];
                sprintf(filename, PROJECT_DIR"/armor_box_photo/%s_%d.jpg", id2name[one_box.tag_id].data(),
                        time(nullptr) + clock());
                auto box_roi = src(one_box.rect);
                cv::resize(box_roi, box_roi, cv::Size(48, 36));
                cv::imwrite(filename, box_roi);
            }
        }
    } else { // 如果分类器不可用，则直接选取候选区中的第一个区域作为目标(往往会误识别)
        auto &one_box = armor_boxes[0];
        res_bbox.rect=one_box.rect;
        res_bbox.tag_id=one_box.tag_id;
        res_bbox.color_id=one_box.box_color;
        for(int i=0;i<4;i++)
            res_bbox.pts[i]=one_box.pts[i];
    }
    return res_bbox;
}