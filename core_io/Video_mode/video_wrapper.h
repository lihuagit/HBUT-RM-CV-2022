/**
 * @file video_wrapper.h
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */

#ifndef _CVRM2022_VIDEO_WRAPPER_H
#define _CVRM2022_VIDEO_WRAPPER_H


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class VideoWrapper {
public:
    VideoWrapper(const std::string& filename);
    ~VideoWrapper();
    bool isOpen();
    bool read(cv::Mat &src);
private:
    cv::VideoCapture video;

};


#endif //_CVRM2022_VIDEO_WRAPPER_H
