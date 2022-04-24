//
// Created by xixiliadorabarry on 1/24/19.
//
#include "video_wrapper.h"


VideoWrapper::VideoWrapper(const std::string &filename) {
    video.open(filename);
}

VideoWrapper::~VideoWrapper() = default;


bool VideoWrapper::isOpen() {
    return video.isOpened();
}

bool VideoWrapper::read(cv::Mat &src) {
    return video.read(src);
}
