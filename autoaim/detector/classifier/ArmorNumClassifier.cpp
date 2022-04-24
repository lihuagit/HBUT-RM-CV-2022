/*
*	@Author: Mountain
*	@Date:	 2020.04.13
*	@Brief:  This cpp file define the ArmorNumClassifier class, realize some function used 
*/

#include "ArmorNumClassifier.h"

ArmorNumClassifier::ArmorNumClassifier(){
	svm = cv::ml::SVM::create();
	armorImgSize = cv::Size(40, 40);
	p = cv::Mat();
	
	warpPerspective_mat = cv::Mat(3, 3, CV_32FC1);
	dstPoints[0] = cv::Point2f(0, 0);
	dstPoints[1] = cv::Point2f(armorImgSize.width, 0);
	dstPoints[2] = cv::Point2f(armorImgSize.width, armorImgSize.height);
	dstPoints[3] = cv::Point2f(0, armorImgSize.height);
}

ArmorNumClassifier::~ArmorNumClassifier(){}

void ArmorNumClassifier::loadSvmModel(const char * model_path, cv::Size armorImgSize) {
	svm = cv::ml::StatModel::load<cv::ml::SVM>(model_path);
    if(svm.empty())
    {
		LOGE("Svm load error! Please check the path!");
        exit(0);
    }
	this->armorImgSize = armorImgSize;

	//set dstPoints (the same to armorImgSize, as it can avoid resize armorImg)
	dstPoints[0] = cv::Point2f(0, 0);
	dstPoints[1] = cv::Point2f(armorImgSize.width, 0);
	dstPoints[2] = cv::Point2f(armorImgSize.width, armorImgSize.height);
	dstPoints[3] = cv::Point2f(0, armorImgSize.height);
}

void ArmorNumClassifier::loadImg(const cv::Mat & srcImg){

	//copy srcImg as warpPerspective_src
	(srcImg).copyTo(warpPerspective_src);

	//preprocess srcImg for the goal of acceleration
	cvtColor(warpPerspective_src, warpPerspective_src, 6);  //CV_BGR2GRAY=6
	threshold(warpPerspective_src, warpPerspective_src, 10, 255, cv::THRESH_BINARY);
}

void ArmorNumClassifier::getArmorImg(cv::Mat & roi, cv::Point2f pts[]){
	//set the armor vertex as srcPoints
	for (int i = 0; i < 4; i++)
		srcPoints[i] = pts[i];

	//get the armor image using warpPerspective
	warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);  // get perspective transform matrix  透射变换矩阵
	warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, armorImgSize, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0)); //warpPerspective to get armorImage
	warpPerspective_dst.copyTo(roi); //copyto armorImg
}

int ArmorNumClassifier::setArmorNum(cv::Mat & roi){

	// adapt armorImg to the SVM model sample-size requirement
	p = roi.reshape(1, 1);
	p.convertTo(p, CV_32FC1);

	//set armor number according to the result of SVM  
	return (int)svm->predict(p);
}
