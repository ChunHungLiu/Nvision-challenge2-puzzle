/********************************************************************
**                                                                 **
**     ChamferMatchi                           -Trial ver-         **
**                                                                 **
**          Created by Ending2012 (103062372) on 2016/8/9          **
**                                                                 **
**        Copyright (c) 2012 End of APP. All rights reserved.      **
**                                                                 **
*********************************************************************/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <string>

#include "Color.h"
#include "Contour.h"
#include "Detector.h"
#include "ChamferMatcher.h"


void read_template(std::string name, cv::Mat &temp){
	cv::Mat gray;
	cv::cvtColor(temp, gray, CV_BGR2GRAY);
	cv::Canny(gray, gray, 50, 100, 3);
	Contour c(gray);
	std::vector<std::vector<cv::Point>> p = c.getContours();
	for (int i = 0; i < p.size(); i++){
		for (int j = 0; j < p[i].size(); j++){

		}
	}
}
/*
std::vector<cv::Point> bestMatching(cv::Mat &image, cv::Mat &temp){
	std::vector<std::vector<cv::Point>> results;
	std::vector<float> costs;

	int best = cv::orientationChamferMatching(image, temp, results, costs,30, 1, 20, 1.0, 3, 3, 5, 0.6, 1.6, 0.5, 20);

	if (best == -1)return std::vector<cv::Point>();
	return results[best];
}*/



int main(void){
	cv::Mat image = cv::imread("12.jpg");
	cv::Mat temp = cv::imread("N1.png", CV_LOAD_IMAGE_GRAYSCALE);


	cv::Mat iedge;
	cv::Mat tedge;
	colorEdgeDetection(image, iedge, true);
	edgeDetection(temp, tedge, false);

	//rotate(tedge, tedge, -10);
	cv::imshow("myedge", tedge);


	//std::vector<cv::Point> best = bestMatching(iedge, tedge);

	//for (int i = 0; i < best.size(); i++){
	//	image.at<cv::Vec3b>(best[i]) = cv::Vec3b(0, 255, 0);
	//}

	ending::Template t(tedge);
	ending::ChamferMatcher cmatcher;

	cv::Mat dis, ori;
	

	cv::imshow("result", ori);

	cv::waitKey(0);
	return 0;
}