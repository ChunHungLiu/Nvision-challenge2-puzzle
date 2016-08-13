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
#include "chamfermatching.h"


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

std::vector<cv::Point> bestMatching(cv::Mat &image, cv::Mat &temp){
	std::vector<std::vector<cv::Point>> results;
	std::vector<float> costs;

	int best = cv::chamerMatching(image, temp, results, costs, 1, 20, 1.0, 3, 3, 5, 0.6, 1.6, 0.5, 20);
	
	return results[best];
}

int main(void){
	cv::Mat image = cv::imread("12.jpg");
	cv::Mat temp = cv::imread("N1.png", CV_LOAD_IMAGE_GRAYSCALE);


	cv::Mat iedge, iedge2;
	cv::Mat tedge;
	cv::cvtColor(image, iedge2, CV_BGR2GRAY);
	edgeDetection(iedge2, iedge2, true);
	colorEdgeDetection(image, iedge, true);
	edgeDetection(temp, tedge, false);


	cv::imshow("image", image);
	cv::imshow("edge", iedge);
	cv::imshow("edge2", iedge2);
	cv::imshow("templ", temp);

	std::vector<cv::Point> best = bestMatching(iedge, tedge);

	for (int i = 0; i < best.size(); i++){
		image.at<cv::Vec3b>(best[i]) = cv::Vec3b(0, 255, 0);
	}

	cv::imshow("result", image);

	cv::waitKey(0);
	return 0;
}