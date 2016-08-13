#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <vector>
#include "Color.h"
#include "Contour.h"

std::vector<cv::Point> getPoint(cv::Mat &templ){
	Contour c(templ);
	std::vector<std::vector<cv::Point>> p = c.getContours();
	std::vector<cv::Point> pn;
	for (int i = 0; i < p.size(); i++){
		for (int j = 0; j < p[i].size(); j++){
			pn.push_back(p[i][j]);
		}
	}
	return pn;
}

void distanceTransform(cv::Mat &input, cv::Mat &output){
	cv::distanceTransform(input, output, CV_DIST_L1, 3);
	cv::normalize(output, output, 0.0, 1.0, cv::NORM_MINMAX);
}

cv::Point myChamferMatching(cv::Mat &image,cv::Mat &templ){
	std::vector<cv::Point> p = getPoint(templ);
	cv::Mat dt;
	image_not(image, dt);
	distanceTransform(dt, dt);

	cv::imshow("dt", dt);

	cv::Size isz = image.size();
	cv::Size tsz = templ.size();

	double min = 100000000;
	cv::Point minpoint;
	cv::Mat m = cv::Mat::zeros(isz, CV_32FC1);

	for (int i = 0; i < isz.height - tsz.height; i++){
		for (int j = 0; j < isz.width - tsz.width; j++){
			float cost = 0.0;

			for (int t = 0; t < p.size(); t++){
				cost += dt.at<float>(i + p[t].y, j + p[t].x);
			}
			m.at<float>(i, j) = cost;
			if (cost < min){
				min = cost;
				minpoint = cv::Point(j, i);
			}

		}
	}
	std::cout << "cost = " << min << std::endl;
	cv::normalize(m, m, 0.0, 1.0, cv::NORM_MINMAX);

	cv::imshow("norm", m);

	return minpoint;
}