/********************************************************************
**                                                                 **
**     ChamferMatching                         - ver 4.3 -         **
**                                                                 **
**          Created by Ending2012 (103062372) on 2016/8/9          **
**                                                                 **
**        Copyright (c) 2012 End of APP. All rights reserved.      **
**                              E-mail: joe1397426985@gmail.com    **
*********************************************************************/

#define __CHAMFER_DEBUG_MODE___
#define __CHAMFER_LOW_MEMORY___

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <ctime>
#include <cstdio>

#include <Windows.h>
#include <Psapi.h>

#include "Color.h"
#include "Contour.h"
#include "Detector.h"
#include "ChamferMatcher.h"

std::vector<cv::Rect> list(){
	std::vector<cv::Rect> b;
	b.push_back(cv::Rect(23, 160, 100, 93));
	b.push_back(cv::Rect(117, 172, 93, 85));
	b.push_back(cv::Rect(203, 171, 99, 90));
	b.push_back(cv::Rect(287, 172, 84, 82));

	return b;
}

int main(void){
	cv::Mat image = cv::imread("image.jpg");
	//cv::Mat temp1 = cv::imread("F.png", CV_LOAD_IMAGE_GRAYSCALE);
	//cv::Mat temp2 = cv::imread("N1.png", CV_LOAD_IMAGE_GRAYSCALE);
	//cv::resize(temp1, temp1, cv::Size(temp1.size().width / 3, temp1.size().height / 3));

	ending::DEBUG_img = image.clone();

	cv::Mat iedge;
	//cv::Mat tedge1;
	//cv::Mat tedge2;
	colorEdgeDetection(image, iedge, true);
	//edgeDetection(temp1, tedge1, false);
	//edgeDetection(temp2, tedge2, false);
	//cv::imshow("edge", iedge);
	//rotate(tedge, tedge, -10);
	//cv::imshow("temp edge1", tedge1);
	//cv::imshow("temp edge2", tedge2);
	//ending::debugimg = image.clone();

	ending::RChamferMatcher cmatcher(1, 20, 1.0, 3, 3, 3, 0.8, 1.2, 0.5, 20, 5);

	std::vector<cv::Rect> boundingBoxList = list();

	std::vector<ending::Matcher::MatchPoints> matchpoints;
	
	for (char a = 'A'; a <= 'D'; a++){
		char name[50];
		sprintf(name, "edge_x04/%c.png", a);
		cv::Mat templ = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
		std::cout << "Loading template: " << name << std::endl;
		cmatcher.addMatcher(templ);
	}

	//cv::chamerMatching(iedge, tedge, results, costs, 1, 20, 1.0, 3, 3, 5, 0.6, 1.6, 0.5, 20);


	double startTime, endTime;
	startTime = (double)clock();
	cmatcher.multimatching(iedge, boundingBoxList, matchpoints);

	endTime = (double)clock();

	PROCESS_MEMORY_COUNTERS beginpms, endpms;

	GetProcessMemoryInfo(GetCurrentProcess(), &beginpms, sizeof(beginpms));

	for (int i = 0; i < matchpoints.size(); i++){
		if (matchpoints[i].size() <= 0){
			std::cout << "No any matching results in matcher [" << i << "] ..." << std::endl;
			continue;
		}

		ending::Matcher::MatchPoint &mp = matchpoints[i][0];
		cv::Point matchCenter = mp.getBoundingBoxCenter();
		double rotateAngle = mp.getAngle();
		double scaleSize = mp.getScale();
		double matchCost = mp.getCost();
		cv::Size matchSize = mp.getBoundingBoxSize();


		int maxRadius_ = (matchSize.width > matchSize.height ? matchSize.width : matchSize.height);
		maxRadius_ = (maxRadius_ + 1) / 2;
		cv::circle(image, matchCenter, maxRadius_, cv::Scalar(0, 255, 0));
		cv::Point lineEnd(matchCenter.x - (int)(sin(rotateAngle)*maxRadius_ + 0.5), matchCenter.y - (int)(cos(rotateAngle)*maxRadius_ + 0.5));
		cv::line(image, matchCenter, lineEnd, cv::Scalar(0, 255, 0));

		if (i == 0)std::cout << "1st ";
		else if (i == 1)std::cout << "2nd ";
		else if (i == 2)std::cout << "3rd ";
		else std::cout << i + 1 << "th ";

		std::cout << "Best matching result: " << std::endl;
		std::cout << "\t Center: (" << matchCenter.x << ", " << matchCenter.y << ") " << std::endl;
		std::cout << "\t Box: (" << matchCenter.x - matchSize.width / 2 << ", " << matchCenter.y - matchSize.height / 2 << ") ";
		std::cout << "(" << matchCenter.x + matchSize.width / 2 << ", " << matchCenter.y + matchSize.height / 2 << ") " << std::endl;
		std::cout << "\t Angle: " << (rotateAngle / CV_PI * 180.0) << std::endl;
		std::cout << "\t Scale: " << scaleSize << std::endl;
		std::cout << "\t Cost: " << matchCost << std::endl;
		std::cout << std::endl;
	}

	GetProcessMemoryInfo(GetCurrentProcess(), &endpms, sizeof(endpms));

	std::cout << "Peak Memory Usage: " << endpms.PeakWorkingSetSize - beginpms.WorkingSetSize << std::endl;
	std::cout << "Memory Usage: " << endpms.WorkingSetSize - beginpms.WorkingSetSize << std::endl;
	std::cout << "Take: " << (endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;


    //cv::imwrite("result2.png", image);
	cv::imshow("result2", image);
	cv::imwrite("results.png", image);
	cv::imshow("asd", ending::DEBUG_img);
	cv::imwrite("debug.png", ending::DEBUG_img);
	cv::waitKey(0);
	return 0;
}
