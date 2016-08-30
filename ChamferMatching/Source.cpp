/********************************************************************
**                                                                 **
**     ChamferMatching                         - ver 4.1 -         **
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

int main(void){
	cv::Mat image = cv::imread("12.jpg");
	cv::Mat temp1 = cv::imread("F.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat temp2 = cv::imread("N1.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::resize(temp1, temp1, cv::Size(temp1.size().width / 3, temp1.size().height / 3));

	ending::DEBUG_img = image.clone();

	cv::Mat iedge;
	cv::Mat tedge1;
	cv::Mat tedge2;
	colorEdgeDetection(image, iedge, true);
	edgeDetection(temp1, tedge1, false);
	edgeDetection(temp2, tedge2, false);
	//cv::imshow("edge", iedge);
	//rotate(tedge, tedge, -10);
	//cv::imshow("temp edge1", tedge1);
	//cv::imshow("temp edge2", tedge2);
	//ending::debugimg = image.clone();

	ending::RChamferMatcher cmatcher(1, 20, 1.0, 3, 3, 5, 0.6, 1.6, 0.5, 20, 5);

	std::vector<ending::Matcher::MatchPoints> matchpoints;
	
	cmatcher.addMatcher(tedge1);
	cmatcher.addMatcher(tedge2);

	//cv::chamerMatching(iedge, tedge, results, costs, 1, 20, 1.0, 3, 3, 5, 0.6, 1.6, 0.5, 20);


	double startTime, endTime;
	startTime = (double)clock();
	cmatcher.multimatching(iedge, matchpoints);

	endTime = (double)clock();

	PROCESS_MEMORY_COUNTERS beginpms, endpms;

	GetProcessMemoryInfo(GetCurrentProcess(), &beginpms, sizeof(beginpms));

	for (int i = 0; i < matchpoints.size(); i++){
		if (matchpoints[i].size() <= 0){
			std::cout << "No matching in matcher [" << i << "] ..." << std::endl;
			continue;
		}
		ending::Matcher::MatchPoint &mp = matchpoints[i][0];
		int max_ = (mp.getBoundingBoxSize().width > mp.getBoundingBoxSize().height ? mp.getBoundingBoxSize().width : mp.getBoundingBoxSize().height);
		cv::circle(image, mp.getBoundingBoxCenter(), max_/2, cv::Scalar(0, 255, 0));
		
	}

	GetProcessMemoryInfo(GetCurrentProcess(), &endpms, sizeof(endpms));

	std::cout << "Peak Memory Usage: " << endpms.PeakWorkingSetSize - beginpms.WorkingSetSize << std::endl;
	std::cout << "Memory Usage: " << endpms.WorkingSetSize - beginpms.WorkingSetSize << std::endl;
	std::cout << "Take: " << (endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;


    //cv::imwrite("result2.png", image);
	cv::imshow("result2", image);
	cv::imshow("asd", ending::DEBUG_img);
	cv::waitKey(0);
	return 0;
}
