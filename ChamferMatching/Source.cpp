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
#include <ctime>

#include "Color.h"
#include "Contour.h"
#include "Detector.h"
#include "ChamferMatcher.h"





int main(void){
	cv::Mat image = cv::imread("12.jpg");
	cv::Mat temp1 = cv::imread("F.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat temp2 = cv::imread("N1.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::resize(temp1, temp1, cv::Size(temp1.size().width / 2, temp1.size().height / 2));

	cv::Mat iedge;
	cv::Mat tedge1;
	cv::Mat tedge2;
	colorEdgeDetection(image, iedge, true);
	edgeDetection(temp1, tedge1, false);
	edgeDetection(temp2, tedge2, false);

	//rotate(tedge, tedge, -10);
	cv::imshow("temp edge1", tedge1);
	cv::imshow("temp edge2", tedge2);


	ending::ChamferMatcher cmatcher;
	std::vector<std::vector<std::vector<cv::Point>>> results;
	std::vector<std::vector<float>> costs;

	cv::Mat dis, ori;
	cv::Mat ori2;

	cmatcher.addMatcher(tedge1);
	cmatcher.addMatcher(tedge2);

	//cv::chamerMatching(iedge, tedge, results, costs, 1, 20, 1.0, 3, 3, 5, 0.6, 1.6, 0.5, 20);


	double startTime, endTime;
	startTime = (double)clock();
	std::vector<int> best = cmatcher.multimatching(iedge, results, costs);

	endTime = (double)clock();
	for (int i = 0; i < best.size(); i++){
		if (best[i] == -1){
			std::cout << "No matching in matcher [" << i << "] ..." << std::endl;
			continue;
		}

		std::cout << "Template " << i + 1 << " center: (" << results[i][best[i]][0].x << ", " << results[i][best[i]][0].y << ")" << std::endl;
		for (int j = 0; j < results[i][best[i]].size(); j++){
			if (j == 0)image.at<cv::Vec3b>(results[i][best[i]][j]) = cv::Vec3b(0, 0, 255);
			else image.at<cv::Vec3b>(results[i][best[i]][j]) = cv::Vec3b(0, 255, 0);
		}
	}
	
	std::cout << "Take: " << (endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

	
	

	cv::imshow("result", image);

	cv::waitKey(0);
	return 0;
}