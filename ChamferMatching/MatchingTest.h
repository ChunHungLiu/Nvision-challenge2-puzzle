#pragma once

/*	Chamfer headers */

#define __CHAMFER_DEBUG_MODE___
#define __CHAMFER_LOW_MEMORY___

#include "Color.h"
#include "Contour.h"
#include "Detector.h"
#include "ChamferMatcher.h"
#include "INIParser.h"

/*	OpenCV headers */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <map>


namespace ending{
	class MatchingTest{
	private:

		ending::INIparser parser;
		cv::Mat cimage;
		cv::Mat _image;
		cv::Mat _distimg;
		cv::Mat _orientimg;

		ending::RChamferMatcher::MatcherConfig platemc;
		ending::RChamferMatcher::MatcherConfig puzzmc;
		ending::RChamferMatcher::MatcherConfig holemc;

		ending::Matcher::MatchPoints platemp;
		std::vector<ending::Matcher::MatchPoints> puzzmp;
		ending::Matcher::MatchPoints holemp;
		std::map<char, cv::Point> posmap;

		std::string platepath;
		std::string puzzpath;
		std::string holepath;
		cv::Vec2d bboxscale;

		void getRChamferMatcherConfig(const std::string &section, ending::RChamferMatcher::MatcherConfig &mc){
			mc.setTemplScale(parser.getDouble(section, "SCALE"));
			mc.setMaxMatches(parser.getInt(section, "MAXMATCHES"));
			mc.setMinMatchDistance(parser.getDouble(section, "MINMATCHDISTANCE"));
			mc.setPadX(parser.getInt(section, "XSTEP"));
			mc.setPadY(parser.getInt(section, "YSTEP"));
			mc.setScales(parser.getInt(section, "SCALES"));
			mc.setMinScale(parser.getDouble(section, "MINSCALE"));
			mc.setMaxScale(parser.getDouble(section, "MAXSCALE"));
			mc.setOrientationWeight(parser.getDouble(section, "ORIENTATIONWEIGHT"));
			mc.setTruncate(parser.getDouble(section, "THRESHOLD"));
			mc.setAngularVelocity(parser.getDouble(section, "ROTATION"));
		}

		cv::Point getCVPoint(const std::string &section, const std::string &name){
			std::string str = parser.getString(section, name);
			std::stringstream ss(str.substr(0, str.find(',')));
			int x, y;
			ss >> x;
			std::stringstream ss1(str.substr(str.find(',') + 1, std::string::npos));
			ss1 >> y;

			return cv::Point(x, y);
		}

		cv::Vec2d getCVVec2d(const std::string &section, const std::string &name){
			std::string str = parser.getString(section, name);
			std::stringstream ss(str.substr(0, str.find(',')));
			double x, y;
			ss >> x;
			std::stringstream ss1(str.substr(str.find(',') + 1, std::string::npos));
			ss1 >> y;
			return cv::Vec2d(x, y);
		}

		void getHolePos(const std::string &section, std::map<char, cv::Point> &map){
			char c[2] = {};
			for (c[0] = 'A'; c[0] <= 'Z'; c[0]++){
				cv::Point p = getCVPoint(section, std::string(c));
				map[c[0]] = p;
			}
		}

		cv::Point pointTrans(cv::Point puz, ending::Matcher::MatchPoint &plate){
			double sc = plate.getScale();
			puz.x = (int)((double)puz.x * sc + 0.5);
			puz.y = (int)((double)puz.y * sc + 0.5);
			ending::RotationMatrix rm(plate.getAngle());
			ending::Orient o;
			rm.rotate(puz, o);
			puz.x += plate.getBoundingBoxCenter().x;
			puz.y += plate.getBoundingBoxCenter().y;
			return puz;
		}

		cv::Rect getHoleBoundingBox(cv::Point puz, cv::Vec2d scale, ending::Matcher::MatchPoint &plate){
			puz = pointTrans(puz, plate);
			cv::Size size = plate.getBoundingBoxSize();
			size.width = (int)(size.width * scale[0] + 0.5);
			size.height = (int)(size.height * scale[1] + 0.5);
			puz.x -= size.width / 2;
			puz.y -= size.height / 2;
			return cv::Rect(puz.x, puz.y, size.width, size.height);
		}



	public:

		MatchingTest(std::string INIpath) : parser(std::ifstream(INIpath)){

		}

		void init(){
			getRChamferMatcherConfig("plate", platemc);
			getRChamferMatcherConfig("puzzle", puzzmc);
			getRChamferMatcherConfig("hole", holemc);

			getHolePos("pos", posmap);

			platepath = parser.getString("plate", "PATH");
			puzzpath = parser.getString("puzzle", "PATH");
			holepath = parser.getString("hole", "PATH");
			bboxscale = getCVVec2d("pos", "PLATE");
		}

		void setImage(cv::Mat &image){
			cimage = image.clone();
			colorEdgeDetection(image, _image, true);
			ending::ChamferMatcher cmatcher;
			cmatcher.createMaps(_image, _distimg, _orientimg);

			cv::waitKey(0);
		}

		ending::Matcher::MatchPoints &matchPlate(cv::Rect bbox){
			ending::RChamferMatcher cmatcher(platemc);

			ending::DEBUG_img = cimage.clone();

			cv::Mat templ = cv::imread(platepath, CV_LOAD_IMAGE_GRAYSCALE);

			cmatcher.addMatcher(templ);
			cmatcher.matching(_distimg, _orientimg, bbox, platemp);

			cv::imshow("plate", ending::DEBUG_img);

			return platemp;
		}

		std::vector<ending::Matcher::MatchPoints> &matchPuzzle(std::vector<char> puzzleList, std::vector<cv::Rect> bboxList){
			ending::RChamferMatcher cmatcher(puzzmc);


			ending::DEBUG_img = cimage.clone();

			for (size_t i = 0; i<puzzleList.size(); i++){
				char c[2] = {};
				c[0] = puzzleList[i];
				std::string tname = puzzpath + std::string(c) + std::string(".png");
				cv::Mat templ = cv::imread(tname, CV_LOAD_IMAGE_GRAYSCALE);

				cmatcher.addMatcher(templ);
			}

			cmatcher.multimatching(_distimg, _orientimg, bboxList, puzzmp);

			cv::imshow("puzzle", ending::DEBUG_img);
			return puzzmp;
		}

		ending::Matcher::MatchPoint &matchHole(char puzzle){
			ending::RChamferMatcher cmatcher(holemc);


			ending::DEBUG_img = cimage.clone();

			char c[2] = {};
			c[0] = puzzle;
			std::string tname = holepath + std::string(c) + std::string(".png");
			cv::Mat templ = cv::imread(tname, CV_LOAD_IMAGE_GRAYSCALE);

			cmatcher.addMatcher(templ);

			cv::Rect bbox = getHoleBoundingBox(posmap[puzzle], bboxscale, platemp[0]);
			cmatcher.matching(_distimg, _orientimg, bbox, holemp);

			for (size_t j = 0; j<holemp.size(); j++){
				double f = fabs(holemp[j].getAngle() - platemp[0].getAngle());
				if (f > 1 && 2 * CV_PI - f > 1)continue;
				else{
					cv::imshow("hole", ending::DEBUG_img);
					return holemp[j];
				}
			}

			return holemp[0];

			
		}
	};
}
