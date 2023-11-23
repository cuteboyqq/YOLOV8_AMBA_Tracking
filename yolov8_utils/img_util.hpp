/*
  (C) 2023-2024 Wistron NeWeb Corporation (WNC) - All Rights Reserved

  This software and its associated documentation are the confidential and
  proprietary information of Wistron NeWeb Corporation (WNC) ("Company") and
  may not be copied, modified, distributed, or otherwise disclosed to third
  parties without the express written consent of the Company.

  Unauthorized reproduction, distribution, or disclosure of this software and
  its associated documentation or the information contained herein is a
  violation of applicable laws and may result in severe legal penalties.
*/

#ifndef __IMG_UTIL__
#define __IMG_UTIL__

#include <math.h>  //sin
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <assert.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "point.hpp"
#include "bounding_box.hpp"

using namespace std;



#define VERY_DARK 0.15
#define DARK 0.35
#define NORMAL 0.55
#define BRIGHT 0.75
#define VERY_BRIGHT 0.95



namespace imgUtil
{
	//
	void calcLinearEquation(cv::Point &pA, cv::Point &pB, vector<float> &equation);
  	int checkPointOnWhichLineSide(Point &p, vector<float> &linearEquation);
  	int checkPointOnWhichLineSide(cv::Point &p, cv::Point &pA, cv::Point &pB);

	//
	float calcBrightnessRatio(cv::Mat &img);
	void brightnessEnhancement(float brightnessRatio, cv::Mat &img);

	//
	void gammaCorrection(const cv::Mat input, cv::Mat& output, const float gamma);
	void autoscaling(const cv::Mat input, cv::Mat& output);
	void contrastStretching(const cv::Mat input, cv::Mat& output, const int r1, const int s1, const int r2, const int s2);
	void contrastEnhancement(cv::Mat &img, cv::Mat &imgEnhance);

	//
	void abs_sobel_thresh(cv::Mat const& src, cv::Mat& dest, char orient, int kernel_size, int thresh_min, int thresh_max);
	void combined_threshold(cv::Mat const& img, cv::Mat& dst);

	//
	cv::Point2f getLine(cv::Point p1, cv::Point p2);
	cv::Point getIntersectionPoint(cv::Point pLeftBot, cv::Point pLeftTop, cv::Point pRightBot, cv::Point pRightTop);
	float calcEuclideanDistance(Point &pA, Point &pB);
	float calcEuclideanDistance(cv::Point &pA, cv::Point &pB);
	float calcEuclideanDistance(cv::Point2f &pA, cv::Point2f &pB);

	//
	double angleBetweenPoints(cv::Point &pA, cv::Point &pB);
	void rotateVector(cv::Point2f &v, cv::Point2f &vRotated, double angle);

	//
	void _poly_fitx(std::vector<double> const& fity, std::vector<double>& fitx, cv::Mat const& line_fit);
	void _polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order);

	//
	bool sortByContourSize(const pair<int, int> &a, const pair<int, int> &b);
	void findMaxContour(cv::Mat &src, cv::Mat &dst);
	void findClosestContour(cv::Mat &src, int y, cv::Mat &dst);
	void removeSmallContour(cv::Mat &src, cv::Mat &dst, int sizeThreshold);

	//
	void dilate(cv::Mat &img, int kernelSize);
	void erode(cv::Mat &img, int kernelSize);

	// //
	// float _calcAngleDiffRatio(LaneLine &currLine, LaneLine &prevLine);
	// float _calcAvgXDiffRatio(LaneLine &currLine, LaneLine &prevLine);
	// float _calcRoiShiftRatio(LaneLine &currLine, LaneLine &prevLine);

	//
	float getBboxOverlapRatio(BoundingBox &bA, BoundingBox &bB);

	//
	void roundedRectangle(
		cv::Mat& src, cv::Point topLeft, cv::Point bottomRight, const cv::Scalar lineColor,
		int thickness, const int lineType , const int cornerRadius, bool filled);

	//
	void cropImages(cv::Mat &img, cv::Mat &imgCrop, BoundingBox &box);

}

#endif


