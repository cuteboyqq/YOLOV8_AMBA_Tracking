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

#ifndef __UTILS__
#define __UTILS__

#include <math.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <assert.h>
#include <numeric>
#include <sstream>
#include <chrono>
#include <ctime>
#include <sys/stat.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "point.hpp"
#include "bounding_box.hpp"
#include "object.hpp"
#include "dataStructures.h"

using namespace std;


namespace utils
{
  //
  bool checkFileExists(const std::string& name);
  bool checkFolderExists(const std::string& folderName);
  bool createFolder(const std::string& folderName);
  bool createDirectory(const std::string& path);
  bool createDirectories(const std::string& path);

  //
  void getDateTime(std::string &timeStr);

	//
	bool sortbysec(
		const pair<int, int> &a,
		const pair<int, int> &b);

	bool sortBySecAscend(
		const pair<int, int> &a,
		const pair<int, int> &b);

  bool sortByBBoxArea(BoundingBox &bboxA, BoundingBox &bboxB);
  // bool sortByObjBBoxArea(Object &objA, Object &objB);

	//
	float calcVecDegree(Point &a, Point &b);

	//
	void updateIntList(vector<int> &list, int x, int maxSize);

  //
  void findMaxScoreKey(vector<pair<int, float>> &dict, int &key, float &score);

  //
  int findListIdx(int key, vector<int> &dict);
  int findListIdx(int key, vector<pair<int, vector<int>>> &dict);
  int findListIdx(int key, vector<pair<int, vector<pair<int, float>>>> &dict);
  int findListIdx(int key, vector<pair<int, pair<int, float>>> &dict);
  int findListIdx(int key, vector<pair<int, vector<Point>>> &dict);

	//
	int findMedian(vector<int> a, int n);

  //
  bool isKeyInDict(int key, vector<int> &dict);
  bool isKeyInDict(int key, vector<pair<int, vector<int>>> &dict);
  bool isKeyInDict(int key, vector<pair<int, vector<pair<int, float>>>> &dict);
  bool isKeyInDict(int key, vector<pair<int, pair<int, float>>> &dict);

	//
	float getWidthRatio(int refWidth, int width);
	float getWidthRatio(int xA, int xB, int width);
	float getWidthRatio(int xA, int xB, int refWidth, int width);

	//
	std::pair<float, float> computeStdDevAndMean(std::vector<int> &v);
	std::pair<double, double> computeStdDevAndMean(std::vector<double> &v);

	//
	vector<int> outlierRemoval(vector<int> &list);

	//
	void applyMovingAverage(
		vector<int> &list, vector<int> &listMA, int windowSize);

  //
  std::string to_string_with_precision(float value, const int n);

  //
  void rescaleBBox(
    BoundingBox &bbox, BoundingBox &rescaleBBox,
    int modelWidth, int modelHeight, int videoWidth, int videoHeight);

  void rescalePoint(
    cv::Point& pSrc, cv::Point& pDst, float xRatio, float yRatio);

  void smoothBBoxes(
    vector<BoundingBox>& bboxList,
    vector<BoundingBox>& procBBoxList,
    int windowSize,
    int maxFrameInterval
  );

  //
  bool isAscendingList(vector<float> &list);
  bool isDescendingList(vector<float> &list);

}

#endif


