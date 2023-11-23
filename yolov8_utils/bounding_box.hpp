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

#ifndef __BOUNDING_BOX__
#define __BOUNDING_BOX__

#include <math.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <assert.h>
#include <opencv2/core.hpp>
#include "point.hpp"

using namespace std;


class BoundingBox
{
 public:
  BoundingBox(int x1, int y1, int x2, int y2, int label);
  ~BoundingBox();

  ///////////////////////////
  /// Member Functions
  //////////////////////////
  int getHeight();
  int getWidth();
  int getArea();
  float getAspectRatio();
  Point getCenterPoint();
  vector<Point> getCornerPoint();
  bool check(int videoWidth, int videoHeight);
  void setFrameStamp(int _frameStamp);

  // === Default value === //
  int x1 = -1;                      // Bounding Box x1
  int y1 = -1;                      // Bounding Box y1
  int x2 = -1;                      // Bounding Box x2
  int y2 = -1;                      // Bounding Box y2
  int label = -1;                   // Bounding Box label
  int frameStamp = 0;               // Some kind of timestamp
  int objID = -1;
  int boxID = -1;
  float distanceToCamera = -1;      // Distance to camera
  float confidence = -1;            // Confidence score
  bool needWarn = 0;
  std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
  std::vector<cv::DMatch> kptMatches; // keypoint matches enclosed by 2D roi
  cv::Rect roi; // 2D region-of-interest in image coordinates


 private:

  ///////////////////////////
  /// Member Variables
  //////////////////////////
  int h = 0;                        // Bounding Box's height
  int w = 0;                        // Bounding Box's width
  int area = 0;                     // Bounding Box's area
  float aspectRatio = 1.0;          // height / width
  Point pCenter = Point(-1, -1);    // Bounding Box's center point

  // Debug
  int debugMode = false;
};
#endif

