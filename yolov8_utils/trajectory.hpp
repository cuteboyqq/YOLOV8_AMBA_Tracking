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

#ifndef __OBJECT_TRAJECTORY__
#define __OBJECT_TRAJECTORY__

#include <math.h>  //sin
#include <algorithm>
#include <iomanip>   //setprecision
#include <iostream>  //cout
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <ctime>
#include <cmath>
#include <vector>
#include <assert.h>
#include <map>

#include "point.hpp"
#include "bounding_box.hpp"
#include "object.hpp"
#include "dla_config.hpp"
#include "utils.hpp"

using namespace std;

#ifdef __cplusplus
extern "C" {
#endif


class Trajectory
{
 public:
  Trajectory(Config_S *_config);
  ~Trajectory();

  ///////////////////////////
  /// Member Functions
  //////////////////////////
  Point bboxToPointSimple(BoundingBox &bbox);
  void bboxToTrajectory(vector<Object> &objectList);
  float updateLocation3D(Object& obj);
  int getBevZone(Point &bevPoint);

 private:
  ///////////////////////////fu/
  /// Member Functions
  //////////////////////////
  float _bboxToDistanceZ(BoundingBox& box);
  void _rotatePoint(Point &p, double angle);
  void _applyMovingAverage(
    vector<int> &list,
    vector<int> &listMA,
    int windowSize);

  void _bboxPreprocessing(
    vector<BoundingBox> &bboxList,
    vector<BoundingBox> &procBboxList,
    int windowSize);
  vector<Point> _postProcessing(vector<Point> &pTrajectory);

  int videoWidth = 0;                       // Video width
  int videoHeight = 0;                      // Video height
  int modelWidth = 0;
  int modelHeight = 0;
  int birdWidth = 0;                        // Bird's eye view width
  int birdHeight = 0;                       // Bird's eye view height
  int frameInterval = 1;                    // Frame interval
  Point pOrig = Point(-1, -1);              // Origin point
  vector<Point> pList;                      // Arc point list

  // === Bounding Box to BEV Point === //
  int maxFrameInterval = 12;                // For discarding bounding boxes

  // === PostProcessing === //
  int maWindowSize = 10;                    // Moving average window size         // TODO: related to bbox list size ?

  // === Bird Eye View Zone === //
  int numBevZone = 15;                      // Split BEV map into multiple zones

  // === Threshold === //
  float tAspectFullBody = 2.2;              // Full body's aspect ratio
  float tAspectFall = 0.85;                 // Fall down's aspect ratio
  // float tWidthRatioCloseLv1 = 0.1;          // Width ratio if close to doorbell (Close)        //TODO:
  // float tWidthRatioCloseLv2 = 0.145;        // Width ratio if close to doorbell (More Close)   //TODO:
  // float tWidthRatioCloseLv3 = 0.5;          // Width ratio if close to doorbell (Extreme Close)//TODO:
  float tWidthRatioCloseLv1 = 0.4;          // Width ratio if close to doorbell (Close)
  float tWidthRatioCloseLv2 = 0.445;        // Width ratio if close to doorbell (More Close)
  float tWidthRatioCloseLv3 = 0.55;

  float tYCloseDoorbell = 0.0;              // Y close to doorbell
  float tNewY = 1.2;                        // Y changes at most 20%
  float tXDiffRatio = 0.08;                 // X diff at most 8%
  float tYDiffLower = 0.2;                  // Y diff lower bound
  float tYDiffUpper = 0.35;                 // Y diff upper bound

  // Debug
  bool debugMode = true;
};

#ifdef __cplusplus
}
#endif

#endif