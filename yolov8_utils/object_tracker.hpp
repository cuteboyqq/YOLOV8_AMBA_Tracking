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

#ifndef __OBJECT_TRACKER__
#define __OBJECT_TRACKER__

#include <math.h>
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video.hpp>
#include "opencv2/features2d.hpp"

#include "point.hpp"
#include "bounding_box.hpp"
#include "object.hpp"
#include "dla_config.hpp"
#include "matching2D.hpp"
#include "trajectory.hpp"
#include "dataStructures.h"
#include "utils.hpp"
#include "img_util.hpp"

#if defined (SPDLOG)
#include "logger.hpp"
#endif

using namespace std;



#define MAX_BBOX_LIST_SIZE 10       // Preserve object's previous bounding boxes
#define MAX_OBJ_ALIVE_COUNT 600     // Refresh when object's alive counter over this value
#define NUM_FRAME_VALID_SINGLE 60     // Remove object from object list when it disappears over this value (single object case)
#define NUM_FRAME_VALID_MULTI 60     // Remove object from object list  when it disappears over this value (multiple object case)
#define NUM_BEV_VALID_SINGLE 30     // Remove object from BEV map list when it disappears over this value (single object case)
#define NUM_BEV_VALID_MULTI 10      // Remove object from BEV map list when it disappears over this value (multiple object case)
#define MIN_TRAJ_LIST_SIZE 3        // Send object's trajectory when its size over this value
#define COMPARE_SAME 0              // Number of detected object is as same as previous one
#define COMPARE_MORE 1              // Number of detected object is more than previous one
#define COMPARE_LESS -1             // Number of detected object is less than previous one

enum trackingTask
{
  TRACK_HUMAN = 0,
  TRACK_CAR = 2,
  TRACK_BIKE = 1,
  TRACK_MOTORBIKE = 3
};

enum detectionTask
{
  DETECT_HUMAN = 0,
  DETECT_BIKE = 1,
  DETECT_VEHICLE = 2,
  DETECT_MOTORBIKE = 3
};


class ObjectTracker
{
 public:
  ObjectTracker(
    Config_S *_config,
    string _task);

  ~ObjectTracker();

  // === Inputs ===//
  void setROI(BoundingBox &roi);
  void setDataBuffer(std::vector<DataFrame>* dataBufferPtr);

  // === Outputs === //
  int getFrameStamp();
  int getSmoothBBoxList(vector<BoundingBox> &bboxList);
  void getObjectList(vector<Object> &objList);

  // === Tracking === //
  void run(cv::Mat &img, vector<BoundingBox> &bboxList);

  // === 3D Location Calculation === //
  void getTrackedObjList(vector<TrackedObj> &objList);

  // === Others === //
  void debugON();
  void showProcTime();

  ///////////////////////////
  /// Member Variables
  //////////////////////////
  int m_task;
  int m_modelWidth = 0;
  int m_modelHeight = 0;
  int m_videoWidth = 0;           // video width
  int m_videoHeight = 0;          // video height
  int m_maxObject = 100;            // max object can show on BEV map
  int m_maxTracking = 0;          // max tracking object at same time
  float m_warnDistance = 145.0;   // over this distance then sys needs to warning user
  bool m_isFirstTracking = true;  // set to false after start tracking human

  cv::Ptr<cv::ORB> m_ftDetector = cv::ORB::create(512);
  Trajectory* m_trajectory;

 private:
  ///////////////////////////
  /// Member Functions
  //////////////////////////

  // === Initilzation === //
  bool _init(Config_S *_config);
  bool _initObjectList();
  void _assignObjectList();

  // === Inputs === //
  void _setCurrFrame(cv::Mat &imgFrame);
  void _setCurrBoundingBox(vector<BoundingBox> &bboxList);

  // === Tracking === //
  void _ReID(
    vector<Object> &currObjectList,
    vector<Object> &prevObjectList,
    int frameInterval
  );
  int _updateCurrObjectList();
  void _updateTrackingObject();

  int _calcDisappearThreshold(int numPrevObj, int numCurrObj, int frameInterval);

  void _updateRemoveObjList(
    vector<Object>& objList,
    vector<int>& list,
    int numPrevObj,
    int threshold);

  void _getMatchScoreDictList(
    Object &queryObj, vector<Object> &prevObjectList,
    vector<std::pair<int, float>> &matchScoreDictListLv1,
    vector<std::pair<int, float>> &matchScoreDictListLv2
  );

  void _getQueryMatchScoreDict(
    vector<Object> &currObjectList,
    vector<Object> &prevObjectList,
    vector<int> &prevKeyIdList,
    vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDict,
    vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDictBkp
  );

  vector<pair<int, pair<int, float>>> _getSelfMatchScoreDict(
    vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDict,
    float thresholdMatch
  );

  vector<pair<int, pair<int, float>>> _getCrossMatchScoreDict(
    vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDict,
    vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDictBkp,
    vector<pair<int, pair<int, float>>> &selfMatchScoreDict,
    vector<int> &prevKeyIdList,
    int numCompareToPrev,
    int numCurrObject,
    int numPrevObject
  );

  void _updatePrevObjectList(
    vector<pair<int, pair<int, float>>> &crossMatchScoreDict,
    vector<Object> &prevObjectList,
    vector<Object> &currObjectList,
    vector<int> &prevKeyIdList
  );

  float _calcSimilarityScore(Object &objKey, Object &objQuery);
  void _calcKeypoint(cv::Mat &img, vector<cv::KeyPoint> &kpt);
  void _filterOverlapObject();
  void _updateBoundingBox(BoundingBox &srcBox, BoundingBox &newBox);

  // === Following Distance === //
  void _calcDistance(Object &obj);
  void _getBoxAnchor(BoundingBox &box, cv::Point &pAnchor);
  void _rotateVector(cv::Point2f &v, cv::Point2f &vRotated, double angle);
  float _calcX2Meter(Object& obj);
  float _calcY2Meter(Object& obj);

  // === 3D Location Calculation === //
  void calcLocation3D(Object& box, cv::Point3f& pLoc);

  // === Utils === //
  int _updateFrameStamp();
  int _getNumEnableObject(vector<Object> &_objList);
  void _increaseDisappearCounter(vector<Object> &objList);
  vector<int> _getEnableKeyIdList(vector<Object> &objList);
  void _disableOverlapObject(vector<Object> &currObjectList);
  bool _isValidHumanBBox(BoundingBox &box);
  bool _isValidBikeBBox(BoundingBox &box);
  bool _isValidMotorbikeBBox(BoundingBox &box);
  bool _isValidVehicleBBox(BoundingBox &box);
  void _bboxPreprocessing(
    vector<BoundingBox> &bboxList,
    vector<BoundingBox> &procBboxList,
    int windowSize);


  // === Camera === //
  float m_cameraHeight;
  float m_cameraFocalLength;
  float m_frameRate;

  // === Image Frame === //
  cv::Mat m_img;

  // === Data Frame === //
  int m_bufferSize = 2;
  std::vector<DataFrame>* m_dataBuffer;

  // === ROI === //
  BoundingBox* m_roi;

  // === Bounding Box === //
  int m_tArea = 8000;                   // BoundingBox's area must > tArea
  int m_tWidth = 200;                   // BoundingBox's width must > tWidth
  int m_tHeight = 200;                  // BoundingBox's height must > tHeight
  float m_tSmallAspectUpper = 1.25;     // Small BoundingBox's aspect ratio (upper)
  float m_tSmallAspectLower = 0.85;     // Small BoundingBox's aspect ratio (lower)
  float m_tAspect = 3.5;                // Valid BoundingBox's aspect ratio
  float m_tAspectMax = 4.2;             // Invalid BoundingBox's aspect ratio

  // === Objects === //
  vector<Object> m_currObjList;         // Object list for current frame
  vector<Object> m_prevObjList;         // Object list for previous frame
  vector<BoundingBox> m_bboxList;       // Bounding box list for current frame

  // === Tracking === //
  int m_frameInterval = 1;              // No use currently
  int m_maxFrameInterval = 10;
  int m_frameStamp = 0;                 // Some kind of timestamp, increase frame by frame

  // === Threshold === //
  float m_tObjDisappearFrame = 10;      // Object disappears how many frames
  float m_tOverlapRatio = 0.3;          // Bounding Box overlap
  float m_tAspectRatio = 3.2;           // Bounding Box aspect ratio
  float m_tMatchMinRequriement = 0.5;   // Valid matching score (non-matched object)
  float m_tMatchSingle = 1.0;
  float m_tMatchMultiple = 0.8;
  int m_tAliveCounter = 1;             // Start tracking object when is alives over X frames

  // === Debug === //
  string m_loggerStr;
  bool m_debugMode = false;
  bool m_estimateTime = false;
};

#endif