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

#include "object_tracker.hpp"


ObjectTracker::ObjectTracker(Config_S *_config, string _task)
{ printf("[ObjectTracker::ObjectTracker(Config_S *_config, string _task)] Start ~~~~~~~~~~~~~ \n");
  //printf("[ObjectTracker::ObjectTracker(Config_S *_config, string _task)] Start Task name \n");
  // Task name
  if (_task == "human")
  {
    m_task = TRACK_HUMAN;
    m_loggerStr = "HumanTracker";
  }
  else if (_task == "bike")
  {
    m_task = TRACK_BIKE;
    m_loggerStr = "BikeTracker";
  }
  else if (_task == "vehicle")
  {
    m_task = TRACK_CAR;
    m_loggerStr = "VehicleTracker";
  }
  else if (_task == "motorbike")
  {
    m_task = TRACK_MOTORBIKE;
    m_loggerStr = "MotorbikeTracker";
  }
  //printf("[ObjectTracker::ObjectTracker(Config_S *_config, string _task)] End Task name \n");
  // Logger
#if defined (SPDLOG)
  auto m_logger = spdlog::stdout_color_mt(m_loggerStr);
  m_logger->set_pattern("[%n] [%^%l%$] %v");

  if (_config->stDebugConfig.objectTracking)
  {
    if (m_task == TRACK_HUMAN && _config->stDebugConfig.humanTracker)
      m_logger->set_level(spdlog::level::debug);
    else if (m_task == TRACK_BIKE && _config->stDebugConfig.bikeTracker)
      m_logger->set_level(spdlog::level::debug);
    else if (m_task == TRACK_CAR && _config->stDebugConfig.vehicleTracker)
      m_logger->set_level(spdlog::level::debug);
    else if (m_task == TRACK_MOTORBIKE && _config->stDebugConfig.motorbikeTracker)
      m_logger->set_level(spdlog::level::debug);
    else
      m_logger->set_level(spdlog::level::info);
  }
  else
    m_logger->set_level(spdlog::level::info);
#endif
  //printf("[ObjectTracker::ObjectTracker(Config_S *_config, string _task)] Start set parameters \n");
  // Image Size
  m_videoWidth = _config->frameWidth;
  m_videoHeight = _config->frameHeight;

  // Model Input Size
  m_modelWidth = _config->modelWidth;
  m_modelHeight = _config->modelHeight;

  // Object Tracking
  m_maxObject = _config->stOdConfig.maxDetection;
  m_maxTracking = _config->stTrackerConifg.maxTracking;

  // Distance Estimation
  m_cameraHeight = _config->stCameraConfig.height;
  m_cameraFocalLength = _config->stCameraConfig.focalLength;

  // Frame Rate
  m_frameRate = _config->procFrameRate;

  // // Data Buffer
  // m_dataBuffer = new boost::circular_buffer<DataFrame>();
  // m_dataBuffer->set_capacity(m_bufferSize);

  // ROI
  m_roi = new BoundingBox(-1, -1, -1, -1, -1);

  // Trajectory
  m_trajectory = new Trajectory(_config);
  //printf("[ObjectTracker::ObjectTracker(Config_S *_config, string _task)] End set parameters \n");
  //printf("[ObjectTracker::ObjectTracker(Config_S *_config, string _task)] Start _init(_config) \n");
  _init(_config);
  //printf("[ObjectTracker::ObjectTracker(Config_S *_config, string _task)] End _init(_config) \n");
  printf("[ObjectTracker::ObjectTracker(Config_S *_config, string _task)] End ~~~~~~~~~~~~~ \n");
};


ObjectTracker::~ObjectTracker()
{
  delete m_trajectory;
  delete m_roi;

  m_roi = nullptr;
  m_trajectory = nullptr;
};


// ============================================
//               Initialization
// ============================================
bool ObjectTracker::_init(Config_S *_config)
{
// #if defined (SPDLOG)
//   auto m_logger = spdlog::get(m_loggerStr);
// #endif
   printf("[bool ObjectTracker::_init(Config_S *_config)] Start Object Detection \n");
  // Object Detection
  if (m_task == TRACK_HUMAN)
  {
    m_tWidth = 10;                   // BoundingBox's width must > tWidth
    m_tHeight = 15;                  // BoundingBox's height must > tHeight
  }
  else if (m_task == TRACK_BIKE)
  {
    m_tWidth = 10;                   // BoundingBox's width must > tWidth
    m_tHeight = 15;                  // BoundingBox's height must > tHeight
  }
  else if (m_task == TRACK_CAR)
  {
    m_tWidth = 20;                   // BoundingBox's width must > tWidth
    m_tHeight = 20;                  // BoundingBox's height must > tHeight
  }
  else if (m_task == TRACK_MOTORBIKE)
  {
    m_tWidth = 20;                   // BoundingBox's width must > tWidth
    m_tHeight = 20;                  // BoundingBox's height must > tHeight
  }
//printf("[bool ObjectTracker::_init(Config_S *_config)] End Object Detection \n");
//printf("[bool ObjectTracker::_init(Config_S *_config)] Start Matching threshold \n");
  // Matching threshold
    // rAppearance settings
    m_tMatchMinRequriement = 0.5;

    // Matching score
    m_tMatchSingle = 0.8;
    m_tMatchMultiple = 0.8;

    if (m_task != TRACK_CAR)
    {
      m_tMatchSingle = 0.3;
      m_tMatchMultiple = 0.3;
    }
  // if (_config->stTrackerConifg.matchingLevel == "Low")      // Level: Low
  // {
  //   // rAppearance settings
  //   m_tMatchMinRequriement = 0.45;

  //   // Matching score
  //   m_tMatchSingle = 0.5;
  //   m_tMatchMultiple = 0.5;
  //   printf("[bool ObjectTracker::_init(Config_S *_config)] End if (_config->stTrackerConifg.matchingLevel == Low)\n");
  // }
  // else if (_config->stTrackerConifg.matchingLevel == "Normal")   // Level: Normal (Default)
  // {
  //   // rAppearance settings
  //   m_tMatchMinRequriement = 0.5;

  //   // Matching score
  //   m_tMatchSingle = 0.8;
  //   m_tMatchMultiple = 0.8;

  //   if (m_task != TRACK_CAR)
  //   {
  //     m_tMatchSingle = 0.3;
  //     m_tMatchMultiple = 0.3;
  //   }
  //   printf("[bool ObjectTracker::_init(Config_S *_config)] End if (_config->stTrackerConifg.matchingLevel == Normal)\n");
  // }
  // else if (_config->stTrackerConifg.matchingLevel == "High")   // Level: High
  // {
  //   // rAppearance settings
  //   m_tMatchMinRequriement = 0.525;

  //   // Matching score
  //   m_tMatchSingle = 0.5;
  //   m_tMatchMultiple = 0.5;
  //   printf("[bool ObjectTracker::_init(Config_S *_config)] End if (_config->stTrackerConifg.matchingLevel == High)\n");
  // }
//   else
//   {
// #if defined (SPDLOG)
//     m_logger->debug("[Tracker] Matching score uses defualt confiuration ...");
// #endif
//   }
//printf("[bool ObjectTracker::_init(Config_S *_config)] End Matching threshold \n");

 

  // Alive counter threshold
  m_tAliveCounter = 10;

  if (m_task != TRACK_CAR)
  {
     m_tAliveCounter *= 0.3;
  }
   //printf("[bool ObjectTracker::_init(Config_S *_config)] Start  _initObjectList()\n");
//printf("2023-11-24~~~~~~~~~~\n");
  _initObjectList();
  printf("[bool ObjectTracker::_init(Config_S *_config)]] End Object Detection \n");
  //printf("[bool ObjectTracker::_init(Config_S *_config)] End  _initObjectList()\n");
  return true;
}


bool ObjectTracker::_initObjectList()
{
  //printf("[bool ObjectTracker::_initObjectList()] Start ----------------------\n");
  // Initialize Object List
  for (int i=0; i<m_maxObject; i++)
  {
    Object tmpObj;
    tmpObj.id = i; // Assign Object ID
    m_currObjList.push_back(tmpObj);
  }

  for (int i=0; i<m_maxObject; i++)
  {
    Object tmpObj;
    m_prevObjList.push_back(tmpObj);
  }
  //printf("[bool ObjectTracker::_initObjectList()] End ----------------------\n");
  return true;  
}


void ObjectTracker::_assignObjectList()
{
  for (int i=0; i<m_maxObject; i++)
  {
    Object *ptrObj = &m_currObjList[i];
    m_prevObjList[i].id = ptrObj->id;
    m_prevObjList[i].updateStatus(ptrObj->status);
    m_prevObjList[i].updateBoundingBox(ptrObj->bbox);
    m_prevObjList[i].updatePointCenter(ptrObj->pCenter);
    m_prevObjList[i].updateKeypoint(ptrObj->m_currKpts);
    m_prevObjList[i].aliveCounter = 1;
  }
}


// ============================================
//                  Tracking
// ============================================
void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);

  if (m_estimateTime)
  {
    if (m_task == TRACK_HUMAN)
    {
      m_logger->info("[Human Tracker Processing Time]");
    }
    else if (m_task == TRACK_BIKE)
    {
      m_logger->info("[Bike Tracker Processing Time]");
    }
    else if (m_task == TRACK_MOTORBIKE)
    {
      m_logger->info("[Motorbike Tracker Processing Time]");
    }
    else if (m_task == TRACK_CAR)
    {
      m_logger->info("[Vehicle Tracker Processing Time]");
    }

    m_logger->info("-----------------------------------------");
  }
#endif
  printf("[void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)] Start  _setCurrBoundingBox(bboxList); \n");
  _setCurrBoundingBox(bboxList);
  printf("[void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)] End  _setCurrBoundingBox(bboxList); \n");
  printf("[void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)] Start  _updateFrameStamp(); \n");
  _updateFrameStamp();
  printf("[void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)] End  _updateFrameStamp(); \n");
  _setCurrFrame(img);
  printf("[void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)] End  _setCurrFrame(); \n");
  _updateCurrObjectList();
  printf("[void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)] End  _updateCurrObjectList(); \n");
  _updateTrackingObject();
  printf("[void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)] End  _updateTrackingObject(); \n");
  _filterOverlapObject();
  printf("[void ObjectTracker::run(cv::Mat &img, vector<BoundingBox> &bboxList)] End  _filterOverlapObject(); \n");
#if defined (SPDLOG)
  if (m_estimateTime)
  {
    m_logger->info("-----------------------------------------");
  }
#endif
}


void ObjectTracker::_updateTrackingObject()
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  // Max Tracking
  int numCurrObj = 0;
  int numPrevObj = 0;
  for (int i=0; i < m_maxTracking; i++)
  {
    // Get number of enabled current object
    if (m_currObjList[i].getStatus() == 1)
    {
#if defined (SPDLOG)
      if (m_debugMode) m_logger->debug(
        "[Tracker] => curr obj [{}] status = {}", i, m_currObjList[i].getStatus());
#endif
      numCurrObj += 1;
    }

    // Get number of enabled previous object
    if (m_prevObjList[i].getStatus() == 1)
    {
#if defined (SPDLOG)
      if (m_debugMode) m_logger->debug(
        "[Tracker] => prev obj [{}] status = {}", i, m_currObjList[i].getStatus());
#endif
      numPrevObj += 1;
    }
  }

  // Update Object List
  // If detect object
  if (numCurrObj > 0)
  {
    // If first frame, dont do tracking but save current objects
    if (m_isFirstTracking)
    {
      _assignObjectList();
      m_isFirstTracking = false;
    }
    // if not first frame and previous frame has no object
    else if ((!m_isFirstTracking) && (numPrevObj == 0))
    {
      _assignObjectList();
    }
    else
    {
      _ReID(m_currObjList, m_prevObjList, m_frameInterval);
    }
  }
  else
  {
    // have to run reid because this function will increase disappear counter
    _ReID(m_currObjList, m_prevObjList, m_frameInterval);
  }

  // Update Tracked Object's Location
  if (m_task == TRACK_HUMAN)
  {
    // cout << "-------------------------------" << endl;
    // cout << m_loggerStr << endl;
    // m_trajectory->bboxToTrajectory(m_prevObjList);

    for (int i=0; i<m_prevObjList.size(); i++)
    {
      Object& obj = m_prevObjList[i];
      if (obj.status == 0)
        continue;

      m_trajectory->updateLocation3D(obj);

      // cout << "Obj[" << obj.id << "] Loc = (" << obj.pLocation3D.x << " m, " << obj.pLocation3D.y << " m, " << obj.pLocation3D.z << " m)" << endl;
    }
  }
}


void ObjectTracker::getTrackedObjList(vector<TrackedObj> &objList)
{
  objList.clear();

  int numPrevObj = 0;
  for (int i=0; i < m_maxTracking; i++)
  {
    // Get number of enabled previous frame object
    if (m_prevObjList[i].getStatus() == 1)
    {
      numPrevObj += 1;
    }
  }

  // If object's disappear counter > threshold,
  // then stop displaying or sending trajectory points
  int bevDisappear = NUM_BEV_VALID_MULTI;
  if (numPrevObj <= 1)
    bevDisappear = NUM_BEV_VALID_SINGLE;

  for (int i=0; i<m_prevObjList.size(); i++)
  {
    int id = m_prevObjList[i].id;
    int aliveCounter = m_prevObjList[i].aliveCounter;
    vector<Point> trajectoryList = m_prevObjList[i].m_trajList;

    if (aliveCounter < m_tAliveCounter)
      continue;

    else if (m_prevObjList[i].status == 0)
      continue;

    // if object disappears too long then don't show point
    if (m_prevObjList[i].disappearCounter > bevDisappear)
    {
      continue;
    }
    else
    {
      // Show last trajcetory point, if trajectory length is enough
      TrackedObj trackObj;
      trackObj.id = id;
      trackObj.type = m_prevObjList[i].bboxList.back().label;
      trackObj.confidence = m_prevObjList[i].bboxList.back().confidence;
      trackObj.pLoc = m_prevObjList[i].pLocation3D;
      objList.push_back(trackObj);
    }
  }
}


int ObjectTracker::_updateCurrObjectList() //TODO: refactor?
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

  int ret = 1;

  // Initialize
  for (int i=0; i<m_maxObject; i++)
  {
    m_currObjList[i].init(m_frameStamp);
  }

  int objIdx = 0;
  for (int i=0; i<(int)m_bboxList.size(); i++)
  {
    BoundingBox bbox = m_bboxList[i];

    // If bounding box is "car"
    time_0 = std::chrono::high_resolution_clock::now();

    if (m_task == TRACK_CAR)
    {
      if (_isValidVehicleBBox(bbox))
      {
        Point pCenter = bbox.getCenterPoint();

        Object *ptrObj = &m_currObjList[objIdx];
        ptrObj->updateBoundingBox(bbox);
        ptrObj->updatePointCenter(pCenter);
        ptrObj->updateStatus(1);    //Enable Object
        objIdx += 1;

        // Appearance Features
        BoundingBox rescaleBBox(-1, -1, -1, -1, m_bboxList[i].label);
        utils::rescaleBBox(
          m_bboxList[i], rescaleBBox, m_modelWidth, m_modelHeight, m_videoWidth, m_videoHeight);

        cv::Mat imgCrop;
        imgUtil::cropImages(m_img, imgCrop, rescaleBBox);

        // Get Keypoints and Descriptors
        cv::Mat imgGray;
        vector<cv::KeyPoint> kpt;
        cv::cvtColor(imgCrop, imgGray, cv::COLOR_BGR2GRAY);
        _calcKeypoint(imgGray, kpt);
        ptrObj->updateKeypoint(kpt);
      }
    }
    else if (m_task == TRACK_BIKE)
    {
      if (_isValidBikeBBox(bbox))
      {
        Point pCenter = bbox.getCenterPoint();

        Object *ptrObj = &m_currObjList[objIdx];
        ptrObj->updateBoundingBox(bbox);
        ptrObj->updatePointCenter(pCenter);
        ptrObj->updateStatus(1);    //Enable Object
        objIdx += 1;
      }
    }
    else if (m_task == TRACK_MOTORBIKE)
    {
      if (_isValidMotorbikeBBox(bbox))
      {
        Point pCenter = bbox.getCenterPoint();

        Object *ptrObj = &m_currObjList[objIdx];
        ptrObj->updateBoundingBox(bbox);
        ptrObj->updatePointCenter(pCenter);
        ptrObj->updateStatus(1);    //Enable Object
        objIdx += 1;
      }
    }
    else if (m_task == TRACK_HUMAN)
    {
      if (_isValidHumanBBox(bbox))
      {
        Point pCenter = bbox.getCenterPoint();

        Object *ptrObj = &m_currObjList[objIdx];
        ptrObj->updateBoundingBox(bbox);
        ptrObj->updatePointCenter(pCenter);
        ptrObj->updateStatus(1);    //Enable Object
        objIdx += 1;

        // Appearance Features
        BoundingBox rescaleBBox(-1, -1, -1, -1, m_bboxList[i].label);
        utils::rescaleBBox(
          m_bboxList[i], rescaleBBox, m_modelWidth, m_modelHeight, m_videoWidth, m_videoHeight);

        cv::Mat imgCrop;
        imgUtil::cropImages(m_img, imgCrop, rescaleBBox);

        // Get Keypoints and Descriptors
        cv::Mat imgGray;
        vector<cv::KeyPoint> kpt;
        cv::cvtColor(imgCrop, imgGray, cv::COLOR_BGR2GRAY);
        _calcKeypoint(imgGray, kpt);
        ptrObj->updateKeypoint(kpt);

        // // Trajectory
        // vector<Point> tmpTrajectoryList;
        // Point pLocation = m_trajectory->bboxToPointSimple(bbox);
        // ptrObj->updatePointLocation(pLocation);
        // ptrObj->updateTrajectoryList(tmpTrajectoryList);
      }
    }
    else
    {
#if defined (SPDLOG)
      m_logger->debug("m_task = {}, undefined bbox label {}", m_task, bbox.label);
#endif
    }

    if (m_estimateTime)
    {
      time_1 = std::chrono::high_resolution_clock::now();
#if defined (SPDLOG)
      m_logger->info("[_updateCurrObjectList]: \t{} ms", std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
#endif
    }
  }

  return ret;
}


void ObjectTracker::_getMatchScoreDictList(
  Object &queryObj, vector<Object> &prevObjectList,
  vector<std::pair<int, float>> &matchScoreDictListLv1,
  vector<std::pair<int, float>> &matchScoreDictListLv2)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  int queryID = queryObj.id;
  BoundingBox queryBoxSmall = queryObj.getScaledBoundingBox(0.3, m_videoHeight, m_videoWidth);
  BoundingBox queryBoxBig = queryObj.getScaledBoundingBox(0.5, m_videoHeight,m_videoWidth);
  Point queryTLPoint = queryObj.bbox.getCornerPoint()[0];

  int numPrevObject = _getNumEnableObject(prevObjectList);

  // Features
  float rSimilarity = 0.0;
  float rArea = 0.0;            // Bounding Box area ratio
  float rAspect = 1.0;          // Bounding Box aspect ratio
  float rOverlapSmall = 0.0;    // Bounding Box overlapping ratio
  float rOverlapBig = 0.0;      // Bounding Box overlapping ratio
  float rDisappear = 0.0;       // Object disappear ratio

  // Weighting
  float wArea = 1.0;
  float wOverlap = 1.0;
  float wSimilarity = 2.0;

  // Disappear Frames
  int maxDisappear = NUM_FRAME_VALID_SINGLE;
  // In multiple tracking case, we reduce max disappear
  // which can prevent system match wrong person (who was disappear for a long time)
  if (numPrevObject > 1)
    maxDisappear = NUM_FRAME_VALID_MULTI;

  // Bounding Box Area & Aspect Ratio
  float queryArea = queryObj.bbox.getArea();

  // Calculate "Match Score" between input object and others
  int idxList = 0;
  int keyID = 0;
  for (auto it=prevObjectList.begin(); it!=prevObjectList.end(); it++)
  {
    float matchScoreLv1 = 0.0;
    float matchScoreLv2 = 0.0;

    // Get key object's features
    Object *ptrKeyObj = &(*it);

    // skip object if its status is disable
    if (ptrKeyObj->getStatus() == 0)
      continue;

    keyID = ptrKeyObj->id;
    BoundingBox keyBoxSmall = ptrKeyObj->getScaledBoundingBox(0.3, m_videoHeight, m_videoWidth);
    BoundingBox keyBoxBig = ptrKeyObj->getScaledBoundingBox(0.5, m_videoHeight,m_videoWidth);
    Point keyTLPoint = ptrKeyObj->bbox.getCornerPoint()[0];
    float keyArea = ptrKeyObj->bbox.getArea();
    int keyDisappear = ptrKeyObj->disappearCounter;

    // ===
    // Step1. Calculate Similarity Score
    // ===
    // if (m_task == TRACK_CAR)
    rSimilarity = _calcSimilarityScore(*ptrKeyObj, queryObj);

    // ===
    // Step2. Calculate "Bounding Box Overlap" ratio
    // ===
    rOverlapSmall = imgUtil::getBboxOverlapRatio(keyBoxSmall, queryBoxSmall);
    rOverlapBig = imgUtil::getBboxOverlapRatio(keyBoxBig, queryBoxBig);

    // ===
    // Step3. Calculate "Bounding Box Area" ratio
    // ===
    rArea = abs(keyArea-queryArea) / queryArea;


    // ===
    // Step4. Calculate "Disppear Frame" ratio
    // ===
    if ((keyDisappear - m_tObjDisappearFrame) > 0)
      rDisappear = 1.0 - (keyDisappear/maxDisappear);
    else
      rDisappear = 1.0;


    // if disappear for a long time set r disappear to 0.5
    // and then make sure rDisappear won't less than 0.5
    if (keyDisappear > static_cast<int>(maxDisappear*0.85))
      rDisappear = 0.5;

    // ===
    // Step5. Calculate "Matching Score"
    // ===
    matchScoreLv1 = (rOverlapSmall * wOverlap) + (rSimilarity * wSimilarity);
    matchScoreLv2 = (rOverlapBig * wOverlap) + (rSimilarity * wSimilarity);

    // Adjust matching score by disappear ratio
    matchScoreLv1 *= rDisappear;
    matchScoreLv2 *= rDisappear;

    // ===
    // Step6. Decrease match socre to distinguish with real match
    // ===

    // Case1: If key bounding box area are two times than query bounding box area
    if (rArea > 2.0)
    {
      matchScoreLv1 *= 0.01;
      matchScoreLv2 *= 0.01;
    }
    // else if (rArea > 1.0)
    // {
    //   matchScoreLv1 *= 0.1;
    //   matchScoreLv2 *= 0.1;
    // }
    // else if (rArea > 0.5)
    // {
    //   matchScoreLv1 *= 0.5;
    //   matchScoreLv2 *= 0.5;
    // }

    if (m_debugMode)
    {
      cout << "----------------------------------------------" << endl;
      cout << "Compare to object list index: [{" << idxList << "}] => Key ID: [{" << keyID << "}]" << endl;
      cout << "----------------------------------------------" << endl;
      cout << "rSimilarity=" << rSimilarity << "\t\t wSimilarity=" << wOverlap << endl;
      cout << "rOverlapSmall=" << rOverlapSmall << "\t\t wOverlap=" << wOverlap << endl;
      cout << "rOverlapBig=" << rOverlapBig << "\t\t wOverlap=" << wOverlap << endl;
      cout << "rArea=" << rArea << "\t\t wArea=" << wArea << endl;
      cout << "rDisappear=" << rDisappear << endl;
      cout << "===> Match Socre (Lv1) = " << matchScoreLv1 << endl;
      cout << "===> Match Socre (Lv2) = " << matchScoreLv2 << endl;
      cout << "----------------------------------------------" << endl;
    }

    // Update match score dict
    matchScoreDictListLv1.push_back(std::make_pair(keyID, matchScoreLv1));
    matchScoreDictListLv2.push_back(std::make_pair(keyID, matchScoreLv2));
    //
    idxList += 1;
  }
}


int ObjectTracker::_calcDisappearThreshold(int numPrevObj, int numCurrObj, int frameInterval)
{
  int tDisappear = static_cast<int>(NUM_FRAME_VALID_MULTI / frameInterval);
  if ((numPrevObj <= 1) || (numCurrObj <= 1))
    tDisappear = static_cast<int>(NUM_FRAME_VALID_SINGLE / frameInterval);

  // if object list is full then speedup cleaning process ...
  if (numPrevObj >= m_maxTracking-1)
    tDisappear = (int)(tDisappear * 0.5);

  return tDisappear;
}


void ObjectTracker::_updateRemoveObjList(
  vector<Object>& objList, vector<int>& list, int numPrevObj, int threshold)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  for (int i=0; i<numPrevObj; i++)
  {
    int disappear = objList[i].disappearCounter;
    if (disappear > threshold)
    {
#if defined (SPDLOG)
      if (m_debugMode)
      {
        m_logger->debug("Remove object profile");
        m_logger->debug("Detect no object and prev object profile disappear");
      }
#endif
      list.push_back(objList[i].id);
    }

    // Revmoe object when it was disappear and out of ROI
    if ((disappear > 2) && (numPrevObj > 1))
    {
      Point pCenter = objList[i].pCenter;
      int height = objList[i].bbox.getHeight();
      // Out of ROI
      if (((pCenter.x < m_roi->x1 || pCenter.x > m_roi->x2) && (height > m_roi->y2)))
        list.push_back(i);
    }
  }
}


void ObjectTracker::_ReID(
  vector<Object> &currObjectList, vector<Object> &prevObjectList, int frameInterval)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

  // Get Number of enabled objects
  int numPrevObject = _getNumEnableObject(prevObjectList);
  int numCurrObject = _getNumEnableObject(currObjectList);

  // Define "Matching Score" threshold
  float tMatch = m_tMatchMultiple;
  if (numPrevObject == 1) tMatch = m_tMatchSingle;

#if defined (SPDLOG)
  if (m_debugMode)
  {
    string taskStr = "";
    if (m_task == TRACK_HUMAN)
      taskStr = "Human";
    else if (m_task == TRACK_BIKE)
      taskStr = "Bike";
    else if (m_task == TRACK_CAR)
      taskStr = "Vehicle";
    else if (m_task == TRACK_MOTORBIKE)
      taskStr = "Motorbike";

    m_logger->debug("\n[{} Re-Identification]", taskStr);
    m_logger->debug("----------------------------------------------");
    m_logger->debug("Num Prev Obj = {}", numPrevObject);
    m_logger->debug("Num Curr Obj = {}", numCurrObject);
  }
#endif

  // Number comparison to previous objects
  int numCompareToPrev = COMPARE_SAME;        // same
  if (numCurrObject < numPrevObject)
    numCompareToPrev = COMPARE_LESS;          // less
  else if (numCurrObject > numPrevObject)
    numCompareToPrev = COMPARE_MORE;          // more
  else
    numCompareToPrev = COMPARE_SAME;

  // STEP0. Calculate Disappear Threshold
  int tDisappear = _calcDisappearThreshold(numPrevObject, numCurrObject, frameInterval);

  // STEP1. Remove objects that disappear for a long time
  vector<int> idxToRemoveList;
  _updateRemoveObjList(prevObjectList, idxToRemoveList, numPrevObject, tDisappear);

  // STEP2. Increase prev objects's disappear counter
  _increaseDisappearCounter(prevObjectList);

  // STEP3. Get all enabled key object id from prev frame
  vector<int> prevKeyIdList = _getEnableKeyIdList(prevObjectList);

  // STEP4. Disable be overlapped bounding boxes
  _disableOverlapObject(currObjectList);
  numCurrObject = _getNumEnableObject(currObjectList);  // update current object number

#if defined (SPDLOG)
  if (m_debugMode)
  {
    m_logger->debug("Num Curr Obj (Disable Overlap Objects) = {}", numCurrObject);
  }
#endif

  // STEP5. Calculate Matching Score
  // => queryMatchScoreDict = {queryID: [{keyID_1: MS_1}, {keyID_2: MS_2}, ...]}
  vector<pair<int, vector<pair<int, float>>>> queryMatchScoreDict;
  vector<pair<int, vector<pair<int, float>>>> queryMatchScoreDictBkp;

  _getQueryMatchScoreDict(
    currObjectList,
    prevObjectList,
    prevKeyIdList,
    queryMatchScoreDict,
    queryMatchScoreDictBkp);

  // STEP6. Self Attention => query vs. key
  // For each query id, find key id with max match score
  // => selfMatchScoreDict = {queryID: {keyID: MS}}
  vector<pair<int, pair<int, float>>> selfMatchScoreDict;
  selfMatchScoreDict = _getSelfMatchScoreDict(queryMatchScoreDict, tMatch);

  // STEP7. Cross Attention => Key vs. Query
  // => crossMatchScoreDict = {keyID: {queryID: MS}}
  vector<pair<int, pair<int, float>>> crossMatchScoreDict;
  crossMatchScoreDict = _getCrossMatchScoreDict(
    queryMatchScoreDict,
    queryMatchScoreDictBkp,
    selfMatchScoreDict,
    prevKeyIdList,
    numCompareToPrev,
    numCurrObject,
    numPrevObject);

  // STEP8. Update previous objects
  _updatePrevObjectList(
    crossMatchScoreDict, prevObjectList, currObjectList, prevKeyIdList);

  // STEP9. Output tracking result
  for (int i=0; i<(int)prevObjectList.size(); i++)
  {
    Object *ptrPrevObj = &prevObjectList[i];
    int id = ptrPrevObj->id;
    if (m_debugMode) cout << "Obj[" << i << "]:";
    // if (!isKeyInDict(id, idxToRemoveList) && ptrPrevObj->getStatus() == 1)
    if ((ptrPrevObj->disappearCounter < tDisappear) && ptrPrevObj->getStatus() == 1)
    {
      prevObjectList[i].updateStatus(1);
      if (m_debugMode) cout << "update satus = 1" << endl;
    }
    else
    {
      prevObjectList[i].init(m_frameStamp);
      if (m_debugMode) cout << "init()" << endl;
    }
  }

  int numFinalObj = _getNumEnableObject(prevObjectList);
#if defined (SPDLOG)
  if (m_debugMode) m_logger->debug("Num Prev Obj = {}", numPrevObject);
  if (m_debugMode) m_logger->debug("Num Curr Obj (Final) = {}", numFinalObj);

  if (m_estimateTime)
  {
    time_1 = std::chrono::high_resolution_clock::now();
    m_logger->info("[_ReID]: \t{} ms", std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
  }
#endif
}


void ObjectTracker::_getQueryMatchScoreDict(
  vector<Object> &currObjectList,
  vector<Object> &prevObjectList,
  vector<int> &prevKeyIdList,
  vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDict,
  vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDictBkp)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif
  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

  vector<pair<int, vector<pair<int, float>>>> matchScoreDictListDict; // For debug

  for (int i=0; i<(int)currObjectList.size(); i++)
  {
    int queryId = i;
    Object *ptrQueryObj = &currObjectList[i];

    if (ptrQueryObj->getStatus() == 0)
      continue;

#if defined (SPDLOG)
    if (m_debugMode) m_logger->debug("Query ID [{}] calculate match score ...", queryId);
    // matchScoreDict => [{KeyID_1: MS_1}, {KeyID_2: MS_2}, ...]
#endif

    vector<std::pair<int, float>> matchScoreDictList;
    vector<std::pair<int, float>> matchScoreDictListBkp;
    _getMatchScoreDictList(*ptrQueryObj, prevObjectList, matchScoreDictList, matchScoreDictListBkp);

    // Push Query ID and corresponding match score dict into queryMatchScoreDict
    // queryMatchScoreDict => {queryID: [{KeyID_1: MS_1}, {KeyID_2: MS_2}, ...]}
    queryMatchScoreDict.push_back(std::make_pair(queryId, matchScoreDictList));
    queryMatchScoreDictBkp.push_back(std::make_pair(queryId, matchScoreDictListBkp));

    if (m_debugMode)
    {
      // Generate match score dict list dict
      // => {keyID: [{q1: ms1}, {q2: ms2}, ...]}
      for (auto it=prevKeyIdList.begin(); it!=prevKeyIdList.end(); it++)
      {
        int keyId = (*it);
        float matchScore = 0.0;

        // Find Key ID's match score
        for (int i=0; i<(int)matchScoreDictList.size(); i++)
        {
          if (keyId == matchScoreDictList[i].first)
            matchScore = matchScoreDictList[i].second;
        }

        // Add Key ID and corresponding {Query ID: match score} into dict list
        if (!utils::isKeyInDict(keyId, matchScoreDictListDict))
        {
          vector<pair<int, float>> _tmpScoreDict;
          _tmpScoreDict.push_back(std::make_pair(queryId, matchScore));
          matchScoreDictListDict.push_back(std::make_pair(keyId, _tmpScoreDict));
        }
        else
        {
          int idx = utils::findListIdx(keyId, matchScoreDictListDict);
          matchScoreDictListDict[idx].second.push_back(std::make_pair(queryId, matchScore));
        }
      }
    }
  }

  if (m_debugMode)
  {
    // matchScoreDictListDict => {keyID: [{q1: ms1}, {q2: ms2}, ...]}
    std::cout << "\nMatch Score Dict List Dict = " << endl;
    for (auto it=matchScoreDictListDict.begin(); it!=matchScoreDictListDict.end(); it++)
    {
      std::cout << "Key [" << (*it).first << "] => ";
      for (auto iit=(*it).second.begin(); iit!=(*it).second.end(); iit++)
      {
        std::cout << "{" << (*iit).first << ": " << setprecision(4) << (float)(*iit).second << "}, ";
      }
      std::cout << endl;
    }


    std::cout << "\nqueryMatchScoreDict = " << endl;
    for (auto it=queryMatchScoreDict.begin(); it!=queryMatchScoreDict.end(); it++)
    {
      std::cout << "QueryID [" << (*it).first << "] => ";
      for (auto iit=(*it).second.begin(); iit!=(*it).second.end(); iit++)
      {
        std::cout << "{KeyID [" << (*iit).first << "]: " << setprecision(4) << (float)(*iit).second << "}, ";
      }
      std::cout << endl;
    }
  }

#if defined (SPDLOG)
  if (m_estimateTime)
  {
    time_1 = std::chrono::high_resolution_clock::now();
    m_logger->info("[_getQueryMatchScoreDict]: \t{} ms", \
      std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
  }
#endif
}


vector<pair<int, pair<int, float>>> ObjectTracker::_getSelfMatchScoreDict(
  vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDict,
  float thresholdMatch)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

  vector<pair<int, pair<int, float>>> selfMatchScoreDict;
  for (auto it=queryMatchScoreDict.begin(); it!=queryMatchScoreDict.end(); it++)
  {
    int queryId = (*it).first;
    vector<pair<int, float>> matchScoreDict = (*it).second;

    int keyId = 0;
    float maxScore = 0.0;

    // Find Max Score Key ID and return max match score
    utils::findMaxScoreKey(matchScoreDict, keyId, maxScore);

    // Make sure "match score" over threshold
    if (maxScore > thresholdMatch)
    {
      pair<int, pair<int, float>> res = std::make_pair(queryId, std::make_pair(keyId, maxScore));
      selfMatchScoreDict.push_back(res);
    }
  }

  if (m_debugMode)
  {
    std::cout << "1st Self Match Score Dict (query, (key, score)) = ";
    for (auto it=selfMatchScoreDict.begin(); it!=selfMatchScoreDict.end(); it++)
    {
      std::cout << "Query [" << (*it).first << "] => ";
      std::cout << "{" << (*it).second.first << ": " << (*it).second.second << "}";
      std::cout << endl;
    }
  }

  // For each Key ID, find best matched Query ID by comparing match score
  for (int i=0; i<(int)selfMatchScoreDict.size(); i++)
  {
    int queryIdA = selfMatchScoreDict[i].first;
    int keyIdA = selfMatchScoreDict[i].second.first;
    float scoreA = selfMatchScoreDict[i].second.second;

    for (int j=0; j<(int)selfMatchScoreDict.size(); j++)
    {
      int queryIdB = selfMatchScoreDict[j].first;
      int keyIdB = selfMatchScoreDict[j].second.first;
      float scoreB = selfMatchScoreDict[j].second.second;

      if ((keyIdA == keyIdB) && (queryIdA != queryIdB))
      {
        // if match score A is less than score B
        // then find Top-2 match score
        if (scoreA < scoreB)
        {
          // Get queryID A's score dict
          vector<pair<int, float>> _scoreDict;
          int idxList = utils::findListIdx(queryIdA, queryMatchScoreDict);
          _scoreDict = queryMatchScoreDict[idxList].second;

          // Update queryID A's score dict
          // Find key id with max match socre while its id != key ID A
          // Initialize key ID with key ID A
          // Initialize max match score with 0
          int keyIdNew = keyIdA;
          float maxMatchScore = 0.0;
          for (auto it=_scoreDict.begin(); it!=_scoreDict.end(); it++)
          {
            int _keyId = (*it).first;
            float _score = (*it).second;
            if ((_score > maxMatchScore) && (_keyId != keyIdA))
            {
              keyIdNew = _keyId;
              maxMatchScore = _score;
            }
          }

          // Update queryID A's score dict
          selfMatchScoreDict[i].second = std::make_pair(keyIdNew, maxMatchScore);
        }
      }
    }
  }

  if (m_debugMode)
  {
    std::cout << "2nd Self Match Score Dict (query, (key, score)) = ";
    for (auto it=selfMatchScoreDict.begin(); it!=selfMatchScoreDict.end(); it++)
    {
      std::cout << "Query [" << (*it).first << "] => ";
      std::cout << "{" << (*it).second.first << ": " << (*it).second.second << "}";
      std::cout << std::endl;
    }
  }

  // Remove query ID if its match score is less than threshold
  vector<int> queryIdRemovedList;
  for (auto it=selfMatchScoreDict.begin(); it!=selfMatchScoreDict.end(); it++)
  {
    int queryId = (*it).first;
    float score = (*it).second.second;

    if (score < thresholdMatch)
    {
      if (m_debugMode) std::cout << "Query [" << queryId << "] match score = " << \
        score << " < threshold " << thresholdMatch << " ... remove!" << std::endl;
      queryIdRemovedList.push_back(queryId);
    }
  }

  for (auto it=queryIdRemovedList.begin(); it!=queryIdRemovedList.end(); it++)
  {
    int queryIdRemoved = (*it);
    vector<pair<int, pair<int, float>>>::iterator itr = selfMatchScoreDict.begin();
    while (itr != selfMatchScoreDict.end())
    {
      int queryId = (*itr).first;
      if (queryId == queryIdRemoved)
      {
        // m_logger->debug("[SelfAttention] BEFORE REMOVE !!!!");
        itr = selfMatchScoreDict.erase(itr);
        // m_logger->debug("[SelfAttention] AFTER REMOVE !!!!");
      }
      else
      {
        ++itr;
      }
    }
  }

  if (m_debugMode)
  {
    std::cout << "Final Self Match Score Dict (query, (key, score)) = ";
    for (auto it=selfMatchScoreDict.begin(); it!=selfMatchScoreDict.end(); it++)
    {
      std::cout << "Query [" << (*it).first << "] => ";
      std::cout << "{" << (*it).second.first << ": " << (*it).second.second << "}";
      std::cout << endl;
    }
  }

#if defined (SPDLOG)
  if (m_estimateTime)
  {
    time_1 = std::chrono::high_resolution_clock::now();
    m_logger->info("[_getSelfMatchScoreDict]: \t{} ms", std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
  }
#endif

  return selfMatchScoreDict;
}


vector<pair<int, pair<int, float>>> ObjectTracker::_getCrossMatchScoreDict(
  vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDict,
  vector<pair<int, vector<pair<int, float>>>> &queryMatchScoreDictBkp,
  vector<pair<int, pair<int, float>>> &selfMatchScoreDict,
  vector<int> &prevKeyIdList,
  int numCompareToPrev,
  int numCurrObject,
  int numPrevObject)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

  vector<pair<int, pair<int, float>>> crossMatchScoreDict;
  vector<int> matchedQueryIdList;
  vector<int> matchedKeyIdList;

  for (auto it=prevKeyIdList.begin(); it!=prevKeyIdList.end(); it++)
  {
    int prevKeyId = (*it);

    // For each key ID from previous frame
    // add its best matched query id and match score into cross attention dict
    for (int i=0; i<(int)selfMatchScoreDict.size(); i++)
    {
      int queryId = selfMatchScoreDict[i].first;
      int bestKeyId = selfMatchScoreDict[i].second.first;
      float bestMatchScore = selfMatchScoreDict[i].second.second;

      // Add best <key ID, <query ID, match score>> into cross match score dict
      if ((prevKeyId == bestKeyId) && (!utils::isKeyInDict(prevKeyId, crossMatchScoreDict)))
      {
        crossMatchScoreDict.push_back(std::make_pair(prevKeyId, std::make_pair(queryId, bestMatchScore)));
        matchedQueryIdList.push_back(queryId);  //  preserve saved query id
        matchedKeyIdList.push_back(prevKeyId);  //  preserve saved key id
      }
      else if (prevKeyId == bestKeyId)  // if conflit then compare match score
      {
        int prevIdx = 0;
        float prevBestScore = 0.0;

        // Find previous best matched Key ID and match score
        for (int j=0; j<(int)crossMatchScoreDict.size(); j++)
        {
          int keyId = crossMatchScoreDict[j].first;
          if (prevKeyId == keyId)
          {
            prevBestScore = crossMatchScoreDict[j].second.second;
            prevIdx = j;
          }
        }

        // And then compare match score
        // if new match score is greater than previous best match score
        // then update matched "crossMatchScoreDict"
        if (bestMatchScore > prevBestScore)
        {
          // Remove previous query id from matched query id list
          int prevQueryId = crossMatchScoreDict[prevIdx].second.first;
          vector<int>::iterator itr = matchedQueryIdList.begin();
          while (itr != matchedQueryIdList.end())
          {
            int queryId = (*itr);
            if (queryId == prevQueryId)
            {
              // m_logger->debug("[CrossAttention] BEFORE REMOVE !!!!");
              itr = matchedQueryIdList.erase(itr);
              // m_logger->debug("[CrossAttention] AFTER REMOVE !!!!");
            }
            else
            {
              ++itr;
            }
          }

          // Replace previous query id and match score with new one
          crossMatchScoreDict[prevIdx].second = std::make_pair(queryId, bestMatchScore);
          matchedQueryIdList.push_back(queryId);  //  preserve saved query id
        }
      }
    }
  }

  if (m_debugMode)
  {
    std::cout << "1st Cross Match Score Dict (key, (query, score)) = ";
    for (auto it=crossMatchScoreDict.begin(); it!=crossMatchScoreDict.end(); it++)
    {
      std::cout << "Key [" << (*it).first << "] => ";
      std::cout << "{" << (*it).second.first << ": " << (*it).second.second << "}";
      std::cout << std::endl;
    }
  }

  // If object is miss matched, we need to match it
  int numMatched = crossMatchScoreDict.size();
  if (numCompareToPrev <= COMPARE_SAME)  // curr is "less than prev" or "equal to prev"
  {
    int numMissMatched = numCurrObject - numMatched;

#if defined (SPDLOG)
    if (m_debugMode)
    {
      m_logger->debug("Curr Object Num = {}", numCurrObject);
      m_logger->debug("Prev Object Num = {}", numPrevObject);
      m_logger->debug("Num Matched = {}", numMatched);
      m_logger->debug("Num Miss Matched = {}", numMissMatched);
    }
#endif
    // // For multi object cases only
    // if ((numPrevObject > 1) && (numMissMatched > 0))
    // {
    //   // queryMatchScoreDict => {queryID: [{keyID1, MatchScore1}, {keyID2, MatchScore2}, ...]}
    //   // For unmatched query Id, find the best match from its key Id list
    //   // for (int i=0; i<(int)queryMatchScoreDict.size(); i++) //TODO:
    //   for (int i=0; i<(int)queryMatchScoreDictBkp.size(); i++) //TODO:
    //   {
    //     int queryId = queryMatchScoreDict[i].first;
    //     // vector<pair<int, float>> matchDictList = queryMatchScoreDict[i].second; //TODO:
    //     vector<pair<int, float>> matchDictList = queryMatchScoreDictBkp[i].second;

    //     // if query ID not in matched query id list
    //     if (!isKeyInDict(queryId, matchedQueryIdList))
    //     {
    //       // Find key id that wasn't matched with max match score
    //       int bestKeyId = 0;
    //       float maxMatchScore = 0.0;
    //       for (int j=0; j<(int)matchDictList.size(); j++)
    //       {
    //         int keyId = matchDictList[j].first;
    //         float matchScore = matchDictList[j].second;
    //         if ((matchScore > maxMatchScore) && (!isKeyInDict(keyId, matchedKeyIdList)))
    //         {
    //           bestKeyId = keyId;
    //           maxMatchScore = matchScore;
    //         }
    //       }

    //       // if max match score > threahold, then this match is valild
    //       if (maxMatchScore > m_tMatchMinRequriement)
    //       {
    //         matchedQueryIdList.push_back(queryId);

    //         crossMatchScoreDict.push_back(
    //           std::make_pair(bestKeyId, std::make_pair(queryId, maxMatchScore)));
    //       }
    //     }
    //   }

    //   if (m_debugMode)
    //   {
    //     m_logger->debug("2nd Cross Match Score Dict (key, (query, score)) = ";
    //     for (auto it=crossMatchScoreDict.begin(); it!=crossMatchScoreDict.end(); it++)
    //     {
    //       m_logger->debug("Key [" << (*it).first << "] => ";
    //       m_logger->debug("{" << (*it).second.first << ": " << (*it).second.second << "}";
    //       m_logger->debug(std::endl;
    //     }
    //   }
    // }
  }
  else if (numCompareToPrev == COMPARE_MORE) // Curr is greater than prev
  {
#if defined (SPDLOG)
    if (m_debugMode) m_logger->debug("numCompareToPrev == COMPARE_MORE");
#endif
  }

#if defined (SPDLOG)
  if (m_estimateTime)
  {
    time_1 = std::chrono::high_resolution_clock::now();
    m_logger->info("[_getCrossMatchScoreDict]: \t{} ms", std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
  }
#endif

  return crossMatchScoreDict;
}


void ObjectTracker::_updatePrevObjectList(
  vector<pair<int, pair<int, float>>> &crossMatchScoreDict,
  vector<Object> &prevObjectList,
  vector<Object> &currObjectList,
  vector<int> &prevKeyIdList)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

#if defined (SPDLOG)
  // Update Object content
  if (m_debugMode)
    m_logger->debug("Match Results:");
#endif

  vector<int> updatedQueryIdList;
  for (int i=0; i<(int)crossMatchScoreDict.size(); i++)
  {
    int keyId = crossMatchScoreDict[i].first;
    int queryId = crossMatchScoreDict[i].second.first;
    float matchScore = crossMatchScoreDict[i].second.second;

    // if (m_debugMode) m_logger->debug("Key [" << keyId << "] / Query [" << queryId << "] / Match Score = " << matchScore);

    // update object list if key id is in previous key id list
    if (utils::isKeyInDict(keyId, prevKeyIdList))
    {
      // Find list index of key id
      int idxKey = 0;
      for (int j=0; j<(int)prevObjectList.size(); j++)
      {
        if (keyId == prevObjectList[j].id)
        {
          idxKey = j;
          break;
        }
      }

      // Find list index of query id
      int idxQuery = 0;
      for (int q=0; q<(int)currObjectList.size(); q++)
      {
        // if (m_debugMode) m_logger->debug("Find query ID ...");
        // if (m_debugMode) m_logger->debug("queryId = " << queryId << ", currObjectList[q].id = " << currObjectList[q].id);

        if (queryId == currObjectList[q].id)
        {
          // if (m_debugMode) m_logger->debug("idxQuery = " << q);
          idxQuery = q;
          break;
        }
      }

      // if (m_debugMode) m_logger->debug("idxKey = " << idxKey << ", keyId = " << keyId);
      // if (m_debugMode) m_logger->debug("idxQuery = " << idxQuery);
      // if (m_debugMode) m_logger->debug("currObjectList[idxQuery].getStatus() = " << currObjectList[idxQuery].getStatus());

      // Save previous object location point
      prevObjectList[idxKey].id = keyId;
      prevObjectList[idxKey].updatePointCenter(currObjectList[idxQuery].pCenter);

      // Update bounding box
      BoundingBox prevBbox = prevObjectList[idxKey].bbox;
      BoundingBox newBbox = currObjectList[idxQuery].bbox;
      int frameInterval = newBbox.frameStamp - prevBbox.frameStamp;

      // If bounding box changed too much, then use previous one
      if (!currObjectList[idxQuery].discardPrevBoundingBox)
      {
        newBbox = prevObjectList[idxKey].bbox;
      }

      prevObjectList[idxKey].updateBoundingBox(newBbox);

      // Each object preserves at most MAX_BBOX_LIST_SIZE bounding boxes.
      if (prevObjectList[idxKey].bboxList.size() < MAX_BBOX_LIST_SIZE)
      {
        prevObjectList[idxKey].bboxList.push_back(newBbox);
        prevObjectList[idxKey].m_lastDetectBoundingBox = newBbox;
      }
      else
      {
        prevObjectList[idxKey].bboxList.erase(prevObjectList[idxKey].bboxList.begin());
        prevObjectList[idxKey].bboxList.push_back(newBbox);
        prevObjectList[idxKey].m_lastDetectBoundingBox = newBbox;
      }

      // Appearance
      prevObjectList[idxKey].updateKeypoint(currObjectList[idxQuery].m_currKpts);

      // // Following Distance
      // _calcDistance(prevObjectList[idxKey]);
      // if (prevObjectList[idxKey].m_distanceList.size() < MAX_BBOX_LIST_SIZE)
      // {
      //   prevObjectList[idxKey].m_distanceList.push_back(prevObjectList[idxKey].distanceToCamera);
      // }
      // else
      // {
      //   prevObjectList[idxKey].m_distanceList.erase(prevObjectList[idxKey].m_distanceList.begin());
      //   prevObjectList[idxKey].m_distanceList.push_back(prevObjectList[idxKey].distanceToCamera);
      // }

      // Refresh disappear counter
      prevObjectList[idxKey].disappearCounter = 0;

      // Increase alive counter
      // m_logger->debug("ID[" << idxKey << "] moving speed = " << speed);
      prevObjectList[idxKey].aliveCounter += 1;

      if (prevObjectList[idxKey].aliveCounter > MAX_OBJ_ALIVE_COUNT)
      {
        prevObjectList[idxKey].aliveCounter = m_tAliveCounter;
      }

      // Update model embedding only when aspect ratio is vaild
      if (m_debugMode) std::cout << "=> Match: Query [" << queryId \
        << "] / Key [" << keyId << "] / Match Score = " << matchScore << std::endl;

      // Save updated query id
      updatedQueryIdList.push_back(queryId);
    }
  }

  // Add new objects if they didn't match anyone
  for (int i=0; i<(int)currObjectList.size(); i++)
  {
    int queryId = i;

    if (currObjectList[i].getStatus() == 0)
      continue;

    if (!utils::isKeyInDict(queryId, updatedQueryIdList))
    {
      // Find new key ID
      if (m_debugMode)
        std::cout << "[Tracker] >>> Find a new query ID: " << queryId << std::endl;

      int numPrevObj = _getNumEnableObject(prevObjectList);
      int keyIdNew = -1;    // initialize

      // Select an empty object which status is disable and p location is default value
      int idx = 0;
      for (int j=0; j<(int)prevObjectList.size(); j++)
      {
        if (prevObjectList[j].getStatus() == 0 && \
            prevObjectList[j].pCenter.x == -1 && \
            prevObjectList[j].pCenter.y == -1)
        {
          keyIdNew = prevObjectList[j].id;
          if (m_debugMode)
            std::cout << "[Tracker] >>> KeyIdNew = " << keyIdNew << std::endl;
          idx = j;
          break;
        }
      }

      // if find an empty object and then assign current object contents to it
      if (keyIdNew != -1)
      {
        if (m_debugMode) std::cout << "[Tracker] >>> queryId = " << queryId << std::endl;
        prevObjectList[idx].id = keyIdNew;
        prevObjectList[idx].updateStatus(1);
        prevObjectList[idx].updateBoundingBox(currObjectList[queryId].bbox);
        prevObjectList[idx].updateBoundingBoxList(currObjectList[queryId].bboxList);
        prevObjectList[idx].updatePointCenter(currObjectList[queryId].pCenter);
        prevObjectList[idx].disappearCounter = 0;
        prevObjectList[idx].aliveCounter = 1;

        prevObjectList[idx].updateKeypoint(currObjectList[queryId].m_currKpts);

        // Distance
        prevObjectList[idx].distanceToCamera = -1;
      }
      else
      {
        if (m_debugMode) std::cout << "[Tracker] >>> Object is full, no room for new object ... " << std::endl;
      }
    }
  }

#if defined (SPDLOG)
  if (m_estimateTime)
  {
    time_1 = std::chrono::high_resolution_clock::now();
    m_logger->info("[_updatePrevObjectList]: \t{} ms", std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
  }
#endif
}


void ObjectTracker::_calcKeypoint(cv::Mat &img, vector<cv::KeyPoint> &kpt)
{
  cv::resize(img, img, cv::Size(256, 256), cv::INTER_NEAREST);
  detKeypointsORB(kpt, img, false, false);
}


float ObjectTracker::_calcSimilarityScore(Object &objKey, Object &objQuery)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  float avgAngleQuery = 0;
  float avgAngleKey = 0;
  for (int i=0; i<objQuery.m_currKpts.size(); i++)
  {
    avgAngleQuery += objQuery.m_currKpts[i].angle;
  }
  for (int i=0; i<objKey.m_currKpts.size(); i++)
  {
    avgAngleKey += objKey.m_currKpts[i].angle;
  }

  avgAngleQuery /= objQuery.m_currKpts.size();
  avgAngleKey /= objKey.m_currKpts.size();

#if defined (SPDLOG)
  m_logger->debug("=> avg angle (Query) = {}", avgAngleQuery);
  m_logger->debug("=> avg angle (Key) = {}", avgAngleKey);
#endif

  float angleDiff = abs(avgAngleQuery - avgAngleKey);
  float similarityScore = 1.0 - angleDiff/360.0;

  // ratio of bounding box area
  float areaKey = objKey.bbox.getArea();
  float areaQuery = objQuery.bbox.getArea();
  float rArea = 1.0 - (abs(areaKey-areaQuery) / max(areaKey, areaQuery));

#if defined (SPDLOG)
  m_logger->debug("=> rArea = {}", rArea);
#endif

  similarityScore *= rArea;

  return similarityScore;
}


void ObjectTracker::_updateBoundingBox(BoundingBox &srcBox, BoundingBox &newBox)
{
  int xOffset = m_roi->x1;
  int yOffset = m_roi->y1;

  BoundingBox _box(-1, -1, -1, -1, -1);
  _box.x1 = srcBox.x1;
  _box.y1 = srcBox.y1;
  _box.x2 = srcBox.x2;
  _box.y2 = srcBox.y2;
  _box.label = srcBox.label;

  utils::rescaleBBox(_box, _box, m_modelWidth, m_modelHeight, m_videoWidth, m_videoHeight);

  int boxWidth = _box.getWidth();
  int boxHeight = _box.getHeight();

  _box.x1 -= xOffset;
  _box.y1 -= yOffset;

  _box.x2 = _box.x1 + boxWidth;
  _box.y2 = _box.y1 + boxHeight;

  cv::Rect roi;
  roi.width = _box.getWidth();
  roi.height = _box.getHeight();
  roi.x = _box.x1;
  roi.y = _box.y1;

  _box.roi = roi;

  newBox = _box;
}


void ObjectTracker::_calcDistance(Object &obj)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);

  if (m_task == TRACK_HUMAN)
  {
    m_logger->debug("Human Distance =>");
  }
  else if (m_task == TRACK_BIKE)
  {
    m_logger->debug("Bike Distance =>");
  }
  else if (m_task == TRACK_MOTORBIKE)
  {
    m_logger->debug("Motorbike Distance =>");
  }
  else if (m_task == TRACK_CAR)
  {
    m_logger->debug("Vehicle Distance =>");
  }
#endif

  int bboxListSize = obj.bboxList.size();
  if (bboxListSize == 0)
  {
#if defined (SPDLOG)
    m_logger->debug("skip object {}, because bboxList size is 0", obj.id);
#endif
    return;
  }

  if (bboxListSize < 2)
  {
#if defined (SPDLOG)
    m_logger->debug("skip object {}, because bbox list size < 2", obj.id);
#endif
    return;
  }

  if (obj.getStatus() == 0)
  {
#if defined (SPDLOG)
    m_logger->debug("skip object {}, because getStatus is 0", obj.id);
#endif
    return;
  }


  /* Distance Calculation */
  vector<float> distanceList;
  for (int j = obj.disappearCounter; j < bboxListSize; j++)
  {
    float distance = 0.0;
    BoundingBox rescaleBBox(-1, -1, -1, -1, -1);
    utils::rescaleBBox(obj.bboxList[j], rescaleBBox, m_modelWidth, m_modelHeight, m_videoWidth, m_videoHeight);

    // //TODO:
    // if (m_yVanish == 0)
    //   distance = NAN;
    // else
    //   distance = _calcFollowDistance(rescaleBBox, (int)(m_videoWidth / 2), m_yVanish);

    // if (distance > 0)
    //   distanceList.push_back(distance);
  }

  // Distance
  float c = 0;
  float avgDistance = 0.0;

  cout << "Object " << obj.id << "distanceList = [";
  for (int j=0; j<distanceList.size(); j++)
  {
    if (distanceList[j] > 0)
    {
      avgDistance += distanceList[j];
      c++;
      cout << distanceList[j] << " ";
    }
  }
  cout << "] " << endl;
  avgDistance = avgDistance / c;

  obj.distanceToCamera = avgDistance;

#if defined (SPDLOG)
  m_logger->debug("Object ID: {} => distance = {}, disappear = {}", obj.id, avgDistance, obj.disappearCounter);
#endif

}


// ============================================
//              Following Distance
// ============================================

void ObjectTracker::_getBoxAnchor(BoundingBox &bbox, cv::Point &pAnchor)
{
  pAnchor.x = bbox.getCenterPoint().x;
  pAnchor.y = bbox.getCenterPoint().y + (int)(bbox.getHeight() / 2);
}


// ============================================
//                  Utils
// ============================================
int ObjectTracker::_getNumEnableObject(vector<Object> &_objList)
{
  int numEnableObj = 0;
  // m_logger->debug("_objList.size() = " << _objList.size());
  for (int i=0; i<(int)_objList.size(); i++)
  {
    numEnableObj += _objList[i].getStatus();
    // m_logger->debug("objList[" << i << "] status = " << _objList[i].getStatus());
  }
  return numEnableObj;
}


void ObjectTracker::_increaseDisappearCounter(vector<Object> &objList)
{
  for (auto it=objList.begin(); it!=objList.end(); it++)
  {
    (*it).disappearCounter += 1;
  }
}


vector<int> ObjectTracker::_getEnableKeyIdList(vector<Object> &objList)
{
  vector<int> enableKeyIdList;
  for (auto it=objList.begin(); it!=objList.end(); it++)
  {
    // if object is enable
    if (it->getStatus() == 1)
      enableKeyIdList.push_back(((*it).id));
  }
  return enableKeyIdList;
}


void ObjectTracker::_filterOverlapObject()
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  for (int i=0; i<m_prevObjList.size(); i++)
  {
    if (m_prevObjList[i].getStatus() == 0)
      continue;

    int areaA = m_prevObjList[i].bbox.getArea();


    for (int j=0; j<m_prevObjList.size(); j++)
    {
      if ((i!=j) && (m_prevObjList[j].getStatus() != 0))
      {
        int areaB = m_prevObjList[j].bbox.getArea();
        float overlapRatio = imgUtil::getBboxOverlapRatio(m_prevObjList[i].bbox, m_prevObjList[j].bbox);
#if defined (SPDLOG)
        m_logger->debug("overlapRatio = {}", overlapRatio);
        m_logger->debug("areaB = {}, areaA = {}, areaB < areaA", areaB, areaA);
#endif
        // if ((overlapRatio > 0.8) && (areaB < areaA))
        if (overlapRatio > 0.35) //TODO:
        {
          if (areaB < areaA)
          {
            m_prevObjList[j].init(m_prevObjList[j].m_frameStamp);
          }
        }
      }
    }
  }
}


void ObjectTracker::_disableOverlapObject(vector<Object> &currObjectList)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

  vector<pair<int, vector<int>>> overlapDict;
  int numCurrObject = _getNumEnableObject(currObjectList);

  for (int i=0; i<numCurrObject; i++)
  {
    Object *ptrCurrObjA = &currObjectList[i];
    BoundingBox boxA = ptrCurrObjA->bbox;
    int statusA = ptrCurrObjA->getStatus();

#if defined (SPDLOG)
    if (m_debugMode) m_logger->debug("objA.status = {}", statusA);
#endif
    // Skip disabled object
    if (statusA == 0)
    {
      continue;
    }

    for (int j=0; j<numCurrObject; j++)
    {
      if (i!=j)
      {
        Object *ptrCurrObjB = &currObjectList[j];
        BoundingBox boxB = ptrCurrObjB->bbox;
        int statusB = ptrCurrObjB->getStatus();
#if defined (SPDLOG)
        if (m_debugMode) m_logger->debug("objB.status = {}", statusB);
#endif
        // Skip disabled object
        if (statusB == 0)
        {
          continue;
        }

        float overlapRatio = imgUtil::getBboxOverlapRatio(boxB, boxA);

#if defined (SPDLOG)
        if (m_debugMode)
        {
          m_logger->debug("Obj[{}], Obj[{}], overlapRatio = {}", i, j, overlapRatio);
        }
#endif
        // Save overlapped object's id
        // Ex: ObjA is the closest to doorbell
        // overlapDict => {ObjA: [ObjB, ObjC, ObjD ...]}
        if (overlapRatio > m_tOverlapRatio)
        {
          if (!utils::isKeyInDict(i, overlapDict))
          {
            vector<int> _tmpIdxList;
            _tmpIdxList.push_back(j);
            overlapDict.push_back(std::make_pair(i, _tmpIdxList));
          }
          else
          {
            for (auto it=overlapDict.begin(); it!=overlapDict.end(); it++)
            {
              if (i==(*it).first)
                (*it).second.push_back(j);
            }
          }
        }
      }
    }
  }

  vector<int> beOverlapedIdxList;
#if defined (SPDLOG)
  if (m_debugMode) m_logger->debug("Overlap dict = ");
#endif

  for (auto it=overlapDict.begin(); it!=overlapDict.end(); it++)
  {
#if defined (SPDLOG)
    if (m_debugMode) m_logger->debug("query ID: {}:", (*it).first);
#endif
    // Save be overlapped object's id into list
    for (auto itt=(*it).second.begin(); itt!=(*it).second.end(); itt++)
    {
      if (!utils::isKeyInDict((*itt), beOverlapedIdxList))
      {
        beOverlapedIdxList.push_back((*itt));
#if defined (SPDLOG)
        if (m_debugMode) m_logger->debug("{}, ", (*itt));
#endif
      }
#if defined (SPDLOG)
      if (m_debugMode) m_logger->debug("");
#endif
    }
  }

  // Enable / Disable object that are "not overlapped" / "overlapped"
  for (int i=0; i<numCurrObject; i++)
  {
    if (!utils::isKeyInDict(i, beOverlapedIdxList))
    {
#if defined (SPDLOG)
      if (m_debugMode) m_logger->debug("[Overlap] Obj[{}] => Enable", i);
#endif
      // currObjectList[i].updateStatus(1);   // Enable Object
      currObjectList[i].status = 1;   // Enable Object
    }
    else
    {
#if defined (SPDLOG)
      if (m_debugMode) m_logger->debug("[Overlap] Obj[{}] => Disable", i);
#endif
      // currObjectList[i].updateStatus(0);   // Disable Object
      currObjectList[i].status = 0;   // Disable Object
    }
  }

#if defined (SPDLOG)
  if (m_estimateTime)
  {
    time_1 = std::chrono::high_resolution_clock::now();
    m_logger->info("[_disableOverlapObject]: \t{} ms",
      std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
  }
#endif
}


bool ObjectTracker::_isValidHumanBBox(BoundingBox &box)
{

  if (box.label != DETECT_HUMAN)
    return false;

  bool isValid = true;

  //
  int tHeightLimit = static_cast<int>(m_videoHeight*0.8);
  int tWidthLimit = static_cast<int>(m_videoWidth*0.8);
  int tYROI = static_cast<int>(m_videoHeight*0.95);

  //
  int h = box.getHeight();
  int w = box.getWidth();
  float aspectRatio = box.getAspectRatio();
  Point pCenter = box.getCenterPoint();

  // Ignore big bounding box
  if ((h > tHeightLimit) || (w > tWidthLimit))
    isValid = false;

  // // Adjust Bounding Box if its aspect ratio is not good
  // m_logger->debug("Bounding box aspect ratio = {}", aspectRatio);
  // if (aspectRatio < 1.0)
  // {
  //   m_logger->debug("Ivalid human (aspect ratio < 1.0)");
  //   isValid = false;
  // }
  return isValid;
}


bool ObjectTracker::_isValidBikeBBox(BoundingBox &box)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  if (box.label != DETECT_BIKE)
    return false;

  bool isValid = true;

  //
  int tHeightLimit = static_cast<int>(m_videoHeight*0.8);
  int tWidthLimit = static_cast<int>(m_videoWidth*0.8);
  int tYROI = static_cast<int>(m_videoHeight*0.95);

  //
  int h = box.getHeight();
  int w = box.getWidth();
  float aspectRatio = box.getAspectRatio();
  Point pCenter = box.getCenterPoint();

  // Ignore big bounding box
  if ((h > tHeightLimit) || (w > tWidthLimit))
    isValid = false;

  // Adjust Bounding Box if its aspect ratio is not good
#if defined (SPDLOG)
  m_logger->debug("Bounding box aspect ratio = {}", aspectRatio);
#endif
  if (aspectRatio < 0.9)
  {
#if defined (SPDLOG)
    m_logger->debug("Ivalid rider (aspect ratio < 0.9)");
#endif
    isValid = false;
  }
  return isValid;
}


bool ObjectTracker::_isValidMotorbikeBBox(BoundingBox &box)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif
  if (box.label != DETECT_MOTORBIKE)
    return false;

  bool isValid = true;

  //
  int tHeightLimit = static_cast<int>(m_videoHeight*0.8);
  int tWidthLimit = static_cast<int>(m_videoWidth*0.8);
  int tYROI = static_cast<int>(m_videoHeight*0.95);

  //
  int h = box.getHeight();
  int w = box.getWidth();
  float aspectRatio = box.getAspectRatio();
  Point pCenter = box.getCenterPoint();

  // Ignore big bounding box
  if ((h > tHeightLimit) || (w > tWidthLimit))
    isValid = false;

  // Adjust Bounding Box if its aspect ratio is not good
#if defined (SPDLOG)
  m_logger->debug("Bounding box aspect ratio = {}", aspectRatio);
#endif
  if (aspectRatio < 0.9)
  {
#if defined (SPDLOG)
    m_logger->debug("Ivalid rider (aspect ratio < 0.9)");
#endif
    isValid = false;
  }
  return isValid;
}


bool ObjectTracker::_isValidVehicleBBox(BoundingBox &box)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif
  if (box.label != DETECT_VEHICLE)
    return false;

  bool isValid = true;

  //
  int tHeightLimit = static_cast<int>(m_videoHeight*0.8);
  int tWidthLimit = static_cast<int>(m_videoWidth*0.8);
  int tYROI = static_cast<int>(m_videoHeight*0.95);

  //
  int h = box.getHeight();
  int w = box.getWidth();
  float aspectRatio = box.getAspectRatio();
  Point pCenter = box.getCenterPoint();

  // Ignore big bounding box
  if ((h > tHeightLimit) || (w > tWidthLimit))
    isValid = false;

  // Adjust Bounding Box if its aspect ratio is not good
#if defined (SPDLOG)
  m_logger->debug("Bounding box aspect ratio = {}", aspectRatio);
#endif
  if (aspectRatio < 0.35)
  {
#if defined (SPDLOG)
    m_logger->debug("Ivalid car (aspect ratio < 0.35)");
#endif
    isValid = false;
  }
  return isValid;
}


void ObjectTracker::_bboxPreprocessing(
  vector<BoundingBox> &bboxList, vector<BoundingBox> &procBboxList, int windowSize)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  int numBbox = bboxList.size();
  int halfWindowSize = (int)(windowSize*0.5);

  for (int i=halfWindowSize; i<numBbox-halfWindowSize; i+=1)
  {
    int newX1 = 0;
    int newY1 = 0;
    int newX2 = 0;
    int newY2 = 0;

    vector<BoundingBox> tmpBboxList;
    vector<int> frameStampList;
    for (int j=i-halfWindowSize; j<=i+halfWindowSize; j++)
    {
      tmpBboxList.push_back(bboxList[j]);
      frameStampList.push_back(bboxList[j].frameStamp);
    }

    vector<int> frameIntervalList;
    frameIntervalList.push_back(0);   // 1st frame interval = 0
    for (int i=1; i<windowSize; i++)
    {
      int interval = frameStampList[i] - frameStampList[i-1];
      frameIntervalList.push_back(interval);
    }

    vector<vector<Point>> cornerPointList;
    for (int i=0; i<(int)tmpBboxList.size(); i++)
    {
      cornerPointList.push_back(tmpBboxList[i].getCornerPoint());
    }

    // 0:TL, 1:TR, 2:BL, 3:BR
    vector<int> x1List;
    vector<int> y1List;
    vector<int> x2List;
    vector<int> y2List;
    for (int i=0; i<(int)cornerPointList.size(); i++)
    {
      int frameInterval = frameIntervalList[i];
      if (frameInterval > m_maxFrameInterval)
      {
        break;  // ignore remain bounding boxes ...
      }
      x1List.push_back(cornerPointList[i][0].x);
      y1List.push_back(cornerPointList[i][0].y);
      x2List.push_back(cornerPointList[i][3].x);
      y2List.push_back(cornerPointList[i][3].y);
    }

    vector<int>::iterator x1It_min;
    vector<int>::iterator x1It_max;

    vector<int>::iterator x2It_min;
    vector<int>::iterator x2It_max;

    vector<int>::iterator y1It;
    vector<int>::iterator y2It;

    x1It_min = std::min_element(x1List.begin(), x1List.end());
    x1It_max = std::max_element(x1List.begin(), x1List.end());

    x2It_min = std::min_element(x2List.begin(), x2List.end());
    x2It_max = std::max_element(x2List.begin(), x2List.end());

    y1It = std::min_element(y1List.begin(), y1List.end());
    y2It = std::min_element(y2List.begin(), y2List.end());

    newX1 = int(((*x1It_min) + (*x1It_max)) * 0.5);
    newX2 = int(((*x2It_min) + (*x2It_max)) * 0.5);
    // newX2 = (*x2It_min);

    newY1 = (*y1It);
    // newX2 = (*x2It);
    // newY2 = (*y2It);
    newY2 = utils::findMedian(y2List, y2List.size());

    BoundingBox bbox = BoundingBox(newX1, newY1, newX2, newY2, bboxList[0].label);
    procBboxList.push_back(bbox);
  }
}


// ============================================
//                  Inputs
// ============================================
void ObjectTracker::_setCurrFrame(cv::Mat &imgFrame)
{
  m_img = imgFrame;
}


void ObjectTracker::setROI(BoundingBox &roi)
{
  m_roi->x1 = roi.x1;
  m_roi->x2 = roi.x2;
  m_roi->y1 = roi.y1;
  m_roi->y2 = roi.y2;
}


void ObjectTracker::setDataBuffer(std::vector<DataFrame>* dataBufferPtr)
{
  m_dataBuffer = dataBufferPtr;
}


void ObjectTracker::_setCurrBoundingBox(vector<BoundingBox> &bboxList)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get(m_loggerStr);
#endif

  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

  // Make sure bboxList size <= MAX_OBJECT
  if ((int)bboxList.size() > m_maxObject)
  {
    // Sorting by bounding box's area (from big to small)
    std::sort(bboxList.begin(), bboxList.end(), utils::sortByBBoxArea);

    for (int i=0; i<(int)bboxList.size(); i++)
    {
      if (i >= m_maxObject)
      {
        bboxList.pop_back();  // pop up small bounding boxes ...
      }
    }
  }

  // Save current bounding boxes
  m_bboxList = bboxList;

#if defined (SPDLOG)
  if (m_estimateTime)
  {
    time_1 = std::chrono::high_resolution_clock::now();
    m_logger->info("[_setCurrBoundingBox]: \t{} ms",
      std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
  }
#endif
}

int ObjectTracker::_updateFrameStamp()
{
  // Refresh frame stamp
  m_frameStamp += 1;
  if (m_frameStamp >= 65535)
  {
    m_frameStamp = 0;
  }
  return 1;
}


// ============================================
//                  Outputs
// ============================================
int ObjectTracker::getFrameStamp()
{
  return m_frameStamp;
}


void ObjectTracker::getObjectList(vector<Object> &objList)
{
  objList = m_prevObjList;
}


int ObjectTracker::getSmoothBBoxList(vector<BoundingBox> &smoothedBBoxList)
{
  int maxWindowSize = 5;
  vector<BoundingBox> tmpBBoxList;

  smoothedBBoxList.clear();

  for(int i=0; i<m_prevObjList.size(); i++)
  {
    vector<BoundingBox> _smoothedBBoxList;

    if (m_prevObjList[i].disappearCounter > 5)
    {
      if (m_prevObjList[i].smoothedBBoxList.size() > 0)
      {
        m_prevObjList[i].smoothedBBoxList.erase(m_prevObjList[i].smoothedBBoxList.begin());
      }
    }

    // if (m_prevObjList[i].aliveCounter > 3 && m_prevObjList[i].bboxList.size() > 5)
    if (m_prevObjList[i].status == 1 && m_prevObjList[i].bboxList.size() > maxWindowSize)
    {
      // BoundingBox box(-1, -1., -1, -1, -1);
      // m_prevObjList[i].getPrevPredBoundingBox(box);

      // if (box.label == -1)
      //   continue;
      // STEP1: bounding box is out of lane

      _bboxPreprocessing(m_prevObjList[i].bboxList, _smoothedBBoxList, maxWindowSize);

      int x1 = (int)((_smoothedBBoxList[_smoothedBBoxList.size()-1].x1 + m_prevObjList[i].bbox.x1)*0.5);
      int y1 = (int)((_smoothedBBoxList[_smoothedBBoxList.size()-1].y1 + m_prevObjList[i].bbox.y1)*0.5);
      int x2 = (int)((_smoothedBBoxList[_smoothedBBoxList.size()-1].x2 + m_prevObjList[i].bbox.x2)*0.5);
      int y2 = (int)((_smoothedBBoxList[_smoothedBBoxList.size()-1].y2 + m_prevObjList[i].bbox.y2)*0.5);

      BoundingBox finalBox(x1, y1, x2, y2, m_prevObjList[i].bbox.label);
      // smoothedBBoxList.push_back(finalBox);
      tmpBBoxList.push_back(finalBox);

      //
      if (m_prevObjList[i].smoothedBBoxList.size() < MAX_BBOX_LIST_SIZE)
      {
        m_prevObjList[i].smoothedBBoxList.push_back(finalBox);
      }
      else
      {
        m_prevObjList[i].smoothedBBoxList.erase(m_prevObjList[i].smoothedBBoxList.begin());
        m_prevObjList[i].smoothedBBoxList.push_back(finalBox);
      }

    }
  }

  for(int i=0; i<tmpBBoxList.size(); i++)
  {
    BoundingBox bbox = tmpBBoxList[i];

    bool isOverlapped = false;
    for (int j=0; j<tmpBBoxList.size(); j++)
    {
      if (j!=i)
      {
        BoundingBox bbox2 = tmpBBoxList[j];
        float overlapRatio = imgUtil::getBboxOverlapRatio(bbox, bbox2);
        if (overlapRatio > 0.5) //TODO:
          isOverlapped = true;
      }
    }

    if (!isOverlapped)
      smoothedBBoxList.push_back(bbox);
  }
}


// void ObjectTracker::calcLocation3D(Object& obj, cv::Point3f& pLoc)
// {
//   // Calculate Z (Distance)
//   _calcDistance(obj);

//   // Calculate X (Horizontal)
//   _calcX2Meter(obj);

//   // Calcualte Y (Vertical)
//   _calcY2Meter(obj);
// }


// ============================================
//                  Others
// ============================================
void ObjectTracker::debugON()
{
  m_debugMode = true;
}


void ObjectTracker::showProcTime()
{
  m_estimateTime = true;
}