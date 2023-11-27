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

#include "vision_tracker.hpp"


VisionTracker::VisionTracker(std::string configPath)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::stdout_color_mt("VisionTracker");

  m_logger->set_pattern("[%n] [%^%l%$] %v");

  m_logger->info("=================================================");
  m_logger->info("=                WNC VisionTracker              =");
  m_logger->info("=================================================");
  m_logger->info("Version: v{}", VERSION);
  m_logger->info("-------------------------------------------------");
#endif

  // === initialize parameters === //
  printf("[VisionTracker(std::string configPath)] Start _init(configPath)\n");
  _init(configPath);
  printf("[VisionTracker(std::string configPath)] End _init(configPath)\n");
#if defined (SPDLOG)
  // Create a file rotating logger with 5 MB size max and 3 rotated files
  auto max_size = 1048576 * 5;
  auto max_files = 3;
  string logName = "log.txt";
  string logPath = m_dbg_logsDirPath + "/" + logName;
  auto m_loggerOutput = spdlog::rotating_logger_mt("VisionTracker_DEBUG", logPath, max_size, max_files);
  m_loggerOutput->flush_on(spdlog::level::info);
  m_loggerOutput->set_pattern("%v");

  if (m_dbg_tracking)
    m_logger->set_level(spdlog::level::debug);
  else
    m_logger->set_level(spdlog::level::info);
#endif
};


VisionTracker::~VisionTracker()
{
  delete m_configReader;
  delete m_config;
  delete m_yolov8;
  delete m_humanTracker;
  delete m_bikeTracker;
  delete m_vehicleTracker;
  delete m_motorbikeTracker;
  delete m_roi;

  m_configReader = nullptr;
  m_config = nullptr;
  m_yolov8 = nullptr;
  m_humanTracker = nullptr;
  m_bikeTracker = nullptr;
  m_vehicleTracker = nullptr;
  m_motorbikeTracker = nullptr;
  m_roi = nullptr;
};


bool VisionTracker::_init(std::string configPath)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get("VisionTracker");
#endif

  // Get Date Time
  utils::getDateTime(m_dbg_dateTime);

  // Read VisionTracker Configuration
  
  m_config = new Config_S();
 
  m_configReader = new TrackerConfigReader();
  
  m_configReader->read(configPath);
 
#if defined (SPDLOG)
  m_logger->info("Read configuration file ... Done");
#endif
  
  m_config = m_configReader->getConfig();
 
#if defined (SPDLOG)
  m_logger->info("Set configuration ... Done");
#endif
 
  // Create AI model
  if (utils::checkFileExists(m_config->modelPath))
  { 
    m_yolov8 = new YOLOv8(m_config);
#if defined (SPDLOG)
    m_logger->info("Init AI model ... Done");
#endif
  }
  else
  {
#if defined (SPDLOG)
    m_logger->error("Can not find AI model {}", m_config->modelPath);
    m_logger->error("Stop VisionTracker ...");
#endif
    exit(0);
  }
 
  // ROI
  _initROI();
#if defined (SPDLOG)
  m_logger->info("Init ROI ... Done");
#endif
 
  // Video Input Size
  m_videoWidth = m_config->frameWidth;
  m_videoHeight = m_config->frameHeight;

  // Model Input Size
  m_modelWidth = m_config->modelWidth;
  m_modelHeight = m_config->modelHeight;

  // Video Frame
  m_img = cv::Mat(cv::Size(m_videoWidth, m_videoHeight), CV_8UC3, cv::Scalar::all(0));

  // Processing
  m_frameStep = m_config->procFrameStep;

  // Distance Estimation
  m_focalRescaleRatio = (float)m_videoHeight / (float)m_modelHeight;

  // Object Tracker
  m_humanTracker = new ObjectTracker(m_config, "human");
  m_bikeTracker = new ObjectTracker(m_config, "bike");
  m_vehicleTracker = new ObjectTracker(m_config, "vehicle");
  m_motorbikeTracker = new ObjectTracker(m_config, "motorbike");

 
  // TODO:
  m_vehicleTracker->setROI(*m_roi);
  m_bikeTracker->setROI(*m_roi);
  m_humanTracker->setROI(*m_roi);
  m_motorbikeTracker->setROI(*m_roi);

#if defined (SPDLOG)
  m_logger->info("Init Object Trackers ... Done");
#endif
 
  // _readDebugConfig();          // Debug Configuration
  // _readDisplayConfig();        // Display Configuration
  // _readShowProcTimeConfig();   // Show Processing Time Configuration
  //  printf("[VisionTracker::_init(std::string configPath)] End Debug /Display/Show Processing Time Configuration,\n");
#if defined (SPDLOG)
  m_logger->info("Init VisionTracker ... Done");
#endif

  return SUCCESS;
}


bool VisionTracker::_readDebugConfig()
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get("VisionTracker");
#endif

  if (m_config->stDebugConfig.tracking)
  {
    m_dbg_tracking = true;
  }
  if (m_config->stDebugConfig.yolov8)
  {
    m_dbg_yolov8 = true;
    // m_yolov8->debugON();
  }
  if (m_config->stDebugConfig.objectDetection)
  {
    m_dbg_objectDetection = true;
  }
  if (m_config->stDebugConfig.objectTracking)
  {
    m_dbg_objectTracking = true;
    if (m_config->stDebugConfig.humanTracker) m_humanTracker->debugON();
    if (m_config->stDebugConfig.bikeTracker) m_bikeTracker->debugON();
    if (m_config->stDebugConfig.vehicleTracker) m_vehicleTracker->debugON();
    if (m_config->stDebugConfig.motorbikeTracker) m_motorbikeTracker->debugON();
  }
  if (m_config->stDebugConfig.saveLogs)
  {
    m_dbg_saveLogs = true;
  }
  if (m_config->stDebugConfig.saveImages)
  {
    m_dbg_saveImages = true;
  }
  if (m_config->stDebugConfig.saveRawImages)
  {
    m_dbg_saveRawImages = true;
  }
  if (m_config->stDebugConfig.logsDirPath != "")
  {
    m_dbg_logsDirPath = m_config->stDebugConfig.logsDirPath + "/" + m_dbg_dateTime;

    if (m_config->stDebugConfig.saveLogs && utils::createDirectories(m_dbg_logsDirPath))
    {
#if defined (SPDLOG)
      m_logger->info("Folders created successfully: {}", m_dbg_logsDirPath);
#endif
    }
    else
    {
#if defined (SPDLOG)
      m_logger->info("Error creating folders: {}", m_dbg_logsDirPath);
#endif
    }
  }
  if (m_config->stDebugConfig.imgsDirPath != "")
  {
    m_dbg_imgsDirPath = m_config->stDebugConfig.imgsDirPath + "/" + m_dbg_dateTime;

    if (m_config->stDebugConfig.saveImages && utils::createDirectories(m_dbg_imgsDirPath))
    {
#if defined (SPDLOG)
      m_logger->info("Folders created successfully: {}", m_dbg_imgsDirPath);
#endif
    }
    else
    {
#if defined (SPDLOG)
      m_logger->info("Error creating folders: {}", m_dbg_imgsDirPath);
#endif
    }
  }
  if (m_config->stDebugConfig.rawImgsDirPath != "")
  {
    m_dbg_rawImgsDirPath = m_config->stDebugConfig.rawImgsDirPath + "/" + m_dbg_dateTime;

    if (m_config->stDebugConfig.saveRawImages && utils::createDirectories(m_dbg_rawImgsDirPath))
    {
#if defined (SPDLOG)
      m_logger->info("Folders created successfully: {}", m_dbg_rawImgsDirPath);
#endif
    }
    else
    {
#if defined (SPDLOG)
      m_logger->info("Error creating folders: {}", m_dbg_rawImgsDirPath);
#endif
    }
  }

  return SUCCESS;
}


bool VisionTracker::_readDisplayConfig()
{
  if (m_config->stDisplayConfig.results)
  {
    m_dsp_results = true;
  }
  if (m_config->stDisplayConfig.objectDetection)
  {
    m_dsp_objectDetection = true;
  }
  if (m_config->stDisplayConfig.objectTracking)
  {
    m_dsp_objectTracking = true;
  }
  if (m_config->stDisplayConfig.information)
  {
    m_dsp_information = true;
  }
  if (m_config->stDisplayConfig.warningZone)
  {
    m_dsp_warningZone = true;
  }
  if (m_config->stDisplayConfig.maxFrameIndex)
  {
    m_dsp_maxFrameIdx = m_config->stDisplayConfig.maxFrameIndex;
  }

  return SUCCESS;
}


bool VisionTracker::_readShowProcTimeConfig()
{
  if (m_config->stShowProcTimeConfig.tracking)
  {
    m_estimateTime = true;
  }
  // if (m_config->stShowProcTimeConfig.yolov8)
  // {
  //   m_yolov8->showProcTime();
  // }
  if (m_config->stShowProcTimeConfig.objectTracking)
  {
    m_humanTracker->showProcTime();
    m_bikeTracker->showProcTime();
    m_vehicleTracker->showProcTime();
    m_motorbikeTracker->showProcTime();
  }
}


bool VisionTracker::_initROI()
{
  //
  int xCenter = static_cast<int>(m_config->frameWidth * 0.5);
  int yCenter = static_cast<int>(m_config->frameHeight * 0.5);

  //
  int newWidth = static_cast<int>(m_config->frameWidth * 1);
  int newHeight = static_cast<int>(m_config->frameHeight * 1);

  //
  int x1 = xCenter - static_cast<int>(newWidth * 0.5);
  int x2 = xCenter + static_cast<int>(newWidth * 0.5);

  //
  int y1 = yCenter - static_cast<int>(m_config->frameHeight * 0.1);
  int y2 = yCenter + static_cast<int>(m_config->frameHeight * 0.2);

  //
  int xOffset = x1;
  int yOffset = y1;

  m_roi = new BoundingBox(x1, y1, x2, y2, -1);

  return SUCCESS;
}


// =================================================================
//                   Main  Alsiter 2023-11-27
// =================================================================
bool VisionTracker::run(cv::Mat &imgFrame,std::vector<BoundingBox> bboxList)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get("VisionTracker");
#endif
  auto time_0 = std::chrono::high_resolution_clock::now();
  auto time_1 = std::chrono::high_resolution_clock::now();

  bool ret = SUCCESS;

  if (m_img.empty())
  {
#if defined (SPDLOG)
    m_logger->warn("Input image is empty");
#endif
   
    return false;
  }

  // Get Image Frame
  if (m_dsp_results) m_dsp_img = imgFrame.clone();

  // Entry Point
  if (m_frameIdx % m_frameStep == 0)
  {
#if defined (SPDLOG)
    m_logger->info("");
    m_logger->info("========================================");
    m_logger->info("Frame Index: {}", m_frameIdx);
    m_logger->info("========================================");
#endif
    // Get Image Frame
    m_img = imgFrame.clone();
   
    // AI Inference
//     if (!_modelInfernece(bboxList))
//     {
// #if defined (SPDLOG)
//       m_logger->error("AI model inference failed ... STOP VisionTracker");
// #endif
//       ret = FAILURE;
//       exit(1);
//     }

    // Get Detected Bounding Boxes
    // Alister 2023-11-22
    if (!_objectDetection(bboxList))
    {
#if defined (SPDLOG)
      m_logger->warn("Detect objects failed ...");
#endif
      ret = FAILURE;
    }
    // Object Tracking
    if (!_objectTracking())
    {
#if defined (SPDLOG)
      m_logger->warn("Track objects failed ...");
#endif
      ret = FAILURE;
    }
    // Show Results
    _showDetectionResults();
#if defined (SPDLOG)
    // Save Results to Debug Logs
    if (m_dbg_saveLogs)
    {
      _saveDetectionResults();
    }
#endif
    //
    m_preparingNextDetection = false;

    if (m_estimateTime)
    {
      time_1 = std::chrono::high_resolution_clock::now();
#if defined (SPDLOG)
      m_logger->info("");
      m_logger->info("Processing Time: \t{} ms", std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
#endif
    }
  }
  else
  {
    m_preparingNextDetection = true;
  }
  // Draw and Save Results
  if (m_dsp_results && ret == SUCCESS)
  {
    _drawResults();
  }
  if (m_dsp_results && m_dbg_saveImages && ret == SUCCESS)
  {
    _saveDrawResults();
  }
  // Save Raw Images
  if (m_dbg_saveRawImages)
  {
    _saveRawImages();
  }
  // Update frame index
  _updateFrameIndex();
  return ret;
}


// ============================================
//              Work Flow Functions
// ============================================

bool VisionTracker::_modelInfernece(std::vector<BoundingBox> bboxList)
{
  if (!m_yolov8->run(m_img,bboxList))
  {
    return FAILURE;
  }

  return SUCCESS;
}

// Alister 2023-11-22
bool VisionTracker::_objectDetection(std::vector<BoundingBox> bboxList)
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get("VisionTracker");
#endif
  std:vector<v8xyxy> m_yoloOut;
 
  for(int i=0;i<bboxList.size();i++)
  {
    v8xyxy box;
    box.x1 = bboxList[i].x1;
    box.y1 = bboxList[i].y1;
    box.x2 = bboxList[i].x2;
    box.y2 = bboxList[i].y2;
    box.c = bboxList[i].label;
    box.c_prob = 0.99;
    m_yoloOut.push_back(box);
  }
  
  m_yolov8->getHumanBoundingBox(
    m_humanBBoxList,
    m_config->stOdConfig.humanConfidence,
    m_videoWidth,
    m_videoHeight,
    m_yoloOut
  );
  
  m_yolov8->getBikeBoundingBox(
    m_bikeBBoxList,
    m_config->stOdConfig.bikeConfidence,
    m_videoWidth,
    m_videoHeight,
    m_yoloOut
  );
 
  m_yolov8->getVehicleBoundingBox(
    m_vehicleBBoxList,
    m_config->stOdConfig.carConfidence,
    m_videoWidth,
    m_videoHeight,
    m_yoloOut
  );
  
  m_yolov8->getMotorbikeBoundingBox(
    m_motorbikeBBoxList,
    m_config->stOdConfig.motorbikeConfidence,
    m_videoWidth,
    m_videoHeight,
    m_yoloOut
  );
 
#if defined (SPDLOG)
  // Debug Logs
  m_logger->debug("Num of Human Bounding Box: {}",\
    (int)m_humanBBoxList.size());

  m_logger->debug("Num of Bike Bounding Box: {}",\
    (int)m_bikeBBoxList.size());

  m_logger->debug("Num of Vehicle Bounding Box: {}",\
    (int)m_vehicleBBoxList.size());

  m_logger->debug("Num of Motorbike Bounding Box: {}",\
    (int)m_motorbikeBBoxList.size());
#endif

  return SUCCESS;
}



bool VisionTracker::_objectTracking()
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get("VisionTracker");
#endif
  // Run Object Tracking
  m_humanTracker->run(m_img, m_humanBBoxList); //TODO:
  m_bikeTracker->run(m_img, m_bikeBBoxList); //TODO:
  m_vehicleTracker->run(m_img, m_vehicleBBoxList); //TODO:
  m_motorbikeTracker->run(m_img, m_motorbikeBBoxList); //TODO:
 
  // Get Tracked Objects
  m_humanTracker->getObjectList(m_humanObjList);
  m_bikeTracker->getObjectList(m_bikeObjList);
  m_vehicleTracker->getObjectList(m_vehicleObjList);
  m_motorbikeTracker->getObjectList(m_motorbikeObjList);
 
  // Get Location of Tracked Objects
  m_humanTracker->getTrackedObjList(m_humanTrackObjList);
  m_bikeTracker->getTrackedObjList(m_bikeTrackObjList);
  m_vehicleTracker->getTrackedObjList(m_vehicleTrackObjList);
  m_motorbikeTracker->getTrackedObjList(m_motorbikeTrackObjList);
  
  // Merge Tracked Objects
  m_trackedObjList.clear();
  m_trackedObjList.insert(m_trackedObjList.end(), m_humanObjList.begin(), m_humanObjList.end());
  m_trackedObjList.insert(m_trackedObjList.end(), m_bikeObjList.begin(), m_bikeObjList.end());
  m_trackedObjList.insert(m_trackedObjList.end(), m_vehicleObjList.begin(), m_vehicleObjList.end());
  m_trackedObjList.insert(m_trackedObjList.end(), m_motorbikeObjList.begin(), m_motorbikeObjList.end());
   
  // Bounding Box Smoothing
  for (int i=0; i<m_trackedObjList.size(); i++)
  {
    m_trackedObjList[i].updateSmoothBoundingBoxList();
  }
  printf("[bool VisionTracker::_objectTracking()] End updateSmoothBoundingBoxList\n");
#if defined (SPDLOG)
  // Debug Logs
  m_logger->debug("Track: {} Human, {} Bike, {} Vehicle, {} Motorbike", \
    (int)m_humanObjList.size(), (int)m_bikeObjList.size(), (int)m_vehicleObjList.size(), (int)m_motorbikeObjList.size());
#endif

  return SUCCESS;
}



// ============================================
//                  Outputs
// ============================================

void VisionTracker::getResults(VisionTrackingResults &res)
{
  // Save Tracked Objects
  m_result.humanObjList = m_humanTrackObjList;
  m_result.bikeObjList = m_vehicleTrackObjList;
  m_result.vehicleObjList = m_bikeTrackObjList;
  m_result.motorbikeOjList = m_motorbikeTrackObjList;

  // Pass reference to res
  res = m_result;
}


void VisionTracker::getResultImage(cv::Mat &imgResult)
{
  if (m_dsp_results)
  {
    imgResult = m_dsp_imgResize;
  }
}



// ============================================
//                  Results
// ============================================

void VisionTracker::_showDetectionResults()
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get("VisionTracker");
#endif

  // Show Tracked Object Information
  for (int i=0; i<m_trackedObjList.size(); i++)
  {
    const Object& obj = m_trackedObjList[i];

    if (obj.status == 1)
    {
      string classType = "";
      if (obj.bbox.label == HUMAN)
        classType = "Pedestrian";
      else if (obj.bbox.label == BIKE)
        classType = "Bike";
      else if (obj.bbox.label == VEHICLE)
        classType = "Vehicle";
      else if (obj.bbox.label == MOTORBIKE)
        classType = "Motorbike";
#if defined (SPDLOG)
      // m_logger->info("Tracking Obj[{}]: Cls = {}, Conf = {:.2f}", \
      //   obj.id, classType, obj.bbox.confidence);
#endif
    }
  }
}

#if defined (SPDLOG)
void VisionTracker::_saveDetectionResult(std::vector<std::string>& logs)
{
  auto m_logger = spdlog::get("VisionTracker_DEBUG");

  for(int i=0; i<logs.size(); i++)
  {
    m_logger->info(logs[i]);
  }
}


void VisionTracker::_saveDetectionResults()
{
  auto m_logger = spdlog::get("VisionTracker_DEBUG");

  m_logger->info("");
  m_logger->info("=================================");
  m_logger->info("Frame Index:{}", m_frameIdx);
  m_logger->info("=================================");

  // --- Object Detection --- //
  std::vector<BoundingBox> bboxList;

  bboxList.insert(bboxList.end(), m_humanBBoxList.begin(), m_humanBBoxList.end());
  bboxList.insert(bboxList.end(), m_bikeBBoxList.begin(), m_bikeBBoxList.end());
  bboxList.insert(bboxList.end(), m_vehicleBBoxList.begin(), m_vehicleBBoxList.end());
  bboxList.insert(bboxList.end(), m_motorbikeBBoxList.begin(), m_motorbikeBBoxList.end());

  m_loggerManager.m_objectDetectionLogger->logObjects(bboxList);
  m_logger->info("");
  m_logger->info("Object Detection");
  m_logger->info("---------------------------------");
  _saveDetectionResult(m_loggerManager.m_objectDetectionLogger->m_logs);

  // --- Object Tracking --- //
  m_loggerManager.m_objectTrackingLogger->logObjects(m_trackedObjList);

  m_logger->info("");
  m_logger->info("Object Tracking");
  m_logger->info("---------------------------------");
  _saveDetectionResult(m_loggerManager.m_objectTrackingLogger->m_logs);
}
#endif

void VisionTracker::_saveDrawResults()
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get("VisionTracker");
#endif
  string imgName = "frame_" + std::to_string(m_frameIdx) + ".jpg";
  // string imgPath = m_dbg_imgsDirPath + "/" + imgName;
  string imgPath = "./AMBA_result/" + imgName;
  printf("imgPath = %c\n",imgPath);
  cv::imwrite(imgPath, m_dsp_imgResize);
#if defined (SPDLOG)
  m_logger->info("Save img to {}", imgPath);
#endif
}


void VisionTracker::_saveRawImages()
{
#if defined (SPDLOG)
  auto m_logger = spdlog::get("VisionTracker");
#endif
  string imgName = "frame_" + std::to_string(m_frameIdx) + ".jpg";
  string imgPath = m_dbg_rawImgsDirPath + "/" + imgName;

  cv::imwrite(imgPath, m_img);
#if defined (SPDLOG)
  m_logger->info("Save raw img to {}", imgPath);
#endif
}


// ============================================
//               Draw Results
// ============================================

void VisionTracker::_drawBoundingBoxes()
{
  // Define the bounding box lists and their corresponding colors
  std::vector<BoundingBox> boundingBoxLists[] =
  {
    m_humanBBoxList,
    m_bikeBBoxList,
    m_vehicleBBoxList,
    m_motorbikeBBoxList
  };
  cout<<"------------------------m_vehicleBBoxList.size() = "<<m_vehicleBBoxList.size()<<endl;
  cv::Scalar colors[] =
  {
    cv::Scalar(0, 255, 0),     // Green for vehicles
    cv::Scalar(255, 0, 255),   // Magenta for bikes
    cv::Scalar(0, 128, 255),   // Orange for humans
    cv::Scalar(255, 255, 0)    // Yellow for motorbikes
  };
  
  for (int j = 0; j < sizeof(boundingBoxLists) / sizeof(boundingBoxLists[0]); j++)
  {
    std::vector<BoundingBox>& boundingBoxList = boundingBoxLists[j];
    cv::Scalar color = colors[j];

    for (int i = 0; i < boundingBoxList.size(); i++)
    {
      BoundingBox lastBox = boundingBoxList[i];
  
      BoundingBox rescaleBox(-1, -1, -1, -1, -1);

       utils::rescaleBBox(
        lastBox, rescaleBox,
        m_config->modelWidth, m_config->modelHeight,
        3840, 2160);

      imgUtil::roundedRectangle(
        m_dsp_img, cv::Point(rescaleBox.x1, rescaleBox.y1),
        cv::Point(rescaleBox.x2, rescaleBox.y2),
        color, 2, 0, 10, false);
    }
  }
}


void VisionTracker::_drawTrackedObjects()
{
  cout<< "    m_trackedObjList.size() = "<<m_trackedObjList.size()<<endl;
  for (int i = 0; i < m_trackedObjList.size(); i++)
  {
    const Object& trackedObj = m_trackedObjList[i];

    // Skip objects with specific conditions
    if (trackedObj.status == 0 || trackedObj.disappearCounter > 5 || trackedObj.bboxList.empty())
    {
      // cout<<"   Skip objects with specific conditions"<<endl;
      // cout<<"trackedObj.status == 0 || trackedObj.disappearCounter > 5 || trackedObj.bboxList.empty()"<<endl;
      // cout<<" trackedObj.status = "<<trackedObj.status<<endl;
      // cout<<"trackedObj.disappearCounter = "<<trackedObj.disappearCounter<<endl;
      // cout<<"trackedObj.bboxList = "<<trackedObj.bboxList.size()<<endl;
      continue;
    }

    // BoundingBox lastBox = trackedObj.bboxList.back();
    BoundingBox lastBox(-1, -1, -1, -1, -1);
    if (trackedObj.smoothedBBoxList.size() > 0)
    {
      lastBox = trackedObj.smoothedBBoxList.back();
    }
    else
    {
      lastBox = trackedObj.bboxList.back();
    }

    BoundingBox rescaleBox(-1, -1, -1, -1, -1);

      // Rescale BBoxes
    utils::rescaleBBox(
      lastBox, rescaleBox,
      m_config->modelWidth, m_config->modelHeight,
      m_config->frameWidth, m_config->frameHeight);

    if (trackedObj.aliveCounter < 3)
    {
      // Draw bounding box in green
      imgUtil::roundedRectangle(
        m_dsp_img, cv::Point(rescaleBox.x1, rescaleBox.y1),
        cv::Point(rescaleBox.x2, rescaleBox.y2),
        cv::Scalar(0, 250, 0), 6, 0, 10, false);
    }
    else
    {
      //TODO:
      cv::Scalar color;
      if (lastBox.label == 0) // Human
        color = cv::Scalar(0, 128, 255);
      else if (lastBox.label == 1) // Rider
        color = cv::Scalar(255, 0, 255);
      else if (lastBox.label == 2) // Vehicle
        color = cv::Scalar(0, 255, 120);
      else if (lastBox.label == 3) // Vehicle
        color = cv::Scalar(110, 255, 120);

      // Draw bounding box with the determined color
      imgUtil::roundedRectangle(
        m_dsp_img, cv::Point(rescaleBox.x1, rescaleBox.y1),
        cv::Point(rescaleBox.x2, rescaleBox.y2),
        color, 6, 0, 10, false);
    }

    // Draw label and ID for valid distance
    string clsLabel = "";
    if (trackedObj.bbox.label == HUMAN)
    {
      clsLabel = "H ";
    }
    else if (trackedObj.bbox.label == BIKE)
    {
      clsLabel = "B ";
    }
    else if (trackedObj.bbox.label == VEHICLE)
    {
      clsLabel = "V ";
    }
    else if (trackedObj.bbox.label == MOTORBIKE)
    {
      clsLabel = "M ";
    }


  cv::rectangle(
      m_dsp_img,
      cv::Point(rescaleBox.x1 + 10, rescaleBox.y1 - 75),
      cv::Point(rescaleBox.x1 + 160, rescaleBox.y1 - 3),
      (0, 0, 0),
      -1/*fill*/);


    Point pCenter = lastBox.getCenterPoint();

    cv::putText(m_dsp_img, clsLabel + std::to_string(trackedObj.id) + " " + std::to_string(pCenter.x) + ", " + std::to_string(pCenter.y),
      cv::Point(int(rescaleBox.x1) + 20, int(rescaleBox.y1) - 15),
      cv::FONT_HERSHEY_DUPLEX, 2.0,
      cv::Scalar(255, 255, 255), 2, 5, 0);
  }
}


void VisionTracker::_drawInformation()
{
  cv::putText(m_dsp_imgResize, "Frame: " + std::to_string(m_frameIdx), \
    cv::Point(10, 540), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, \
    cv::Scalar(0, 255, 0), 1, 2, 0);
}


void VisionTracker::_drawObjectLocation()
{
  cv::Mat locationMap = cv::Mat::zeros(400, 400, CV_8UC3);
  cv::circle(locationMap, cv::Point(200, 395), 3, cv::Scalar(0, 255, 0), -1);

  for (int i=0; i<m_humanTrackObjList.size(); i++)
  {
    TrackedObj& obj = m_humanTrackObjList[i];
    int id = obj.id;
    cv::Point3f& p = obj.pLoc;

    float xRatio = (p.x + 6.5) / 13.0; // TODO: assume half of x distance is 6.5 meters
    float zRatio = 1.0 - (p.z / 13.0); //TODO: assume z distance is 13 meters

    int x = (int)(xRatio * 400);
    int z = (int)(zRatio * 400);

    cv::Point pLoc(x, z);
    cv::Point pLabel(x+5, z);
    cv::circle(locationMap, pLoc, 2, cv::Scalar(255, 255, 255), -1);
    cv::putText(locationMap, std::to_string(id), \
      pLabel, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, \
      cv::Scalar(255, 255, 255), 1, 2, 0);
  }

  // cv::imshow("Location", locationMap);
}


void VisionTracker::_drawResults()
{
  int waitKey = 1;
  cout<<"====================m_dsp_objectDetection = "<<m_dsp_objectDetection<<endl;
  // if (m_dsp_objectDetection)
  // {
  //   _drawBoundingBoxes();
  // }
  
  if (m_dsp_objectTracking)
  {
    _drawTrackedObjects();
  }
 
  cv::resize(m_dsp_img, m_dsp_imgResize, cv::Size(1024, 640), cv::INTER_LINEAR);
 
  if (m_dsp_information)
  {
    _drawInformation();
  }
 
  // if (m_dsp_location)
  if (1)
  {
    _drawObjectLocation();
  }

  if (m_frameIdx < m_dsp_maxFrameIdx)
  {
    waitKey = 1;
  }
  else if (m_dsp_maxFrameIdx == 0)
  {
    waitKey = 1;
  }
  else
  {
    waitKey = 0;
  }

  // cv::imshow("Vision Tracker", m_dsp_imgResize);

  // cv::waitKey(waitKey);
}


// ============================================
//                    Utils
// ============================================
void VisionTracker::_updateFrameIndex()
{
  m_frameIdx = (m_frameIdx % 65535) + 1;
  printf("m_frameIdx = %d \n",m_frameIdx);
}
