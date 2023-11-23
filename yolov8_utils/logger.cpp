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

#include "logger.hpp"

// --- Base Logger --- //
BaseLogger::BaseLogger(const std::string& loggerName)
{
  m_loggerName = "[" + loggerName + "]";
}

BaseLogger::~BaseLogger() {}


// --- Object Detection Logger --- //
void ObjectDetectionLogger::logObject(const BoundingBox& box)
{
  std::string logStr = "";
  std::string classStr = "";
  std::string boxStr = "";
  std::string confStr = "";

  if (box.label == 0)
  {
    classStr = "Human";
  }
  else if (box.label == 1)
  {
    classStr = "SmallVehicle";
  }
  else if (box.label == 2)
  {
    classStr = "BigVehicle";
  }
  else if (box.label == 3)
  {
    classStr = "StopSign";
  }
  else if (box.label == 4)
  {
    classStr = "RoadSign";
  }

  boxStr = "(" \
    + std::to_string(box.x1) + ", " \
    + std::to_string(box.y1) + ", " \
    + std::to_string(box.x2) + ", " \
    + std::to_string(box.y2) + ")";

  confStr = std::to_string(box.confidence);

  logStr = "Cls: " + classStr + " Box: " + boxStr + " Conf: " + confStr;

  log(logStr);
}


void ObjectDetectionLogger::logObjects(const vector<BoundingBox>& boxList)
{
  m_logs.clear();

  for (int i=0; i<boxList.size(); i++)
  {
    logObject(boxList[i]);
  }
}


// --- Object Tracking Logger --- //
void ObjectTrackingLogger::logObject(const Object& obj)
{
  std::string logStr = "";
  std::string classStr = "";
  std::string boxStr = "";
  std::string confStr = "";
  std::string distStr = "";
  std::string ttcStr = "";
  std::string ttcCounterStr = "";

  const BoundingBox& box = obj.bbox;

  if (box.label == 0)
  {
    classStr = "Human";
  }
  else if (box.label == 1)
  {
    classStr = "SmallVehicle";
  }
  else if (box.label == 2)
  {
    classStr = "BigVehicle";
  }
  else if (box.label == 3)
  {
    classStr = "StopSign";
  }
  else if (box.label == 4)
  {
    classStr = "RoadSign";
  }

  boxStr = "(" \
    + std::to_string(box.x1) + ", " \
    + std::to_string(box.y1) + ", " \
    + std::to_string(box.x2) + ", " \
    + std::to_string(box.y2) + ")";

  confStr = std::to_string(box.confidence);
  distStr = std::to_string(obj.distanceToCamera);
  // ttcStr = std::to_string(obj.currTTC);
  // ttcCounterStr = std::to_string(obj.ttcCounter);

  logStr = \
    "Cls: " + classStr + \
    " Box: " + boxStr + \
    " Conf: " + confStr + \
    " Distance: " + distStr;
    //  + \
    // " TTC: " + ttcStr + \
    // " TTCCounter: " + ttcCounterStr;

  log(logStr);
}


void ObjectTrackingLogger::logObjects(const vector<Object>& objList)
{
  m_logs.clear();

  for (int i=0; i<objList.size(); i++)
  {
    if (objList[i].status == 1)
      logObject(objList[i]);
  }
}



// --- Logger Manager --- //
LoggerManager::LoggerManager()
{
  m_objectDetectionLogger = new ObjectDetectionLogger("ObjDetect");
  m_objectTrackingLogger = new ObjectTrackingLogger("ObjTrack ");
}

LoggerManager::~LoggerManager()
{
  delete m_objectDetectionLogger;
  delete m_objectTrackingLogger;

  m_objectDetectionLogger = nullptr;
  m_objectTrackingLogger = nullptr;
}