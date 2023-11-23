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

#ifndef __LOGGER__
#define __LOGGER__

#ifndef SPDLOG_COMPILED_LIB
#define SPDLOG_COMPILED_LIB
#endif

#include <iostream>
#include <string>
#include <vector>
// #include <spdlog/spdlog.h>
// #include <spdlog/sinks/daily_file_sink.h>
// #include <spdlog/sinks/stdout_color_sinks.h>
// #include <spdlog/sinks/rotating_file_sink.h>

// WNC
#include "dataStructures.h"
#include "bounding_box.hpp"
#include "object.hpp"


// --- Base Logger --- //
class BaseLogger
{
  public:
    BaseLogger(const std::string& loggerName);
    virtual ~BaseLogger();

    void log(const std::string& message)
    {
      m_logs.push_back(message);
    }

    void log(const std::string &logType, const std::string &message)
    {
      std::string logStr = m_loggerName + " " + logType + ": " + message;
      m_logs.push_back(logStr);
    }

    void log(const std::string &logType, int value)
    {
      std::string logStr = m_loggerName + " " + logType + ": " + std::to_string(value);
      m_logs.push_back(logStr);
    }

    void log(const std::string &logType, float value)
    {
      std::string logStr = m_loggerName + " " + logType + ": " + std::to_string(value);
      m_logs.push_back(logStr);
    }

    void log(const std::string &logType, const cv::Point& p)
    {
      std::string logStr = m_loggerName + " " + logType + ": (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
      m_logs.push_back(logStr);
    }

    std::vector<std::string> m_logs;
    std::string m_loggerName;
};


// --- Object Detection Logger --- //
class ObjectDetectionLogger : public BaseLogger
{
  public:
    ObjectDetectionLogger(const std::string& loggerName):BaseLogger(loggerName){};

    void logObject(const BoundingBox& bbox);
    void logObjects(const vector<BoundingBox>& bboxList);
};

// --- Object Tracking Logger --- //
class ObjectTrackingLogger : public BaseLogger
{
  public:
    ObjectTrackingLogger(const std::string& loggerName):BaseLogger(loggerName){};

    void logObject(const Object& obj);
    void logObjects(const vector<Object>& objList);
};


// --- Logger Manager --- //
class LoggerManager
{
  public:
    LoggerManager();
    ~LoggerManager();

    ObjectDetectionLogger* m_objectDetectionLogger;
    ObjectTrackingLogger* m_objectTrackingLogger;
};
#endif

