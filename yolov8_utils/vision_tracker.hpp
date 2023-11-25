#ifndef __VISION_TRACKER__
#define __VISION_TRACKER__

#include <cstring>
#include <memory>
#include <iostream>
#include <getopt.h>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>

// WNC
#include "dataStructures.h"
#include "dla_config.hpp"
#include "config_reader.hpp"
#include "yolov8.hpp"
//#include "yolov8_class.h"
#include "matching2D.hpp"
#include "object_tracker.hpp"

#if defined (SPDLOG)
#include "logger.hpp"
#endif

using namespace std;

#define VERSION 0.1
#define SUCCESS 1
#define FAILURE 0


class VisionTracker
{
 public:
  VisionTracker(std::string configPath);
  ~VisionTracker();

  // bool run_ori(cv::Mat &imgFrame);
  bool run(cv::Mat &imgFrame,std::vector<BoundingBox> bboxList);
  void getResults(VisionTrackingResults &result);
  void getResultImage(cv::Mat &imgResult);
  bool isFinishDetection() { return !m_preparingNextDetection; };

 private:

  // === Initilization === //
  bool _init(std::string configPath);
  bool _initROI();
  bool _readDebugConfig();
  bool _readDisplayConfig();
  bool _readShowProcTimeConfig();

  // === Work Flow === //
  bool _modelInfernece(std::vector<BoundingBox> bboxList);
  // bool _objectDetection_ori();
  bool _objectDetection(std::vector<BoundingBox> bboxList);
  bool _objectTracking();

  // === Results === //
  void _saveRawImages();
  void _saveDrawResults();
  void _showDetectionResults();
#if defined (SPDLOG)
  void _saveDetectionResult(std::vector<std::string>& logs);
  void _saveDetectionResults();
#endif

  // === Debug === //
  void _drawResults();
  void _drawBoundingBoxes();
  void _drawTrackedObjects();
  void _drawInformation();
  void _drawObjectLocation();

  // === Utils === //
  void _updateFrameIndex();


  // === Config === //
  TrackerConfigReader* m_configReader;
  Config_S* m_config;

  // === Frame === //
  int m_frameIdx = 0;
  int m_frameStep = 4;

  // === Input Size === //
  int m_videoWidth = 0;
  int m_videoHeight = 0;
  int m_modelWidth = 0;
  int m_modelHeight = 0;

  // === YOLO-ADAS === //
  YOLOv8* m_yolov8;
  //YoloV8_Class m_yolov8;

  // === Camera === //


  // === Input Frame === //
  cv::Mat m_img;

  // === Bounding Box === //

  // Output Bounding Boxes from AI
  std::vector<BoundingBox> m_humanBBoxList;
  std::vector<BoundingBox> m_bikeBBoxList;
  std::vector<BoundingBox> m_vehicleBBoxList;
  std::vector<BoundingBox> m_motorbikeBBoxList;

  // === Objects === //
  std::vector<Object> m_humanObjList;
  std::vector<Object> m_bikeObjList;
  std::vector<Object> m_vehicleObjList;
  std::vector<Object> m_motorbikeObjList;
  std::vector<Object> m_trackedObjList;

  // === Object Tracker === //
  ObjectTracker* m_humanTracker;
  ObjectTracker* m_bikeTracker;
  ObjectTracker* m_vehicleTracker;
  ObjectTracker* m_motorbikeTracker;

  // === Object Tracking ROI === //
  BoundingBox* m_roi;

  // === Object BEV Location === //
  std::vector<TrackedObj> m_humanTrackObjList;
  std::vector<TrackedObj> m_bikeTrackObjList;
  std::vector<TrackedObj> m_vehicleTrackObjList;
  std::vector<TrackedObj> m_motorbikeTrackObjList;

  // === Following Distance === //
  float m_focalRescaleRatio = 0;

  // === Result === //
  VisionTrackingResults m_result;

  // === Display === //
  cv::Mat m_dsp_img;
  cv::Mat m_dsp_imgResize;
  bool m_dsp_results = true;
  bool m_dsp_objectDetection = true;
  bool m_dsp_objectTracking = true;
  bool m_dsp_warningZone = true;
  bool m_dsp_information = true;
  int m_dsp_maxFrameIdx;

  // === Debug === //
  bool m_dbg_tracking = true;
  bool m_dbg_yolov8 = true;
  bool m_dbg_objectDetection = true;
  bool m_dbg_objectTracking = true;
  bool m_dbg_saveLogs = false;
  bool m_dbg_saveImages = true;
  bool m_dbg_saveRawImages = false;
  std::string m_dbg_logsDirPath = "";
  std::string m_dbg_imgsDirPath = "";
  std::string m_dbg_rawImgsDirPath = "";
  std::string m_dbg_dateTime = "";

  // === Others === //
  bool m_preparingNextDetection = true;

#if defined (SPDLOG)
  LoggerManager m_loggerManager;
#endif

  // === Other === //
  bool m_estimateTime = false;
};


#endif

