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

#include "config_reader.hpp"


/////////////////////////
// public member functions
////////////////////////
TrackerConfigReader::TrackerConfigReader()
{
  m_config = new Config_S();

#if defined (SPDLOG)
  auto m_logger = spdlog::stdout_color_mt("TrackerConfigReader");
  m_logger->set_pattern("[%n] [%^%l%$] %v");
#endif

};

TrackerConfigReader::~TrackerConfigReader() {};


Config_S* TrackerConfigReader::getConfig()
{
  return m_config;
}


bool TrackerConfigReader::read(std::string configPath)
{
  // Create object of the class ConfigReader
  ConfigReader *configReader = ConfigReader::getInstance();

#if defined (SPDLOG)
  auto m_logger = spdlog::get("TrackerConfigReader");
#endif

  if (utils::checkFileExists(configPath) && configReader->parseFile(configPath))
  {
    // Print divider on the console to understand the output properly
#if defined (SPDLOG)
    m_logger->info("=================================================");
#endif
    // Define variables to store the value

    // Platform Runtime
    string runtime = "";      // For QCS6490

    // Model Information
    string modelPath = "";
    int modelWidth = 0;
    int modelHeight = 0;

    // Camera Information
    string cameraHeight = "";
    string cameraFocalLength = "";
    int frameWidth = 0;
    int frameHeight = 0;

    // Processing Time
    string procFrameRate = "";
    int procFrameStep = 4;

    // Object Detection
    string humanConfidence = "";
    string carConfidence = "";
    string bikeConfidence = "";
    string motorbikeConfidence = "";
    string detectionRange = "";
    int maxDetection = 5;
    int removeOverlap = 1;

    // Object Tracking
    string matchingLevel = "";
    int maxTracking = 3;

    // Debug Information
    int debugConfig = 0;
    int debugTracking = 0;
    int debugYolov8 = 0;
    int debugObjectDetection = 0;
    int debugObjectTracking = 0;
    int debugHumanTracker = 0;
    int debugBikeTracker = 0;
    int debugVehicleTracker = 0;
    int debugMotorbikeTracker = 0;
    int debugSaveLogs = 0;
    int debugSaveImages = 0;
    int debugSaveRawImages = 0;
    string debugLogsDirPath = "";
    string debugImagesDirPath = "";
    string debugRawImagesDirPath = "";

    // Display Results
    int displayResults = 0;
    int displayObjectDetection = 0;
    int displayObjectTracking = 0;
    int displayWarningZone = 0;
    int displayInformation = 0;
    int displayMaxFrameIndex = 0;

    // Show Processing Time
    int showProcTimeTracking = 0;
    int showProcTimeYolov8 = 0;
    int showProcTimeObjectTracking = 0;

    // Update the variable by the value present in the configuration file.
    configReader->getValue("Runtime", runtime);

    // Model Information
    configReader->getValue("ModelPath", modelPath);
    configReader->getValue("ModelWidth", modelWidth);
    configReader->getValue("ModelHeight", modelHeight);

    // Camera Information
    configReader->getValue("CameraHeight", cameraHeight);
    configReader->getValue("CameraFocalLength", cameraFocalLength);
    configReader->getValue("FrameWidth", frameWidth);
    configReader->getValue("FrameHeight", frameHeight);

    // Processing Time
    configReader->getValue("ProcessingFrameRate", procFrameRate);
    configReader->getValue("ProcessingFrameStep", procFrameStep);

    // Object Detection
    configReader->getValue("MaxDetection", maxDetection);
    configReader->getValue("HumanConfidence", humanConfidence);
    configReader->getValue("CarConfidence", carConfidence);
    configReader->getValue("BikeConfidence", bikeConfidence);
    configReader->getValue("MotorbikeConfidence", motorbikeConfidence);
    configReader->getValue("DetectionRange", detectionRange);
    configReader->getValue("RemoveOverlap", removeOverlap);

    // Object Tracking
    configReader->getValue("MaxTracking", maxTracking);
    configReader->getValue("MatchingLevel", matchingLevel);

    // Debug Information
    configReader->getValue("DebugConfig", debugConfig);
    configReader->getValue("DebugTracking", debugTracking);
    configReader->getValue("DebugYolov8", debugYolov8);
    configReader->getValue("DebugObjectDetection", debugObjectDetection);
    configReader->getValue("DebugObjectTracking", debugObjectTracking);
    configReader->getValue("DebugHumanTracker", debugHumanTracker);
    configReader->getValue("DebugBikeTracker", debugBikeTracker);
    configReader->getValue("DebugVehicleTracker", debugVehicleTracker);
    configReader->getValue("DebugMotorbikeTracker", debugMotorbikeTracker);
    configReader->getValue("DebugSaveLogs", debugSaveLogs);
    configReader->getValue("DebugSaveImages", debugSaveImages);
    configReader->getValue("DebugSaveRawImages", debugSaveRawImages);
    configReader->getValue("DebugLogsDirPath", debugLogsDirPath);
    configReader->getValue("DebugImagesDirPath", debugImagesDirPath);
    configReader->getValue("DebugRawImagesDirPath", debugRawImagesDirPath);

    // Display Results
    configReader->getValue("DisplayResults", displayResults);
    configReader->getValue("DisplayObjectDetection", displayObjectDetection);
    configReader->getValue("DisplayObjectTracking", displayObjectTracking);
    configReader->getValue("DisplayWarningZone", displayWarningZone);
    configReader->getValue("DisplayInformation", displayInformation);
    configReader->getValue("DisplayMaxFrameIndex", displayMaxFrameIndex);

    // Show Processing Time
    configReader->getValue("ShowProcTimeTracking", showProcTimeTracking);
    configReader->getValue("ShowProcTimeYOLOv8", showProcTimeYolov8);
    configReader->getValue("ShowProcTimeObjectTracking", showProcTimeObjectTracking);

#if defined (SPDLOG)
    // Variables has been updated. Print them on the console.
    if (debugConfig)
    {
      m_logger->info("-------------------------------------------------");
      m_logger->info("[Platform Runtime Information]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("Runtime \t\t= {}", runtime);

      m_logger->info("-------------------------------------------------");
      m_logger->info("[Model Information]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("ModelPath \t\t= {}", modelPath);
      m_logger->info("ModelWidth \t\t= {}", modelWidth);
      m_logger->info("ModelHeight \t\t= {}", modelHeight);

      m_logger->info("-------------------------------------------------");
      m_logger->info("[Camera Information]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("CameraHeight \t= {}", cameraHeight);
      m_logger->info("CameraFocalLength \t= {}", cameraFocalLength);
      m_logger->info("FrameWidth \t\t= {}", frameWidth);
      m_logger->info("FrameHeight \t\t= {}", frameHeight);

      m_logger->info("-------------------------------------------------");
      m_logger->info("[Processing Time]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("ProcessingFrameRate \t= {}", procFrameRate);
      m_logger->info("ProcessingFrameStep \t= {}", procFrameStep);

      m_logger->info("-------------------------------------------------");
      m_logger->info("[Object Detection]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("MaxDetection \t= {}", maxDetection);
      m_logger->info("DetectionRange \t= {}", detectionRange);
      m_logger->info("RemoveOverlap \t= {}", removeOverlap);
      m_logger->info("HumanConfidence \t= {}", humanConfidence);
      m_logger->info("BikeConfidence \t= {}", bikeConfidence);
      m_logger->info("CarConfidence \t= {}", carConfidence);
      m_logger->info("MotorbikeConfidence \t= {}", motorbikeConfidence);

      m_logger->info("-------------------------------------------------");
      m_logger->info("[Object Tracking]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("MaxTracking \t\t= {}", maxTracking);
      m_logger->info("MatchingLevel \t= {}", matchingLevel);

      // Debug Information
      m_logger->info("-------------------------------------------------");
      m_logger->info("[Debug Information]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("Config \t\t= {}", debugConfig);
      m_logger->info("Tracking \t\t= {}", debugTracking);
      m_logger->info("Yolov8 \t\t= {}", debugYolov8);
      m_logger->info("ObjectDetection \t= {}", debugObjectDetection);
      m_logger->info("ObjectTracking \t= {}", debugObjectTracking);
      m_logger->info("HumanTracker \t= {}", debugHumanTracker);
      m_logger->info("RiderTracker \t= {}", debugBikeTracker);
      m_logger->info("VehicleTracker \t= {}", debugVehicleTracker);
      m_logger->info("MotorbikeTracker \t= {}", debugMotorbikeTracker);
      m_logger->info("DebugSaveLogs \t= {}", debugSaveLogs);
      m_logger->info("DebugSaveImages \t= {}", debugSaveImages);
      m_logger->info("DebugSaveRawImages \t= {}", debugSaveRawImages);
      m_logger->info("DebugLogsDirPath \t= {}", debugLogsDirPath);
      m_logger->info("DebugImagesDirPath \t= {}", debugImagesDirPath);
      m_logger->info("DebugRawImagesDirPath \t= {}", debugRawImagesDirPath);

      // Display Results
      m_logger->info("-------------------------------------------------");
      m_logger->info("[Display Information]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("Results \t\t= {}", displayResults);
      m_logger->info("ObjectDetection \t= {}", displayObjectDetection);
      m_logger->info("ObjectTracking \t= {}", displayObjectTracking);
      m_logger->info("ForwardCollision \t= {}", displayWarningZone);
      m_logger->info("Information \t\t= {}", displayInformation);
      m_logger->info("MaxFrameIndex \t= {}", displayMaxFrameIndex);

      // Show Processing Time Information
      m_logger->info("-------------------------------------------------");
      m_logger->info("[Show Processing Time Information]");
      m_logger->info("-------------------------------------------------");
      m_logger->info("Tracking \t\t= {}", showProcTimeTracking);
      m_logger->info("Yolov8 \t\t= {}", showProcTimeYolov8);
      m_logger->info("ObjectTracking \t= {}", showProcTimeObjectTracking);
      m_logger->info("=================================================");
    }
#endif
    // Pass configuration

    // Platform Runtime
    m_config->runtime = runtime;

    // Model Information
    m_config->modelPath = modelPath;
    m_config->modelWidth = modelWidth;
    m_config->modelHeight = modelHeight;

    // Camera Information
    m_config->stCameraConfig.height = std::stof(cameraHeight);
    m_config->stCameraConfig.focalLength = std::stof(cameraFocalLength);
    m_config->frameWidth = frameWidth;
    m_config->frameHeight = frameHeight;

    // Processing Time
    m_config->procFrameRate = std::stof(procFrameRate);
    m_config->procFrameStep = procFrameStep;

    // Object Detection
    m_config->stOdConfig.maxDetection = maxDetection;
    m_config->stOdConfig.detectionRange = detectionRange;
    m_config->stOdConfig.removeOverlap = removeOverlap;
    m_config->stOdConfig.humanConfidence = std::stof(humanConfidence);
    m_config->stOdConfig.bikeConfidence = std::stof(bikeConfidence);
    m_config->stOdConfig.carConfidence = std::stof(carConfidence);
    m_config->stOdConfig.motorbikeConfidence = std::stof(motorbikeConfidence);

    // Object Tracking
    m_config->stTrackerConifg.maxTracking = maxTracking;
    m_config->stTrackerConifg.matchingLevel = matchingLevel;

    // Debug Information
    m_config->stDebugConfig.config = debugConfig;
    m_config->stDebugConfig.tracking = debugTracking;
    m_config->stDebugConfig.yolov8 = debugYolov8;
    m_config->stDebugConfig.objectDetection = debugObjectDetection;
    m_config->stDebugConfig.objectTracking = debugObjectTracking;
    m_config->stDebugConfig.humanTracker = debugHumanTracker;
    m_config->stDebugConfig.bikeTracker = debugBikeTracker;
    m_config->stDebugConfig.vehicleTracker = debugVehicleTracker;
    m_config->stDebugConfig.motorbikeTracker = debugMotorbikeTracker;
    m_config->stDebugConfig.saveLogs = debugSaveLogs;
    m_config->stDebugConfig.saveImages = debugSaveImages;
    m_config->stDebugConfig.saveRawImages = debugSaveRawImages;
    m_config->stDebugConfig.logsDirPath = debugLogsDirPath;
    m_config->stDebugConfig.imgsDirPath = debugImagesDirPath;
    m_config->stDebugConfig.rawImgsDirPath = debugRawImagesDirPath;

    // Display Results
    m_config->stDisplayConfig.results = displayResults;
    m_config->stDisplayConfig.objectDetection = displayObjectDetection;
    m_config->stDisplayConfig.objectTracking = displayObjectTracking;
    m_config->stDisplayConfig.warningZone = displayWarningZone;
    m_config->stDisplayConfig.information = displayInformation;
    m_config->stDisplayConfig.maxFrameIndex = displayMaxFrameIndex;

    // Show Processing Time
    m_config->stShowProcTimeConfig.tracking = showProcTimeTracking;
    m_config->stShowProcTimeConfig.yolov8 = showProcTimeYolov8;
    m_config->stShowProcTimeConfig.objectTracking = showProcTimeObjectTracking;
  }
  else
  {
#if defined (SPDLOG)
    m_logger->error("=================================================");
    m_logger->error("Read Config Failed! ===> Use default configs");
    m_logger->error("=================================================");
#endif
    return false;
  }

  //
  configReader = NULL;

  return true;
}