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

#ifndef _DLA_CONFIG_
#define _DLA_CONFIG_

#include <map>
#include <string>
#include <iostream>

using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

#define LABEL_CLASS_COUNT       1200
#define LABEL_NAME_MAX_SIZE     60

#define DETECT_THRESHOLD        0.5
#define MAX_DETECT_RECT_NUM     30
#define INNER_MOST_ALIGNMENT    8


typedef struct
{
  int maxDetection;               // How many people can be detected at most
  int removeOverlap;
  int boundingBoxShow;
  float humanConfidence;
  float carConfidence;
  float bikeConfidence;
  float motorbikeConfidence;
  std::string detectionRange;     // Near / Medium / Far

} OD_Config_S;


typedef struct
{
  int maxTracking;                // How many people can be tracked at most
  std::string matchingLevel;      // Low / Normal / High

} TRACKER_Config_S;


typedef struct
{
  float height; //
  float focalLength;

} CAMERA_Config_S;


typedef struct
{
  bool config;
  bool tracking;
  bool yolov8;
  bool objectDetection;
  bool objectTracking;
  bool humanTracker;
  bool bikeTracker;
  bool vehicleTracker;
  bool motorbikeTracker;
  bool saveLogs;
  bool saveImages;
  bool saveRawImages;
  std::string logsDirPath;
  std::string imgsDirPath;
  std::string rawImgsDirPath;

} Debug_Config_S;


typedef struct
{
  bool results;
  bool objectDetection;
  bool objectTracking;
  bool warningZone;
  bool information;
  int maxFrameIndex;

} Display_Config_S;


typedef struct
{
  bool tracking;
  bool yolov8;
  bool objectTracking;

} ShowProcTime_Config_S;


typedef struct
{
  std::string runtime;        // QCS6490
  std::string modelPath;

  // Model Input Size
  int modelWidth;
  int modelHeight;

  // Frame Size
  int frameWidth;
  int frameHeight;

  // Processing
  float procFrameRate;
  int procFrameStep;

  // Other config
  OD_Config_S stOdConfig;
  TRACKER_Config_S stTrackerConifg;
  CAMERA_Config_S stCameraConfig;
  Debug_Config_S stDebugConfig;
  Display_Config_S stDisplayConfig;
  ShowProcTime_Config_S stShowProcTimeConfig;

} Config_S;


class ConfigReader
{
private:

  // Define the map to store data from the config file
  std::map<std::string, std::string> m_ConfigSettingMap;

  // Static pointer instance to make this class singleton.
  static ConfigReader* m_pInstance;

public:

  // Public static method getInstance(). This function is
  // responsible for object creation.
  static ConfigReader* getInstance();

  // Parse the config file.
  bool parseFile(std::string fileName = "/tmp/default_config");

  // Overloaded getValue() function.
  // Value of the tag in cofiguration file could be
  // string or integer. So the caller need to take care this.
  // Caller need to call appropiate function based on the
  // data type of the value of the tag.

  bool getValue(std::string tag, int& value);
  bool getValue(std::string tag, std::string& value);

  // Function dumpFileValues is for only debug purpose
  void dumpFileValues();

private:

  // Define constructor in the private section to make this
  // class as singleton.
  ConfigReader();

  // Define destructor in private section, so no one can delete
  // the instance of this class.
  ~ConfigReader();

  // Define copy constructor in the private section, so that no one can
  // voilate the singleton policy of this class
  ConfigReader(const ConfigReader& obj){}
  // Define assignment operator in the private section, so that no one can
  // voilate the singleton policy of this class
  void operator=(const ConfigReader& obj){}

  // Helper function to trim the tag and value. These helper function is
  // calling to trim the un-necessary spaces.
  std::string trim(const std::string& str, const std::string& whitespace = " \t");
  std::string reduce(const std::string& str,
      const std::string& fill = " ",
      const std::string& whitespace = " \t");
};


#ifdef __cplusplus
}

#endif
#endif
