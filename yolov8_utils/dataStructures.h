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

#ifndef __DATA_STRUCTURES__
#define __DATA_STRUCTURES__

#include <vector>
#include <map>
#include <opencv2/core.hpp>

#include "bounding_box.hpp"
#include "object.hpp"

using namespace std;

class Object;

struct DataFrame
{
    cv::Mat cameraImg; // camera image

    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
    std::map<int, int> bbMatches; // bounding box matches between previous and current frame
};


struct Hyperparameters
{
    Hyperparameters()= default;

    string keypointDetector = "Shi_Tomasi"; // Shi_Tomasi, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptor = "BRIEF";                    // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    string matcherType = "MAT_BF";                  // MAT_BF, MAT_FLANN
    string descriptorType = "DES_BINARY";           // DES_BINARY, DES_HOG
    string selectorType = "SEL_KNN";                // SEL_NN, SEL_KNN
};


struct KeypointMatchResult
{
    KeypointMatchResult()= default;
    std::pair<unsigned int, unsigned int> matchedImagePair = {0,0};
    unsigned int totalMatches = 0;
    unsigned int knnMatches = 0;
    unsigned int removed = 0;
    double percentageRemoved = 0.0;
};


struct KeypointCountResult
{
    KeypointCountResult()= default;
    string imageName = "No image name specified";
    unsigned int totalKeypoints = 0;
    double descriptorMatchingTime = 0.0;
    unsigned int precedingVehicleKeypoints = 0;
};


struct ResultLineItem
{
    ResultLineItem()= default;
    unsigned int frame = 0;
    double ttcLidar = 0.0;
    double ttcCamera = 0.0;
    unsigned int lidarPoints = 0;

    KeypointMatchResult keypointMatch;
    KeypointCountResult keypointCount;

    double descriptorExtractionTime = 0.0;
};


struct ResultSet
{
    ResultSet()= default;
    std::string detector = "";
    std::string descriptor = "";
    std::vector<ResultLineItem> data;
};


struct Experiment
{
    Experiment()= default;
    std::vector<ResultSet> resultSet;
    Hyperparameters hyperparameters;

    // Visualization and image saving options
    bool displayImageWindows = false;               // visualize matches between current and previous image?
    bool isFocusOnPrecedingVehicleOnly = true;      // only keep keypoints on the preceding vehicle?
    bool saveKeypointDetectionImagesToFile = false;  // save keypoint detection images to file
    bool saveKeypointMatchImagesToFile = true;      // save keypoint matching images to file
};


struct TrackedObj
{
    int id;
    int type;
    float confidence;
    cv::Point3f pLoc;
};


struct VisionTrackingResults
{
  std::vector<TrackedObj> humanObjList;
  std::vector<TrackedObj> bikeObjList;
  std::vector<TrackedObj> vehicleObjList;
  std::vector<TrackedObj> motorbikeOjList;
};


#endif /* dataStructures_h */
