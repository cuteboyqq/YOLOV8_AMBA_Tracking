
#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
// #include <opencv2/xfeatures2d.hpp>
// #include <opencv2/xfeatures2d/nonfree.hpp>
#include "bounding_box.hpp"
#include "dataStructures.h"
#include "utils.hpp"

#include "gms_matcher.hpp"


void visualizeKeypoints(const vector<cv::KeyPoint> &keypoints, const cv::Mat &img, const string& windowName, bool displayImageWindows, bool saveImageFiles);
void detectKeypoints(cv::Ptr<cv::FeatureDetector> &detector, const std::string& detectorName, std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, bool displayImageWindows, bool saveImageFiles);
void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool displayImageWindows, bool saveImageFiles, ResultLineItem &result);
void detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool displayImageWindows, bool saveImageFiles, ResultLineItem &result);
void detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool displayImageWindows, bool saveImageFiles);
void detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool displayImageWindows, bool saveImageFiles, ResultLineItem &result);
void detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool displayImageWindows, bool saveImageFiles);
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool displayImageWindows, bool saveImageFiles, ResultLineItem &result);
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool displayImageWindows, bool saveImageFiles, ResultLineItem &result);
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, const std::string& descriptorType);
void matchDescriptors(vector<cv::KeyPoint> &kPtsSource,
                      vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource,
                      cv::Mat &descRef,
                      cv::Mat &imgSource,
                      cv::Mat &imgRef,
                      vector<cv::DMatch> &matches,
                      const string &descriptorType,
                      const string& matcherType,
                      const string& selectorType);

void matchBoundingBoxes(std::vector<cv::DMatch> & matches,
                        std::map<int, int> & boundingBoxBestMatches,
                        DataFrame & previousFrame,
                        DataFrame & currentFrame);

void clusterKptMatchesWithROI(BoundingBox & boundingBox,
                              std::vector<cv::KeyPoint> & kptsPrev,
                              std::vector<cv::KeyPoint> & kptsCurr,
                              std::vector<cv::DMatch> & kptMatches);

#endif /* matching2D_hpp */
