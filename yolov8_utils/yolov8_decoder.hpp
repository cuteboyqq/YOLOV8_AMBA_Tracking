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

#ifndef __YOLOv8_DECODER__
#define __YOLOv8_DECODER__

#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "yolov8.hpp"

using namespace std;

struct bbxv8_candidate
{
  float x;
  float y;
  float w;
  float h;
  float c_prob;   // class probability for the max_cls
  int c; // class with maximum probability : 0,1..num_classes-1
};

struct v8xyxy
{
  int x1;
  int y1;
  int x2;
  int y2;
  float c_prob; // class probability, apply sigmoid()
  int c; // class
};

class YOLOv8_Decoder
{
 public:
  YOLOv8_Decoder(int inputH, int inputW);
  ~YOLOv8_Decoder();

  ///////////////////////////
  /// Member Functions
  //////////////////////////
  unsigned int decode(
    const float *m_detection_buff,
    const float confThreshold,
    const float iouThreshold,
    struct v8xyxy out[]
  );

  unsigned int decode(
    const float *m_detection_box_buff,
    const float *m_detection_conf_buff,
    const float *m_detection_cls_buff,
    const float confThreshold,
    const float iouThreshold,
    struct v8xyxy out[]
  );

  ///////////////////////////
  /// Member Variables
  //////////////////////////

 private:
  ///////////////////////////
  /// Member Functions
  //////////////////////////
  float sigmoid(float x);
  int argmax(float cls_begin[], int n);
  float iou(v8xyxy a, v8xyxy b);
  float getBboxOverlapRatio(v8xyxy boxA, v8xyxy boxB);
  int doNMS(vector<v8xyxy> &bboxList, const float iouThreshold, vector<v8xyxy> &picked);
  v8xyxy decodeBoundingBox(bbxv8_candidate c);
  int genBoundingBox(const vector<bbxv8_candidate> &detList, vector<v8xyxy> &bboxList);
  int getCandidates(float *detection, float confThreshold, vector<v8xyxy> &bboxList);
  int getCandidates(
    float *detectionBox, float *detectionConf, float *detectionClass, float confThreshold, vector<v8xyxy> &bboxList);

  ///////////////////////////
  /// Member Variables
  //////////////////////////
  int m_inputH = 348;
  int m_inputW = 640;

  // debug
  bool debugMode = true;
};

#endif