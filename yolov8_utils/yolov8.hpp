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

#ifndef __YOLOv8__
#define __YOLOv8__

#include <chrono>
#include <iostream>
#include <string>

// SNPE SDK
//#include "CheckRuntime.hpp"
//#include "LoadContainer.hpp"
//#include "SetBuilderOptions.hpp"
//#include "DlSystem/DlError.hpp"
//#include "DlSystem/RuntimeList.hpp"
//#include "DlSystem/UserBufferMap.hpp"
//#include "DlSystem/IUserBuffer.hpp"
//#include "DlContainer/IDlContainer.hpp"
//#include "SNPE/SNPE.hpp"
//#include "SNPE/SNPEFactory.hpp"
//#include "DlSystem/ITensorFactory.hpp"
//#include "DlSystem/TensorMap.hpp"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// WNC
#include "point.hpp"
#include "object.hpp"
#include "bounding_box.hpp"
#include "yolov8_decoder.hpp"
#include "img_util.hpp"
#include "utils.hpp"
#include "dla_config.hpp"
#include "logger.hpp"


using namespace std;

#define FILE_MODE 0
#define MAX_YOLO_BBX  100

// --- YOLOv8-nano --- //
#define NUM_BBOX 8400
#define INPUT_WIDTH 640
#define INPUT_HEIGHT 640
#define NUM_DET_CLASSES 4


enum DetectionLabel
{
  HUMAN = 0,
  BIKE = 1,
  VEHICLE = 2,
  MOTORBIKE = 3
};


// YOLO output
unsigned int YOLOv8_Decode(
  const float *lane,
  const float *drive,
  const float *detection,
  const float conf_thr,
  const float iou_thr,
  struct v8xyxy out[]);


class YOLOv8
{
 public:
  YOLOv8(Config_S *config);
  ~YOLOv8();

  ///////////////////////////
  /// Member Functions
  //////////////////////////

  // Inference
  bool run(cv::Mat &imgFrame,vector<BoundingBox> bboxlist);
  // void close();

  // I/O
  bool loadInput(std::string filePath);
  bool loadInput(cv::Mat &imgFrame);
  bool preProcessingFile(std::string imgPath);
  bool preProcessingMemory(cv::Mat &imgFrame);
  bool postProcessing(vector<BoundingBox> bboxlist);

  // Detection
  bool getHumanBoundingBox(
    vector<BoundingBox> &_outHumanBboxList,
    float confidenceHuman,
    int videoWidth,
    int videoHeight,
    std::vector<v8xyxy> m_yoloOut);
    //BoundingBox &fcwROI);

  bool getBikeBoundingBox(
    vector<BoundingBox> &_outRiderBboxList,
    float confidenceRider,
    int videoWidth,
    int videoHeight,
    std::vector<v8xyxy> m_yoloOut);
    // BoundingBox &fcwROI);

  bool getMotorbikeBoundingBox(
    vector<BoundingBox> &_outRiderBboxList,
    float confidenceRider,
    int videoWidth,
    int videoHeight,
    std::vector<v8xyxy> m_yoloOut);
    //BoundingBox &fcwROI);

  bool getVehicleBoundingBox(
    vector<BoundingBox> &_outBboxList,
    float confidenceVehicle,
    int videoWidth,
    int videoHeight,
    std::vector<v8xyxy> m_yoloOut);
    //BoundingBox &fcwROI);

  // Others
  // Debug
  // void getDebugLogs();
  void debugON();
  void showProcTime();


 private:
  ///////////////////////////
  /// Member Functions
  //////////////////////////

  // I/O
  bool _initModelIO();
  bool _loadImageFile(const std::string& inputFile);
  bool _imgPreprocessing();
  // bool _getITensor(float* yoloOutput, const zdl::DlSystem::ITensor* tensor);
  bool _getOutputTensor();

  // Detection
  void _OD_postProcessing(vector<BoundingBox> bboxlist);

  float _getBboxOverlapRatio(
    BoundingBox &boxA, BoundingBox &boxB);

  void _rescaleBoundingBox(
    int bbx_num,
    struct v8xyxy *out,
    struct v8xyxy *scaledOut,
    int inputW,
    int inputH,
    int frameW,
    int frameH
  );

  void _bboxMerging(
    BoundingBox &bboxA, BoundingBox &bboxB, int label, BoundingBox &bboxMerge);


  ///////////////////////////
  /// Member Variables
  //////////////////////////

  // Mat
  cv::Mat m_img;
  cv::Mat m_imgResized;

  // Model Related
  // std::unique_ptr<zdl::SNPE::SNPE> m_snpe = nullptr;

  // I/O information

  // Input
  int m_inputChannel = 0;
  int m_inputHeight = 0;
  int m_inputWidth = 0;
  int m_detectionSize = 0;
  int m_detectionBoxSize = 0;
  int m_detectionConfSize = 0;
  int m_detectionClassSize = 0;

  std::vector<float> m_inputBuff;

  // zdl::DlSystem::TensorShape m_inputTensorShape;
  // std::unique_ptr<zdl::DlSystem::ITensor> m_inputTensor;
  cv::Size inputSize;

  // Output Tensor
  // zdl::DlSystem::TensorMap m_outputTensorMap;
  BoundingBox* m_dummyBox;

  // Output (Yolo Decoder)
  YOLOv8_Decoder *m_decoder;

  float* m_detectionBoxBuff;
  float* m_detectionConfBuff;
  float* m_detectionClsBuff;

  std::vector<std::string> m_outputTensorList = {
    "box",
    "conf",
    "cls"};

  // Inference
  bool m_inference = true;

  // Bounding Box
  float m_bboxExpandRatio = 1.0;
  struct v8xyxy m_yoloOut[MAX_YOLO_BBX];
  struct v8xyxy m_scaledOut[MAX_YOLO_BBX];
  int m_numBox = 0;

  // Threshold
  float confidenceThreshold = 0.2;
  float iouThreshold = 0.4;

  // debug
  bool m_debugMode = false;
  bool m_showMask = false;
  bool m_estimateTime = false;
};

#endif