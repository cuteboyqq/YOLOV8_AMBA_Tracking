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

#include "yolov8.hpp"


// Object color
const std::vector<cv::Scalar> object_colors = {
  {255, 51, 123}, // Person
  {0, 0, 255},    // Bicycle
  {0, 255, 0},    // Vehicle
  {0, 255, 128},  // Motorbike
};


/////////////////////////
// public member functions
////////////////////////
YOLOv8::YOLOv8(Config_S *config)  // main function
{
  // auto m_logger = spdlog::stdout_color_mt("YOLOv8");
  // m_logger->set_pattern("[%n] [%^%l%$] %v");

  // if (config->stDebugConfig.yolov8)
  // {
  //   m_logger->set_level(spdlog::level::debug);
  // }
  // else
  // {
  //   m_logger->set_level(spdlog::level::info);
  // }

  std::string dlcFilePath = config->modelPath;
  std::string rumtimeStr = config->runtime;

  // Runtime
  // static zdl::DlSystem::Runtime_t runtime = zdl::DlSystem::Runtime_t::CPU;
  // static zdl::DlSystem::RuntimeList runtimeList;
  bool usingInitCaching = false;
  bool staticQuantization = false;
  bool useUserSuppliedBuffers = false;

  // m_logger->info("Creating YOLOv8 Model ...");

  // Check if both runtimelist and runtime are passed in
  // if (rumtimeStr == "gpu")
  // {
  //   runtime = zdl::DlSystem::Runtime_t::GPU;
  // }
  // else if (rumtimeStr == "aip")
  // {
  //   runtime = zdl::DlSystem::Runtime_t::AIP_FIXED8_TF;
  // }
  // else if (rumtimeStr == "dsp")
  // {
  //   runtime = zdl::DlSystem::Runtime_t::DSP;
  // }
  // else if (rumtimeStr == "cpu")
  // {
  //   runtime = zdl::DlSystem::Runtime_t::CPU;
  // }
  // else
  // {
  //   m_logger->warn("The runtime option provide is not valid. Defaulting to the CPU runtime.");
  // }

  // if(runtimeList.empty() == false)
  // {
  //   m_logger->error("Invalid option cannot mix runtime order -l with runtime -r ");
  //   std::exit(1);
  // }

  // STEP1: Get Available Runtime
  // m_logger->info("Rumtime = {}", rumtimeStr);
  // runtime = checkRuntime(runtime, staticQuantization);
  // runtimeList.add(runtime);

  // STEP2: Create Deep Learning Container and Load Network File
  // m_logger->info("DLC File Path = {}",  dlcFilePath);
  // std::unique_ptr<zdl::DlContainer::IDlContainer> container = loadContainerFromFile(dlcFilePath);
  // if (container == nullptr)
  // {
  //   m_logger->error("Error while opening the container file.");
  //   std::exit(1);
  // }

  // STEP3: Set Network Builder
  // zdl::DlSystem::PlatformConfig platformConfig;
  // m_snpe = setBuilderOptions(
  //   container, runtimeList, useUserSuppliedBuffers, m_outputTensorList, platformConfig, usingInitCaching);

  // if (m_snpe == nullptr)
  // {
  //   m_logger->error("Error while building SNPE object.");
  //   std::exit(1);
  // }
  // if (usingInitCaching)
  // {
  //   if (container->save(dlcFilePath))
  //   {
  //     m_logger->info("Saved container into archive successfully");
  //   }
  //   else
  //   {
  //     m_logger->warn("Failed to save container into archive");
  //   }
  // }

  // STEP4: Init Model Input/Output Tensor
  printf("[YOLOv8::YOLOv8(Config_S *config)] Start   _initModelIO()\n");
  _initModelIO();
  printf("[YOLOv8::YOLOv8(Config_S *config)] End   _initModelIO()\n");
  // Output Decoder
  m_decoder = new YOLOv8_Decoder(m_inputHeight, m_inputWidth);
};


YOLOv8::~YOLOv8()  // clear object memory
{
  delete m_decoder;
  delete m_detectionBoxBuff;
  delete m_detectionConfBuff;
  delete m_detectionClsBuff;

  m_decoder = nullptr;
  m_detectionBoxBuff = nullptr;
  m_detectionConfBuff = nullptr;
  m_detectionClsBuff = nullptr;

  // close();
};


// void YOLOv8::close()
// {
//   m_snpe.reset();
// }


// ============================================
//               Tensor Settings
// ============================================

bool YOLOv8::_initModelIO()
{
  // auto m_logger = spdlog::get("YOLOv8");
  // m_logger->info("[YOLOv8] => Create Model Input Tensor");
  // m_logger->info("-------------------------------------------");
  // m_inputTensorShape = m_snpe->getInputDimensions();
  // m_inputHeight = m_inputTensorShape.getDimensions()[1];
  // m_inputWidth = m_inputTensorShape.getDimensions()[2];
  // m_inputChannel = m_inputTensorShape.getDimensions()[3];
  // m_logger->info("Input H: {}", m_inputHeight);
  // m_logger->info("Input W: {}", m_inputWidth);
  // m_logger->info("Input C: {}", m_inputChannel);

  // Get input names and number
  // const auto& inputTensorNamesRef = m_snpe->getInputTensorNames();
  // if (!inputTensorNamesRef) throw std::runtime_error("Error obtaining Input tensor names");
  // const zdl::DlSystem::StringList& inputTensorNames = *inputTensorNamesRef;  // inputTensorNames refers to m_snpe->getInputTensorNames()'s returned variable

  // Make sure the network requires only a single input
  // assert (inputTensorNames.size() == 1);

  /* Create an input tensor that is correctly sized to hold the input of the network.
    Dimensions that have no fixed size will be represented with a value of 0. */
  // const auto &inputDims_opt = m_snpe->getInputDimensions(inputTensorNames.at(0));
  // const auto &inputShape = *inputDims_opt;  // 384 * 640

  /* Calculate the total number of elements that can be stored in the tensor
    so that we can check that the input contains the expected number of elements.
    With the input dimensions computed create a tensor to convey the input into the network. */
  // m_inputTensor = zdl::SNPE::SNPEFactory::getTensorFactory().createTensor(inputShape);

  // Create a buffer to store image data
  m_inputBuff.resize(m_inputChannel*m_inputHeight*m_inputWidth);

  // m_logger->info("Create Model Output Buffers");
  // m_logger->info("-------------------------------------------");

  m_detectionBoxSize = 5*NUM_BBOX;
  m_detectionConfSize = NUM_BBOX;
  m_detectionClassSize = NUM_BBOX;
  m_detectionBoxBuff = new float[m_detectionBoxSize];
  m_detectionConfBuff = new float[m_detectionConfSize];
  m_detectionClsBuff = new float[m_detectionClassSize];

  return true;
}


// bool YOLOv8::_getITensor(float *yoloOutputBuff, const zdl::DlSystem::ITensor* tensor)
// {
//   int batchChunk = tensor->getSize();

//   std::memcpy(
//     yoloOutputBuff,
//     &tensor->cbegin()[0],
//     batchChunk * sizeof(float));

//   return true;
// }


bool YOLOv8::_getOutputTensor()
{
  // auto m_logger = spdlog::get("YOLOv8");
  auto time_0 = std::chrono::high_resolution_clock::now();

  // zdl::DlSystem::StringList tensorNames = m_outputTensorMap.getTensorNames();

  // for (auto it=tensorNames.begin(); it!=tensorNames.end(); it++)
  //   cout << (*it) << endl;
  // auto detectionBox_tensorPtr = m_outputTensorMap.getTensor(m_outputTensorList[0].c_str());
  // auto detectionConf_tensorPtr = m_outputTensorMap.getTensor(m_outputTensorList[1].c_str());
  // auto detectionCls_tensorPtr = m_outputTensorMap.getTensor(m_outputTensorList[2].c_str());

  // if(!_getITensor(m_detectionBoxBuff, detectionBox_tensorPtr))
  // {
  //   // m_logger->error("Failed to get detection box tensor");
  //   return false;
  // }

  // if(!_getITensor(m_detectionConfBuff, detectionConf_tensorPtr))
  // {
  //   // m_logger->error("Failed to get detection conf tensor");
  //   return false;
  // }

  // if(!_getITensor(m_detectionClsBuff, detectionCls_tensorPtr))
  // {
  //   // m_logger->error("Failed to get detection cls tensor");
  //   return false;
  // }

  auto time_1 = std::chrono::high_resolution_clock::now();
  // m_logger->debug("[Get Output]: \t{} ms", \
    std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
  return true;
}


// ============================================
//            Inference Entrypoint
// ============================================

bool YOLOv8::run(cv::Mat &imgFrame,vector<BoundingBox> bboxlist)
{
  // auto m_logger = spdlog::get("YOLOv8");

  // if (m_estimateTime)
  // {
  //   m_logger->info("\n[YOLOv8 Processing Time]");
  //   m_logger->info("-----------------------------------------");
  // }

  // STEP1: load image to input tensor
  if (!loadInput(imgFrame))
  {
    // m_logger->error("Load Input Data Failed");

    return false;
  }

  // STEP2: run inference
  // auto time_0 = std::chrono::high_resolution_clock::now();

  // m_inference = m_snpe->execute(m_inputTensor.get(), m_outputTensorMap);

  // if (!m_inference)
  // {
  //   m_logger->error("AI Inference Failed");

  //   return false;
  // }

  auto time_1 = std::chrono::high_resolution_clock::now();
  // m_logger->debug("[Inference]: \t{} ms", \
  //   std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));

  // STEP3: post processing
  if (!postProcessing(bboxlist))
  {
    // m_logger->error("AI Post Processing Failed");

    return false;
  }

  // if (m_estimateTime)
  // {
  //   m_logger->info("-----------------------------------------");
  // }

  return true;
}


// ============================================
//                Load Inputs
// ============================================

bool YOLOv8::loadInput(std::string filePath)
{
  // auto m_logger = spdlog::get("YOLOv8");

  // m_logger->debug("\n===================================");
  // m_logger->debug("[File] => {}", filePath);
  // m_logger->debug("===================================");

  // Preprocessing
  preProcessingFile(filePath);

  auto time_0 = std::chrono::high_resolution_clock::now();

  // if (m_inputTensor->getSize() != m_inputBuff.size())
  // {
    // m_logger->error("Size of input does not match network.");
    // m_logger->error("Expecting: {}", m_inputTensor->getSize());
    // m_logger->error("Got: {}", m_inputBuff.size());

  //   return false;
  // }

  /* Copy the loaded input file contents into the networks input tensor.
    SNPE's ITensor supports C++ STL functions like std::copy() */
  // std::copy(m_inputBuff.begin(), m_inputBuff.end(), m_inputTensor->begin());

  auto time_1 = std::chrono::high_resolution_clock::now();

  // m_logger->debug("[Load Input]: \t{} ms", \
  //   std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));

  return true;
}


bool YOLOv8::loadInput(cv::Mat &imgFrame)
{
  // auto m_logger = spdlog::get("YOLOv8");

  // Preprocessing
  if (!preProcessingMemory(imgFrame))
  {
    // m_logger->error("Data Preprocessing Failed");

    return false;
  }

  auto time_0 = std::chrono::high_resolution_clock::now();

  // if (m_inputTensor->getSize() != m_inputBuff.size())
  // {
    // m_logger->error("Size of input does not match network.");
    // m_logger->error("Expecting: {}", m_inputTensor->getSize());
    // m_logger->error("Got: {}", m_inputBuff.size());

  //   return false;
  // }

  /* Copy the loaded input file contents into the networks input tensor.
    SNPE's ITensor supports C++ STL functions like std::copy() */
  // std::copy(m_inputBuff.begin(), m_inputBuff.end(), m_inputTensor->begin());

  auto time_1 = std::chrono::high_resolution_clock::now();

  // m_logger->debug("[Load Input]: \t{} ms", \
  //   std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));

  return true;
}


bool YOLOv8::_loadImageFile(const std::string& inputFile)
{
  // auto m_logger = spdlog::get("YOLOv8");
  auto time_0 = std::chrono::high_resolution_clock::now();

  m_img = cv::imread(inputFile, -1);
  if (m_img.empty())
  {
    // m_logger->error("image don't exist!");
    return false;
  }

  auto time_1 = std::chrono::high_resolution_clock::now();
  // m_logger->debug("[Read Image]: \t{} ms", \
  //   std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));

  return true;
}


// ============================================
//               Pre Processing
// ============================================
bool YOLOv8::preProcessingFile(std::string imgPath)
{
  _loadImageFile(imgPath);
  _imgPreprocessing();
  return true;
}


bool YOLOv8::preProcessingMemory(cv::Mat &imgFrame)
{
  // auto m_logger = spdlog::get("YOLOv8");

  if (imgFrame.empty())
  {
    // m_logger->error("Image don't exists!");
    return false;
  }
  else
  {
    m_img = imgFrame;
  }

  _imgPreprocessing();

  return true;
}


bool YOLOv8::_imgPreprocessing()
{
  // auto m_logger = spdlog::get("YOLOv8");

  int imageSize = m_inputChannel*m_inputHeight*m_inputWidth;
  cv::Size inputSize = cv::Size(m_inputWidth, m_inputHeight);
  cv::Mat imgResized;
  cv::Mat sample;
  cv::Mat sampleNorm;

  auto time_0 = std::chrono::high_resolution_clock::now();

  if (m_img.size() != inputSize)
  {
    cv::resize(m_img, imgResized, inputSize, cv::INTER_LINEAR);
  }
  else
  {
    imgResized = m_img;
  }
  m_imgResized = imgResized;

  // BGR to RGB
  cv::cvtColor(imgResized, sample, cv::COLOR_BGR2RGB);

  // Normalize
  sample.convertTo(sampleNorm, CV_32F, 1.0 / 255, 0);


  std::memcpy(&m_inputBuff[0], sampleNorm.data, imageSize*sizeof(float));

  auto time_1 = std::chrono::high_resolution_clock::now();
  // m_logger->debug("[Pre-Proc]: \t{}",\
  //   std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));

  return true;
}


// ============================================
//               Post Processing
// ============================================

bool YOLOv8::postProcessing(vector<BoundingBox> bboxlist)
{
  // auto m_logger = spdlog::get("YOLOv8");

  if (m_inference == true)
  {
    if(!_getOutputTensor())
    {
      // m_logger->error("Unable to get output tensors!");
      return false;
    }
  }
  else
  {
    // m_logger->error("Error while executing the network.");
    return false;
  }

  // _OD_postProcessing();
  // Alister add 2023-11-22
  _OD_postProcessing(bboxlist);

  return true;
}


// void YOLOv8::_OD_postProcessing_ori()
// {
//     // auto m_logger = spdlog::get("YOLOv8");

//     auto time_0 = std::chrono::high_resolution_clock::now();
//     auto time_1 = std::chrono::high_resolution_clock::now();


//     // if (!(m_detectionBoxBuff && m_detectionConfBuff && m_detectionClsBuff))  // Missing output(s)
//     // {
//     //   m_logger->error("Not all outputs of the network are available");
//     // }

//     // m_logger->debug("Starting object detection post-processing......");

    // m_numBox = m_decoder->decode((float *)m_detectionBoxBuff, 
    //                               (float *)m_detectionConfBuff, 
    //                               (float *)m_detectionClsBuff, 
    //                               confidenceThreshold, 
    //                               iouThreshold, 
    //                               m_yoloOut);

//     // m_logger->debug("[Post-Proc]: \t{}", \
//     //   std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));

//     // m_logger->debug("=> GET # of raw BBOX(es): {}", m_numBox);
//     for(int i=0; i< m_numBox; i++)
//     {
//       struct v8xyxy b = m_yoloOut[i];
//       // m_logger->debug("=> bbx {}: ({},{})-({},{}), c={}, conf={}", i, b.x1, b.y1, b.x2, b.y2, b.c, b.c_prob);
//     }

//     // m_logger->debug("Finished object detection post-processing");

//     if (m_estimateTime)
//     {
//       time_1 = std::chrono::high_resolution_clock::now();
//       // m_logger->info("[_OD_postProcessing]: \t{} ms", \
//       //   std::chrono::duration_cast<std::chrono::nanoseconds>(time_1 - time_0).count() / (1000.0 * 1000));
//     }
// }
// Alister add 2023-11-22
void YOLOv8::_OD_postProcessing(vector<BoundingBox> bboxlist)
{
  struct vector<v8xyxy> m_yoloOut;
  for (unsigned int i = 0; i < bboxlist.size(); i++)
  { 
    v8xyxy bb;
    bb.x1 = bboxlist[i].x1;
    bb.y1 = bboxlist[i].y1;
    bb.x2 = bboxlist[i].x2;
    bb.y2 = bboxlist[i].y2;
    bb.c  = bboxlist[i].label;
    bb.c_prob  = 1.0;
    m_yoloOut.push_back(bb);
  }
  m_numBox = m_yoloOut.size();
  // m_logger->debug("=> GET # of raw BBOX(es): {}", m_numBox);

  for(int i=0; i< m_numBox; i++)
  {
    struct v8xyxy b = m_yoloOut[i];
    // m_logger->debug("=> bbx {}: ({},{})-({},{}), c={}, conf={}", i, b.x1, b.y1, b.x2, b.y2, b.c, b.c_prob);
  }

}


void YOLOv8::_rescaleBoundingBox(
    int bbx_num,
    struct v8xyxy *out,
    struct v8xyxy *scaledOut,
    int inputW, int inputH,
    int frameW, int frameH)
{
  float w_ratio = (float)inputW / (float)frameW;
  float h_ratio = (float)inputH / (float)frameH;

  for(int i=0; i<bbx_num; i++)
  {
    scaledOut[i].c = out[i].c;
    scaledOut[i].c_prob = out[i].c_prob;
    scaledOut[i].x1 = (int)((float)out[i].x1 / w_ratio);
    scaledOut[i].y1 = (int)((float)out[i].y1 / h_ratio);
    scaledOut[i].x2 = (int)((float)out[i].x2 / w_ratio);
    scaledOut[i].y2 = (int)((float)out[i].y2 / h_ratio);

    // Expand Bounding Box
    int w = scaledOut[i].x2 - scaledOut[i].x1;
    int h = scaledOut[i].y2 - scaledOut[i].y1;

    int c_x = scaledOut[i].x1 + (int)((float)w / 2.0);
    int c_y = scaledOut[i].y1 + (int)((float)h / 2.0);
    w = w * (m_bboxExpandRatio+0.15);
    h = h * m_bboxExpandRatio;

    scaledOut[i].x1 = c_x - (int)((float)w / 2.0);
    scaledOut[i].y1 = c_y - (int)((float)h / 2.0);
    scaledOut[i].x2 = c_x + (int)((float)w / 2.0);
    scaledOut[i].y2 = c_y + (int)((float)h / 2.0);

    if (scaledOut[i].x1 < 0)
      scaledOut[i].x1 = 0;
    if (scaledOut[i].x2 > frameW-1)
      scaledOut[i].x2 = frameW-1;
    if (scaledOut[i].y1 < 0)
      scaledOut[i].y1 = 0;
    if (scaledOut[i].y2 > frameH-1)
      scaledOut[i].y2 = frameH-1;
  }
}


float YOLOv8::_getBboxOverlapRatio(BoundingBox &boxA, BoundingBox &boxB)
{
  int iouX = max(boxA.x1, boxB.x1);
  int iouY = max(boxA.y1, boxB.y1);
  int iouW = min(boxA.x2, boxB.x2) - iouX;
  int iouH = min(boxA.y2, boxB.y2) - iouY;
  iouW = max(iouW, 0);
  iouH = max(iouH, 0);

  if (boxA.getArea() == 0)
    return 0;

  float iouArea = iouW * iouH;
  float ratio = iouArea / (float)boxA.getArea();

  return ratio;
}


void YOLOv8::_bboxMerging(BoundingBox &bboxA, BoundingBox &bboxB, int label, BoundingBox &bboxMerge)
{
  int newX1 = 0;
  int newY1 = 0;
  int newX2 = 0;
  int newY2 = 0;

  vector<BoundingBox> tmpBboxList;
  tmpBboxList.push_back(bboxA);
  tmpBboxList.push_back(bboxB);

  vector<vector<Point>> cornerPointList;
  for (int i=0; i<(int)tmpBboxList.size(); i++)
  {
    cornerPointList.push_back(tmpBboxList[i].getCornerPoint());
  }

  // 0:TL, 1:TR, 2:BL, 3:BR
  vector<int> x1List;
  vector<int> y1List;
  vector<int> x2List;
  vector<int> y2List;
  for (int i=0; i<(int)cornerPointList.size(); i++)
  {
    x1List.push_back(cornerPointList[i][0].x);
    y1List.push_back(cornerPointList[i][0].y);
    x2List.push_back(cornerPointList[i][3].x);
    y2List.push_back(cornerPointList[i][3].y);
  }

  vector<int>::iterator x1It;
  vector<int>::iterator x2It;
  vector<int>::iterator y1It;
  vector<int>::iterator y2It;

  x1It = std::min_element(x1List.begin(), x1List.end());
  x2It = std::max_element(x2List.begin(), x2List.end());

  y1It = std::min_element(y1List.begin(), y1List.end());
  y2It = std::min_element(y2List.begin(), y2List.end());

  // newX1 = int(((*x1It_min) + (*x1It_max)) * 0.5);
  // newX2 = int(((*x2It_min) + (*x2It_max)) * 0.5);
  newX1 = (*x1It);
  newX2 = (*x2It);

  newY1 = (*y1It);
  newY2 = (*y2It);

  bboxMerge.x1 = newX1;
  bboxMerge.y1 = newY1;
  bboxMerge.x2 = newX2;
  bboxMerge.y2 = newY2;
  bboxMerge.label = label;
}


// ============================================
//                  Outputs
// ============================================

bool YOLOv8::getVehicleBoundingBox(
                                  vector<BoundingBox> &_outBboxList,
                                  float confidence,
                                  int videoWidth, 
                                  int videoHeight,
                                  std::vector<v8xyxy> m_yoloOut)
                                  // BoundingBox &fcwROI)
{
  // auto m_logger = spdlog::get("YOLOv8");

  // Clear previous bounding boxes
  _outBboxList.clear();

  // m_logger->debug("Get vehicle box => m_numBox = {}", m_numBox);

  // Point pROI_TL = fcwROI.getCornerPoint()[0];
  // Point pROI_TR = fcwROI.getCornerPoint()[1];

  float wRatio = (float)m_inputWidth / (float)videoWidth;
  float hRatio = (float)m_inputHeight / (float)videoHeight;
 
  for(int i=0; i<m_yoloOut.size(); i++)
  {
    v8xyxy box = m_yoloOut[i];
    if ((box.c == VEHICLE) && (box.c_prob >= confidence))
    {
      // m_logger->debug("Get vehicle box [{}] : ({}, {}, {}, {}, {}, {})", \
      //   i, box.x1, box.y1, box.x2, box.y2, box.c, box.c_prob);
      BoundingBox bbox(box.x1, box.y1, box.x2, box.y2, box.c);
      
      float bboxWidth = (float)bbox.getWidth()/wRatio;
      float bboxHeight = (float)bbox.getHeight()/hRatio;

      // filter out outliers
      if ((bboxWidth > (float)videoWidth*0.8) || (bboxHeight > (float)videoHeight*0.8))
      {
        // m_logger->debug("filter out outliers - (1)");
        // m_logger->debug("bboxWidth > videoWidth*0.8) || (bboxHeight > videoHeight*0.8)");
        continue;
      }
      else if (bboxWidth < 15 || bboxHeight < 10)
      {
        // m_logger->debug("filter out outliers - (2)");
        // m_logger->debug("bboxWidth < 15 || bboxHeight < 10");
        cout<<"bboxWidth < 15 || bboxHeight < 10"<<endl;
        continue;
      }
      else if (bbox.getAspectRatio() < 0.4)
      {
        // m_logger->debug("filter out outliers - (3)");
        // m_logger->debug("bboxA.getAspectRatio() < 0.4");
        cout<<"bbox.getAspectRatio() < 0.4"<<endl;
        continue;
      }

      // filter out vehicles that out of ROI
      // Point cp = bbox.getCenterPoint();
      // if (cp.x/wRatio < pROI_TL.x || cp.x/wRatio > pROI_TR.x)
      // {
      //   m_logger->debug("cp = ({}, {})", cp.x, cp.y);
      //   m_logger->debug("pROI_TL = ({}, {})", pROI_TL.x, pROI_TL.y);
      //   m_logger->debug("pROI_TR = ({}, {})", pROI_TR.x, pROI_TR.y);
      //   m_logger->debug("filter out outliers - (3)");
      //   m_logger->debug("Out of FCW ROI");
      //   continue;
      // }


      bbox.confidence = box.c_prob;
      _outBboxList.push_back(bbox);
    }
    // else if (box.c_prob < confidence)
    // {
    //   m_logger->debug("Get vehicle box [{}] : ({}, {}, {}, {}, {}, {})", \
    //     i, box.x1, box.y1, box.x2, box.y2, box.c, box.c_prob);
    // }
  }

  return true;
}


bool YOLOv8::getMotorbikeBoundingBox(
                                  vector<BoundingBox> &_outBboxList,
                                  float confidence,
                                  int videoWidth, 
                                  int videoHeight,
                                  std::vector<v8xyxy> m_yoloOut)
//                                   BoundingBox &fcwROI)
{
  // auto m_logger = spdlog::get("YOLOv8");

  // Clear previous bounding boxes
  _outBboxList.clear();

  //
  // m_logger->debug("Get rider box => m_numBox = {}", m_numBox);

  // Point pROI_TL = fcwROI.getCornerPoint()[0];
  // Point pROI_TR = fcwROI.getCornerPoint()[1];

  float wRatio = (float)m_inputWidth / (float)videoWidth;
  float hRatio = (float)m_inputHeight / (float)videoHeight;

  //
  vector<BoundingBox> tmpBboxList;

  for(int i=0; i<m_yoloOut.size(); i++)
  {
    v8xyxy box = m_yoloOut[i];
    if ((box.c == MOTORBIKE) && (box.c_prob >= confidence))  // Rider class
    {
      // m_logger->debug("Get rider box [{}] : ({}, {}, {}, {}, {}, {})", \
      //   i, box.x1, box.y1, box.x2, box.y2, box.c, box.c_prob);

      BoundingBox bbox(box.x1, box.y1, box.x2, box.y2, box.c);

      // filter out vehicles that out of ROI
      // Point cp = bbox.getCenterPoint();
      // if (cp.x/wRatio < pROI_TL.x || cp.x/wRatio > pROI_TR.x)
      // {
      //   m_logger->debug("filter out outliers - (1)");
      //   m_logger->debug("Out of FCW ROI");
      //   continue;
      // }

      // TODO: Aspect Ratio

      BoundingBox bboxRider(box.x1, box.y1, box.x2, box.y2, box.c);

      // Merge human box
      for (int j=0; j<m_numBox; j++)
      {
        if (j!=i)
        {
          v8xyxy boxB = m_yoloOut[j];
          BoundingBox bboxB(boxB.x1, boxB.y1, boxB.x2, boxB.y2, boxB.c);

          if (boxB.c == HUMAN)
          {
            int areaB = bboxB.getArea();
            float overlapRatio = _getBboxOverlapRatio(bbox, bboxB);

            if (overlapRatio > 0.1)
            {
              _bboxMerging(bbox, bboxB, 3, bboxRider); //TODO:
            }
          }
        }
      }
      bboxRider.confidence = box.c_prob;
      tmpBboxList.push_back(bboxRider);
    }
  }


  for(int i=0; i<tmpBboxList.size(); i++)
  {
    int areaA = tmpBboxList[i].getArea();

    bool keepThisRider = true;

    for (int j=0; j<tmpBboxList.size(); j++)
    {
      if (j!=i)
      {
        int areaB = tmpBboxList[j].getArea();

        float overlapRatio = _getBboxOverlapRatio(tmpBboxList[i], tmpBboxList[j]);

        if ((overlapRatio > 0.1) && (areaB < areaA))
          keepThisRider = false;
      }
    }

    if (keepThisRider)
      _outBboxList.push_back(tmpBboxList[i]);
  }

  return true;
}


bool YOLOv8::getBikeBoundingBox(
                            vector<BoundingBox> &_outBboxList,
                            float confidence,
                            int videoWidth, 
                            int videoHeight,
                            std::vector<v8xyxy> m_yoloOut)
                            // BoundingBox &fcwROI)
{
  // auto m_logger = spdlog::get("YOLOv8");

  // Clear previous bounding boxes
  _outBboxList.clear();

  //
  // m_logger->debug("Get rider box => m_numBox = {}", m_numBox);

  // Point pROI_TL = fcwROI.getCornerPoint()[0];
  // Point pROI_TR = fcwROI.getCornerPoint()[1];

  float wRatio = (float)m_inputWidth / (float)videoWidth;
  float hRatio = (float)m_inputHeight / (float)videoHeight;

  //
  vector<BoundingBox> tmpBboxList;

  for(int i=0; i<m_yoloOut.size(); i++)
  {
    v8xyxy box = m_yoloOut[i];
    if ((box.c == MOTORBIKE) && (box.c_prob >= confidence))  // Rider class
    {
      // m_logger->debug("Get rider box [{}] : ({}, {}, {}, {}, {}, {})", \
      //   i, box.x1, box.y1, box.x2, box.y2, box.c, box.c_prob);

      BoundingBox bbox(box.x1, box.y1, box.x2, box.y2, box.c);

      // filter out vehicles that out of ROI
      // Point cp = bbox.getCenterPoint();
      // if (cp.x/wRatio < pROI_TL.x || cp.x/wRatio > pROI_TR.x)
      // {
      //   m_logger->debug("filter out outliers - (1)");
      //   m_logger->debug("Out of FCW ROI");
      //   continue;
      // }

      // TODO: Aspect Ratio

      BoundingBox bboxRider(box.x1, box.y1, box.x2, box.y2, box.c);

      // Merge human box
      for (int j=0; j<m_numBox; j++)
      {
        if (j!=i)
        {
          v8xyxy boxB = m_yoloOut[j];
          BoundingBox bboxB(boxB.x1, boxB.y1, boxB.x2, boxB.y2, boxB.c);

          if (boxB.c == HUMAN)
          {
            int areaB = bboxB.getArea();
            float overlapRatio = _getBboxOverlapRatio(bbox, bboxB);

            if (overlapRatio > 0.1)
            {
              _bboxMerging(bbox, bboxB, 3, bboxRider); //TODO:
            }
          }
        }
      }
      bboxRider.confidence = box.c_prob;
      tmpBboxList.push_back(bboxRider);
    }
  }


  for(int i=0; i<tmpBboxList.size(); i++)
  {
    int areaA = tmpBboxList[i].getArea();

    bool keepThisRider = true;

    for (int j=0; j<tmpBboxList.size(); j++)
    {
      if (j!=i)
      {
        int areaB = tmpBboxList[j].getArea();

        float overlapRatio = _getBboxOverlapRatio(tmpBboxList[i], tmpBboxList[j]);

        if ((overlapRatio > 0.1) && (areaB < areaA))
          keepThisRider = false;
      }
    }

    if (keepThisRider)
      _outBboxList.push_back(tmpBboxList[i]);
  }

  return true;
}


bool YOLOv8::getHumanBoundingBox(
                                vector<BoundingBox> &_outBboxList,
                                float confidence,
                                int videoWidth, 
                                int videoHeight,
                                std::vector<v8xyxy> m_yoloOut
                               )
{
  // auto m_logger = spdlog::get("YOLOv8");

  // Clear previous bounding boxes
  _outBboxList.clear();
  // m_logger->debug("Get human box => m_numBox = {}", m_numBox);
  // Point pROI_TL = fcwROI.getCornerPoint()[0];
  // Point pROI_TR = fcwROI.getCornerPoint()[1];
  float wRatio = (float)m_inputWidth / (float)videoWidth;
  float hRatio = (float)m_inputHeight / (float)videoHeight;
  // for(int i=0; i<m_numBox; i++)
  // for(int i=0; i<m_yoloOut.size(); i++)
  for(int i=0; i<m_yoloOut.size(); i++)
  {
    v8xyxy box = m_yoloOut[i];
    if ((box.c == HUMAN) && (box.c_prob >= confidence))
    {
      // m_logger->debug("Get human box [{}] : ({}, {}, {}, {}, {}, {})", \
      //   i, box.x1, box.y1, box.x2, box.y2, box.c, box.c_prob);
      cout<<"box.x1:"<<box.x1<<endl;
      cout<<"box.y1:"<<box.y1<<endl;
      cout<<"box.x2:"<<box.x2<<endl;
      cout<<"box.y2:"<<box.y2<<endl;
      cout<<"box.c:"<<box.c<<endl;
      BoundingBox bbox(box.x1, box.y1, box.x2, box.y2, box.c);
      // filter out vehicles that out of ROI
      // Point cp = bbox.getCenterPoint();
      // if (cp.x/wRatio < pROI_TL.x || cp.x/wRatio > pROI_TR.x)
      // {
      //   m_logger->debug("filter out outliers - (1)");
      //   m_logger->debug("Out of FCW ROI");
      //   continue;
      // }

      // Aspect Ratio TODO:

      bbox.confidence = box.c_prob;
     
      cout<<"box.c_prob:"<<box.c_prob<<endl;
      _outBboxList.push_back(bbox);
      cout<<"_outBboxList.size()="<<_outBboxList.size()<<endl;
    }
  }
  return true;
}

// ============================================
//                  Others
// ============================================

void YOLOv8::debugON()
{
  m_debugMode = true;
}


void YOLOv8::showProcTime()
{
  m_estimateTime = true;
}