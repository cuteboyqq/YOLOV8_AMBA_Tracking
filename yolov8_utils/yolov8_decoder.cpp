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

#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "yolov8.hpp"

using namespace cv;
using namespace std;


bool xyxyobj_compare(v8xyxy a, v8xyxy b)
{
    return a.c_prob > b.c_prob; // decreasing order
}


YOLOv8_Decoder::YOLOv8_Decoder(int inputH, int inputW)
{
    m_inputH = inputH;
    m_inputW = inputW;
};

YOLOv8_Decoder::~YOLOv8_Decoder()
{};


unsigned int YOLOv8_Decoder::decode(const float *m_detection_buff, const float confThreshold, const float iouThreshold, struct v8xyxy out[])
{
    vector<v8xyxy> bboxlist;
    vector<v8xyxy> picked;

    // YOLOv8 Decode
    getCandidates((float *)m_detection_buff, confThreshold, bboxlist);
    doNMS(bboxlist, iouThreshold, picked);

    for(unsigned int i=0; i<picked.size(); i++){
        out[i] = picked[i];

        // handle the case when the total picked bbx number is larger than or equal to maximum allowable BBX Num
        if(i==99)
            return i;
    }
    return picked.size();
}


unsigned int YOLOv8_Decoder::decode(const float *m_detection_box_buff, 
                                    const float *m_detection_conf_buff, 
                                    const float *m_detection_class_buff,
                                    const float confThreshold, 
                                    const float iouThreshold, 
                                    struct v8xyxy out[])
{
    vector<v8xyxy> bboxlist;
    vector<v8xyxy> picked;

    // YOLOv8 Decode
    getCandidates((float *)m_detection_box_buff, 
                    (float*)m_detection_conf_buff, 
                    (float *)m_detection_class_buff, 
                    confThreshold, 
                    bboxlist);
    doNMS(bboxlist, 
          iouThreshold, 
          picked);

    for(unsigned int i=0; i<picked.size(); i++){
        out[i] = picked[i];

        // handle the case when the total picked bbx number is larger than or equal to maximum allowable BBX Num
        if(i==99)
            return i;
    }
    return picked.size();
}



v8xyxy YOLOv8_Decoder::decodeBoundingBox(bbxv8_candidate c)
{
    /***
    Input:
        c (bbxv8_candidate): candidate box extracted from detection output, in xywh coordinates.
    Output:
        box (box): processed box in xyxy (top-left, right-bottom) coordinates.
    ***/

    float x,y,w,h;
    float bx,by,bw,bh;
    v8xyxy box;

    x = c.x;
    y = c.y;
    w = c.w;
    h = c.h;
    // x = sigmoid(c.x);
    // y = sigmoid(c.y);
    // w = sigmoid(c.w);
    // h = sigmoid(c.h);

    box.x1 = x;
    box.y1 = y;
    box.x2 = w;
    box.y2 = h;

    // box.x1 = x;
    // box.y1 = y;
    // box.x2 = w;
    // box.y2 = h;

    box.c = c.c;
    box.c_prob = c.c_prob;

    // cout << "box xyxy is " << box.x1 << " " << box.y1 << " " << box.x2 << " " << box.y2 <<
    // " " << box.c << " " << box.c_prob << endl;
    return box;
}


int YOLOv8_Decoder::genBoundingBox(const vector<bbxv8_candidate> &det_list, vector<v8xyxy> &bbox_list)
{
    /***
    Input:
        &det_list (vector of bbxv8_candidate): pointer to the list containing candidates boxes
    Output:
        &bbox_list (vector of v8xyxy box): pointer to the list containing boxes in v8xyxy format
    ***/

    for (auto it = det_list.begin(); it != det_list.end(); it++)
    {
        v8xyxy box = decodeBoundingBox(*it);
        bbox_list.push_back(box);
    }
    return 0;
}


int YOLOv8_Decoder::getCandidates(float *detection, float conf_thr, vector<v8xyxy> &bbox_list)
{
    /***
    *  get_candidates: get the bounding box with objectiveness > confidence threshold
    *    inputs:
    *      detection : pointer to the detection output buffer: converted to float *
    *      conf_thr: minimal objectivess threshlod to pick-up the candidates
    *
    *    output:
    *      bboxlist: the selected bbox candidates in xyxy format
    *
    ***/
    // // int count = 0;
    // // const int num_loc = 4; // x, y, w, h, class scores
    // const int cutoff = 20160;
    // const int num_val = 14;
    int bbx = 0;
    vector<bbxv8_candidate> detList;

    // NOTE: detection output is in raw shape due to failed compability with SNPE converter:
    // the bbox is serialized to first 4 * 5040 elements of (detection) and the class scores
    // is serialized to the remaining 10*5040 elements of (detection)


    // cout << "aaa " << endl;

    for (int i = 0; i < NUM_BBOX; i++)  // each box
    {
        // int c_begin = i + (NUM_BBOX*4);

        // // cout << "detection sample: " << detection[loc_begin] << " " << detection[loc_begin+1] << " " << detection[loc_begin+2] << " "
        // // << detection[loc_begin+3] << " " << detection[c_begin] << " " << detection[c_begin+1] << " " << detection[c_begin+2] << " " << detection[c_begin+3]
        // // << " " << detection[c_begin+4] << endl;
        // // cout << "Checking: " << detection[20159] << " and " << detection[20160] << endl;
        // // if (i> 20160 - 20 and i < 20160) cout << "detection: " << detection[i] << endl;
        // // if (i > 20160)
        // // {
        // //     cout << "detection: " << detection[i] << " " << detection[i+1] << " "
        // //     << detection[i+2] << " " << detection[i+3] << " " << detection[i+4] << " " << detection[i+5]
        // //     << " " << detection[i+6] << endl;
        // //     cin >> count;
        // // }
        // // count++;
        // // cout << "count: " << count << endl;
        // int c_idx = argmax(detection+c_begin, num_classes_det);  // Finding the class with highest confidence TOOD: Optimize
        // float c_prob = detection[c_begin + c_idx*NUM_BBOX];
        // // c_prob = sigmoid(c_prob);
        // // cout << "c_prob: " << c_prob << endl;
        // // cout << "c_idx: " << c_idx << endl;

        int c_begin = i + (NUM_BBOX*5);
        float c_prob = detection[NUM_BBOX*4 + i];

        // cout << "c_prob = " << c_prob << endl;

        if (c_prob > conf_thr)  // If class score higher than theshold
        {
            bbx++;
            bbxv8_candidate candidate;
            candidate.x = detection[i];
            candidate.y = detection[NUM_BBOX + i];
            candidate.w = detection[NUM_BBOX*2 + i];
            candidate.h = detection[NUM_BBOX*3 + i];
            // candidate.c = c_idx;
            candidate.c = argmax(detection+c_begin, NUM_DET_CLASSES);
            candidate.c_prob = c_prob;
            // cout << "loc_begin " << loc_begin << endl;
            cout << "c_begin " << c_begin << endl;
            cout << "decoding xywh is " << candidate.x << " " << candidate.y << " " << candidate.w << " " << candidate.h << 
            " " << candidate.c << " " << candidate.c_prob << endl;
            detList.push_back(candidate);
        }
    }

    genBoundingBox(detList, bbox_list);

    return bbx;
}


int YOLOv8_Decoder::getCandidates(
    float *detectionBox, float *detectionConf, float *detectionClass, float conf_thr, vector<v8xyxy> &bbox_list)
{
    int bbx = 0;
    vector<bbxv8_candidate> detList;

    // NOTE: detection output is in raw shape due to failed compability with SNPE converter:
    // the bbox is serialized to first 4 * 5040 elements of (detection) and the class scores
    // is serialized to the remaining 10*5040 elements of (detection)

    // box: x, y, w, h
    // cls: c1, c2, c3, c4, c5, c6

    for (int i = 0; i < NUM_BBOX; i++)  // each box
    {

        // int c_begin = i + (NUM_BBOX*5);
        // float c_prob = detection[NUM_BBOX*4 + i];
        float c_prob = detectionConf[i];


        // if (i < 5)
        // {
        //     cout << "candidate[" << i << "]:" << endl;
        //     cout << "x = " << detectionBox[i] << endl;
        //     cout << "y = " << detectionBox[NUM_BBOX + i] << endl;
        //     cout << "w = " << detectionBox[NUM_BBOX*2 + i] << endl;
        //     cout << "h = " << detectionBox[NUM_BBOX*3 + i] << endl;
        //     cout << "c prob = " << detectionConf[i] << endl;
        //     cout << "c idx = " << detectionClass[i] << endl << endl;
        // }


        if (c_prob > conf_thr)  // If class score higher than theshold
        {
            bbx++;
            bbxv8_candidate candidate;
            candidate.x = detectionBox[i];
            candidate.y = detectionBox[NUM_BBOX + i];
            candidate.w = detectionBox[NUM_BBOX*2 + i];
            candidate.h = detectionBox[NUM_BBOX*3 + i];
            // candidate.c = c_idx;
            candidate.c = detectionClass[i];
            candidate.c_prob = detectionConf[i];
            // cout << "loc_begin " << loc_begin << endl;
            // cout << "c_begin " << c_begin << endl;
            // cout << "decoding xywh is " << candidate.x << " " << candidate.y << " " << candidate.w << " " << candidate.h << 
            // " " << candidate.c << " " << candidate.c_prob << endl;
            detList.push_back(candidate);
        }

    }

    genBoundingBox(detList, bbox_list);

    return bbx;
}


float YOLOv8_Decoder::iou(v8xyxy a, v8xyxy b)
{
    /***
    *  iou: return the intersection over union ratio between rectangle a and b
    ***/

    int area_a = (a.x2 - a.x1)*(a.y2 - a.y1);
    int area_b = (b.x2 - b.x1)*(b.y2 - b.y1);
    int intersection_w = max(0, min(a.x2, b.x2)-max(a.x1, b.x1));
    int intersection_h = max(0, min(a.y2, b.y2)-max(a.y1, b.y1));
    int intersection_area = intersection_w * intersection_h;
    float iou = (float)intersection_area / (float)(area_a + area_b - intersection_area);
    return iou;
}


float YOLOv8_Decoder::getBboxOverlapRatio(v8xyxy boxA, v8xyxy boxB)
{
    int iouX = max(boxA.x1, boxB.x1);
    int iouY = max(boxA.y1, boxB.y1);
    int iouW = min(boxA.x2, boxB.x2) - iouX;
    int iouH = min(boxA.y2, boxB.y2) - iouY;
    iouW = max(iouW, 0);
    iouH = max(iouH, 0);

    float areaBoxA = (boxA.x2-boxA.x1) * (boxA.y2-boxA.y1);
    if (areaBoxA <= 0)
        areaBoxA = 1;

    float iouArea = iouW * iouH;
    float ratio = iouArea / areaBoxA;

    return ratio;
}


int YOLOv8_Decoder::doNMS(vector<v8xyxy> &bboxlist, const float iou_thr, vector<v8xyxy> &picked)
{
    /***
    *  do_nms : apply non-maximum-suppresion algorithm
    *    inputs:
    *      bboxlist: the content of bboxlist will be modified afer applying do_nms() (with side-effect)
    *      iou_thr: the iou threshold
    *
    *    outputs:
    *      picked: the picked bounding boxes after applying NMS
    *
    ***/

    // sort the bboxlist based on its class probability, in non-increasing order
    sort(bboxlist.begin(), bboxlist.end(), xyxyobj_compare);

    for (int cls = 0; cls < NUM_DET_CLASSES; cls++)
    {
        for (unsigned int i = 0; i < bboxlist.size(); i++)
        {
            bool keep = true;
            
            if (bboxlist[i].c != cls)      // not target class, skip it to the next cls round
                continue;
            
            for (unsigned int j = 0; j < picked.size(); j++)
            {
                if (picked[j].c != cls)   // not target class, skip it
                    continue;

                if (iou(bboxlist[i], picked[j]) >= iou_thr)
                {
                    bboxlist[i].c_prob = 0; // reset score to 0
                    keep = false;
                    break;
                }
                
                //TODO:
                else if (getBboxOverlapRatio(bboxlist[i], picked[j]) >= 0.6)
                {
                    bboxlist[i].c_prob = 0; // reset score (obj) to 0
                    keep = false;
                    break;
                }
                else   // < iou_thr
                { 
                    keep = true;
                }
            }
            if (keep)
                picked.push_back(bboxlist[i]);
        }
    }

    return picked.size();
}


float YOLOv8_Decoder::sigmoid(float x)
{
    float result;
    result = 1 / (1 + exp(-x));
    return result;
}


int YOLOv8_Decoder::argmax(float cls_begin[], int n)
{
    float max_val = 0.0;
    int max_cls = 0;
    for (int i = 0; i < n; i++)
    {
        int c_idx = i * NUM_BBOX;
        float s = cls_begin[c_idx];
        if (s >= max_val)
        {
            max_val = s;
            max_cls = i;
        }
    }
    return max_cls;
}