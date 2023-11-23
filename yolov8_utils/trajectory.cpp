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
#include "trajectory.hpp"


/////////////////////////
// public member functions
////////////////////////
Trajectory::Trajectory(Config_S *_config)
{
  // === initialize parameters === //
  videoWidth = _config->frameWidth;
  videoHeight = _config->frameHeight;
  modelWidth = _config->modelWidth;
  modelHeight = _config->modelHeight;
  birdWidth = 400;
  birdHeight = 400;
  frameInterval = 1; //_config->procFrameStep;

  // Threshold
  tYCloseDoorbell = static_cast<int>(birdHeight*0.98);
  pOrig = Point(static_cast<int>(birdWidth*0.5), birdHeight);
  Point pCenter = Point(static_cast<int>(birdWidth*0.5), 0);

  double pi = 3.14159265359;
  for (int degree=-180; degree<181; degree++)
  {
    double theta = (degree * (pi / 180));
    Point p = Point(-1, -1);
    p.x = pCenter.x;
    p.y = pCenter.y;
    _rotatePoint(p, theta);

    pList.push_back(p);
  }

  //m_logMsg = "[INFO] Create a Sleep Time Estimator";
};


Trajectory::~Trajectory() {};


void Trajectory::_rotatePoint(Point &p, double angle)
{
  int oX = pOrig.x;
  int oY = pOrig.y;
  int pX = p.x;
  int pY = p.y;

  double qX = oX + (cos(angle) * (pX - oX)) - (sin(angle) * (pY - oY));
  double qY = oY + (sin(angle) * (pX - oX)) + (cos(angle) * (pY - oY));

  p.x = static_cast<int>(qX);
  p.y = static_cast<int>(qY);
}


void Trajectory::_applyMovingAverage(
  vector<int> &list, vector<int> &listMA, int windowSize)
{
  float s = 0;
  vector<int> cumsumList;
  vector<int> tmpCumsumList;
  int listLen = list.size();

  // SMA- Simple Moving Average, the following is an efficient O(3n) algorithm,
  //.better than Brute Force algorithm (O(n^2))

  // Step1: calculate culmulate sum
  for(auto it=list.begin(); it!=list.end(); it++)
  {
    s += (*it);
    tmpCumsumList.push_back(s);
  }

  // Step2: padding and subtracting
  for (int i=0; i < windowSize; i++)
    cumsumList.push_back(tmpCumsumList[i]);
  for (int i=0; i < listLen-windowSize; i++ )
    cumsumList.push_back(tmpCumsumList[i+windowSize]-tmpCumsumList[i]);

  // Step3: calculate average
  for (int i=0; i < listLen-windowSize+1; i++)
    listMA.push_back(static_cast<int>(cumsumList[i+windowSize-1]/windowSize));
}


vector<Point> Trajectory::_postProcessing(vector<Point> &pTrajectory)
{
  vector<Point> finalPTrajectory;
  vector<int> xList;
  vector<int> yList;
  vector<int> xListMA;
  vector<int> yListMA;

  for (auto it=pTrajectory.begin(); it!=pTrajectory.end(); it++)
  {
    xList.push_back((*it).x);
    yList.push_back((*it).y);
  }
  // cout << "[DBG] >>> before _applyMovingAverage" << endl;
  _applyMovingAverage(xList, xListMA, maWindowSize);
  // cout << "[DBG] >>> after _applyMovingAverage" << endl;
  _applyMovingAverage(yList, yListMA, maWindowSize);
  // cout << "[DBG] >>> after _applyMovingAverage" << endl;
  for (int i=0; i<(int)xListMA.size(); i++)
  {
    Point p = Point(xListMA[i], yListMA[i]);
    finalPTrajectory.push_back(p);
  }

  return finalPTrajectory;
}


void Trajectory::_bboxPreprocessing(
  vector<BoundingBox> &bboxList, vector<BoundingBox> &procBboxList, int windowSize)
{
  int numBbox = bboxList.size();
  int halfWindowSize = (int)(windowSize*0.5);

  for (int i=halfWindowSize; i<numBbox-halfWindowSize; i+=frameInterval)
  {
    int newX1 = 0;
    int newY1 = 0;
    int newX2 = 0;
    int newY2 = 0;

    vector<BoundingBox> tmpBboxList;
    vector<int> frameStampList;
    for (int j=i-halfWindowSize; j<=i+halfWindowSize; j++)
    {
      tmpBboxList.push_back(bboxList[j]);
      frameStampList.push_back(bboxList[j].frameStamp);
    }

    vector<int> frameIntervalList;
    frameIntervalList.push_back(0);   // 1st frame interval = 0
    for (int i=1; i<windowSize; i++)
    {
      int interval = frameStampList[i] - frameStampList[i-1];
      frameIntervalList.push_back(interval);
    }

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
      int frameInterval = frameIntervalList[i];
      if (frameInterval > maxFrameInterval)
      {
        break;  // ignore remain bounding boxes ...
      }
      x1List.push_back(cornerPointList[i][0].x);
      y1List.push_back(cornerPointList[i][0].y);
      x2List.push_back(cornerPointList[i][3].x);
      y2List.push_back(cornerPointList[i][3].y);
    }

    vector<int>::iterator x1It;
    vector<int>::iterator y1It;
    vector<int>::iterator x2It;
    vector<int>::iterator y2It;

    x1It = std::max_element(x1List.begin(), x1List.end());
    y1It = std::min_element(y1List.begin(), y1List.end());
    x2It = std::min_element(x2List.begin(), x2List.end());
    y2It = std::max_element(y2List.begin(), y2List.end());

    newX1 = (*x1It);
    newY1 = (*y1It);
    newX2 = (*x2It);
    newY2 = (*y2It);

    BoundingBox bbox = BoundingBox(newX1, newY1, newX2, newY2, bboxList[0].label);
    procBboxList.push_back(bbox);
  }
}


Point Trajectory::bboxToPointSimple(BoundingBox &bbox)
{
  Point pBirdCenter = Point(static_cast<int>(birdWidth*0.5), 0);
  Point pCenter = bbox.getCenterPoint();
  float humanWidth = (float)bbox.getWidth();
  float humanHeight = (float)bbox.getHeight();
  float aspectRatio = (float)bbox.getAspectRatio();

  float dist = (-0.0319 * (float)humanHeight) + 10.8;
  float hRatio = 1 - (dist / 13.0); // TODO: assume max valid distance = 12 meters
  float wRatio = humanWidth / (float)videoWidth;

  float pX = ((pCenter.x / (float)videoWidth) * (float)birdWidth);
  float pY = (hRatio * (float)birdHeight);

  // Step1. Detect non full body
  if ((aspectRatio >= tAspectFall) && (aspectRatio < tAspectFullBody) && (wRatio > tWidthRatioCloseLv1))
  {
    humanHeight = ((tAspectFullBody-wRatio) * humanWidth);
    hRatio = humanHeight / (float)videoHeight;
    float newY = (hRatio * (float)birdHeight);

    // if new y change too much and not close to doorbell
    if (((newY / pY) > tNewY) && (wRatio < tWidthRatioCloseLv2))
    {
      if (pY > tYCloseDoorbell)
      {
        pY = tYCloseDoorbell;
      }
      else
      {
        pY = newY;
      }
    }
  }

  // Fall Down
  else if (aspectRatio < tAspectFall)
  {
    hRatio = humanWidth / (float)videoHeight;
    pY = (hRatio * (float)birdHeight);
  }

  // Step2. X calibration if difference changes too much
  // When person is closing to doorbell,
  // the x difference needs to be adjusted more

  // Find cross point between line and circle and adjust x
  float yRatio = pY / (float)birdHeight;
  for (int i=0; i<(int)pList.size(); i++)
  {
    Point p = pList[i];
    if ((abs(pY - p.y) < 5) && (pX > pBirdCenter.x) && (p.x > pBirdCenter.x))
    {
      float _pX = pX - pBirdCenter.x;
      float _arcX = p.x - pBirdCenter.x;
      float _ratio = _pX / _arcX;
      pX = pX - (_pX * _ratio * pow(yRatio, 2));
      break;
    }
    else if ((abs(pY - p.y) < 5) && (pX <= pBirdCenter.x) && (p.x <= pBirdCenter.x))
    {
      float _pX = pBirdCenter.x - pX;
      float _arcX = pBirdCenter.x - p.x;
      float _ratio = _pX / _arcX;
      pX = pX + (_pX * _ratio * pow(yRatio, 2));
      break;
    }
  }

  // Step3. Limitation
  if (pY > tYCloseDoorbell)
    pY = tYCloseDoorbell;

  Point pResult = Point((int)pX, (int)pY);
  return pResult;
}


void Trajectory::bboxToTrajectory(vector<Object> &objectList)
{
  Point pBirdCenter = Point(static_cast<int>(birdWidth*0.5), 0);
  vector<pair<int, vector<Point>>> pTrajectoryListDict;

  for (int j=0; j<(int)objectList.size(); j++)
  {
    Object *ptrObj = &objectList[j];

    int id = ptrObj->id;

    // Skip if object is disable
    if (ptrObj->getStatus() == 0)
    {
      continue;
    }
    pTrajectoryListDict.push_back(std::make_pair(id, vector<Point>()));

    // cout << "[DBG] >>> bboxToTrajectory push" << endl;

    float prev_pX = 0;
    float prev_pY = 0;

    // StepA. Bounding Box Preprocessing
    // 5 or 9 Boxes
    vector<BoundingBox> rescaleBboxList;
    vector<BoundingBox> smoothedBboxList;

    for (int i=0; i<ptrObj->bboxList.size(); i++)
    {
      BoundingBox rescaleBox(-1, -1, -1, -1, ptrObj->bboxList[i].label);
      utils::rescaleBBox(
        ptrObj->bboxList[i], rescaleBox,
        modelWidth, modelHeight,
        videoWidth, videoHeight);

      rescaleBboxList.push_back(rescaleBox);
    }

    _bboxPreprocessing(rescaleBboxList, smoothedBboxList, 3);

    cout << "[DBG] >>>>>> smoothedBboxList.size() = " << smoothedBboxList.size() << endl;
    // StepB. Bounding Box to Bird's Eye view
    for (int i=0; i<(int)smoothedBboxList.size(); i++)
    {
      BoundingBox box = smoothedBboxList[i];
      Point pCenter = box.getCenterPoint();
      // float humanHeight = (float)box.getHeight();
      float humanHeight = (float)ptrObj->bboxList.back().getHeight();
      float humanWidth = (float)box.getWidth();
      float aspectRatio = (float)box.getAspectRatio();

      if ((humanHeight == 0) || (humanWidth == 0))
        continue;

      float dist = (-0.0319 * (float)humanHeight) + 10.8;
      // _bboxToDistanceZ(ptrObj->bboxList.back());
      float hRatio = 1 - (dist / 13.0); // TODO: assume max valid distance = 12 meters
      float wRatio = humanWidth / (float)videoWidth;
      float pX = ((float)pCenter.x / (float)videoWidth) * (float)birdWidth;
      float pY = hRatio * (float)birdHeight;

      if (debugMode)
      {
        cout << "\n\n\n[REID] obj ID = " << id << endl;
        cout << "[REID] cls = " << box.label << endl;
        cout << "[REID] bbox = " << box.x1 << ", " << box.y1 << ", " << box.x2 << ", " << box.y2 << endl;
        cout << "[REID] Y dist = " << dist << endl;
        cout << "[REID] humanHeight = " << humanHeight << endl;
        cout << "[REID] humanWidth = " << humanWidth << endl;
        cout << "[REID] h ratio = " << hRatio << endl;
        cout << "[REID] w ratio = " << wRatio << endl;
        cout << "[REID] aspectRatio = " << aspectRatio << endl;

        cout << "[DBG] >>> StepA" << endl;
        cout << ">>> pX = " << (pCenter.x / (float)videoWidth) * (float)birdWidth << endl;
        cout << ">>> pY = " << hRatio * (float)birdHeight << endl;
      }


      if (debugMode)
      {
        cout << "[DBG] >>> StepB" << endl;
        cout << ">>> pX = " << pX << endl;
        cout << ">>> pY = " << pY << endl;
      }

      // TODO: childen detection

      // Initialization (Skip 1st Bounding Box)
      if (i == 0)
      {
        if (debugMode) cout << "[DBG] >>> i == 0" << endl;
        prev_pX = pX;
        prev_pY = pY;
      }

      // Preprocessing
      if (i > 0)
      {
        if (debugMode) cout << "[DBG] >>> i > 0" << endl;
        bool detectFullBody = true;

        // Step1. Detect non full body
        if ((aspectRatio >= tAspectFall) && (aspectRatio < tAspectFullBody) && (wRatio > tWidthRatioCloseLv1) && \
          (box.y1 < (float)videoHeight*0.4) && (box.y2 > (float)videoHeight*0.98))
        {
          humanHeight = ((tAspectFullBody-wRatio) * humanWidth);
          hRatio = humanHeight / (float)videoHeight;
          float newY = hRatio * (float)birdHeight;

          if (debugMode)
          {
            cout << "humanHeight = " << humanHeight << endl;
            cout << "hRatio = " << hRatio << endl;
            cout << "aspectRatio = " << aspectRatio << endl;
            cout << "tAspectFall = " << tAspectFall << endl;
            cout << "tAspectFullBody = " << tAspectFullBody << endl;
            cout << "(newY / pY)  = " << (newY / pY) << endl;
            cout << "tNewY = " << tNewY << endl;
            cout << "wRatio = " << wRatio << endl;
            cout << "tWidthRatioCloseLv1 = " << tWidthRatioCloseLv1 << endl;
            cout << "tWidthRatioCloseLv2 = " << tWidthRatioCloseLv2 << endl;
            cout << "newY = " << newY << endl;
          }

          // if new y change too much and ...
          // case1. not close to doorbell
          if (((newY / pY) > tNewY) && (wRatio < tWidthRatioCloseLv2))
          {
            if (pY > tYCloseDoorbell)
            {
              if (debugMode) cout << "pY = " << pY << ", tYCloseDoorbell = " << tYCloseDoorbell << endl;
              pY = prev_pY;
            }
            else
            {
              if (debugMode) cout << "pY = " << pY << ", newY = " << newY << endl;
              pY = newY;
            }
          }
          // case2. close to doorbell
          else
          {
            float pYDiff = abs(prev_pY - newY);
            float pYRatio = pYDiff / prev_pY;

            if (debugMode)
            {
              cout << "pYDiff = " << pYDiff << endl;
              cout << "pYRatio = " << pYRatio << endl;
            }

            if (pYRatio > tYDiffLower)
            {
              pY = ((prev_pY + newY) * 0.5);
              if (debugMode) cout << "(pYRatio > tYDiffLower)" << endl;
            }
            //TODO:
          //   else
          //   {
          //     pY = newY;
          //     cout << "pY = newY" << endl;
          //     cout << "newY = " << newY << endl;
          //   }
          }

          detectFullBody = false;

          if (debugMode)
          {
            cout << "[DBG] >>> StepC1" << endl;
            cout << ">>> pX = " << pX << endl;
            cout << ">>> pY = " << pY << endl;
          }
        }

        // Fall Down
        else if (aspectRatio < tAspectFall)
        {
          pY = prev_pY;
        }

        // Step2. Y calibration
        if ((aspectRatio < tAspectFullBody) && (detectFullBody))
        {
          float pYDiff = abs(prev_pY - pY);
          float pYRatio = pYDiff / (float)birdHeight;

          if ((pYRatio > tYDiffLower) && (pYRatio <= tYDiffUpper))
          {
            pY = ((pY + prev_pY) * 0.5);
          }
          else if (pYRatio > tYDiffUpper)
          {
            pY = prev_pY;
          }
        }

        if (debugMode)
        {
          cout << "[DBG] >>> StepC2" << endl;
          cout << ">>> pX = " << pX << endl;
          cout << ">>> pY = " << pY << endl;
        }
        // Step3. X calibration if difference changes too much

        // When person is closing to doorbell,
        // the x difference needs to be adjusted more

        // Find cross point between line and circle and adjust x
        float yRatio = pY / (float)birdHeight;
        for (int idx=0; idx<(int)pList.size(); idx++)
        {
          Point p = pList[idx];
          // cout << " abs(pY - p.y) = " << abs(pY - p.y) << endl;
          int diff = abs(pY - p.y);
          if ((diff < 5) && (pX > pBirdCenter.x) && (p.x > pBirdCenter.x))
          {
            float _pX = pX - pBirdCenter.x;
            float _arcX = p.x - pBirdCenter.x;
            float _ratio = _pX / _arcX;
            pX = pX - (_pX * _ratio * pow(yRatio, 2));
            break;
          }
          else if ((diff < 5) && (pX <= pBirdCenter.x) && (p.x <= pBirdCenter.x))
          {
            float _pX = pBirdCenter.x - pX;
            float _arcX = pBirdCenter.x - p.x;
            float _ratio = _pX / _arcX;
            pX = pX + (_pX * _ratio * pow(yRatio, 2));

            if (debugMode)
            {
              cout << " >>> pBirdCenter.x = " << pBirdCenter.x << endl;
              cout << " >>> _pX = " << _pX << endl;
              cout << " >>> _arcX = " << _arcX << endl;
              cout << " >>> _ratio = " << _ratio << endl;
              cout << " >>> pX = " << pX << endl;
            }
            break;
          }
        }

        if (debugMode)
        {
          cout << "[DBG] >>> StepC2" << endl;
          cout << ">>> pX = " << pX << endl;
          cout << ">>> pY = " << pY << endl;
        }
        // if extreme close to doorbell then fix p x
        if ((wRatio > tWidthRatioCloseLv3) && (prev_pY >= tYCloseDoorbell))
        {
          pX = prev_pX;
        }

        // if p x changes too much
        float pXDiff = abs(pX - prev_pX);
        float pXRatio = pXDiff/(float)birdWidth;

        if (debugMode)
        {
          cout << "pXRatio = " << pXRatio << endl;
          cout << "tXDiffRatio = " << tXDiffRatio << endl;
        }

        if (pXRatio > tXDiffRatio)
        {
          pX = ((pX + prev_pX) * 0.5);
        }
      }

      if (debugMode)
      {
        cout << "[DBG] >>> StepD" << endl;
        cout << ">>> pX = " << pX << endl;
        cout << ">>> pY = " << pY << endl;
      }

      // Step4. Limitation
      if (pY > tYCloseDoorbell)
      {
        pY = tYCloseDoorbell;
      }

      int idxDict = utils::findListIdx(id, pTrajectoryListDict);

      if (debugMode)
      {
        cout << "[DBG] >>> StepE1" << endl;
        cout << ">>> pX = " << pX << endl;
        cout << ">>> pY = " << pY << endl;
      }

      pX = (int)(pX+0.5);
      pY = (int)(pY+0.5);

      if (debugMode)
      {
        cout << "[DBG] >>> StepE2" << endl;
        cout << ">>> pX = " << pX << endl;
        cout << ">>> pY = " << pY << endl;
      }

      pTrajectoryListDict[idxDict].second.push_back(Point(pX, pY));

      // Ignore bounding box if out of ROI
      if (pCenter.y > ((float)videoHeight*0.85) && (hRatio < 0.3))
        continue;

      if (debugMode)
      {
        cout << "[DBG] >>> StepE3" << endl;
        cout << ">>> pX = " << pX << endl;
        cout << ">>> pY = " << pY << endl;
      }

      // Update previous point
      prev_pX = pX;
      prev_pY = pY;
    }
    // cout << "[DBG] >>> StepB done" << endl;

    // StepC. Update object's location point
    if ((int)ptrObj->bboxList.size() >= 3) //TODO:
    {
      Point pLast = bboxToPointSimple(ptrObj->bboxList[ptrObj->bboxList.size()-1]);
      ptrObj->updatePointLocation(pLast);
    }
    // cout << "[DBG] >>> StepC done" << endl;
  }

  // Trajectory post-processing (Smoothing ...)
  for (int i=0; i<(int)pTrajectoryListDict.size(); i++)
  {
    int len = pTrajectoryListDict[i].second.size();
    if (len > maWindowSize)
    {
      pTrajectoryListDict[i].second = _postProcessing(pTrajectoryListDict[i].second);
    }
  }

  // Update Trajectory List
  for (int i=0; i<(int)pTrajectoryListDict.size(); i++)
  {
    int id = pTrajectoryListDict[i].first;
    int trajSize = pTrajectoryListDict[i].second.size();

    for (int j=0; j<(int)objectList.size(); j++)
    {
      if ((id == objectList[j].id) && (trajSize > 0) && (objectList[j].getStatus() == 1))
      {
        Point p = pTrajectoryListDict[i].second[trajSize-1];

        // cout << "[DBG] >>> Convert trajectory point = " << p.x << ", " << p.y << endl;
        objectList[j].m_trajList.push_back(p);
      }
    }
  }
}


int Trajectory::getBevZone(Point &bevPoint)
{
  int y = bevPoint.y;
  int currZone = (int)((y / (float)birdHeight) * numBevZone);
  return currZone;
}


float Trajectory::_bboxToDistanceZ(BoundingBox& box)
{
  // When the camera is installed at a height of 3 meters with a downward angle of 30 degrees,
  // it records many distance information corresponding to the Bounding Boxes of people.
  // Through linear regression, a linear equation is obtained to estimate the distance of people from the camera.

  float boxHeight = (float)box.getHeight();
  float distance = (-0.0319 * boxHeight) + 10.8;

  // cout << "bbox to distance z : " << distance << endl;

  return distance;
}


float Trajectory::updateLocation3D(Object& obj)
{
  if (obj.bboxList.size() == 0)
    return -1;

  BoundingBox& box = obj.bboxList.back();
  Point pCenter = box.getCenterPoint();
  float height = (float)box.getHeight();
  float width = (float)box.getWidth();
  float distanceZ = _bboxToDistanceZ(box);

  float hRatio = 1 - (distanceZ / 13.0); // TODO: assume max valid distance = 13 meters
  float wRatio = width / (float)modelWidth;
  float xCenter = (float)(modelWidth * 0.5);
  float xDist = (pCenter.x - xCenter);
  float pX = (xDist / (float)(modelWidth * 0.5)) * (float)(birdWidth*0.5);
  float pY = hRatio * (float)birdHeight;

  float distanceX = (distanceZ / pY) * (pX);
  float distanceY = 0;

  // cout << "ID = " << obj.id << endl;
  // cout << "Location = (" << distanceX << ", " << distanceY << ", " << distanceZ << ")" << endl;
  // cout << "DistX = " << xDist << endl;
  // cout << "xCenter = " << xCenter << endl;
  // cout << "pCenter x = " << pCenter.x << endl;
  cv::Point3f pLoc3D(distanceX, distanceY, distanceZ);
  obj.pLocation3D = pLoc3D;
}