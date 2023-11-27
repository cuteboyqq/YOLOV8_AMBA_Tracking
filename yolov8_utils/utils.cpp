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

#include "utils.hpp"

namespace utils
{
  //
  //
  //
  bool checkFileExists(const std::string& name)
  {
    if (FILE *file = fopen(name.c_str(), "r"))
    {
      fclose(file);
      return true;
    } else
      return false;
  }


  bool checkFolderExists(const std::string& folderName)
  {
    struct stat info;
    return stat(folderName.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
  }


  bool createFolder(const std::string& folderName)
  {
    if (!checkFolderExists(folderName))
    {
      // Create the folder using mkdir
      if (mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)
      {
        std::cout << "Folder created successfully: " << folderName << std::endl;
      }
      else
      {
        std::cerr << "Error creating folder: " << strerror(errno) << std::endl;
      }
    }
    else
    {
      std::cout << "Folder already exists: " << folderName << std::endl;
    }
  }


  bool createDirectory(const std::string& path)
  {
    // Check if the directory already exists
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        // Directory does not exist, try to create it
        if (mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) {
            return true;
        } else {
            return false;
        }
    } else if (info.st_mode & S_IFDIR) {
        // Directory already exists
        return true;
    } else {
        // Path exists but is not a directory
        return false;
    }
  }

  bool createDirectories(const std::string& path)
  {
    std::vector<std::string> parts;
    std::istringstream iss(path);
    std::string part;

    // Split the path into parts
    while (std::getline(iss, part, '/'))
    {
      if (!part.empty())
      {
        parts.push_back(part);
      }
    }

    std::string currentPath = "";
    for (const std::string& part : parts)
    {
      currentPath += part + "/";
      if (!createDirectory(currentPath))
      {
        return false; // Error occurred while creating the directory
      }
    }

    return true; // All directories created successfully
  }
  //
  //
  //
  void getDateTime(std::string &timeStr)
  {
    // Get the current time point
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    // Convert the time point to a time_t object
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // Convert the time_t object to a local time structure
    std::tm* now_tm = std::localtime(&now_time_t);

    // Extract date and time components
    int year = now_tm->tm_year + 1900; // Years since 1900
    int month = now_tm->tm_mon + 1;    // Months start from 0
    int day = now_tm->tm_mday;         // Day of the month
    int hour = now_tm->tm_hour;        // Hour of the day
    int minute = now_tm->tm_min;       // Minute of the hour

    timeStr = std::to_string(year) + "-" + std::to_string(month) + "-" + \
      std::to_string(day) + "-" + std::to_string(hour) + "-" + std::to_string(minute);
  }


  //
  //
  //
  bool sortbysec(
    const pair<int, int> &a,
    const pair<int, int> &b)
  {
    return (a.second > b.second);
  }

  bool sortBySecAscend(
    const pair<int, int> &a,
    const pair<int, int> &b)
  {
    return (a.second < b.second);
  }

  bool sortByBBoxArea(BoundingBox &bboxA, BoundingBox &bboxB)
  {
    return bboxA.getArea() > bboxB.getArea();
  }

  // bool sortByObjBBoxArea(Object &objA, Object &objB)
  // {
  //   return objA.m_lastDetectBoundingBox.getArea() > objB.m_lastDetectBoundingBox.getArea();
  // }

  //
  //
  //
  float calcVecDegree(Point &a, Point &b)
  {
    float dot;
    float det;
    float angle;
    float degree;

    // cout << "a.x = " << a.x << endl;
    // cout << "b.x = " << b.x << endl;
    // cout << "a,y = " << a.y << endl;
    // cout << "b.y = " << b.y << endl;
    dot = a.x*b.x + a.y*b.y;      // dot product between [x1, y1] and [x2, y2]
    det = a.x*b.y - a.y*b.x;      // determinant

    // cout << "dot = " << dot << endl;
    // cout << "det = " << det << endl;

    angle = atan2(det, dot);

    // cout << "angle = " << angle << endl;

    degree = abs(angle * 180 / M_PI);

    return degree;
  }

  ///
  //
  //
  void updateIntList(vector<int> &list, int x, int maxSize)
  {
    if (list.size() < maxSize)
    {
      list.push_back(x);
    }
    else
    {
      list.erase(list.begin());
      list.push_back(x);
    }
  }

  //
  //
  //
  void findMaxScoreKey(vector<pair<int, float>> &dict, int &key, float &score)
  {
    int idxMaxScore = 0;
    float maxScore = 0.0;
    for (auto it=dict.begin(); it!=dict.end(); it++)
    {
      int keyId = (*it).first;
      float matchScore = (*it).second;
      if (matchScore > maxScore)
      {
        maxScore = matchScore;
        idxMaxScore = keyId;
      }
    }
    key = idxMaxScore;
    score = maxScore;
  }

  //
  //
  //
  int findListIdx(int key, vector<int> &dict)
  {
    int idx = 0;
    for (int i=0; i<dict.size(); i++)
    {
      if (key == dict[i])
      {
        idx = i;
        break;
      }
    }
    return idx;
  }


  int findListIdx(int key, vector<pair<int, vector<int>>> &dict)
  {
    int idx = 0;
    for (int i=0; i<dict.size(); i++)
    {
      if (key == dict[i].first)
      {
        idx = i;
        break;
      }
    }
    return idx;
  }


  int findListIdx(int key, vector<pair<int, vector<pair<int, float>>>> &dict)
  {
    int idx = 0;
    for (int i=0; i<dict.size(); i++)
    {
      if (key == dict[i].first)
      {
        idx = i;
        break;
      }
    }
    return idx;
  }


  int findListIdx(int key, vector<pair<int, pair<int, float>>> &dict)
  {
    int idx = 0;
    for (int i=0; i<dict.size(); i++)
    {
      if (key == dict[i].first)
      {
        idx = i;
        break;
      }
    }
    return idx;
  }


  int findListIdx(int key, vector<pair<int, vector<Point>>> &dict)
  {
    int idx = 0;
    for (int i=0; i<dict.size(); i++)
    {
      if (key == dict[i].first)
      {
        idx = i;
        break;
      }
    }
    return idx;
  }

  //
  //
  //
  int findMedian(vector<int> a, int n)
  {
    // If size of the arr[] is even
    if (n % 2 == 0)
    {
      // Applying nth_element
      // on n/2th index
      nth_element(a.begin(),
                  a.begin() + n / 2,
                  a.end());
      // Applying nth_element
      // on (n-1)/2 th index
      nth_element(a.begin(),
                  a.begin() + (n - 1) / 2,
                  a.end());
      // Find the average of value at
      // index N/2 and (N-1)/2
      return (int)(a[(n - 1) / 2]
                      + a[n / 2])
              / 2.0;
    }
    // If size of the arr[] is odd
    else
    {
      // Applying nth_element
      // on n/2
      nth_element(a.begin(),
                  a.begin() + n / 2,
                  a.end());
      // Value at index (N/2)th
      // is the median
      return (int)a[n / 2];
    }
  }

  //
  //
  //
  bool isKeyInDict(int key, vector<int> &dict)
  {
    bool inDict = false;
    for (auto it=dict.begin(); it!=dict.end(); it++)
    {
      if (key == (*it))
      {
        inDict = true;
        break;
      }
    }
    return inDict;
  }

  bool isKeyInDict(int key, vector<pair<int, vector<int>>> &dict)
  {
    bool inDict = false;
    for (auto it=dict.begin(); it!=dict.end(); it++)
    {
      if (key == (*it).first)
      {
        inDict = true;
        break;
      }
    }
    return inDict;
  }


  bool isKeyInDict(int key, vector<pair<int, vector<pair<int, float>>>> &dict)
  {
    bool inDict = false;
    for (auto it=dict.begin(); it!=dict.end(); it++)
    {
      if (key == (*it).first)
      {
        inDict = true;
        break;
      }
    }
    return inDict;
  }


  bool isKeyInDict(int key, vector<pair<int, pair<int, float>>> &dict)
  {
    bool inDict = false;
    for (auto it=dict.begin(); it!=dict.end(); it++)
    {
      if (key == (*it).first)
      {
        inDict = true;
        break;
      }
    }
    return inDict;
  }


  //
  //
  //
  float getWidthRatio(int refWidth, int width)
  {
    return (float)refWidth / (float)width;
  }

  float getWidthRatio(int xA, int xB, int width)
  {
    int dist = abs(xA - xB);
    float widthRatio = (float)dist / (float)width;

    return widthRatio;
  }

  float getWidthRatio(int xA, int xB, int refWidth, int width)
  {
    int dist = abs(xA - xB);
    float diff = abs(refWidth - dist);
    float widthRatio = (float)diff / (float)width;

    return widthRatio;
  }

  //
  //
  //
  std::pair<float, float> computeStdDevAndMean(std::vector<int> &v)
  {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
    return std::make_pair(mean, stdev);
  }

  std::pair<double, double> computeStdDevAndMean(std::vector<double> &v)
  {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
    return std::make_pair(mean, stdev);
  }

  //
  //
  //
  vector<int> outlierRemoval(vector<int> &list)
  {
    std::pair<float, float> meanAndStdDev = computeStdDevAndMean(list);
    int median = findMedian(list, list.size());
    float upperBound = (meanAndStdDev.first + 2.3*meanAndStdDev.second);
    float lowerBound = (meanAndStdDev.first - 2.3*meanAndStdDev.second);

    int mean = (int)meanAndStdDev.first;
    // cout << "upperBound = " << upperBound << endl;
    // cout << "lowerBound = " << lowerBound << endl;
    // cout << "mean = " << mean << endl;
    // cout << "median = " << median << endl;

    vector<int> filteredList;
    for (int i=0; i<list.size(); i++)
    {
      if (list[i] < upperBound && list[i] > lowerBound)
      {
        filteredList.push_back(list[i]);
      }
      else
      {
        filteredList.push_back(mean);
      }
    }

    return filteredList;
  }

  //
  //
  //
  void applyMovingAverage(vector<int> &list, vector<int> &listMA, int windowSize)
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

  //
  //
  //
  std::string to_string_with_precision(float a_value, const int n = 6)
  {
      std::ostringstream out;
      out.precision(n);
      out << std::fixed << a_value;
      return std::move(out).str();
  }

  //
  //
  //
  void rescaleBBox(
    BoundingBox &bbox, BoundingBox &rescaleBBox,
    int modelWidth, int modelHeight, int videoWidth, int videoHeight)
  {
    float w_ratio = (float)modelWidth / (float)videoWidth;
    float h_ratio = (float)modelHeight / (float)videoHeight;
    cout<<"w_ratio = "<<w_ratio<<endl;
    cout<<"h_ratio = "<<h_ratio<<endl;
    cout<<"bbox.x1 = "<<bbox.x1<<endl;
    cout<<"bbox.y1 = "<<bbox.y1<<endl;
    cout<<"bbox.x2 = "<<bbox.x2<<endl;
    cout<<"bbox.y2 = "<<bbox.y2<<endl;

    rescaleBBox.x1 = (int)((float)bbox.x1 / w_ratio);
    rescaleBBox.y1 = (int)((float)bbox.y1 / h_ratio);
    rescaleBBox.x2 = (int)((float)bbox.x2 / w_ratio);
    rescaleBBox.y2 = (int)((float)bbox.y2 / h_ratio);
    rescaleBBox.confidence = bbox.confidence;
    cout<<"---------------------------------------"<<endl;
    cout<<"rescaleBBox.x1 = "<<rescaleBBox.x1<<endl;
    cout<<"rescaleBBox.y1 = "<<rescaleBBox.y1<<endl;
    cout<<"rescaleBBox.x2 = "<<rescaleBBox.x2<<endl;
    cout<<"rescaleBBox.y2 = "<<rescaleBBox.y2<<endl;

    if (rescaleBBox.x1 < 0)
      rescaleBBox.x1 = 0;
    if (rescaleBBox.x2 > videoWidth-1)
      rescaleBBox.x2 = videoWidth-1;
    if (rescaleBBox.y1 < 0)
      rescaleBBox.y1 = 0;
    if (rescaleBBox.y2 > videoHeight-1)
      rescaleBBox.y2 = videoHeight-1;
  }


  void rescalePoint(
    cv::Point& pSrc, cv::Point& pDst, float xRatio, float yRatio)
  {
    pDst.x = (int)((float)pSrc.x / xRatio);
    pDst.y = (int)((float)pSrc.y / yRatio);
  }


  void smoothBBoxes(
    vector<BoundingBox>& bboxList, vector<BoundingBox>& procBboxList, int windowSize, int maxFrameInterval)
  {
    int numBbox = bboxList.size();
    int halfWindowSize = (int)(windowSize*0.5);

    for (int i=halfWindowSize; i<numBbox-halfWindowSize; i+=1)
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

      vector<int>::iterator x1It_min;
      vector<int>::iterator x1It_max;

      vector<int>::iterator x2It_min;
      vector<int>::iterator x2It_max;

      vector<int>::iterator y1It;
      vector<int>::iterator y2It;

      x1It_min = std::min_element(x1List.begin(), x1List.end());
      x1It_max = std::max_element(x1List.begin(), x1List.end());

      x2It_min = std::min_element(x2List.begin(), x2List.end());
      x2It_max = std::max_element(x2List.begin(), x2List.end());

      y1It = std::min_element(y1List.begin(), y1List.end());
      y2It = std::min_element(y2List.begin(), y2List.end());

      // newX1 = int(((*x1It_min) + (*x1It_max)) * 0.5);
      // newX2 = int(((*x2It_min) + (*x2It_max)) * 0.5);
      // // newX2 = (*x2It_min);

      // newY1 = (*y1It);
      // // newX2 = (*x2It);
      // // newY2 = (*y2It);
      newX1 = utils::findMedian(x1List, x1List.size());
      newX2 = utils::findMedian(x2List, x2List.size());
      newY1 = utils::findMedian(y1List, y1List.size());
      newY2 = utils::findMedian(y2List, y2List.size());

      BoundingBox bbox = BoundingBox(newX1, newY1, newX2, newY2, bboxList[0].label);
      procBboxList.push_back(bbox);
    }
  }


  //
  //
  //
  bool isAscendingList(vector<float> &list)
  {
    float counter = 0;
    for (int i=1; i<list.size(); i++)
    {
      if (list[i] > list[i-1])
        counter += 1;
    }

    // cout << "isAscendingList counter = " << counter << endl;
    if ((counter / (float)list.size()) > 0.3)
    {
      return true;
    }
    else
    {
      return false;
    }
  }


  bool isDescendingList(vector<float> &list)
  {
    float counter = 0;
    for (int i=1; i<list.size(); i++)
    {
      if (list[i] < list[i-1])
        counter += 1;
    }
    // cout << "isDescendingList counter = " << counter << endl;
    if ((counter / (float)list.size()) > 0.3)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}