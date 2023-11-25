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

#include "img_util.hpp"

namespace imgUtil
{
  //
  //
  //
	void calcLinearEquation(cv::Point &pA, cv::Point &pB, vector<float> &equation)
  {
    // Calculate the slope (m)
    float slope = (float)(pB.y - pA.y) / (float)(pB.x - pA.x);

    // Slope *= -1 due to image is from top-left to bottom-right
    slope *= -1;

    // Calculate the y-intercept (b) using y = mx + b
    float intercept = pA.y - slope * pA.x;

    equation.push_back(slope);
    equation.push_back(intercept);
  }

  int checkPointOnWhichLineSide(Point &p, vector<float> &linearEquation)
  {
    // Slope (m)
    float m = linearEquation[0];

    // Intercept (b)
    float b = linearEquation[1];

    float res = p.x * m + b;

    if (res > 0)
      return 0;   // left side
    else if (res == 0)
      return -1;  // on the line
    else
      return 1;   // right side
  }

  int checkPointOnWhichLineSide(cv::Point &p, cv::Point &pA, cv::Point &pB)
  {
    float tmpx = ((float)pA.x - (float)pB.x) / ((float)pA.y - (float)pB.y) * ((float)p.y - (float)pB.y) + (float)pB.x;

    // Left Side
    if (tmpx > (float)p.x)
        return false;

    // Right Side
    else
      return true;
  }


  //
  //
  //
  float calcBrightnessRatio(cv::Mat &img)
  {
    cv::Mat resize;
    cv::resize(img, resize, cv::Size(36, 24), cv::INTER_LINEAR);

    cv::Mat hls;
    cv::cvtColor(resize, hls, cv::COLOR_BGR2HLS);
    cv::Mat hls_channels[3];
    cv::split(hls, hls_channels);

    cv::Mat imgLumin = hls_channels[1];
    // cv::normalize(imgLumin, imgLumin, 0, 1.0, cv::NORM_MINMAX, CV_32F);

    // float mean = cv::mean(imgLumin)[0];
    cv::Mat dummy;
    double th = cv::threshold(imgLumin, dummy, 150, 255, cv::THRESH_OTSU);
    double mean = th/255.0;

    return mean;
  }

  void brightnessEnhancement(float brightnessRatio, cv::Mat &img)
  {
    // cout << "brightnessRatio: " << brightnessRatio << endl;

    // cv::imshow("before BE", img);
    if (brightnessRatio < VERY_DARK)
    {
      imgUtil::gammaCorrection(img, img, 0.5);
    }
    else if (brightnessRatio < DARK)
    {
      imgUtil::gammaCorrection(img, img, 0.7);
    }
    else if (brightnessRatio > VERY_BRIGHT)
    {
      imgUtil::gammaCorrection(img, img, 1.4);
    }
    else if (brightnessRatio > BRIGHT)
    {
      imgUtil::gammaCorrection(img, img, 1.2);
    }
    // cv::imshow("after BE", img);
  }

  //
  // Image Enhancement
  //
  void gammaCorrection(const cv::Mat input, cv::Mat& output, const float gamma)
  {
    // std::array<uchar, 256> table;
    // int table[256];
    cv::Mat table(1, 256, CV_8UC1);
    for (int i = 0; i < 256; i++)
    {
      table.at<uchar>(i) = cv::saturate_cast<uchar>(pow((i / 255.0), gamma) * 255.0);
    }

    cv::LUT(input, table, output);
  }

  void autoscaling(const cv::Mat input, cv::Mat& output)
  {
    double minVal, maxVal;
    cv::minMaxLoc(input, &minVal, &maxVal, NULL, NULL);
    output = 255 * (input - minVal) / (maxVal - minVal);
  }

  void contrastStretching(const cv::Mat input, cv::Mat& output, const int r1, const int s1, const int r2, const int s2)
  {
    // std::array<uchar, 256> table;
    // int table[256];
    cv::Mat table(1, 256, CV_8UC1);
    for (int i = 0; i < 256; i++)
    {
      if (i <= r1)
      {
          table.at<uchar>(i) = cv::saturate_cast<uchar>(((float)s1 / (float)r1) * i);
      }
      else if (r1 < i && i <= r2)
      {
          table.at<uchar>(i) = cv::saturate_cast<uchar>(((float)(s2 - s1)/(float)(r2 - r1)) * (i - r1) + s1);
      }
      else // (r2 < i)
      {
          table.at<uchar>(i) = cv::saturate_cast<uchar>(((float)(255 - s2)/(float)(255 - r2)) * (i - r2) + s2);
      }
    }

    cv::LUT(input, table, output);
  }

  void contrastEnhancement(cv::Mat &img, cv::Mat &imgEnhance)
  {
    // STEP1. Autoscaling
    cv::Mat imgAutoscaled;
    autoscaling(img, imgAutoscaled);
    // cv::imshow("autoscaling", imgAutoscaled);

    // STEP2. Gamma correction
    int gamma = 150;
    cv::Mat imgGamma;
    gammaCorrection(imgAutoscaled, imgGamma, gamma / 100.0f);
    // cv::imshow("gamma correction", imgGamma);

    imgEnhance = imgGamma;
  }


  //
  // Image Thresholding
  //
  void abs_sobel_thresh(cv::Mat const& src, cv::Mat& dest, char orient = 'x', int kernel_size = 3, int thresh_min = 0, int thresh_max = 255)
  {
    int dx, dy;
    int ddepth = CV_64F;

    cv::Mat grad_img, scaled;

    if (orient == 'x') {
      dy = 0;
      dx = 1;
    }
    else {
      dy = 1;
      dx = 0;
    }

    cv::Sobel(src, grad_img, ddepth, dx, dy, kernel_size);
    grad_img = cv::abs(grad_img);

    // Scaling
    double min, max;
    cv::minMaxLoc(grad_img, &min, &max);
    scaled = 255 * (grad_img / max);
    scaled.convertTo(scaled, CV_8UC1);

    assert(scaled.type() == CV_8UC1);
    cv::inRange(scaled, cv::Scalar(thresh_min), cv::Scalar(thresh_max), dest);

    return;
  }


  void combined_threshold(cv::Mat const& img, cv::Mat& dst)
  {
    cv::Mat sobel_x, sobel_y, combined;

    // perform thresholding on parallel
    abs_sobel_thresh(img, sobel_x, 'x', 3, 10, 170);
    abs_sobel_thresh(img, sobel_y, 'y', 3, 10, 170);

    dst = sobel_x & sobel_y; // combine gradient images

    return;
  }


  //
  // Line
  //
  cv::Point2f _getLine(cv::Point p1, cv::Point p2)
  {
    cv::Point2f poly;
    //""" y = ax + b """
    if (p1.x == p2.x)
    {
      poly.x = 0;
      poly.y = 0;
      // printf("return NULL \n ");
    }
    else
    {
      float a = float(p1.y - p2.y) / float(p1.x - p2.x);
      float b = (float)p1.y - a * (float)p1.x;
      poly.x = a;
      poly.y = b;
      // printf("poly.x = %f", poly.x);
      // printf("poly.y = %f \n", poly.y);
    }
    return poly;
  }

  cv::Point getIntersectionPoint(cv::Point pLeftBot, cv::Point pLeftTop, cv::Point pRightBot, cv::Point pRightTop)
  {
    // """ y = ax + b """
    cv::Point2f a1_b1 = _getLine(pLeftBot, pLeftTop);
    cv::Point2f a2_b2 = _getLine(pRightBot, pRightTop);
    float x = (a1_b1.y - a2_b2.y) / (a2_b2.x - a1_b1.x);
    float y = a1_b1.x * x + a1_b1.y;

    if (x < 0)
    {
      x = 0;
    }
    if (y < 0)
    {
      y = 0;
    }
    cv::Point x_y = cv::Point(x, y);

    return x_y;
  }


  float calcEuclideanDistance(Point &pA, Point &pB)
  {
    float distance = 0;
    if ((pB.x == -1) && (pB.y == -1))
      distance = -1.0;
    else
    {
      int dx = pA.x - pB.x;
      int dy = pA.y - pB.y;
      distance = sqrt(pow(dx, 2) + pow(dy, 2));
    }

    return distance;
  }


  float calcEuclideanDistance(cv::Point &pA, cv::Point &pB)
  {
    float distance = 0;
    if ((pB.x == -1) && (pB.y == -1))
      distance = -1.0;
    else
    {
      int dx = pA.x - pB.x;
      int dy = pA.y - pB.y;
      distance = sqrt(pow(dx, 2) + pow(dy, 2));
    }

    return distance;
  }

  float calcEuclideanDistance(cv::Point2f &pA, cv::Point2f &pB)
  {
    float distance = 0;
    if ((pB.x == -1) && (pB.y == -1))
      distance = -1.0;
    else
    {
      int dx = pA.x - pB.x;
      int dy = pA.y - pB.y;
      distance = sqrt(pow(dx, 2)+pow(dy, 2));
    }

    return distance;
  }


  //
  // Angle
  //
  double angleBetweenPoints(cv::Point &pA, cv::Point &pB)
  {
    double x1 = (double)pA.x;
    double x2 = (double)pB.x;
    double y1 = (double)pA.y;
    double y2 = (double)pB.y;

    double dot_product = x1 * x2 + y1 * y2;
    double magnitude1 = std::sqrt(x1 * x1 + y1 * y1);
    double magnitude2 = std::sqrt(x2 * x2 + y2 * y2);
    double cosine = dot_product / (magnitude1 * magnitude2);
    double angle_in_radians = std::acos(cosine);

    return angle_in_radians;
  }

  void rotateVector(cv::Point2f &v, cv::Point2f &vRotated, double angle)
  {
    double cosAngle = cos(angle);
    double sinAngle = sin(angle);
    vRotated = cv::Point2f(
        v.x * cosAngle - v.y * sinAngle,
        v.x * sinAngle + v.y * cosAngle
    );
  }

  //
  // PolyLine
  //
  template<typename T>
  std::vector<double> linspace(T start_in, T end_in, int num_in)
  {
    std::vector<double> linspaced;
    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0) { return linspaced; }
    if (num == 1) {
      linspaced.push_back(start);
      return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for (int i = 0; i < num - 1; ++i) {
      linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end);

    return linspaced;
  }


  void _poly_fitx(std::vector<double> const& fity, std::vector<double>& fitx, cv::Mat const& line_fit)
  {
    for (auto const& y : fity) {
      double x = line_fit.at<float>(2, 0) * y * y + line_fit.at<float>(1, 0) * y + line_fit.at<float>(0, 0);
      fitx.push_back(x);
    }

    return;
  }


  void _polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order)
  {
    CV_Assert((src_x.rows > 0) && (src_y.rows > 0) && (src_x.cols == 1) && (src_y.cols == 1)
      && (dst.cols == 1) && (dst.rows == (order + 1)) && (order >= 1));
    cv::Mat X;
    X = cv::Mat::zeros(src_x.rows, order + 1, CV_32FC1);
    cv::Mat copy;
    for (int i = 0; i <= order; i++)
    {
      copy = src_x.clone();
      pow(copy, i, copy);
      cv::Mat M1 = X.col(i);
      copy.col(0).copyTo(M1);
    }
    cv::Mat X_t, X_inv;
    transpose(X, X_t);
    cv::Mat temp = X_t * X;
    cv::Mat temp2;
    invert(temp, temp2);
    cv::Mat temp3 = temp2 * X_t;
    cv::Mat W = temp3 * src_y;
    W.copyTo(dst);
  }


  //
  // Contour
  //
  bool sortByContourSize(const pair<int, int> &a, const pair<int, int> &b)
  {
    return (a.second > b.second);
  }

  void findMaxContour(cv::Mat &src, cv::Mat &dst)
  {
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(src, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat drawing = cv::Mat::zeros(src.size(), CV_8UC1);

    int maxContourIdx = 0;
    int maxContourSize = 0;
    for (int i=0; i<contours.size(); i++)
    {
      int contourSize = contours[i].size();
      if (contourSize > maxContourSize)
      {
        maxContourSize = contourSize;
        maxContourIdx = i;
      }
    }

    for(int i = 0; i< contours.size(); i++)
    {
      if (i != maxContourIdx)
        drawContours(drawing, contours, (int)i, cv::Scalar(0), -1, cv::LINE_4, hierarchy, 0);
      else
      {
        drawContours(drawing, contours, (int)i, cv::Scalar(255), -1, cv::LINE_4, hierarchy, 0);
      }
    }

    dst = drawing;
  }


  void findClosestContour(cv::Mat &src, int y, cv::Mat &dst)
  {
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(src, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat drawing = cv::Mat::zeros(src.size(), CV_8UC1);

    /*
    int maxContourIdx = 0;
    int maxContourSize = 0;
    for (int i=0; i<contours.size(); i++)
    {
      int contourSize = contours[i].size();
      if (contourSize > maxContourSize)
      {
        maxContourSize = contourSize;
        maxContourIdx = i;
      }
    }
    */

    int minDiff = 1000;
    int idx = 0;

    for (int i=0; i<contours.size(); i++)
    {
      cv::Moments M = cv::moments(contours[i]);
      cv::Point center(M.m10/M.m00, M.m01/M.m00);
      int diff = abs(center.y - y);
      if (diff < minDiff)
      {
        minDiff = diff;
        idx = i;
      }
    }


    for(int i = 0; i< contours.size(); i++)
    {
      if (i != idx)
        drawContours(drawing, contours, (int)i, cv::Scalar(0), -1, cv::LINE_4, hierarchy, 0);
      else
      {
        drawContours(drawing, contours, (int)i, cv::Scalar(255), -1, cv::LINE_4, hierarchy, 0);
      }
    }

    dst = drawing;
  }


  void removeSmallContour(cv::Mat &src, cv::Mat &dst, int sizeThreshold)
  {
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(src, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat drawing = cv::Mat::zeros(src.size(), CV_8UC1);

    for (int i=0; i<contours.size(); i++)
    {
      int contourSize = contours[i].size();

      // cout << i << ": size = " << contourSize << ", th = " << sizeThreshold << endl;

      // if (contourSize < sizeThreshold)
      // {
      //   drawContours(drawing, contours, (int)i, cv::Scalar(0), -1, cv::LINE_4, hierarchy, 0);
      // }
      if (contourSize >= sizeThreshold)
      {
        drawContours(drawing, contours, (int)i, cv::Scalar(255), -1, cv::LINE_4, hierarchy, 0);
      }
    }
    dst = drawing;
  }


  // Bounding Box
  float getBboxOverlapRatio(BoundingBox &boxA, BoundingBox &boxB)
  {
    // boxA: key bounidng box
    // boxB: query bounidng box
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


  //
  void roundedRectangle(
    cv::Mat& src, cv::Point topLeft, cv::Point bottomRight, const cv::Scalar lineColor,
    int thickness, const int lineType , const int cornerRadius, bool filled)
  {
    cv::Point p1 = topLeft;
    cv::Point p2 = cv::Point (bottomRight.x, topLeft.y);
    cv::Point p3 = bottomRight;
    cv::Point p4 = cv::Point (topLeft.x, bottomRight.y);


    cv::line(src, cv::Point (p1.x+cornerRadius,p1.y), cv::Point (p2.x-cornerRadius,p2.y), lineColor, thickness, lineType);
    cv::line(src, cv::Point (p2.x,p2.y+cornerRadius), cv::Point (p3.x,p3.y-cornerRadius), lineColor, thickness, lineType);
    cv::line(src, cv::Point (p4.x+cornerRadius,p4.y), cv::Point (p3.x-cornerRadius,p3.y), lineColor, thickness, lineType);
    cv::line(src, cv::Point (p1.x,p1.y+cornerRadius), cv::Point (p4.x,p4.y-cornerRadius), lineColor, thickness, lineType);

    cv::ellipse(src, p1+cv::Point(cornerRadius, cornerRadius), cv::Size( cornerRadius, cornerRadius ), 180.0, 0, 90, lineColor, thickness, lineType);
    cv::ellipse(src, p2+cv::Point(-cornerRadius, cornerRadius), cv::Size( cornerRadius, cornerRadius ), 270.0, 0, 90, lineColor, thickness, lineType);
    cv::ellipse(src, p3+cv::Point(-cornerRadius, -cornerRadius), cv::Size( cornerRadius, cornerRadius ), 0.0, 0, 90, lineColor, thickness, lineType);
    cv::ellipse(src, p4+cv::Point(cornerRadius, -cornerRadius), cv::Size( cornerRadius, cornerRadius ), 90.0, 0, 90, lineColor, thickness, lineType);

    // choose arbitrary starting point for fill => Top left plus 10,10
    if (filled)
    {
      cv::Point fillFrom(topLeft.x+10, topLeft.y+10);
      // You may want to use `lineColor` instead of `fillColor`
      cv::floodFill(src, fillFrom, lineColor);

      // TODO: use find contour
    }
  }

  //
  void dilate(cv::Mat &img, int kernelSize)
  {
    cv::Mat element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(2 * kernelSize + 1, 2 * kernelSize + 1),
      cv::Point(kernelSize, kernelSize));

    cv::dilate(img, img, element);
  }


	void erode(cv::Mat &img, int kernelSize)
  {
    cv::Mat element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(2 * kernelSize + 1, 2 * kernelSize + 1),
      cv::Point(kernelSize, kernelSize));

    cv::erode(img, img, element);
  }

  //
  void cropImages(cv::Mat &img, cv::Mat &imgCrop, BoundingBox &box)
  {
    int w = 0;
    int h = 0;

    if (box.x1 < 0)
        box.x1 = 0;
    else if (box.x2 > img.cols)
        box.x2 = img.cols;

    if (box.y1 < 0)
        box.y1 = 0;
    else if (box.y2 > img.rows)
        box.y2 = img.rows;

    w = box.x2 - box.x1;
    h = box.y2 - box.y1;
    cout<<"[cropImages]box.x1 = "<<box.x1<<endl;
    cout<<"[cropImages]box.y1 = "<<box.y1<<endl;
    cout<<"[cropImages]w = "<<w<<endl;
    cout<<"[cropImages]h = "<<h<<endl;
    cv::Rect roi(box.x1, box.y1, w, h);
    imgCrop = img(roi);
  }
}