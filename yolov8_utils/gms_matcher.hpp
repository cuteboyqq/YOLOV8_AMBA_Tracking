#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <ctime>
using namespace std;


#define THRESH_FACTOR 6


class gms_matcher
{
public:
	// OpenCV Keypoints & Correspond Image Size & Nearest Neighbor Matches
	gms_matcher(
		const vector<cv::KeyPoint> &vkp1,
		const cv::Size size1,
		const vector<cv::KeyPoint> &vkp2,
		const cv::Size size2,
		const vector<cv::DMatch> &vDMatches);
	~gms_matcher();

private:

	// Normalized Points
	vector<cv::Point2f> mvP1, mvP2;

	// Matches
	vector<pair<int, int> > mvMatches;

	// Number of Matches
	size_t mNumberMatches;

	// Grid Size
	cv::Size mGridSizeLeft, mGridSizeRight;
	int mGridNumberLeft;
	int mGridNumberRight;

	// x	  : left grid idx
	// y      :  right grid idx
	// value  : how many matches from idx_left to idx_right
	cv::Mat mMotionStatistics;

	//
	vector<int> mNumberPointsInPerCellLeft;

	// Inldex  : grid_idx_left
	// Value   : grid_idx_right
	vector<int> mCellPairs;

	// Every Matches has a cell-pair
	// first  : grid_idx_left
	// second : grid_idx_right
	vector<pair<int, int> > mvMatchPairs;

	// Inlier Mask for output
	vector<bool> mvbInlierMask;

	//
	cv::Mat mGridNeighborLeft;
	cv::Mat mGridNeighborRight;

public:

	// Get Inlier Mask
	// Return number of inliers
	int GetInlierMask(vector<bool> &vbInliers, bool WithScale = false, bool WithRotation = false);

private:

	// Normalize Key Points to Range(0 - 1)
	void NormalizePoints(const vector<cv::KeyPoint> &kp, const cv::Size &size, vector<cv::Point2f> &npts);

	// Convert OpenCV DMatch to Match (pair<int, int>)
	void ConvertMatches(const vector<cv::DMatch> &vDMatches, vector<pair<int, int> > &vMatches);

	int GetGridIndexLeft(const cv::Point2f &pt, int type);

	int GetGridIndexRight(const cv::Point2f &pt);

	// Assign Matches to Cell Pairs
	void AssignMatchPairs(int GridType);

	// Verify Cell Pairs
	void VerifyCellPairs(int RotationType);

	// Get Neighbor 9
	vector<int> GetNB9(const int idx, const cv::Size& GridSize);

	void InitalizeNiehbors(cv::Mat &neighbor, const cv::Size& GridSize);

	void SetScale(int Scale);

	// Run
	int run(int RotationType);
};
