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

#include <iostream>
#include <fstream>
#include <string>
#include <getopt.h>

// WNC
#include "vision_tracker.hpp"




std::vector<std::string> getInputFileList(const char* filePath)
{
	// Read lines from the input lists file
	// and store the paths to inputs in strings
	std::ifstream inputList(filePath);
	std::string fileLine;
	std::vector<std::string> lines;
	while (std::getline(inputList, fileLine))
	{
		if (fileLine.empty()) continue;
		lines.push_back(fileLine);
	}
	return lines;
}


void showTrackedHumanResults(std::vector<TrackedObj>& objList)
{
	for (int i=0; i<objList.size(); i++)
  {
    TrackedObj& obj = objList[i];

	  cout << "Obj[" << obj.id << "] ";
    cout << "Type: " << obj.type << " ";
    cout << "Conf: " << obj.confidence << " ";
    cout << "Loc: (" << obj.pLoc.x << " m, " << obj.pLoc.y << " m, " << obj.pLoc.z << " m)" << endl;
  }
}


int main(int argc, char** argv)
{
	// Command line arguments
	const char* videoFilePath = "";

	// Process command line arguments
	int opt = 0;
	while ((opt = getopt(argc, argv, "hi:d:o:r:l:u")) != -1)
	{
		switch (opt)
		{
			case 'h':
				std::cout
					<< "\nDESCRIPTION:\n"
					<< "------------\n"
					<< "An ADAS application (with LDW, FCW).\n"
					<< "\n\n"
					<< "REQUIRED ARGUMENTS:\n"
					<< "-------------------\n"
					<< "  -i  <FILE>   Path to a video file for the network.\n"
					<< "\n"
					<< std::endl;

				std::exit(0);
			case 'i':
				videoFilePath = optarg;
				break;
			default:
				std::cout << "Invalid parameter specified. Please run snpe-sample with the -h flag to see required arguments" << std::endl;
				std::exit(1);
		}
	}


	// ============================================ //
	//                  Entry Point                 //
	// ============================================ //
	int idxFrame = 0;
	cv::Mat img;
	cv::VideoCapture cap(videoFilePath);
	VisionTracker vTracker("config/config.txt");

	VisionTrackingResults result;

	std::cout << "Start WNC Vision Tracking" << std::endl;
	std::cout << "-------------------------------------------------" << std::endl;

	while(1)
	{
		// Read video frame
		cap >> img;
		if (img.empty())
		{
			break;
		}
		idxFrame += 1;

		// Run Object Tracking
		vTracker.run(img);

		// Get Tracked Results
		vTracker.getResults(result);

		// Show Tracked Results
		if (vTracker.isFinishDetection())
		{
			std::cout << "\nFrame: [" << idxFrame << "]" << std::endl;
			std::cout << "-------------------------------------------------" << std::endl;
			showTrackedHumanResults(result.humanObjList);
		}
	}

	std::cout << "-------------------------------------------------" << std::endl;
	std::cout << "Stop WNC Vision Tracking" << std::endl;

	cap.release();

	return 0;
}
