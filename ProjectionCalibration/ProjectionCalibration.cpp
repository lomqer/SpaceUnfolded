/*
*   Single Point-of-view Projector Calibration System
*	Copyright (C) 2023 Damian Ziaber
*
*	This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*
*	This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License along with this program. If not, see https://www.gnu.org/licenses/.
*/
#include "pch.h"
#include "ProjectionCalibration.h"
#include "GrayCodeCalibration.h"
#include <opencv2/calib3d.hpp>


void calibrateRefined(const char* dataFolder, const int* projectionSize, const int* contourPoints, int meshRefinementCount, int meshRefinementDistLimit) {
	//Load in captured frames from the data folder
	std::vector<cv::Mat> frames = FileUtil::getPatternImages(dataFolder);

	//Order the desired projection contour points for calculating the initial homography
	std::vector<cv::Point> contour;
	uint8_t min_sum = 0, max_sum = 2, min_diff = 3, max_diff = 1;
	for (int i = 0; i < 4; i++) {
		const int x = contourPoints[2 * i], y = contourPoints[2 * i + 1];
		contour.push_back(cv::Point(x, y));
		if (x + y < contourPoints[2 * min_sum] + contourPoints[2 * min_sum + 1])
			min_sum = i;
		else if (x + y > contourPoints[2 * max_sum] + contourPoints[2 * max_sum + 1])
			max_sum = i;
		if (x - y < contourPoints[2 * min_diff] + contourPoints[2 * min_diff + 1])
			min_diff = i;
		else if (x - y > contourPoints[2 * max_diff] + contourPoints[2 * max_diff + 1])
			max_diff = i;
	}

	const std::vector<cv::Point> sortedContour({ contour[min_sum], contour[max_diff], contour[max_sum], contour[min_diff] });
	const std::vector<cv::Point> projectionContour({ cv::Point(0, 0), cv::Point(projectionSize[0], 0), cv::Point(projectionSize[0], projectionSize[1]), cv::Point(0, projectionSize[1])});
	
	// Process the frames and save pixel maps to the data folder
	cv::Mat map1, map2;
	GrayCodeCalibration::process(frames, map1, map2, FileUtil::getGrayCodeSize(dataFolder), cv::Size(projectionSize[0], projectionSize[1]), cv::findHomography(sortedContour, projectionContour, cv::noArray()), meshRefinementCount, meshRefinementDistLimit);
	FileUtil::saveMap(dataFolder, map1, map2);
}

void calibrate(const char* dataFolder, const int* projectionSize, const int* contourPoints) {
	calibrateRefined(dataFolder, projectionSize, contourPoints, 2, 7);
}