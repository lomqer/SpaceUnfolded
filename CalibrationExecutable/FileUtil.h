/*
*   Single Point-of-view Projector Calibration System
*	Copyright (C) 2023 Damian Ziaber
*
*	This program is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
*
*	This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
*
*	You should have received a copy of the GNU Lesser General Public License along with this program. If not, see https://www.gnu.org/licenses/.
*/

#pragma once
#include <vector>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>

namespace FileUtil {

	void savePatternImages(const char* dataDirName, std::vector<cv::Mat> frames);

	std::vector<cv::Mat> getPatternImages(const char* dataDirName);

	void saveMap(const char* dataDirName, cv::Mat map1, cv::Mat map2);

	void getMap(const char* dataDirName, cv::Mat& map1, cv::Mat& map2);

	void saveProjectionSize(const char* dataDirName, int projectionWidth, int projectionHeight);

	cv::Size getProjectionSize(const char* dataDirName);
}
