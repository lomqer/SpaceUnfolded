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
#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include "FileUtil.h"
#include "DelaunayBasedCorrection.h"

namespace GrayCodeCalibration {
	void process(std::vector<cv::Mat> frames, cv::Mat& map1, cv::Mat& map2, cv::Size projectionSize, cv::Mat initial_homography = cv::Mat::eye(3, 3, CV_64F));

	void getCameraProjectionPairs(std::vector<cv::Point>& cameraPoints, std::vector<cv::Point>& projectionPoints, const std::vector<cv::Mat> frames, const cv::Size projectionSize, const cv::Mat mask, const int min_point_count = 1000, const int threshold = 4);

	cv::Mat getMask(const cv::Mat white_frame_bgr, const cv::Mat black_frame_bgr, const int threshold = 4);
}
