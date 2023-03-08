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
#include "FileUtil.h"

using namespace std;
namespace fs = std::filesystem;

vector<cv::Mat> FileUtil::getPatternImages(const char* dataDirName) {
	vector<cv::Mat> matVector;
	fs::path dataDir(dataDirName);
	if (fs::is_directory(dataDir)) {
		fs::path whiteFrame("whiteFrame.png");
		fs::path blackFrame("blackFrame.png");
		matVector.push_back(cv::imread((dataDir / whiteFrame).string()));
		matVector.push_back(cv::imread((dataDir / blackFrame).string()));
		string patternName = "pattern0.png";
		for (int i = 0; fs::is_regular_file(dataDir / fs::path(patternName)); patternName = "pattern" + std::to_string(++i) + ".png") {
			matVector.push_back(cv::imread((dataDir / patternName).string()));
		}
	}
	return matVector;
}

cv::Size FileUtil::getProjectionSize(const char* dataDirName) {
	fs::path dataDir(dataDirName);
	fs::path fileName("projection_size.ext");

	cv::FileStorage file((dataDir / fileName).string(), cv::FileStorage::READ);
	int h, w;
	file["h"] >> h;
	file["w"] >> w;
	file.release();
	return cv::Size(w, h);
}

void FileUtil::saveMap(const char* dataDirName, cv::Mat map1, cv::Mat map2) {
	fs::path dataDir(dataDirName);
	fs::path fileName("map.ext");

	cv::FileStorage file((dataDir / fileName).string(), cv::FileStorage::WRITE);
	file << "map1" << map1 << "map2" << map2;
	file.release();
}