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

#include <iostream>
#include "ProjectionCalibration.h"

int main(int argc, char** argv)
{
	if (argc <= 1) {
		std::cout << "Please specify if you want to calibrate or capture image by using a 'calibrate' or 'capture' argument.";
		return 0;
	}
	if (strcmp(argv[1], "capture") == 0) {
		if (argc < 8) {
			std::cout << "Too few arguments for capture, specify data folder, camera id, gray code width, height and projector screen offset x and y.";
			return 0;
		}
		if (argc > 8) {
			std::cout << "Too many arguments for capture, specify only data folder, camera id, gray code width, height and projector screen offset x and y.";
			return 0;
		}
		const char* dataFolder = argv[2];
		const int cameraId = atoi(argv[3]);
		const int width= atoi(argv[4]);
		const int height = atoi(argv[5]);
		const int offsetX = atoi(argv[6]);
		const int offsetY = atoi(argv[7]);
		capture(dataFolder, cameraId, width, height, offsetX, offsetY);
	}
	else if (strcmp(argv[1], "calibrate") == 0) {
		if (argc < 15) {
			std::cout << "Too few arguments for calibrate, specify data folder, size width, height, 4 x y pairs for projection corners, refinement count and distance limit.";
			return 0;
		}
		if (argc > 15) {
			std::cout << "Too many arguments for calibrate, specify ONLY data folder, size width, height, 4 x y pairs for projection corners, refinement count and distance limit.";
			return 0;
		}
		const char* dataFolder = argv[2];
		int* projectionSize = (int*)malloc(2 * sizeof(int));
		projectionSize[0] = atoi(argv[3]);
		projectionSize[1] = atoi(argv[4]);
		int* cornerCoordinates = (int*)malloc(8 * sizeof(int));
		for (int i = 0; i < 8; i++)
			cornerCoordinates[i] = atoi(argv[i+5]);
		const int refinementCount = atoi(argv[13]);
		const int distLimit = atoi(argv[14]);
		calibrate(dataFolder, projectionSize, cornerCoordinates, refinementCount, distLimit);
		free(cornerCoordinates);
	}
	else {
		std::cout << "Unrecognized argument '" << argv[0] <<"', please choose either 'calibrate' or 'capture'.";
	}
	return 0;
}
