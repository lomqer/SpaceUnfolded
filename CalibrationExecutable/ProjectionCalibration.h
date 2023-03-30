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

void calibrate(const char* dataFolder, const int* projectionSize, const int* contourPoints, int meshRefinementCount, int meshRefinementDistLimit);

void capture(const char* dataFolder, int cameraId, int projectionWidth, int projectionHeight, int projectionMonitorOffsetX, int projectionMonitorOffsetY);
