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
#ifdef PROJECTIONCALIBRATION_EXPORTS
#define PROJECTIONCALIBRATION_API __declspec(dllexport)
#else
#define PROJECTIONCALIBRATION_API __declspec(dllimport)
#endif

extern "C" PROJECTIONCALIBRATION_API void calibrateRefined(const char* dataFolder, const int* contourPoints, int meshRefinementCount, int meshRefinementDistLimit);

extern "C" PROJECTIONCALIBRATION_API void calibrate(const char* dataFolder, const int* contourPoints);