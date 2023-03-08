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
#include <queue>
#include <opencv2/imgproc/imgproc.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/boost/iterator/transform_iterator.hpp>
#include <CGAL/draw_triangulation_2.h>


namespace DelaunayBasedCorrection {
	void findMaps(std::vector<cv::Point> cameraPoints, std::vector<cv::Point> projectionPoint, cv::Mat& map1, cv::Mat& map2, cv::Size projectionSize, cv::Mat initialHomography);
}
